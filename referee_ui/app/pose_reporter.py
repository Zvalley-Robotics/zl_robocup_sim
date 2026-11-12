#!/usr/bin/env python3
"""NX 侧位姿上报器：Brain 场地系位姿 -> UDP JSON（默认 10 Hz）-> 裁判机。

话题（实机 real_robot:=true 时 brain 的 robot_name 为空，故默认无后缀）:
    /soccer/robot_pose         geometry_msgs/Pose2D       必需
    /soccer/field_dimensions   std_msgs/Float64MultiArray 可选（transient_local）
    /soccer/ball_position      geometry_msgs/Point        可选（--no-ball 关闭）

报文（UTF-8 文本，魔数 + 空格 + JSON，单包 < 512 B）:
    RPSE1 {"v":1,"team":29,"player":1,"robot":"","seq":321,
           "x":1.234,"y":-0.567,"theta":0.891,
           "age_ms":70,"stamp":1761.23,
           "field":[14.0,8.5,2.1,2.6,1.5,3.0,5.8,1.0,4.0],
           "ball":{"x":2.1,"y":0.3,"age_ms":120}}

    age_ms = 本机最近一次 robot_pose 距今毫秒：即使定位断流也继续发包，
             裁判机据此区分"网络断"与"定位断"（接收年龄 vs 位姿年龄）。

用法（NX，域 100，DDS 环境必须与 brain 完全一致）:
    python3 referee_ui/app/pose_reporter.py --target 10.0.0.10 --player-id 1   # 裁判机 IP
    referee_ui/scripts/start_pose_reporter.sh run --target 10.0.0.10 --player-id 1

只读观测，不发布任何 ROS 话题，不影响 brain / 桥接 / zl 栈。
"""
from __future__ import annotations

import argparse
import json
import math
import os
import socket
import sys
import time

import rclpy
from geometry_msgs.msg import Point, Pose2D
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import Float64MultiArray

MAGIC = b"RPSE1"
BROADCAST_TARGETS = ("broadcast", "255.255.255.255")
# brain 失球后会停止发布 /soccer/ball_position；超过该时长就不再
# 随包转发球位，避免裁判机把最后收到的旧位置当成“当前球”（幽灵球）。
BALL_POSITION_STALE_MS = 500.0


def _reliable(depth: int) -> QoSProfile:
    return QoSProfile(depth=depth, reliability=ReliabilityPolicy.RELIABLE,
                      history=HistoryPolicy.KEEP_LAST,
                      durability=DurabilityPolicy.VOLATILE)


def _latched(depth: int = 1) -> QoSProfile:
    """匹配 brain 的 QoS(1).transient_local()，启动晚也能收到已发布的场地参数。"""
    return QoSProfile(depth=depth, reliability=ReliabilityPolicy.RELIABLE,
                      history=HistoryPolicy.KEEP_LAST,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


class PoseReporter(Node):
    def __init__(self, args) -> None:
        super().__init__("pose_reporter")
        self.args = args
        suffix = args.suffix or ""
        pose_topic = "/soccer/robot_pose" + suffix
        field_topic = "/soccer/field_dimensions" + suffix
        ball_topic = "/soccer/ball_position" + suffix

        self._pose = None          # (x, y, theta, ros_stamp, monotonic)
        self._field = None         # 9 元列表
        self._ball = None          # (x, y, monotonic)
        self._seq = 0
        self._sent = 0
        self._send_err = 0
        self._pose_count = 0
        self._last_print = time.monotonic()
        self._last_no_pose_warn = 0.0
        self._announced = False
        self._send_err_warned = False

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        host = args.target
        if host in BROADCAST_TARGETS:
            host = "255.255.255.255"
            self._broadcast = True
        else:
            self._broadcast = False
        self._addr = (host, args.port)

        self.create_subscription(Pose2D, pose_topic, self._on_pose, _reliable(10))
        self.create_subscription(Float64MultiArray, field_topic,
                                 self._on_field, _latched(1))
        if not args.no_ball:
            self.create_subscription(Point, ball_topic, self._on_ball, _reliable(10))

        self.create_timer(1.0 / max(args.hz, 0.1), self._tick)

        print("=" * 64, flush=True)
        print("位姿上报器（只读观测，不影响 brain / 桥接 / zl 栈）", flush=True)
        print(f"  订阅: {pose_topic}  (+ {field_topic}"
              + ("" if args.no_ball else f" + {ball_topic}") + ")", flush=True)
        print(f"  上报: UDP {self._addr[0]}:{self._addr[1]} @ {args.hz:g} Hz"
              f"{'（广播）' if self._broadcast else ''}", flush=True)
        print(f"  标识: team={args.team_id} player={args.player_id} "
              f"robot={args.robot_name or '(空)'}", flush=True)
        print(f"  环境: ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '(未设)')} "
              f"RMW={os.environ.get('RMW_IMPLEMENTATION', '(默认)')}", flush=True)
        cyclonedds = os.environ.get("CYCLONEDDS_URI", "")
        if cyclonedds:
            print(f"        CYCLONEDDS_URI={cyclonedds}", flush=True)
        if os.environ.get("ROS_DOMAIN_ID") not in (None, "", str(args.domain_id)):
            print(f"[警告] ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID')} 与 brain 期望的 "
                  f"{args.domain_id} 不同，可能收不到位姿", flush=True)
        print("  提示: RMW/CYCLONEDDS_URI 必须与 brain 完全一致，否则同域也收不到（静默）",
              flush=True)
        print("=" * 64, flush=True)

    # ---- 订阅回调 --------------------------------------------------------
    def _on_pose(self, msg: Pose2D) -> None:
        if not all(math.isfinite(v) for v in (msg.x, msg.y, msg.theta)):
            return
        stamp = self.get_clock().now().nanoseconds / 1e9
        self._pose = (float(msg.x), float(msg.y), float(msg.theta),
                      stamp, time.monotonic())
        self._pose_count += 1
        if not self._announced:
            self._announced = True
            print(f"[位姿] 已收到 {msg.x:+.3f} {msg.y:+.3f} "
                  f"θ={msg.theta:+.3f}，开始上报", flush=True)

    def _on_field(self, msg: Float64MultiArray) -> None:
        if len(msg.data) >= 9 and all(math.isfinite(v) for v in msg.data[:9]):
            self._field = [float(v) for v in msg.data[:9]]

    def _on_ball(self, msg: Point) -> None:
        if math.isfinite(msg.x) and math.isfinite(msg.y):
            self._ball = (float(msg.x), float(msg.y), time.monotonic())

    # ---- 周期上报 --------------------------------------------------------
    def _tick(self) -> None:
        now = time.monotonic()
        pose = self._pose
        if pose is None:
            # 没有定位输出就不发：裁判机侧表现为"无此机器人"，而非假位置。
            if now - self._last_no_pose_warn >= 5.0:
                self._last_no_pose_warn = now
                print("[等待] 尚未收到 /soccer/robot_pose；"
                      "检查 brain 是否运行、域号与 RMW 是否一致", flush=True)
            self._print_periodic(now)
            return

        x, y, theta, ros_stamp, pose_mono = pose
        payload = {
            "v": 1,
            "team": self.args.team_id,
            "player": self.args.player_id,
            "robot": self.args.robot_name or "",
            "seq": self._seq,
            "x": round(x, 4),
            "y": round(y, 4),
            "theta": round(theta, 4),
            "age_ms": round((now - pose_mono) * 1000.0, 1),
            "stamp": round(ros_stamp, 3),
        }
        if self._field is not None:
            payload["field"] = [round(v, 4) for v in self._field]
        if self._ball is not None and not self.args.no_ball:
            bx, by, ball_mono = self._ball
            ball_age_ms = (now - ball_mono) * 1000.0
            if ball_age_ms <= BALL_POSITION_STALE_MS:
                payload["ball"] = {"x": round(bx, 4), "y": round(by, 4),
                                   "age_ms": round(ball_age_ms, 1)}

        frame = MAGIC + b" " + json.dumps(
            payload, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
        try:
            self._sock.sendto(frame, self._addr)
            self._seq += 1
            self._sent += 1
        except OSError as exc:
            self._send_err += 1
            if not self._send_err_warned:
                self._send_err_warned = True
                print(f"[错误] 发送失败 {self._addr}: {exc}（后续同类错误不再逐条打印）",
                      flush=True)
        self._print_periodic(now)

    def _print_periodic(self, now: float) -> None:
        span = now - self._last_print
        if span < max(self.args.print_every, 1.0):
            return
        hz = self._pose_count / span
        self._pose_count = 0
        self._last_print = now
        pose_age = ("--" if self._pose is None
                    else f"{(now - self._pose[4]) * 1000.0:.0f}ms")
        print(f"[状态] 位姿回调 {hz:4.1f} Hz · 位姿年龄 {pose_age} · "
              f"已发 {self._sent} 包 · 发送错误 {self._send_err}", flush=True)

    def close(self) -> None:
        try:
            self._sock.close()
        except OSError:
            pass


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="NX 位姿上报器：/soccer/robot_pose -> 裁判机 UDP")
    ap.add_argument("--target", default=os.environ.get("POSEGW_TARGET", ""),
                    help="裁判机 IP（或 broadcast / 255.255.255.255）；"
                         "也可用环境变量 POSEGW_TARGET")
    ap.add_argument("--port", type=int, default=3940, help="裁判机监听端口（默认 3940）")
    ap.add_argument("--hz", type=float, default=10.0, help="上报频率（默认 10 Hz）")
    ap.add_argument("--team-id", type=int, default=29, help="队号（与 brain game.team_id 一致）")
    ap.add_argument("--player-id", type=int, required=True,
                    help="球员号（与 brain game.player_id 一致）")
    ap.add_argument("--robot-name", default="",
                    help="显示名（实机 brain 为空；仅用于面板标签）")
    ap.add_argument("--suffix", default="",
                    help="话题后缀覆盖（如 /robot_01；实机默认空）")
    ap.add_argument("--no-ball", action="store_true", help="不附带球位置")
    ap.add_argument("--domain-id", type=int, default=100, help="期望的 ROS 域号（默认 100，仅自检）")
    ap.add_argument("--print-every", type=float, default=10.0, help="状态行间隔秒数（默认 10）")
    args = ap.parse_args(argv)

    if not args.target:
        ap.error("必须指定 --target（裁判机 IP），或设置 POSEGW_TARGET 环境变量")
    if not 0 <= args.team_id <= 255 or not 1 <= args.player_id <= 255:
        ap.error("team-id 需在 0..255，player-id 需在 1..255")

    rclpy.init()
    node = PoseReporter(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:  # 已 shutdown（SIGINT 路径）
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
