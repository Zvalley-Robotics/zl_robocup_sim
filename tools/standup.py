#!/usr/bin/env python3
"""六机仿真立起：ready → reload（回到初始站立位姿）→ walk → 等 RL_ACTIVE 且 alt≥阈值。

运行在域 10（sim / zl 同域），由 `robocup_sim standup` 调用。只依赖 robot_interfaces /
yesense_interface 与 std_srvs，不依赖旧 joint_test 脚本。
"""
from __future__ import annotations

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist  # noqa: F401  (类型支持预加载)
from robot_interfaces.msg import RobotIntent, RobotState
from std_srvs.srv import Trigger
from yesense_interface.msg import NavAll

IDLE = 2
DO_READY = 3
READY = 4
RL_STANDBY = 5
RL_ACTIVE = 6
FAULT = 10
DAMP = 11

STATE_NAME = {
    0: "INITIAL", 1: "?", 2: "IDLE", 3: "DO_READY", 4: "READY", 5: "RL_STANDBY",
    6: "RL_ACTIVE", 10: "FAULT", 11: "DAMP",
}


class Standup(Node):
    def __init__(self, robots: list[str], alt_threshold: float, timeout: float):
        super().__init__("robocup_sim_standup")
        self.robots = robots
        self.alt_threshold = alt_threshold
        self.deadline = time.monotonic() + timeout
        self.state = {r: None for r in robots}
        self.alt = {r: None for r in robots}
        self.reloaded = {r: 0 for r in robots}
        self.last_reload = {r: 0.0 for r in robots}
        self.done = {r: False for r in robots}

        high_freq = QoSProfile(
            depth=5, history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT)
        intent_qos = QoSProfile(
            depth=10, history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE)
        self.pubs = {}
        self.reload_clients = {}
        for r in robots:
            self.create_subscription(
                RobotState, f"/{r}/state",
                lambda msg, name=r: self.state.__setitem__(name, int(msg.state)), high_freq)
            self.create_subscription(
                NavAll, f"/{r}/imu/nav_all",
                lambda msg, name=r: self.alt.__setitem__(name, float(msg.pos.altitude)), high_freq)
            self.pubs[r] = self.create_publisher(RobotIntent, f"/{r}/input/intent", intent_qos)
            self.reload_clients[r] = self.create_client(Trigger, f"/reload_{r}")

    def send_status(self, robot: str, command: str) -> None:
        msg = RobotIntent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.source = "api"
        msg.intent_type = "status"
        msg.command = command
        msg.action = ""
        msg.priority = 60
        self.pubs[robot].publish(msg)

    def call_reload(self, robot: str) -> None:
        client = self.reload_clients[robot]
        if not client.service_is_ready():
            return
        req = Trigger.Request()
        client.call_async(req)
        self.reloaded[robot] += 1
        self.last_reload[robot] = time.monotonic()
        self.get_logger().info(f"{robot}: reload #{self.reloaded[robot]}")

    def tick(self) -> bool:
        now = time.monotonic()
        if now > self.deadline:
            return False
        for r in self.robots:
            st = self.state[r]
            alt = self.alt[r]
            if st is None or alt is None:
                continue
            if self.done[r]:
                continue
            if st == FAULT:
                self.get_logger().warn(f"{r}: FAULT，等待人工 reset_fault")
                continue
            if st == DAMP:
                self.send_status(r, "reinit")
                continue
            if alt >= self.alt_threshold and st == RL_ACTIVE:
                self.done[r] = True
                self.get_logger().info(f"{r}: 已站立 alt={alt:.2f} state={STATE_NAME.get(st, st)}")
                continue
            if st < READY:
                self.send_status(r, "ready")
                continue
            # READY / RL_STANDBY / RL_ACTIVE 但 alt 不足：重载到初始站立位姿再 walk。
            if now - self.last_reload[r] > 5.0 and self.reloaded[r] < 3:
                self.call_reload(r)
            self.send_status(r, "walk")
        return not all(self.done.values())


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--robots", nargs="+", required=True)
    ap.add_argument("--alt-threshold", type=float, default=0.5)
    ap.add_argument("--timeout", type=float, default=60.0)
    args = ap.parse_args()

    rclpy.init()
    node = Standup(args.robots, args.alt_threshold, args.timeout)
    try:
        while rclpy.ok() and node.tick():
            rclpy.spin_once(node, timeout_sec=0.2)
            time.sleep(0.1)
    finally:
        pending = [r for r in args.robots if not node.done[r]]
        if pending:
            print(f"[standup][WARN] 未达标: {' '.join(pending)}", file=sys.stderr)
        else:
            print("[standup] 全部机器人已站立", file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
    return 0 if not pending else 1


if __name__ == "__main__":
    raise SystemExit(main())
