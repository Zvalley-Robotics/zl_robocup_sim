#!/usr/bin/env python3
"""实机位置监视：UDP 3940 位姿上报监听（pose_reporter → 裁判机，默认 10 Hz）。

协议（详见 pose_reporter.py 头注释）：

    RPSE1 {"v":1,"team":29,"player":1,"robot":"","seq":321,
           "x":1.234,"y":-0.567,"theta":0.891,
           "age_ms":70,"stamp":1761.23,
           "field":[14.0,8.5,2.1,2.6,1.5,3.0,5.8,1.0,4.0],
           "ball":{"x":2.1,"y":0.3,"age_ms":120}}

两种用法:
  1) 独立终端工具（本文件 __main__）:
       python3 pose_watch.py [--bind 0.0.0.0] [--port 3940] [--stale 0.6] [--timeout 3.0]
     周期打印位置表；脚本化检查用 --wait 3（等到首个有效位姿，0=成功 / 1=超时）。
  2) 被 gc_ui.py 复用（同目录 import）: PoseWatch 线程 + snapshot()，
     位置表经 /api/poses 下发到网页面板。

与 robot_watch.py（UDP 3939 心跳）互不干扰：两套监听、两个端口。
"""
from __future__ import annotations

import argparse
import json
import math
import socket
import sys
import threading
import time

MAGIC = b"RPSE1"
DEFAULT_PORT = 3940
DEFAULT_STALE = 0.6                  # 秒；超过即"位置陈旧"（10 Hz 约 6 帧）
DEFAULT_TIMEOUT = 3.0                # 秒；超过即判离线（保留最后位置灰显）
FIELD_MARGIN_M = 1.5                 # 场外待命/入场区：边线外 1.5 m 内不算越界


def parse_packet(data: bytes) -> dict | None:
    """解析一帧位姿包；非法返回 None（调用方只累加 bad_total）。"""
    if len(data) < len(MAGIC) + 2 or data[:len(MAGIC)] != MAGIC:
        return None
    try:
        payload = json.loads(data[len(MAGIC):].decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError):
        return None
    if not isinstance(payload, dict):
        return None
    try:
        team = int(payload["team"])
        player = int(payload["player"])
        x = float(payload["x"])
        y = float(payload["y"])
        theta = float(payload["theta"])
    except (KeyError, TypeError, ValueError):
        return None
    if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(theta)):
        return None

    entry = {
        "team": team,
        "player": player,
        "robot": str(payload.get("robot", "") or ""),
        "x": x,
        "y": y,
        "theta": theta,
        "seq": _as_int(payload.get("seq"), -1),
        "age_ms": _as_float(payload.get("age_ms"), 0.0),
        "stamp": _as_float(payload.get("stamp"), 0.0),
        "field": _as_field(payload.get("field")),
        "ball": _as_ball(payload.get("ball")),
    }
    return entry


def _as_int(value, default: int) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _as_float(value, default: float) -> float:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return default
    return out if math.isfinite(out) else default


def _as_field(value) -> list | None:
    """9 元场地参数（length/width/penaltyDist/goalWidth/circle/...）；非法返回 None。"""
    if not isinstance(value, (list, tuple)) or len(value) < 9:
        return None
    try:
        vals = [float(v) for v in value[:9]]
    except (TypeError, ValueError):
        return None
    if not all(math.isfinite(v) and v > 0 for v in vals[:2]):
        return None
    return vals


def _as_ball(value) -> dict | None:
    if not isinstance(value, dict):
        return None
    try:
        bx = float(value["x"])
        by = float(value["y"])
    except (KeyError, TypeError, ValueError):
        return None
    if not (math.isfinite(bx) and math.isfinite(by)):
        return None
    return {"x": bx, "y": by, "age_ms": _as_float(value.get("age_ms"), 0.0)}


def _wrap_pi(angle: float) -> float:
    """归一化到 [-π, π)。"""
    while angle >= math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _mirror180(value: dict) -> dict:
    """场地系 180° 旋转（对手 Brain 的 +x 指向我方球门）。"""
    out = dict(value)
    out["x"] = -float(value["x"])
    out["y"] = -float(value["y"])
    if "theta" in value:
        out["theta"] = _wrap_pi(float(value["theta"]) + math.pi)
    return out


class PoseWatch(threading.Thread):
    """UDP 3940 位姿监听线程；按 (team, player) 维护最新位姿与年龄。

    线程自身不打印（供 gc_ui.py 静默复用）；终端展示由 __main__ 负责。
    """

    def __init__(self, bind: str = "0.0.0.0", port: int = DEFAULT_PORT,
                 stale: float = DEFAULT_STALE, timeout: float = DEFAULT_TIMEOUT,
                 our_team: int | None = None) -> None:
        super().__init__(name="pose-watch", daemon=True)
        self.bind = bind
        self.port = port
        self.stale = float(stale)
        self.timeout = float(timeout)
        # 显示基准 = 我方 Brain 场地系；对手（team != our_team）上报的坐标
        # 旋转 180° 后叠加——两队 Brain 场地系天生互为镜像，直接混画会重叠在
        # 同一半场。None = 不转换（单队 / 不关心方向）。
        self.our_team = None if our_team is None else int(our_team)
        self._lock = threading.Lock()
        self._robots: dict = {}      # (team, player) -> 解析后的条目 + 接收统计
        self._recv_total = 0
        self._bad_total = 0
        self._field: list | None = None
        self._stop_flag = threading.Event()
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # 不设 SO_REUSEADDR：与 robot_watch 同样的策略，端口被占立即暴露。

    # ---- 生命周期 --------------------------------------------------------
    def start(self) -> None:  # noqa: A003（沿用 threading.Thread 接口）
        """绑定端口并启动线程；端口被占用时抛 OSError（未启动线程）。"""
        self._sock.bind((self.bind, self.port))
        self._sock.settimeout(0.5)
        super().start()

    def stop(self) -> None:
        self._stop_flag.set()
        try:
            self._sock.close()
        except OSError:
            pass
        if self.is_alive():
            self.join(timeout=2.0)

    # ---- 线程主体 --------------------------------------------------------
    def run(self) -> None:
        while not self._stop_flag.is_set():
            try:
                data, addr = self._sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break                # stop() 关闭套接字
            entry = parse_packet(data)
            if entry is None:
                with self._lock:
                    self._bad_total += 1
                continue
            now = time.time()
            key = (entry["team"], entry["player"])
            with self._lock:
                self._recv_total += 1
                prev = self._robots.get(key)
                gaps = prev.get("gaps", 0) if prev else 0
                resets = prev.get("resets", 0) if prev else 0
                last_seq = prev.get("seq", -1) if prev else -1
                seq = entry["seq"]
                if prev is not None and seq >= 0 and last_seq >= 0:
                    if seq > last_seq:
                        gaps += seq - last_seq - 1
                    elif seq < last_seq:
                        resets += 1          # 上报器重启，序号归零
                entry.update(
                    last_seen=now,
                    first_seen=prev["first_seen"] if prev else now,
                    count=(prev.get("count", 0) if prev else 0) + 1,
                    gaps=gaps,
                    resets=resets,
                    addr=addr[0],
                )
                self._robots[key] = entry
                if entry["field"] is not None:
                    self._field = entry["field"]

    # ---- 快照 ------------------------------------------------------------
    def snapshot(self, now: float | None = None) -> dict:
        """返回位置表（JSON 可序列化）；fresh/online 按本地接收年龄判定。"""
        now = time.time() if now is None else now
        with self._lock:
            bad = self._bad_total
            total_recv = self._recv_total
            field = list(self._field) if self._field else None
            rows = []
            for entry in self._robots.values():
                age = now - entry["last_seen"]
                x, y, theta = entry["x"], entry["y"], entry["theta"]
                field_e = entry["field"] or field
                # 初值 0/0/0 是 brain 未定位收敛时的 robotPoseToField 原值。
                unlocalized = (abs(x) < 1e-6 and abs(y) < 1e-6
                               and abs(theta) < 1e-6)
                in_field = True
                if field_e:
                    in_field = (abs(x) <= field_e[0] / 2.0 + FIELD_MARGIN_M
                                and abs(y) <= field_e[1] / 2.0 + FIELD_MARGIN_M)
                is_our = (self.our_team is None or entry["team"] == self.our_team)
                shown = {"x": x, "y": y, "theta": theta} if is_our \
                    else _mirror180({"x": x, "y": y, "theta": theta})
                ball = entry["ball"]
                if ball is not None and not is_our:
                    ball = _mirror180(ball)
                rows.append({
                    "team": entry["team"],
                    "player": entry["player"],
                    "robot": entry["robot"],
                    # x/y/theta = 显示系（我方 Brain 场地系，对手已旋转）
                    "x": round(shown["x"], 4),
                    "y": round(shown["y"], 4),
                    "theta": round(shown["theta"], 4),
                    # raw_* = 该机 Brain 原始场地系（排障用）
                    "raw_x": round(x, 4),
                    "raw_y": round(y, 4),
                    "raw_theta": round(theta, 4),
                    "is_our": is_our,
                    "age_sec": round(age, 2),
                    "age_ms": round(entry["age_ms"], 1),
                    "fresh": age <= self.stale,
                    "online": age <= self.timeout,
                    "valid": in_field and not unlocalized,
                    "unlocalized": unlocalized,
                    "seq": entry["seq"],
                    "gaps": entry["gaps"],
                    "resets": entry["resets"],
                    "count": entry["count"],
                    "stamp": entry["stamp"],
                    "addr": entry["addr"],
                    "field": entry["field"],
                    "ball": ball,
                })
        rows.sort(key=lambda r: (r["team"], r["player"]))
        return {
            "stale_sec": self.stale,
            "timeout_sec": self.timeout,
            "listening": f"{self.bind}:{self.port}",
            "our_team": self.our_team,
            "total": len(rows),
            "fresh": sum(1 for r in rows if r["fresh"]),
            "online": sum(1 for r in rows if r["online"]),
            "received_total": total_recv,
            "bad_total": bad,
            "field": field,
            "list": rows,
        }


# ---- 终端展示 ------------------------------------------------------------
def _print_table(snap: dict) -> None:
    stamp = time.strftime("%H:%M:%S")
    head = (f"[{stamp}] 位置 {snap['fresh']}/{snap['total']} 台新鲜 "
            f"(累计 {snap['received_total']} 包，坏包 {snap['bad_total']})")
    if not snap["list"]:
        print(head + "  —— 暂无位姿包（检查 pose_reporter 的 --target/端口/防火墙）",
              flush=True)
        return
    print(head, flush=True)
    for r in snap["list"]:
        mark = "●" if r["fresh"] else ("○" if r["online"] else "×")
        name = r["robot"] or f"P{r['player']}"
        side = "[对]" if not r["is_our"] else ""
        flag = " 未定位" if r["unlocalized"] else ("" if r["valid"] else " 越界")
        print(f"  {mark} {name:<10}{side:<3} team={r['team']:>3} player={r['player']:<2} "
              f"x={r['x']:>7.3f} y={r['y']:>7.3f} θ={r['theta']:>7.3f} "
              f"收={r['age_sec']:>5.2f}s前 位姿={r['age_ms']:>6.0f}ms"
              f" 丢包={r['gaps']:<4}{flag}", flush=True)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="实机位置监视（pose_reporter UDP 位姿包，无 ROS 依赖）")
    ap.add_argument("--bind", default="0.0.0.0", help="监听地址（默认 0.0.0.0）")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT,
                    help=f"监听端口（默认 {DEFAULT_PORT}）")
    ap.add_argument("--stale", type=float, default=DEFAULT_STALE,
                    help="陈旧判定秒数（默认 0.6）")
    ap.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT,
                    help="离线判定秒数（默认 3.0）")
    ap.add_argument("--our-team-id", type=int, default=None,
                    help="我方队号；对手上报自动旋转 180° 到同一显示系（默认不转换）")
    ap.add_argument("--interval", type=float, default=1.0, help="刷新间隔秒数（默认 1.0）")
    ap.add_argument("--wait", type=float, default=0.0, metavar="SECS",
                    help="等待首个新鲜位姿至多 SECS 秒，打印一次后退出（0=成功 1=超时）")
    args = ap.parse_args(argv)

    watch = PoseWatch(bind=args.bind, port=args.port,
                      stale=args.stale, timeout=args.timeout,
                      our_team=args.our_team_id)
    try:
        watch.start()
    except OSError as exc:
        print(f"[错误] 绑定 UDP {args.bind}:{args.port} 失败: {exc}", file=sys.stderr)
        print("       （端口可能已被 gc_ui.py 内嵌监视或另一实例占用）", file=sys.stderr)
        return 1

    print("=" * 64, flush=True)
    print("实机位置监视（pose_reporter UDP 位姿包）", flush=True)
    print(f"  监听:     UDP {args.bind}:{args.port}"
          f"（陈旧 {args.stale}s / 离线 {args.timeout}s）", flush=True)
    print(f"  显示系:   " + (f"我方队 {args.our_team_id} 场地系（对手自动旋转 180°）"
          if args.our_team_id is not None else "原始 Brain 系（未指定 --our-team-id，不转换）"),
          flush=True)
    print("  前提:     各 NX 上运行 pose_reporter.py 且 --target 指向本机", flush=True)
    print("  互斥:     gc_ui.py 默认内嵌同一监听；两者只可运行一个", flush=True)
    print("=" * 64, flush=True)

    if args.wait > 0:
        deadline = time.time() + args.wait
        seen = False
        while time.time() < deadline:
            snap = watch.snapshot()
            if snap["fresh"] > 0:
                _print_table(snap)
                seen = True
                break
            time.sleep(0.1)
        if not seen:
            print(f"[超时] {args.wait}s 内未收到任何新鲜位姿包", file=sys.stderr)
        watch.stop()
        return 0 if seen else 1

    try:
        while True:
            time.sleep(args.interval)
            _print_table(watch.snapshot())
    except KeyboardInterrupt:
        pass
    finally:
        watch.stop()
        print("\n已停止", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
