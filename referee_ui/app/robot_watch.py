#!/usr/bin/env python3
"""实机在线监视：UDP 3939 保活心跳监听（brain → 裁判机，1 Hz）。

协议（robocup_demo/src/brain/include/RoboCupGameControlData.h
     + brain 侧 unicastToGameController()）：

    struct HlRoboCupGameControlReturnData {   // 固定 8 字节
        char    header[4];    // "RGrt"
        uint8_t version;      // 2
        uint8_t team;         // 队号（29=A / 30=B）
        uint8_t player;       // 号码，从 1 开始
        uint8_t message;      // 0=MAN_PENALISE 1=MAN_UNPENALISE 2=ALIVE
    };

每台 brain 每秒向 `game_control_ip:3939` 发送一次；把各机器人的
`game_control_ip` 指向裁判机本机 IP，裁判机即可凭此表判断每台实机
"在线 / 离线"（离线判定阈值默认 3 s ≈ 3 个心跳周期）。

两种用法:
  1) 独立终端工具（本文件 __main__）:
       python3 robot_watch.py [--bind 0.0.0.0] [--port 3939] [--timeout 3]
     周期打印在线表；脚本化检查用 --wait 2（等首个心跳，0=成功 / 1=超时）。
     启停封装: referee_ui/scripts/start_robot_watch.sh run|fg|stop|status
  2) 被 gc_ui.py 复用（同目录 import）: RobotWatch 线程 + snapshot()，
     在线表经 /api/snapshot 的 robots 字段下发到网页面板。

互斥提醒: UDP 3939 单绑定——gc_ui.py 内嵌监视（默认开启）与
独立监视进程二者只能存活一个；面板场景用内嵌，无面板场景用本工具。
"""
from __future__ import annotations

import argparse
import socket
import sys
import threading
import time

HEADER = b"RGrt"
STRUCT_SIZE = 8                      # char[4] + 4 × uint8，无对齐填充
MSG_NAMES = {0: "MAN_PENALISE", 1: "MAN_UNPENALISE", 2: "ALIVE"}
DEFAULT_PORT = 3939
DEFAULT_TIMEOUT = 3.0                # 秒；> 3 个心跳周期判离线


class RobotWatch(threading.Thread):
    """UDP 3939 保活包监听线程；按 (team, player) 维护在线表。

    线程自身不打印任何内容（供 gc_ui.py 静默复用）；
    终端展示由 __main__ 的 CLI 包装层负责。
    """

    def __init__(self, bind: str = "0.0.0.0", port: int = DEFAULT_PORT,
                 timeout: float = DEFAULT_TIMEOUT) -> None:
        super().__init__(name="robot-watch", daemon=True)
        self.bind = bind
        self.port = port
        self.timeout = float(timeout)
        self._lock = threading.Lock()
        self._robots: dict = {}      # (team, player) -> {last_seen, count, ...}
        self._recv_total = 0
        self._bad_total = 0
        self._stop_flag = threading.Event()
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # 不设 SO_REUSEADDR：端口被占（如另一监视实例）时立即报 EADDRINUSE，
        # 让"双实例同时监听"这类配置错误显式暴露。

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
                data, addr = self._sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break                # stop() 关闭套接字
            if len(data) < STRUCT_SIZE or data[:4] != HEADER:
                with self._lock:
                    self._bad_total += 1
                continue
            version, team, player, message = data[4], data[5], data[6], data[7]
            now = time.time()
            with self._lock:
                self._recv_total += 1
                entry = self._robots.get((team, player))
                if entry is None:
                    entry = {"team": team, "player": player, "first_seen": now}
                    self._robots[(team, player)] = entry
                entry["last_seen"] = now
                entry["count"] = entry.get("count", 0) + 1
                entry["message"] = message
                entry["version"] = version
                entry["addr"] = addr[0]

    # ---- 快照 ------------------------------------------------------------
    def snapshot(self, now: float | None = None) -> dict:
        """返回在线表（JSON 可序列化）；online = 距最后心跳 ≤ timeout。"""
        now = time.time() if now is None else now
        with self._lock:
            bad = self._bad_total
            total_recv = self._recv_total
            rows = []
            for entry in self._robots.values():
                age = now - entry["last_seen"]
                rows.append({
                    "team": entry["team"],
                    "player": entry["player"],
                    "online": age <= self.timeout,
                    "age_sec": round(age, 2),
                    "count": entry["count"],
                    "message": entry.get("message", -1),
                    "message_name": MSG_NAMES.get(entry.get("message", -1),
                                                  f"?({entry.get('message', -1)})"),
                    "version": entry.get("version", -1),
                    "addr": entry.get("addr", ""),
                })
        rows.sort(key=lambda r: (r["team"], r["player"]))
        return {
            "timeout_sec": self.timeout,
            "listening": f"{self.bind}:{self.port}",
            "total": len(rows),
            "online": sum(1 for r in rows if r["online"]),
            "received_total": total_recv,
            "bad_total": bad,
            "list": rows,
        }


# ---- 终端展示 ------------------------------------------------------------
def _print_table(snap: dict) -> None:
    stamp = time.strftime("%H:%M:%S")
    head = (f"[{stamp}] 在线 {snap['online']}/{snap['total']} "
            f"(累计 {snap['received_total']} 包，坏包 {snap['bad_total']})")
    if not snap["list"]:
        print(head + "  —— 暂无 RGrt 心跳（检查各 brain 的 game_control_ip）",
              flush=True)
        return
    print(head, flush=True)
    for r in snap["list"]:
        mark = "●" if r["online"] else "○"
        print(f"  {mark} team={r['team']:>3} player={r['player']:<2} "
              f"{r['message_name']:<14} {r['age_sec']:>6.1f}s前 "
              f"{r['count']:>5}包  {r['addr']}", flush=True)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="实机在线监视（brain 3939 保活包，1 Hz，无 ROS 依赖）")
    ap.add_argument("--bind", default="0.0.0.0", help="监听地址（默认 0.0.0.0）")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT, help="监听端口（默认 3939）")
    ap.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT,
                    help="离线判定秒数（默认 3.0 ≈ 3 个心跳周期）")
    ap.add_argument("--interval", type=float, default=1.0, help="刷新间隔秒数（默认 1.0）")
    ap.add_argument("--quiet", action="store_true", help="仅打印上下线事件，不周期刷表")
    ap.add_argument("--wait", type=float, default=0.0, metavar="SECS",
                    help="等待首个心跳至多 SECS 秒，打印一次后退出（0=成功 1=超时）")
    args = ap.parse_args(argv)

    watch = RobotWatch(bind=args.bind, port=args.port, timeout=args.timeout)
    try:
        watch.start()
    except OSError as exc:
        print(f"[错误] 绑定 UDP {args.bind}:{args.port} 失败: {exc}", file=sys.stderr)
        print("       （3939 可能已被 gc_ui.py 内嵌监视或另一实例占用）", file=sys.stderr)
        return 1

    print("=" * 64, flush=True)
    print("实机在线监视（brain 3939 保活包，1 Hz）", flush=True)
    print(f"  监听:     UDP {args.bind}:{args.port}（离线阈值 {args.timeout}s）", flush=True)
    print("  前提:     每台 brain 的 game_control_ip 指向本机（裁判机）IP", flush=True)
    print("  互斥:     gc_ui.py 默认内嵌同一监听；两者只可运行一个", flush=True)
    print("=" * 64, flush=True)

    # --wait：脚本化检查模式（不进入周期刷新）。
    if args.wait > 0:
        deadline = time.time() + args.wait
        seen = False
        while time.time() < deadline:
            snap = watch.snapshot()
            if snap["online"] > 0:
                _print_table(snap)
                seen = True
                break
            time.sleep(0.2)
        if not seen:
            print(f"[超时] {args.wait}s 内未收到任何 RGrt 保活包", file=sys.stderr)
        watch.stop()
        return 0 if seen else 1

    # 周期刷新模式：始终打印上下线事件；--quiet 时跳过整表打印。
    prev_online: set = set()
    try:
        while True:
            time.sleep(args.interval)
            snap = watch.snapshot()
            cur_online = {(r["team"], r["player"]) for r in snap["list"] if r["online"]}
            for team, player in sorted(cur_online - prev_online):
                print(f"[上线] team={team} player={player}", flush=True)
            for team, player in sorted(prev_online - cur_online):
                print(f"[离线] team={team} player={player}", flush=True)
            prev_online = cur_online
            if not args.quiet:
                _print_table(snap)
    except KeyboardInterrupt:
        pass
    finally:
        watch.stop()
        print("\n已停止", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
