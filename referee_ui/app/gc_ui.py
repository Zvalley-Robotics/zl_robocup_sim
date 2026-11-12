#!/usr/bin/env python3
"""独立裁判机 UI：网页面板 + HL v12 UDP 发送（仅标准库，无 ROS 依赖）。

协议/状态机来自同目录 referee_core.py（ctypes 复刻 HL v12，纯标准库）：
  - 以 5 Hz 向 game_controller_node 监听的 UDP 端口发送 688 字节数据包；
  - 本进程同时提供 HTTP 面板（web/index.html + /api/*），供 MuJoCo 仿真
    联调时随时手动执法。sim_console 与真实 zl 栈不能在域 10 共存，本工具
    不占任何 ROS 资源，可与完整栈并行使用。

实机在线监视（默认开启，可 --no-robot-watch 关闭）：
  - 内嵌 robot_watch.RobotWatch 线程监听 UDP 3939 的 brain 保活包（RGrt 8B，1 Hz）；
  - 各实机 brain 的 game_control_ip 指向本机时，面板"机器人在线状态"卡
    即显示每台机器人 team/player、心跳时延与累计包数（3s 无心跳判离线）；
  - 3939 单绑定：独立运行 referee_ui/app/robot_watch.py 时须关闭本内嵌监视。

实机位置监视（默认开启，可 --no-pose-watch 关闭）：
  - 内嵌 pose_watch.PoseWatch 线程监听 UDP 3940 的位姿包（各 NX 上
    pose_reporter.py 上报 /soccer/robot_pose，默认 10 Hz）；
  - 面板"场地位置"卡按接收时刻画机器人/球，区分网络断流与定位断流；
  - 3940 单绑定：独立运行 referee_ui/app/pose_watch.py 时须关闭本内嵌监视。

典型用法:
    # 1) 另起 game_controller_node（域 100，与 Brain 同域）
    ros2 launch game_controller launch.py
    # 2) 启动本 UI（默认 http://0.0.0.0:8090 -> UDP 127.0.0.1:3838）
    referee_ui/scripts/start_referee_ui.sh

HTTP 接口:
    GET  /               面板页面
    GET  /api/snapshot   裁判机完整状态（JSON；含 robots 实机在线表）
    GET  /api/poses      实机位置表（JSON；PoseWatch 快照，独立高频轮询）
    POST /api/command    控制指令（JSON）:
        {"cmd":"patch","patch":{...}}                  透传 RefereeSim.update
        {"cmd":"goal","side":"our"|"opp"}              进球结算（比分+交换开球权+重开仪式）
        {"cmd":"freekick","secondary":4..9,"team":"our"|"opp",
         "wait_kicker":bool}                          出界判罚（自动子状态时序；
            wait_kicker=true 时 GET_READY 挂起，等发球方到位信号后进 SET）
        {"cmd":"kicker_ready"}                      发球方已就位（放行挂起的 GET_READY）
        {"cmd":"cancel_freekick"}                     打断任意球时序
        {"cmd":"goal_restart"}                        手动触发重开仪式
        {"cmd":"reset_scores"}                        比分清零
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit

# 本文件只依赖 referee_ui 目录内部：app/ 下的同目录模块 + ../web 静态页。
# 可整目录（referee_ui/）单独拷贝/打包到任意位置运行，无需本仓库其他目录。
APP_DIR = os.path.dirname(os.path.abspath(__file__))
UI_ROOT = os.path.dirname(APP_DIR)
if APP_DIR not in sys.path:
    sys.path.insert(0, APP_DIR)          # 兼容 `python3 -m` / 从别处启动脚本

from referee_core import (  # noqa: E402  同目录模块：HL v12 协议 + 裁判机状态机
    SECONDARY_NAMES,
    STATE2_DIRECT_FREEKICK,
    RefereeSim,
)
from pose_watch import PoseWatch  # noqa: E402  同目录模块：3940 位姿监听
from robot_watch import RobotWatch  # noqa: E402  同目录模块：3939 保活监听


def apply_command(ref: RefereeSim, payload: dict) -> dict:
    """执行面板指令并返回结果描述；非法输入抛 ValueError。"""
    cmd = str(payload.get("cmd", ""))

    if cmd == "patch":
        patch = payload.get("patch")
        if not isinstance(patch, dict):
            raise ValueError("patch 必须是对象")
        ref.update(patch)
        return {"ok": True, "cmd": cmd, "applied": sorted(patch.keys())}

    if cmd == "goal":
        side = payload.get("side")
        if side == "our":
            ref.apply_goal(1, 0)
        elif side == "opp":
            ref.apply_goal(0, 1)
        else:
            raise ValueError("side 必须是 our/opp")
        return {"ok": True, "cmd": cmd, "side": side}

    if cmd == "freekick":
        try:
            secondary = int(payload.get("secondary", -1))
        except (TypeError, ValueError):
            secondary = -1
        if secondary not in SECONDARY_NAMES or secondary < STATE2_DIRECT_FREEKICK:
            raise ValueError("secondary 必须是 4..9 的出界判罚类型")
        team_key = payload.get("team")
        if team_key == "our":
            team = ref.team_id
        elif team_key == "opp":
            team = ref.opponent_team_id
        else:
            raise ValueError("team 必须是 our/opp")
        ref.begin_freekick(secondary, team,
                           wait_kicker=bool(payload.get("wait_kicker", False)))
        return {"ok": True, "cmd": cmd, "secondary": secondary, "team": team,
                "wait_kicker": bool(payload.get("wait_kicker", False))}

    if cmd == "kicker_ready":
        ref.note_kicker_ready()
        return {"ok": True, "cmd": cmd}

    if cmd == "cancel_freekick":
        ref.cancel_freekick()
        return {"ok": True, "cmd": cmd}

    if cmd == "goal_restart":
        ref.begin_goal_restart()
        return {"ok": True, "cmd": cmd}

    if cmd == "reset_scores":
        ref.update({"our_score": 0, "opp_score": 0})
        return {"ok": True, "cmd": cmd}

    raise ValueError(f"未知指令: {cmd!r}")


class RefereeUiHandler(BaseHTTPRequestHandler):
    """静态页面 + /api/snapshot + /api/command；轮询请求不刷日志。"""

    ref: RefereeSim = None
    watch: "RobotWatch | None" = None
    poses: "PoseWatch | None" = None
    web_dir: str = ""
    server_version = "referee-ui/1.0"

    # ---- 工具 -----------------------------------------------------------
    def _send_json(self, obj: dict, status: int = 200) -> None:
        body = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _send_file(self, path: str) -> None:
        try:
            with open(path, "rb") as fh:
                data = fh.read()
        except OSError:
            self.send_error(404, "not found")
            return
        ctype = "text/html; charset=utf-8" if path.endswith(".html") else "application/octet-stream"
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    # ---- 请求处理 --------------------------------------------------------
    def do_GET(self) -> None:  # noqa: N802（http.server 命名约定）
        path = urlsplit(self.path).path
        if path in ("/", "/index.html"):
            self._send_file(os.path.join(self.web_dir, "index.html"))
        elif path == "/api/snapshot":
            try:
                snap = self.ref.snapshot()
                snap["robots"] = self.watch.snapshot() if self.watch else None
                self._send_json(snap)
            except Exception as exc:  # 兜底：不让单个请求打断服务
                self._send_json({"ok": False, "error": f"internal: {exc}"}, 500)
        elif path == "/api/poses":
            try:
                snap = self.poses.snapshot() if self.poses else None
                self._send_json(snap if snap is not None
                                else {"enabled": False, "list": []})
            except Exception as exc:  # 兜底：不让单个请求打断服务
                self._send_json({"ok": False, "error": f"internal: {exc}"}, 500)
        elif path == "/favicon.ico":
            self.send_response(204)
            self.end_headers()
        else:
            self.send_error(404, "not found")

    def do_POST(self) -> None:  # noqa: N802
        if urlsplit(self.path).path != "/api/command":
            self.send_error(404, "not found")
            return
        try:
            length = int(self.headers.get("Content-Length", 0) or 0)
            raw = self.rfile.read(length) if length > 0 else b"{}"
            payload = json.loads(raw.decode("utf-8") or "{}")
            if not isinstance(payload, dict):
                raise ValueError("请求体必须是 JSON 对象")
            result = apply_command(self.ref, payload)
        except (ValueError, TypeError) as exc:
            self._send_json({"ok": False, "error": str(exc)}, 400)
            return
        except Exception as exc:  # JSON 解析失败等
            self._send_json({"ok": False, "error": f"bad request: {exc}"}, 400)
            return
        print(f"[cmd] {json.dumps(payload, ensure_ascii=False)} "
              f"-> {json.dumps(result, ensure_ascii=False)}", flush=True)
        self._send_json(result)

    def log_message(self, fmt: str, *args) -> None:  # 轮询/静态资源不刷屏
        msg = fmt % args
        if ("/api/snapshot" in msg or "/api/poses" in msg
                or "GET / " in msg or "/favicon.ico" in msg):
            return
        sys.stderr.write(f"[http] {msg}\n")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="独立裁判机 UI（网页面板 + HL v12 UDP 发送，无 ROS 依赖）")
    ap.add_argument("--bind", default="0.0.0.0", help="HTTP 监听地址（默认 0.0.0.0，便于 LAN 访问）")
    ap.add_argument("--http-port", type=int, default=8090, help="HTTP 端口（默认 8090）")
    ap.add_argument("--target", default="127.0.0.1", help="裁判机 UDP 目标 IP（默认 127.0.0.1）")
    ap.add_argument("--port", type=int, default=3838, help="裁判机 UDP 目标端口（默认 3838）")
    ap.add_argument("--team-id", type=int, default=29, help="我方队号（需与 brain config 一致）")
    ap.add_argument("--opponent-team-id", type=int, default=30, help="对手队号")
    ap.add_argument("--player-id", type=int, default=1, help="我方球员号（兼容旧 penalize_our 布尔）")
    ap.add_argument("--players-per-team", type=int, default=3,
                    help="每队上场人数（3v3=3；未用槽位自动填 SUBSTITUTE 保证 brain liveCount 正确）")
    ap.add_argument("--quiet", action="store_true", help="不打印周期状态行")
    ap.add_argument("--watch-port", type=int, default=3939,
                    help="实机保活监听端口（默认 3939，brain game_control_ip 指向本机）")
    ap.add_argument("--watch-timeout", type=float, default=3.0,
                    help="实机离线判定秒数（默认 3.0，约 3 个心跳周期）")
    ap.add_argument("--no-robot-watch", action="store_true",
                    help="关闭内嵌实机在线监视（释放 3939，供独立 robot_watch.py 使用）")
    ap.add_argument("--pose-port", type=int, default=3940,
                    help="实机位姿监听端口（默认 3940，各 NX pose_reporter 上报）")
    ap.add_argument("--pose-stale", type=float, default=0.6,
                    help="位姿陈旧判定秒数（默认 0.6，面板变半透明）")
    ap.add_argument("--pose-timeout", type=float, default=3.0,
                    help="位姿离线判定秒数（默认 3.0，面板灰显）")
    ap.add_argument("--no-pose-watch", action="store_true",
                    help="关闭内嵌位姿监视（释放 3940，供独立 pose_watch.py 使用）")
    args = ap.parse_args(argv)

    web_dir = os.path.join(UI_ROOT, "web")
    if not os.path.isfile(os.path.join(web_dir, "index.html")):
        print(f"[错误] 页面文件缺失: {web_dir}/index.html", file=sys.stderr)
        return 1

    ref = RefereeSim(target=args.target, port=args.port, team_id=args.team_id,
                     opponent_team_id=args.opponent_team_id, player_id=args.player_id,
                     players_per_team=args.players_per_team)
    ref.start()

    watch = None
    if not args.no_robot_watch:
        watch = RobotWatch(port=args.watch_port, timeout=args.watch_timeout)
        try:
            watch.start()
        except OSError as exc:
            print(f"[警告] 3939 实机监视未启用（{exc}）；面板将不显示机器人在线状态"
                  f"（如为有意让位独立监视，请加 --no-robot-watch 静默关闭）", flush=True)
            watch = None

    RefereeUiHandler.ref = ref

    poses = None
    if not args.no_pose_watch:
        poses = PoseWatch(port=args.pose_port, stale=args.pose_stale,
                          timeout=args.pose_timeout, our_team=args.team_id)
        try:
            poses.start()
        except OSError as exc:
            print(f"[警告] {args.pose_port} 位置监视未启用（{exc}）；"
                  f"面板将不显示场地位置"
                  f"（如为有意让位独立监视，请加 --no-pose-watch 静默关闭）", flush=True)
            poses = None

    RefereeUiHandler.watch = watch
    RefereeUiHandler.poses = poses
    RefereeUiHandler.web_dir = web_dir
    httpd = ThreadingHTTPServer((args.bind, args.http_port), RefereeUiHandler)
    httpd.daemon_threads = True

    shown = "localhost" if args.bind in ("0.0.0.0", "::") else args.bind
    print("=" * 64, flush=True)
    print("裁判机独立面板（无 ROS 依赖）", flush=True)
    print(f"  面板:     http://{shown}:{args.http_port}", flush=True)
    print(f"  UDP:      5 Hz 688B HL v12 -> {args.target}:{args.port}", flush=True)
    print(f"  球队:     team_id={args.team_id} opp={args.opponent_team_id} "
          f"players_per_team={args.players_per_team}", flush=True)
    print("  实机:     " + (f"UDP {args.watch_port} 保活监听中（离线阈值 {args.watch_timeout}s）"
          if watch else "在线监视关闭（--no-robot-watch 或 3939 被占）"), flush=True)
    print("  位置:     " + (f"UDP {args.pose_port} 位姿监听中（陈旧 {args.pose_stale}s / "
          f"离线 {args.pose_timeout}s）"
          if poses else "位置监视关闭（--no-pose-watch 或 3940 被占）"), flush=True)
    print("  配合:     ros2 launch game_controller launch.py（域 100）", flush=True)
    print("  说明:     发送开关/自动时间线/手动执法均在面板操作；Ctrl+C 退出", flush=True)
    print("=" * 64, flush=True)

    if not args.quiet:
        def status_loop() -> None:
            while True:
                time.sleep(10)
                snap = ref.snapshot()
                robots = ""
                if watch is not None:
                    ws = watch.snapshot()
                    robots = f" 实机 {ws['online']}/{ws['total']}"
                if poses is not None:
                    ps = poses.snapshot()
                    robots += f" 位置 {ps['fresh']}/{ps['total']}"
                print(f"[状态] {snap['state_name']:>8} 剩余 {snap['secs_remaining']:>3}s "
                      f"二级 {snap['secondary_name']:<16} 比分 {snap['our_score']}:{snap['opp_score']} "
                      f"发送 {snap['sent_count']} "
                      f"启用={'on' if snap['enabled'] else 'off'} "
                      f"auto={'on' if snap['auto'] else 'off'}{robots}", flush=True)

        threading.Thread(target=status_loop, name="status", daemon=True).start()

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        httpd.server_close()
        ref.stop()
        if watch is not None:
            watch.stop()
        if poses is not None:
            poses.stop()
        print("\n已停止", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
