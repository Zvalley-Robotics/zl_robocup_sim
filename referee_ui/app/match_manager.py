#!/usr/bin/env python3
"""场外裁判组件（边裁 + 摆球 + 比赛状态面板），配合 joint_test 无围栏仿真。

职责（域 10，与 sim / zl 栈同域直连）：
  1) 边裁：盯 /sim/ball_pose（球真值，MuJoCo 世界系），球越线即按足球规则处置：
       边线 |y|>4.5            -> 界外球（GC 二级 9），触球方出界则球权归对方；
       底线 |x|>7.0 非进门区   -> 防守方碰出=角球（7），进攻方踢出=球门球（8）；
       底线 |x|>7.0 且 |y|<1.30 且 z<1.3 -> 进球（与 sim CheckBallGoalReset 同一判定）；
     无触球记录时按"防守方碰出"处理（球权归对方/进攻方）。
  2) 摆球：把球摆回场内（/sim/set_ball_pose 幂等重发 + 清零球速 /sim/set_ball_twist），
      摆点内缩 0.60m：边线 (clamp(x),±3.90)、角球 (±6.40,±3.90)、球门球 (±5.80,clamp(y,±2))、
      进球回场心 (0,0)；球飞出仿真范围（|x|>10/|y|>7/z<-0.5）兜底收回场心。
      （2026-10-09 由 0.15m 加大到 0.40m；再按联调反馈 0.40→0.60m：机器人在球附近
      走动蹭碰仍会把贴线球带出界——日志实证同一点位 14s 内连续再判；0.60m 留出更大容错，
      仍 > brain 判罚到位容差 0.2m）
  3) 联动 referee_ui（HTTP，默认 127.0.0.1:8090）：判定后异步 POST
      {"cmd":"freekick","secondary":7|8|9,"team":"our"|"opp","wait_kicker":true} /
      {"cmd":"goal","side":...}，上报前检查快照 enabled && state==PLAYING，跳过/失败
      都记入事件流（面板可见）。判罚时序以 wait_kicker=true 启动：referee_ui 的
      GET_READY 不按固定计时推进，而是挂起等待“发球方就位”信号——摆球完成后本组件
      监视发球方任一机器人距球 <1.2m，即 POST {"cmd":"kicker_ready"} 放行进入 SET
      （referee_ui 另有 30s 兜底超时，防机器人故障永久挂起）——修正
      “没到位就 SET→PLAYING，恢复比赛后仓促触球把球再弄出界”的循环。
      our/opp 以 referee_ui 启动参数为准（start_all.sh 用 --team-id 29 --opponent-team-id 39，
      即 our=A=robot_01~03，攻 x=-7.02 门；如不一致用 --our-side 对齐）。
  4) 面板（默认 0.0.0.0:8092，轮询式单页，无外部依赖）：
      GET  /            俯视球场 SVG + 逐机状态/策略链卡片 + 事件流 + 控制按钮
      GET  /api/snapshot 全部数据（JSON）
      POST /api/command  {"cmd":"set_auto","on":bool} / {"cmd":"place_ball","x":..,"y":..} /
                         {"cmd":"place_center"} / {"cmd":"judge","secondary":7|8|9,"team":"our"|"opp"} /
                         {"cmd":"goal","side":"our"|"opp"}

触球启发式：导航位姿没有触球传感器，用 0.8s 回溯窗口内"机身与球心距离 <0.70m"近似，
  取窗口内最晚满足者；混战场景可能误判，面板可手动判罚覆盖。

用法：
  source scripts/joint_test/env_joint_d10.sh        # 或经 match_official.sh 包装
  python3 scripts/joint_test/match_official.py --robots robot_01 robot_02 robot_03 \
          --our-side a --http-port 8092           # 单队 3 台
  python3 scripts/joint_test/match_official.py    # 缺省读 active_players.env（六台双队）

依赖：域 10（sim / bridge 在跑）；referee_ui 可选（不可达时仅摆球，联动降级并记事件）。
退出码：0 正常退出（--duration 到期 / SIGTERM）；2 环境缺 rclpy 或消息包。
"""
import argparse
import json
import math
import os
import queue
import re
import signal
import sys
import threading
import time
import urllib.request
from collections import deque
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                           ReliabilityPolicy)
    from geometry_msgs.msg import Point, Vector3
    from std_msgs.msg import String
    from yesense_interface.msg import NavAll
    from robot_interfaces.msg import RobotState
    from game_controller_interface.msg import GameControlData
except ImportError as _e:  # 未 source 环境时给可读提示
    print(f"[match_official] ROS 环境不完整（{_e}）；请先 source scripts/joint_test/env_joint_d10.sh",
          file=sys.stderr)
    sys.exit(2)

ROOT = Path(__file__).resolve().parents[2]

# ---- 场地几何（MuJoCo 世界系；与 soccer_field.xml / sim CheckBallGoalReset 一致）----
FIELD_HALF_LEN = 7.0        # 门线 |x|
FIELD_HALF_WID = 4.5        # 边线 |y|
GOAL_MOUTH_HALF_Y = 1.30    # 门柱 |y|
GOAL_MOUTH_Z = 1.3          # 横梁高判定
LINE_Z = 1.5                # 球高于该高度越线不算出界（等落地再判）
BALL_LINE_MARGIN = 0.60     # 摆点内缩 m（2026-10-09 由 0.15 加大到 0.40，再因机器人在球
                            # 附近走动把贴线球蹭出界上调到 0.60；仍 > 到位容差 0.2m）
OUT_CONFIRM = 0.30          # 出界确认时长 s（连续在界外超过该时长才判，防围球推挤抖动）
BALL_Z = 0.11               # 摆球高度（sim 会夹到 >=0.11）
GOAL_KICK_X = 5.80          # 球门球摆点 |x|（门线内 1.2m）
GOAL_KICK_HALF_Y = 2.0      # 球门球摆点 |y| 上限
HARD_X, HARD_Y = 10.0, 7.0  # 球飞出仿真范围兜底
BALL_STALE = 1.5            # 球位静默视为数据断流
NAV_MAX_AGE = 0.5           # 触球判定只采信该时延内的机身位姿

# ---- 判罚参数 ----
TOUCH_DIST = 0.70           # 触球判定距离阈值 m（机身中心到球心；脚部触球约 0.4~0.6m）
TOUCH_WINDOW = 0.8          # 触球回溯窗口 s
OUT_LOCK = 2.0              # 出界判定后免疫窗 s（防抖 + 摆球重发窗口）
GOAL_LOCK = 3.0             # 进球判定后免疫窗 s（等 sim 回中完成）
RESCUE_COOLDOWN = 3.0       # 兜底收球冷却 s
SPAWN_RESEND = 4            # 摆球幂等重发次数
SPAWN_INTERVAL = 0.15       # 摆球重发间隔 s
TICK = 0.05                 # 判定节拍 s

# ---- 发球方就位放行（防"没到位就 SET→PLAYING 后仓促触球再出界"）----
HOLD_DIST = 1.20            # 就位判定：发球方任一机器人距球 < 该值即发 kicker_ready 放行 m

STATE2_THROW_IN = 9
STATE2_GOAL_KICK = 8
STATE2_CORNER_KICK = 7
GC_PLAYING = 3

STATE_NAME = {0: "INITIAL", 1: "READY", 2: "SET", 3: "PLAYING", 4: "FINISHED"}
SECONDARY_NAME = {0: "NONE", 1: "PENALTYSHOOT", 2: "OVERTIME", 3: "TIMEOUT",
                  4: "直接任意球", 5: "间接任意球", 6: "点球",
                  7: "角球", 8: "球门球", 9: "界外球"}
JUDGE_STATE_NAME = {"inside": "监视中", "out_locked": "出界已判·锁定",
                    "goal_locked": "进球已判·锁定"}


def resolve_robots():
    """缺省名单（高→低）：ROBOT_NAMES 环境变量 > active_players.env > robots.env > robot_01~03。"""
    env = os.environ.get("ROBOT_NAMES", "").split()
    if env:
        return env
    # robocup_sim：默认跟随当前 run 的 roster 名单。
    try:
        run_id = json.loads((ROOT / "runtime/active.json").read_text())["run_id"]
        state = json.loads((ROOT / "runtime" / run_id / "state.json").read_text())
        if state.get("robots"):
            return list(state["robots"])
    except (OSError, KeyError, ValueError):
        pass
    for p in (ROOT / "logs/tmp/joint_test/active_players.env", ROOT / "robots.env"):
        try:
            lines = p.read_text().splitlines()
        except OSError:
            continue
        for line in lines:
            m = re.match(r"\s*export\s+ROBOT_NAMES=(.+)$", line)
            if not m:
                continue
            val = m.group(1).strip().strip('"').strip("'")
            m2 = re.match(r"\$\{ROBOT_NAMES:-([^}]*)\}", val)
            if m2:
                val = m2.group(1).strip()
            if val:
                return val.split()
    return ["robot_01", "robot_02", "robot_03"]


class RefereeLink:
    """referee_ui 异步联动：后台线程 1s 刷新快照 + 串行 POST 上报，绝不阻塞 ROS 执行器。"""

    def __init__(self, base_url, on_result=None):
        self.base_url = base_url.rstrip("/")
        self.on_result = on_result
        self.q = queue.Queue()
        self.snap = None            # 最近一次 /api/snapshot
        self.snap_t = 0.0
        self.reachable = False
        self.ok_count = 0
        self.fail_count = 0
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, name="referee-link", daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()

    def enqueue_freekick(self, secondary, team, ctx):
        # wait_kicker=true：referee_ui 的 GET_READY 挂起等待就位信号，不再固定计时推进
        self.q.put(("freekick", {"cmd": "freekick", "secondary": int(secondary),
                                 "team": team, "wait_kicker": True}, ctx))

    def enqueue_kicker_ready(self, ctx):
        self.q.put(("kicker_ready", {"cmd": "kicker_ready"}, ctx))

    def enqueue_goal(self, side, ctx):
        self.q.put(("goal", {"cmd": "goal", "side": side}, ctx))

    def _record(self, msg, ok):
        if ok:
            self.ok_count += 1
        else:
            self.fail_count += 1
        if self.on_result:
            self.on_result(msg)

    def _loop(self):
        last_snap = 0.0
        while not self._stop.is_set():
            now = time.time()
            if now - last_snap >= 1.0:
                self._fetch_snapshot()
                last_snap = now
            try:
                kind, payload, ctx = self.q.get(timeout=0.2)
            except queue.Empty:
                continue
            self._dispatch(kind, payload, ctx)

    def _fetch_snapshot(self):
        try:
            with urllib.request.urlopen(self.base_url + "/api/snapshot", timeout=2.0) as r:
                self.snap = json.loads(r.read().decode())
            self.snap_t = time.time()
            self.reachable = True
        except Exception:
            self.reachable = False

    def _dispatch(self, kind, payload, ctx):
        if not self.reachable or self.snap is None:
            self._record(f"未上报（裁判机不可达 {self.base_url}）：{ctx}", False)
            return
        if not self.snap.get("enabled", False):
            self._record(f"未上报（referee_ui enabled=false）：{ctx}", False)
            return
        state = self.snap.get("state")
        if state != GC_PLAYING:
            self._record(f"未上报（GC state={state}≠PLAYING，{STATE_NAME.get(state, state)}）：{ctx}", False)
            return
        try:
            req = urllib.request.Request(
                self.base_url + "/api/command",
                data=json.dumps(payload).encode(),
                headers={"Content-Type": "application/json"}, method="POST")
            with urllib.request.urlopen(req, timeout=3.0) as r:
                body = r.read().decode()[:160]
            self._record(f"已上报裁判机：{ctx} → {body}", True)
        except Exception as e:
            self._record(f"上报失败：{ctx}（{e}）", False)


class MatchOfficial:
    """边裁判定状态机 + 摆球执行 + 面板数据源（全部在 ROS spin 线程内执行）。"""

    def __init__(self, args):
        self.args = args
        self.robots = list(args.robots)
        self.our_side = args.our_side.lower()
        # 队伍划分：名单前半 = A 队（start_all.sh 生成顺序：A 队在前），与 GT 逐机
        # gt_world_reverse 派生一致（A=true 守 +x 攻 -x）。
        half = (len(self.robots) + 1) // 2
        self.team_of = {n: ("A" if i < half else "B") for i, n in enumerate(self.robots)}
        self.our_team = "A" if self.our_side == "a" else "B"
        self.opp_team = "B" if self.our_team == "A" else "A"
        self.auto = not args.no_auto

        # ---- ROS 数据缓存（回调写，tick/面板读；同线程写 + GIL 读，原子够用）----
        self.ball = None            # (x, y, z, t)
        self.nav = {n: None for n in self.robots}       # (x, y, yaw, alt, t)
        self.rstate = {n: None for n in self.robots}    # (state, name, mode, t)
        self.strategy = {n: None for n in self.robots}  # dict
        self.gc = None              # dict
        self._ball_cnt = 0
        self._ball_hz = 0.0
        self._hz_t = time.time()
        self._ball_stale = False

        # ---- 判定状态机 ----
        self.judge_state = "inside"
        self.lock_until = 0.0
        self.out_since = None           # 出界确认计时起点（防抖）
        self.fk_hold = None             # 就位放行监视 {secondary, side, ctx, at, ready}（None=无）
        self.frames = deque(maxlen=60)  # (t, (bx,by)|None, {name:(x,y)}) 供触球回溯

        # ---- 摆球幂等重发 ----
        self.spawn_target = None    # (x, y, next_t, left)

        # ---- 事件流 / 计数 ----
        self.events = deque(maxlen=300)
        self.counters = {"out": 0, "corner": 0, "goal_kick": 0, "goal": 0,
                         "rescue": 0, "manual": 0, "referee_ok": 0, "referee_fail": 0}
        self.last_judge = None      # 最近一次判罚摘要（面板标题条）

        # ---- 面板手动命令队列（HTTP 线程 → tick 线程）----
        self.cmd_q = queue.Queue()
        self.referee = RefereeLink(args.referee_url, on_result=self._on_referee_result)
        self.node = None
        self.pub_set_ball = None
        self.pub_set_twist = None
        self.httpd = None

    # ================================================================ 日志 ----
    def log_event(self, kind, msg):
        ev = {"t": time.time(), "ts": datetime.now().strftime("%H:%M:%S"),
              "kind": kind, "msg": msg}
        self.events.append(ev)
        print(f"[{ev['ts']}][{kind}] {msg}", flush=True)

    def _on_referee_result(self, msg):
        """RefereeLink 工作线程回调：仅追加事件（deque.append 线程安全）。"""
        ok = msg.startswith("已上报")
        self.counters["referee_ok" if ok else "referee_fail"] += 1
        self.log_event("referee", msg)

    # ============================================================ ROS 接口 ----
    def bind_ros(self, node):
        self.node = node
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)

        def on_ball(msg):
            self.ball = (msg.x, msg.y, msg.z, time.time())
            self._ball_cnt += 1

        node.create_subscription(Point, "/sim/ball_pose", on_ball, qos)

        for name in self.robots:
            node.create_subscription(NavAll, f"/{name}/imu/nav_all",
                                     self._make_nav_cb(name), qos)
            node.create_subscription(RobotState, f"/{name}/state",
                                     self._make_state_cb(name), qos)
            node.create_subscription(String, f"/soccer/brain_strategy/{name}",
                                     self._make_strategy_cb(name), qos)

        def on_gc(msg):
            info = msg.secondary_state_info
            self.gc = {
                "state": int(msg.state),
                "secondary": int(msg.secondary_state),
                "sub_type": int(info[0]) if len(info) > 0 else 0,
                "sub_stage": int(info[1]) if len(info) > 1 else 0,
                "secs": int(msg.secs_remaining),
                "kick_off_team": int(msg.kick_off_team),
                "team_ids": [int(msg.teams[0].team_number), int(msg.teams[1].team_number)],
                "scores": [int(msg.teams[0].score), int(msg.teams[1].score)],
                "t": time.time(),
            }

        node.create_subscription(GameControlData, "/soccer/game_controller", on_gc, qos)

        # 摆球发布必须可靠（sim 的 /sim/set_ball_pose 订阅用默认 reliable，be 发布不匹配）
        self.pub_set_ball = node.create_publisher(Point, "/sim/set_ball_pose", 10)
        self.pub_set_twist = node.create_publisher(Vector3, "/sim/set_ball_twist", 10)
        self.timer = node.create_timer(TICK, self.tick)

    def _make_nav_cb(self, name):
        def cb(msg):
            self.nav[name] = (msg.pos.longitude, msg.pos.latitude,
                              msg.euler.yaw, msg.pos.altitude, time.time())
        return cb

    def _make_state_cb(self, name):
        def cb(msg):
            self.rstate[name] = (int(msg.state), str(msg.state_name),
                                 str(msg.active_mode), time.time())
        return cb

    def _make_strategy_cb(self, name):
        def cb(msg):
            try:
                self.strategy[name] = json.loads(msg.data)
            except (ValueError, TypeError):
                self.strategy[name] = {"raw": msg.data[:200]}
        return cb

    # ============================================================== 判定核心 ----
    def tick(self):
        now = time.time()
        self._drain_cmds()
        self._service_spawn(now)
        self._sample_frame(now)
        self._maybe_signal_kicker_ready(now)

        # 球位频率统计（2s 窗）
        if now - self._hz_t >= 2.0:
            self._ball_hz = self._ball_cnt / (now - self._hz_t)
            self._ball_cnt = 0
            self._hz_t = now

        # 数据静默边沿告警（启动初期尚未收到过球位时不报静默）
        stale = self.ball is None or now - self.ball[3] > BALL_STALE
        if stale != self._ball_stale:
            if stale and self.ball is not None:
                self._ball_stale = True
                self.log_event("info", "球位数据静默（/sim/ball_pose 断流）：暂停自动判定")
            elif not stale:
                if self._ball_stale:
                    self.log_event("info", "球位数据恢复：继续自动判定")
                self._ball_stale = False

        # 锁释放
        if self.judge_state != "inside" and now >= self.lock_until:
            self.judge_state = "inside"
            self.out_since = None

        if self.auto and self.judge_state == "inside" and not stale:
            self._check_lines(now)

    def _sample_frame(self, now):
        """每 tick 采样一帧（球 + 逐机位姿）供触球回溯；位姿过期机位记 None。"""
        ball_xy = (self.ball[0], self.ball[1]) if self.ball else None
        robs = {}
        for n in self.robots:
            p = self.nav.get(n)
            robs[n] = (p[0], p[1]) if p and now - p[4] <= NAV_MAX_AGE else None
        self.frames.append((now, ball_xy, robs))

    def _last_toucher(self, now):
        """回溯窗口内最近的触球机器人名；无记录返回 None。"""
        best = None  # (t, name)
        for ft, fball, frobs in self.frames:
            if ft < now - TOUCH_WINDOW or fball is None:
                continue
            for nm, fp in frobs.items():
                if fp is None:
                    continue
                if math.hypot(fball[0] - fp[0], fball[1] - fp[1]) < TOUCH_DIST:
                    if best is None or ft >= best[0]:
                        best = (ft, nm)
        return best[1] if best else None

    def _maybe_signal_kicker_ready(self, now):
        """发球方就位放行：判罚后监视发球方位置，任一机器人距球 <HOLD_DIST 即通知
        referee_ui 放行（GET_READY 挂起 → SET）。

        referee_ui 判罚时序以 wait_kicker=true 启动后，GET_READY 不再固定计时推进，
        而是挂起等待本信号（referee_ui 另有 30s 兜底超时，防机器人故障永久挂起）。
        这里在摆球完成（spawn_target 为空，防拿旧出界点误判）且球位新鲜时，检查
        发球方到球的最小距离：就位即发 kicker_ready，一次性（ready 置位去重）。
        """
        h = self.fk_hold
        if h is None or h.get("ready"):
            return
        if self.spawn_target:               # 摆球未完成前不判（球位可能仍是旧出界点）
            return
        if not self.ball or now - self.ball[3] > BALL_STALE:
            return
        # 发球方各机到球的最小距离（nav 位姿缺失/过期视为无数据）
        team = self.our_team if h["side"] == "our" else self.opp_team
        bx, by = self.ball[0], self.ball[1]
        d_min = None
        for n in self.robots:
            if self.team_of.get(n) != team:
                continue
            p = self.nav.get(n)
            if not p or now - p[4] > NAV_MAX_AGE:
                continue
            d = math.hypot(bx - p[0], by - p[1])
            d_min = d if d_min is None else min(d_min, d)
        if d_min is None or d_min > HOLD_DIST:
            return
        h["ready"] = True
        self.referee.enqueue_kicker_ready(h["ctx"])
        self.log_event("out", f"发球方已就位（最近距球 {d_min:.2f}m）→ 通知裁判机放行进入 SET")

    def _check_lines(self, now):
        x, y, z = self.ball[0], self.ball[1], self.ball[2]

        # 硬兜底：球飞出仿真范围/掉出场外 -> 收回场心（不判罚）
        if abs(x) > HARD_X or abs(y) > HARD_Y or z < -0.5:
            self._rescue(now, x, y, z)
            return

        # 进球（与 sim CheckBallGoalReset 同一判定式）
        if abs(x) > FIELD_HALF_LEN and abs(y) < GOAL_MOUTH_HALF_Y and z < GOAL_MOUTH_Z:
            self._on_goal(now, x, y)
            return

        # 出界（球高于 LINE_Z 视为空中掠过，等落地再判）；连续 OUT_CONFIRM 秒在界外
        # 才判定，防"球员围球推挤/贴线滚动"的瞬时抖动造成判定轰炸。
        outside = z < LINE_Z and (abs(y) > FIELD_HALF_WID or abs(x) > FIELD_HALF_LEN)
        if not outside:
            self.out_since = None
            return
        if self.out_since is None:
            self.out_since = now
            return
        if now - self.out_since < OUT_CONFIRM:
            return
        self.out_since = None
        if abs(y) > FIELD_HALF_WID:
            self._on_out(now, x, y, "边线")
        else:
            self._on_out(now, x, y, "底线")

    def _on_out(self, now, x, y, kind):
        toucher = self._last_toucher(now)
        toucher_team = self.team_of.get(toucher) if toucher else None

        if kind == "边线":
            # 界外球：触球方出界 -> 对方发球；无触球 -> 归我方（演示默认，面板可手动改）
            kick_team = self.opp_team if toucher_team == self.our_team else self.our_team
            secondary = STATE2_THROW_IN
            label = "界外球"
            px = max(-(FIELD_HALF_LEN - BALL_LINE_MARGIN), min(FIELD_HALF_LEN - BALL_LINE_MARGIN, x))
            py = math.copysign(FIELD_HALF_WID - BALL_LINE_MARGIN, y or 1.0)
        else:
            # 底线出界：区分触球方是"攻该门"还是"守该门"。
            # our_side=a（我方攻 x=-7 门）：x<0 门由我方进攻、对方把守。
            attacker = self.our_team if ((x < 0) == (self.our_side == "a")) else self.opp_team
            defender = self.opp_team if attacker == self.our_team else self.our_team
            if toucher_team == attacker:
                # 进攻方踢出底线 -> 球门球（防守方发）
                kick_team = defender
                secondary = STATE2_GOAL_KICK
                label = "球门球"
                px = math.copysign(GOAL_KICK_X, x)
                py = max(-GOAL_KICK_HALF_Y, min(GOAL_KICK_HALF_Y, y))
            else:
                # 防守方碰出（或无触球）-> 角球（进攻方发）
                kick_team = attacker
                secondary = STATE2_CORNER_KICK
                label = "角球"
                px = math.copysign(FIELD_HALF_LEN - BALL_LINE_MARGIN, x)
                py = math.copysign(FIELD_HALF_WID - BALL_LINE_MARGIN, y or 1.0)

        side = "our" if kick_team == self.our_team else "opp"
        who = f"{toucher}触球" if toucher else "无触球记录"
        self._apply_ball(px, py)
        self.counters["out" if secondary == STATE2_THROW_IN else
                      ("corner" if secondary == STATE2_CORNER_KICK else "goal_kick")] += 1
        self.judge_state = "out_locked"
        self.lock_until = now + OUT_LOCK
        self.last_judge = {"t": now, "kind": label, "msg": f"{label}→{kick_team}队发球"}
        self.log_event("out", f"{label}出界 ({x:+.2f},{y:+.2f})：{who} → "
                              f"摆回 ({px:+.2f},{py:+.2f})，{kick_team} 队发球（secondary={secondary}）")
        self.referee.enqueue_freekick(secondary, side, f"{label}→{kick_team}队")
        self.fk_hold = {"secondary": secondary, "side": side,
                        "ctx": f"{label}→{kick_team}队", "at": now, "ready": False}

    def _on_goal(self, now, x, y):
        # 球进 x<0 门时：our_side=a 则我方得分，否则对方得分
        self.fk_hold = None   # 进球结束出界判罚，停止就位监视
        side = "our" if ((x < 0) == (self.our_side == "a")) else "opp"
        team = self.our_team if side == "our" else self.opp_team
        self._apply_ball(0.0, 0.0)   # 立即回场心（sim ball_goal_reset 1s 后同样回中，幂等）
        self.counters["goal"] += 1
        self.judge_state = "goal_locked"
        self.lock_until = now + GOAL_LOCK
        self.last_judge = {"t": now, "kind": "进球", "msg": f"进球→{team}队得分"}
        self.log_event("goal", f"进球！球越门线 ({x:+.2f},{y:+.2f}) → {team} 队得分"
                               f"（sim 侧 ball_goal_reset 也会回中，幂等）")
        self.referee.enqueue_goal(side, f"进球→{team}队得分")

    def _rescue(self, now, x, y, z):
        self.fk_hold = None   # 兜底收球，停止就位监视
        self._apply_ball(0.0, 0.0)
        self.counters["rescue"] += 1
        self.judge_state = "out_locked"
        self.lock_until = now + RESCUE_COOLDOWN
        self.last_judge = {"t": now, "kind": "兜底", "msg": "球飞出仿真范围→收回场心"}
        self.log_event("rescue", f"球飞出仿真范围 ({x:+.2f},{y:+.2f},{z:+.2f}) → 兜底收回场心")

    # ============================================================== 摆球执行 ----
    def _apply_ball(self, x, y):
        """登记摆球目标：tick 内幂等重发 SPAWN_RESEND 次并清零球速。"""
        self.spawn_target = (x, y, 0.0, SPAWN_RESEND)

    def _service_spawn(self, now):
        if not self.spawn_target:
            return
        sx, sy, nt, left = self.spawn_target
        if left <= 0:
            self.spawn_target = None
            return
        # 到位 = 目标点附近（0.08m）且确实在场内（防"球停在界外 5cm 内"被误判到位）
        if (self.ball and abs(self.ball[0] - sx) < 0.08 and abs(self.ball[1] - sy) < 0.08
                and abs(self.ball[0]) <= FIELD_HALF_LEN - 0.02
                and abs(self.ball[1]) <= FIELD_HALF_WID - 0.02):
            self.spawn_target = None
            return
        if now >= nt:
            msg = Point()
            msg.x, msg.y, msg.z = sx, sy, BALL_Z
            self.pub_set_ball.publish(msg)
            self.pub_set_twist.publish(Vector3())  # 全 0 = 清零球速
            self.spawn_target = (sx, sy, now + SPAWN_INTERVAL, left - 1)

    # ========================================================== 手动命令处理 ----
    def _drain_cmds(self):
        while True:
            try:
                c = self.cmd_q.get_nowait()
            except queue.Empty:
                return
            try:
                self._exec_cmd(c)
            except Exception as e:  # 面板输入非法不让 tick 崩
                self.log_event("info", f"手动命令失败：{c!r}（{e}）")

    def _ball_xy_or(self, dx, dy):
        return (self.ball[0], self.ball[1]) if self.ball else (dx, dy)

    def _exec_cmd(self, c):
        k = c.get("cmd")
        now = time.time()
        if k == "set_auto":
            self.auto = bool(c.get("on", True))
            self.counters["manual"] += 1
            self.log_event("info", f"自动判罚已{'开启' if self.auto else '关闭'}")
        elif k == "place_ball":
            x = max(-6.95, min(6.95, float(c.get("x", 0.0))))
            y = max(-4.45, min(4.45, float(c.get("y", 0.0))))
            self._apply_ball(x, y)
            self.fk_hold = None   # 人工干预摆球，停止就位监视
            self.counters["manual"] += 1
            self.log_event("info", f"手动摆球 → ({x:+.2f},{y:+.2f})")
        elif k == "place_center":
            self._apply_ball(0.0, 0.0)
            self.fk_hold = None   # 人工干预摆球，停止就位监视
            self.counters["manual"] += 1
            self.log_event("info", "手动摆球 → 场心")
        elif k == "judge":
            secondary = int(c.get("secondary", STATE2_THROW_IN))
            if secondary not in (STATE2_CORNER_KICK, STATE2_GOAL_KICK, STATE2_THROW_IN):
                raise ValueError(f"secondary 仅支持 7/8/9，收到 {secondary}")
            team = c.get("team", "our")
            if team not in ("our", "opp"):
                raise ValueError(f"team 仅支持 our/opp，收到 {team}")
            bx, by = self._ball_xy_or(0.0, 0.0)
            if secondary == STATE2_THROW_IN:
                px = max(-(FIELD_HALF_LEN - BALL_LINE_MARGIN),
                         min(FIELD_HALF_LEN - BALL_LINE_MARGIN, bx))
                py = math.copysign(FIELD_HALF_WID - BALL_LINE_MARGIN, by or 1.0)
                label = "界外球"
            elif secondary == STATE2_CORNER_KICK:
                # 角球点取"我方/对方进攻端"角落
                sx = -1.0 if (team == "our") == (self.our_side == "a") else 1.0
                px = sx * (FIELD_HALF_LEN - BALL_LINE_MARGIN)
                py = math.copysign(FIELD_HALF_WID - BALL_LINE_MARGIN, by or 1.0)
                label = "角球"
            else:
                px = math.copysign(GOAL_KICK_X, -1.0 if (team == "our") == (self.our_side == "a") else 1.0)
                py = max(-2.0, min(2.0, by))
                label = "球门球"
            self._apply_ball(px, py)
            self.referee.enqueue_freekick(secondary, team, f"手动{label}→{'我方' if team == 'our' else '对方'}")
            self.fk_hold = {"secondary": secondary, "side": team,
                            "ctx": f"手动{label}→{'我方' if team == 'our' else '对方'}",
                            "at": now, "ready": False}
            self.counters["manual"] += 1
            self.judge_state = "out_locked"
            self.lock_until = now + OUT_LOCK
            self.last_judge = {"t": now, "kind": f"手动{label}", "msg": f"{label}→{team}"}
            self.log_event("manual", f"手动判罚 {label}（{team} 发球）→ 摆球 ({px:+.2f},{py:+.2f})")
        elif k == "goal":
            side = c.get("side", "our")
            if side not in ("our", "opp"):
                raise ValueError(f"side 仅支持 our/opp，收到 {side}")
            self._apply_ball(0.0, 0.0)
            self.referee.enqueue_goal(side, f"手动进球→{'我方' if side == 'our' else '对方'}")
            self.fk_hold = None   # 进球结束出界判罚，停止就位监视
            self.counters["manual"] += 1
            self.judge_state = "goal_locked"
            self.lock_until = now + GOAL_LOCK
            self.last_judge = {"t": now, "kind": "手动进球", "msg": f"{side} 得分"}
            self.log_event("manual", f"手动判定进球（{side} 得分）→ 球回场心")
        elif k == "reset_counters":
            for key in self.counters:
                self.counters[key] = 0
            self.log_event("info", "计数器已清零")
        else:
            raise ValueError(f"未知命令: {k!r}")

    # ============================================================== 面板数据 ----
    def snapshot(self):
        now = time.time()

        def _age(t):
            return round(now - t, 2) if t else None

        robots = []
        for n in self.robots:
            p, st, sp = self.nav.get(n), self.rstate.get(n), self.strategy.get(n)
            team = self.team_of.get(n, "?")
            robots.append({
                "name": n, "team": team, "is_our": team == self.our_team,
                "x": round(p[0], 2) if p else None,
                "y": round(p[1], 2) if p else None,
                "yaw": round(p[2], 1) if p else None,
                "alt": round(p[3], 2) if p else None,
                "age": _age(p[4]) if p else None,
                "state": st[0] if st else None,
                "state_name": st[1] if st else None,
                "mode": st[2] if st else None,
                "strategy": sp,
            })

        ref = None
        if self.referee.reachable and self.referee.snap:
            s = self.referee.snap
            ref = {k: s.get(k) for k in ("state", "state_name", "secondary_name",
                                         "freekick_stage", "waiting_kicker",
                                         "goal_restart_stage", "our_score", "opp_score",
                                         "kick_off_team", "team_id", "opponent_team_id",
                                         "enabled")}
            ref["age"] = _age(self.referee.snap_t)
        gc = None
        if self.gc:
            gc = dict(self.gc)
            gc["state_name"] = STATE_NAME.get(gc["state"], str(gc["state"]))
            gc["secondary_name"] = SECONDARY_NAME.get(gc["secondary"], str(gc["secondary"]))
            gc["age"] = _age(gc["t"])

        ball = None
        if self.ball:
            ball = {"x": round(self.ball[0], 3), "y": round(self.ball[1], 3),
                    "z": round(self.ball[2], 3), "age": _age(self.ball[3]),
                    "hz": round(self._ball_hz, 1),
                    "inside": abs(self.ball[0]) <= FIELD_HALF_LEN and abs(self.ball[1]) <= FIELD_HALF_WID}

        events = list(self.events)[-100:][::]
        return {
            "now": now,
            "mode": {"auto": self.auto, "our_side": self.our_side.upper(),
                     "our_team": self.our_team, "opp_team": self.opp_team,
                     "robots": self.robots},
            "ball": ball,
            "judge": {"state": self.judge_state,
                      "state_name": JUDGE_STATE_NAME.get(self.judge_state, self.judge_state),
                      "lock_left": round(max(0.0, self.lock_until - now), 1),
                      "last": self.last_judge},
            "counters": dict(self.counters),
            "referee": {"url": self.referee.base_url,
                        "reachable": self.referee.reachable, "detail": ref},
            "gc": gc,
            "robots": robots,
            "events": events,
        }

    # ============================================================== 生命周期 ----
    def start_threads(self):
        self.referee.start()
        PanelHandler.app = self
        self.httpd = ThreadingHTTPServer((self.args.http_host, self.args.http_port), PanelHandler)
        self.httpd.daemon_threads = True
        threading.Thread(target=self.httpd.serve_forever,
                         name="panel-http", daemon=True).start()

    def shutdown(self):
        self.referee.stop()
        if self.httpd:
            self.httpd.shutdown()


# ================================================================ HTTP 面板 ----
class PanelHandler(BaseHTTPRequestHandler):
    """内联单页：GET / + /api/snapshot；POST /api/command（命令只入队，tick 线程执行）。"""

    app = None  # start_threads 注入（class 属性，与 referee_ui 同风格）
    server_version = "match-official/1.0"

    def _send(self, code, body, ctype):
        data = body if isinstance(body, bytes) else body.encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def _send_json(self, obj, code=200):
        self._send(code, json.dumps(obj, ensure_ascii=False),
                   "application/json; charset=utf-8")

    def do_GET(self):  # noqa: N802（http.server 命名约定）
        path = urlsplit(self.path).path
        if path in ("/", "/index.html"):
            self._send(200, INDEX_HTML, "text/html; charset=utf-8")
        elif path == "/api/snapshot":
            try:
                self._send_json(self.app.snapshot())
            except Exception as exc:  # 兜底：不让单个请求打断服务
                self._send_json({"error": str(exc)}, 500)
        else:
            self.send_error(404, "not found")

    def do_POST(self):  # noqa: N802
        if urlsplit(self.path).path != "/api/command":
            self.send_error(404, "not found")
            return
        try:
            length = int(self.headers.get("Content-Length", 0) or 0)
            raw = self.rfile.read(length) if length > 0 else b"{}"
            cmd = json.loads(raw.decode() or "{}")
            if not isinstance(cmd, dict):
                raise ValueError("body 必须是 JSON 对象")
            # HTTP 线程只入队；由 ROS tick 线程执行（发布/状态机不跨线程）
            self.app.cmd_q.put(cmd)
            self._send_json({"ok": True, "queued": cmd})
        except Exception as exc:
            self._send_json({"ok": False, "error": str(exc)}, 400)

    def log_message(self, fmt, *args):  # 轮询/静态资源不刷屏
        msg = fmt % args
        if "/api/snapshot" in msg or "GET / " in msg or "favicon" in msg:
            return
        sys.stderr.write(f"[http] {msg}\n")


INDEX_HTML = """<!doctype html>
<html lang="zh">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>场外裁判 · robocup_sim</title>
<style>
  :root { --bg:#0f1115; --card:#171a21; --line:#2a2f3a; --fg:#e5e7eb; --dim:#9ca3af;
          --our:#4ade80; --opp:#f87171; }
  * { box-sizing: border-box; }
  body { margin:0; background:var(--bg); color:var(--fg);
         font:14px/1.5 "Segoe UI", system-ui, "Noto Sans CJK SC", sans-serif; }
  header { padding:10px 16px; border-bottom:1px solid var(--line);
           display:flex; align-items:center; gap:16px; flex-wrap:wrap; }
  h1 { font-size:16px; margin:0; }
  .badge { padding:1px 8px; border-radius:10px; background:#1f2430;
           border:1px solid var(--line); font-size:12px; }
  .dot-ok { color:var(--our); } .dot-bad { color:var(--opp); }
  main { display:grid; grid-template-columns:minmax(560px, 720px) minmax(320px, 1fr);
         gap:12px; padding:12px; align-items:start; }
  @media (max-width:1100px) { main { grid-template-columns:1fr; } }
  .card { background:var(--card); border:1px solid var(--line);
          border-radius:10px; padding:10px; }
  .card h2 { font-size:13px; margin:0 0 8px; color:var(--dim); font-weight:600; }
  svg#field { width:100%; height:auto; display:block; border-radius:8px; }
  .grass { fill:#12301a; } .line { stroke:#e7e7e7; stroke-width:2; fill:none; }
  .goal { fill:#e7e7e7; opacity:.85; } .dot { fill:#e7e7e7; }
  .glabel { fill:#9ca3af; font-size:12px; }
  .robots { display:grid; grid-template-columns:repeat(auto-fill, minmax(290px,1fr)); gap:8px; }
  .rb { background:#12151c; border:1px solid var(--line); border-left-width:4px;
        border-radius:8px; padding:8px 10px; }
  .rb.our { border-left-color:var(--our); } .rb.opp { border-left-color:var(--opp); }
  .rb .nm { font-weight:700; } .rb .sub { color:var(--dim); font-size:12px; }
  .strat { color:#c7d2fe; font-size:12px; margin-top:4px; word-break:break-all; }
  .events { max-height:420px; overflow:auto; font-size:12.5px; }
  .ev { padding:3px 6px; border-bottom:1px dashed #1f2430; }
  .ev .t { color:var(--dim); margin-right:6px; }
  .ev .k { display:inline-block; min-width:52px; margin-right:6px; padding:0 6px;
           border-radius:8px; text-align:center; background:#1f2430; font-size:11px; }
  .ev.goal .k { background:#7c5a12; color:#fde68a; }
  .ev.out .k { background:#1e3a8a; color:#bfdbfe; }
  .ev.rescue .k { background:#4c1d95; color:#ddd6fe; }
  .ev.manual .k { background:#164e63; color:#a5f3fc; }
  .ev.referee .k { background:#334155; color:#cbd5e1; }
  .ev.info .k { background:#27272a; color:#d4d4d8; }
  .btns { display:flex; flex-wrap:wrap; gap:6px; margin-top:8px; align-items:center; }
  button { background:#1f2430; color:var(--fg); border:1px solid var(--line);
           border-radius:8px; padding:5px 10px; cursor:pointer; font-size:12.5px; }
  button:hover { background:#283042; }
  input.num { width:64px; background:#0c0e12; color:var(--fg);
              border:1px solid var(--line); border-radius:6px; padding:4px 6px; }
</style>
</head>
<body>
<header>
  <h1>场外裁判 · 边裁+摆球+面板</h1>
  <span class="badge" id="b-gc">GC -</span>
  <span class="badge" id="b-score">比分 -</span>
  <span class="badge" id="b-sub">二级 -</span>
  <span class="badge" id="b-judge">判定 -</span>
  <span class="badge" id="b-ref">裁判机 -</span>
  <span class="badge" id="b-ball">球 -</span>
</header>
<main>
  <section>
    <div class="card">
      <h2>球场俯视（MuJoCo 世界系；绿=我方 红=对方；箭头=朝向）</h2>
      <svg id="field" viewBox="0 0 656 446">
        <rect class="grass" x="0" y="0" width="656" height="446"></rect>
        <rect class="line" x="33.6" y="33.6" width="588" height="378"></rect>
        <line class="line" x1="327.6" y1="33.6" x2="327.6" y2="411.6"></line>
        <circle class="line" cx="327.6" cy="222.6" r="58.8"></circle>
        <circle class="dot" cx="327.6" cy="222.6" r="3"></circle>
        <rect class="goal" x="16.8" y="168" width="16.8" height="109.2"></rect>
        <rect class="goal" x="621.6" y="168" width="16.8" height="109.2"></rect>
        <text class="glabel" id="lblL" x="33.6" y="24"></text>
        <text class="glabel" id="lblR" x="621.6" y="24" text-anchor="end"></text>
        <g id="dyn"></g>
      </svg>
      <div class="btns">
        <button id="btn-auto">自动判罚 -</button>
        <button data-cmd='{"cmd":"place_center"}'>球回场心</button>
        <button data-cmd='{"cmd":"judge","secondary":9,"team":"our"}'>界外球·我方发</button>
        <button data-cmd='{"cmd":"judge","secondary":9,"team":"opp"}'>界外球·对方发</button>
        <button data-cmd='{"cmd":"judge","secondary":7,"team":"our"}'>角球·我方发</button>
        <button data-cmd='{"cmd":"judge","secondary":7,"team":"opp"}'>角球·对方发</button>
        <button data-cmd='{"cmd":"goal","side":"our"}'>进球·我方</button>
        <button data-cmd='{"cmd":"goal","side":"opp"}'>进球·对方</button>
      </div>
      <div class="btns">
        <span style="color:var(--dim)">摆球：</span>
        <input class="num" id="bx" placeholder="x" value="0">
        <input class="num" id="by" placeholder="y" value="0">
        <button id="btn-place">摆球到 (x,y)</button>
      </div>
    </div>
    <div class="card" style="margin-top:12px">
      <h2>事件流（最新在上，最多 100 条）</h2>
      <div class="events" id="events"></div>
    </div>
  </section>
  <section>
    <div class="card">
      <h2>机器人（<span id="rb-count">-</span>）</h2>
      <div class="robots" id="robots"></div>
    </div>
    <div class="card" style="margin-top:12px">
      <h2>计数器</h2>
      <div id="counters" style="font-size:12.5px;color:var(--dim)"></div>
    </div>
  </section>
</main>
<script>
const SCALE = 42, X0 = 7.8, Y0 = 5.3;
const px = x => (x + X0) * SCALE;
const py = y => (Y0 - y) * SCALE;
let autoState = true;

const esc = s => String(s).replace(/[&<>"']/g,
  c => ({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;","'":"&#39;"}[c]));

async function refresh() {
  try {
    const r = await fetch("/api/snapshot", {cache: "no-store"});
    if (!r.ok) throw new Error("HTTP " + r.status);
    render(await r.json());
  } catch (e) {
    document.getElementById("b-gc").textContent = "面板离线：" + e;
  }
}

function render(s) {
  // ---- 顶部徽章 ----
  const gc = s.gc;
  document.getElementById("b-gc").textContent = gc
    ? "GC " + gc.state_name + (gc.secs ? " " + gc.secs + "s" : "")
    : "GC 无数据";
  const rd = s.referee.detail;
  document.getElementById("b-score").textContent = rd
    ? "比分(裁判机) 我 " + rd.our_score + " : " + rd.opp_score + " 对"
    : (gc ? "比分(GC) " + gc.scores[0] + ":" + gc.scores[1] : "比分 -");
  document.getElementById("b-sub").textContent = gc && gc.secondary
    ? "二级 " + gc.secondary_name + "(子阶段 " + gc.sub_stage + ")"
      + (rd && rd.waiting_kicker ? "·待就位" : "")
    : "二级 无";
  document.getElementById("b-judge").textContent =
    "判定 " + s.judge.state_name + (s.judge.lock_left > 0 ? " " + s.judge.lock_left + "s" : "");
  document.getElementById("b-ref").innerHTML = rd
    ? '<span class="dot-ok">●</span> 裁判机' + (rd.enabled ? "" : "（停发）")
    : '<span class="dot-bad">●</span> 裁判机不可达';
  document.getElementById("b-ball").textContent = s.ball
    ? "球 (" + s.ball.x + "," + s.ball.y + ") " + s.ball.hz + "Hz"
    : "球 无数据";

  // ---- 门标注（our_side=A 时我方攻 -x 左门）----
  const leftIsOurs = (s.mode.our_side === "A");
  document.getElementById("lblL").textContent = leftIsOurs ? "我方进攻门 →" : "← 对方进攻门";
  document.getElementById("lblR").textContent = leftIsOurs ? "← 对方进攻门" : "我方进攻门 →";

  // ---- 动态图层：机器人 + 球 ----
  let h = "";
  for (const r of s.robots) {
    if (r.x === null || r.y === null) continue;
    const cx = px(r.x), cy = py(r.y);
    const rad = (r.yaw || 0) * Math.PI / 180;
    const ex = cx + 18 * Math.cos(rad), ey = cy - 18 * Math.sin(rad);
    const color = r.is_our ? "#4ade80" : "#f87171";
    h += '<g><circle cx="' + cx + '" cy="' + cy + '" r="11" fill="' + color
      + '" opacity="0.9"></circle>'
      + '<line x1="' + cx + '" y1="' + cy + '" x2="' + ex + '" y2="' + ey
      + '" stroke="#0b0d10" stroke-width="2.5"></line>'
      + '<text x="' + cx + '" y="' + (cy + 3.5) + '" text-anchor="middle" font-size="9"'
      + ' fill="#0b0d10" font-weight="700">' + esc(r.name.slice(-2)) + '</text>'
      + '<title>' + esc(r.name) + ' ' + esc(r.state_name || "") + '</title></g>';
  }
  if (s.ball) {
    h += '<circle cx="' + px(s.ball.x) + '" cy="' + py(s.ball.y)
      + '" r="6" fill="#fff" stroke="#0b0d10" stroke-width="1.5"></circle>';
  }
  document.getElementById("dyn").innerHTML = h;

  // ---- 机器人卡片 ----
  document.getElementById("rb-count").textContent = s.robots.length;
  document.getElementById("robots").innerHTML = s.robots.map(r => {
    const st = r.strategy || {};
    const path = (st.path || []).slice(-3).map(esc).join(" → ");
    const pos = (r.x === null) ? "无位姿数据"
      : "x " + r.x + " y " + r.y + " yaw " + r.yaw + "° alt " + r.alt;
    const meta = [];
    if (st.dec) meta.push("dec=" + esc(st.dec));
    if (st.ctl !== undefined) meta.push("ctl=" + esc(st.ctl));
    if (st.gc) meta.push("gc=" + esc(st.gc));
    if (st.sub) meta.push("sub=" + esc(st.sub));
    return '<div class="rb ' + (r.is_our ? "our" : "opp") + '">'
      + '<div><span class="nm">' + esc(r.name) + '</span> <span class="sub">' + r.team
      + ' 队' + (r.is_our ? "·我方" : "·对方") + '</span></div>'
      + '<div class="sub">' + esc(r.state_name || "无状态")
      + (r.age !== null && r.age !== undefined ? "（" + r.age + "s 前）" : "") + '</div>'
      + '<div class="sub">' + esc(pos) + '</div>'
      + (path ? '<div class="strat">' + path + '</div>' : "")
      + (meta.length ? '<div class="sub">' + meta.join(" · ") + '</div>' : "")
      + '</div>';
  }).join("");

  // ---- 事件流（最新在上）----
  document.getElementById("events").innerHTML = s.events.slice().reverse().map(e =>
    '<div class="ev ' + esc(e.kind) + '"><span class="t">' + esc(e.ts) + '</span>'
    + '<span class="k">' + esc(e.kind) + '</span>' + esc(e.msg) + '</div>').join("");

  // ---- 计数器 ----
  const c = s.counters;
  document.getElementById("counters").textContent =
    "出界球 " + c.out + " · 角球 " + c.corner + " · 球门球 " + c.goal_kick
    + " · 进球 " + c.goal + " · 兜底 " + c.rescue + " · 手动 " + c.manual
    + " · 裁判机上报 " + c.referee_ok + " 成 / " + c.referee_fail + " 败";

  // ---- 自动开关按钮 ----
  autoState = s.mode.auto;
  document.getElementById("btn-auto").textContent =
    "自动判罚：" + (autoState ? "开（点击关闭）" : "关（点击开启）");
}

async function cmd(obj) {
  try {
    await fetch("/api/command", {method: "POST",
      headers: {"Content-Type": "application/json"}, body: JSON.stringify(obj)});
  } finally {
    refresh();
  }
}

document.querySelectorAll("[data-cmd]").forEach(b => {
  b.addEventListener("click", () => cmd(JSON.parse(b.dataset.cmd)));
});
document.getElementById("btn-auto").addEventListener("click",
  () => cmd({cmd: "set_auto", on: !autoState}));
document.getElementById("btn-place").addEventListener("click", () =>
  cmd({cmd: "place_ball",
       x: parseFloat(document.getElementById("bx").value || "0"),
       y: parseFloat(document.getElementById("by").value || "0")}));

setInterval(refresh, 700);
refresh();
</script>
</body>
</html>
"""


def main():
    ap = argparse.ArgumentParser(
        description="场外裁判（边裁+摆球+面板），用法与判罚规则见文件头 docstring")
    ap.add_argument("--robots", nargs="+", default=None,
                    help="机器人名单（缺省读 active_players.env / robots.env）")
    ap.add_argument("--our-side", choices=["a", "b"], default="a",
                    help="我方对应队伍（a = A 队 robot_01~03，攻 x=-7.02 门；"
                         "须与 referee_ui --team-id 29 对齐）")
    ap.add_argument("--http-port", type=int, default=8092, help="面板端口（缺省 8092）")
    ap.add_argument("--http-host", default="0.0.0.0", help="面板绑定地址")
    ap.add_argument("--referee-url", default="http://127.0.0.1:8090",
                    help="referee_ui 地址（不可达时仅摆球，联动降级）")
    ap.add_argument("--no-auto", action="store_true",
                    help="启动时关闭自动判罚（面板仍可手动摆球/判罚）")
    ap.add_argument("--duration", type=float, default=0.0, help="运行时长秒；0 = 常驻")
    args = ap.parse_args()
    if not args.robots:
        args.robots = resolve_robots()

    rclpy.init()
    node = Node("match_official")
    app = MatchOfficial(args)
    app.bind_ros(node)
    app.start_threads()

    half = (len(app.robots) + 1) // 2
    a_list = [n for i, n in enumerate(app.robots) if i < half]
    b_list = [n for i, n in enumerate(app.robots) if i >= half]
    print("=" * 68, flush=True)
    print("场外裁判（边裁 + 摆球 + 比赛面板）", flush=True)
    print(f"  名单：{' '.join(app.robots)}", flush=True)
    print(f"        A 队 = {','.join(a_list) or '-'} | B 队 = {','.join(b_list) or '-'}", flush=True)
    print(f"  我方：{app.our_team} 队（--our-side {app.our_side}；须与 referee_ui --team-id 对齐）", flush=True)
    print(f"  自动判罚：{'关（--no-auto）' if args.no_auto else '开'}"
          f"；触球近似阈值 <{TOUCH_DIST}m / {TOUCH_WINDOW}s", flush=True)
    print(f"  面板：http://{args.http_host}:{args.http_port}/   裁判机：{args.referee_url}", flush=True)
    print("=" * 68, flush=True)

    def _term(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, _term)
    signal.signal(signal.SIGINT, _term)

    t0 = time.time()
    try:
        while rclpy.ok():
            if args.duration > 0 and time.time() - t0 >= args.duration:
                break
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        app.shutdown()
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
    print("[match_official] 已退出", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
