#!/usr/bin/env python3
"""裁判机（Game Controller）模拟器 —— HL v12 协议/状态机唯一实现（归属 referee_ui）。

复刻 robocup_demo/src/game_controller/include/RoboCupGameControlData.h 的
HL（Humanoid League）v12 布局，向 game_controller_node 监听的 UDP 端口
（默认 127.0.0.1:3838）以 5 Hz 发送 688 字节数据包。本文件属 referee_ui，只依赖
标准库，可随 referee_ui 整目录单独打包；sim_console/app/referee.py 是转发兼容层
（老写法 from referee import ... 仍有效），改协议/状态机只改本文件。

3+3 建模：playersPerTeam 由调用方给定（多机 3 / 单机 4）；每名球员 penalty 可
独立设置（WS patch 键 penalize_our_players / penalize_opp_players，列表全量替换；
旧布尔 penalize_our 兼容保留，映射 player_id 槽位）；未用槽位（players_per_team+1..11）
恒填 PENALTY_SUBSTITUTE —— brain 侧 liveCount/gcAliveCount 遍历全部 11 槽位统计
PENALTY_NONE，填充后即等于真实在场上人数。

自检（不需要 ROS 环境）: python3 referee_ui/app/referee_core.py --selfcheck
"""

from __future__ import annotations

import ctypes
import socket
import threading
import time

# ---------------------------------------------------------------------------
# 常量（对齐 RoboCupGameControlData.h / SPLCoachMessage.h）
# ---------------------------------------------------------------------------
SPL_COACH_MESSAGE_SIZE = 253
HL_MAX_NUM_PLAYERS = 11

GAMECONTROLLER_PORT = 3838
GAMECONTROLLER_STRUCT_HEADER = b"RGme"
HL_GAMECONTROLLER_STRUCT_VERSION = 12

TEAM_CYAN = 0
TEAM_MAGENTA = 1
DROPBALL = 255

GAME_ADULT = 1

STATE_INITIAL = 0
STATE_READY = 1
STATE_SET = 2
STATE_PLAYING = 3
STATE_FINISHED = 4

STATE2_NORMAL = 0
STATE2_PENALTYSHOOT = 1
STATE2_OVERTIME = 2
STATE2_TIMEOUT = 3
STATE2_DIRECT_FREEKICK = 4
STATE2_INDIRECT_FREEKICK = 5
STATE2_PENALTYKICK = 6
STATE2_CORNER_KICK = 7
STATE2_GOAL_KICK = 8
STATE2_THROW_IN = 9

PENALTY_NONE = 0
PENALTY_SUBSTITUTE = 14   # 未用槽位填充：brain 只统计 PENALTY_NONE 为活人数
PENALTY_MANUAL = 15

STATE_NAMES = {
    STATE_INITIAL: "INITIAL",
    STATE_READY: "READY",
    STATE_SET: "SET",
    STATE_PLAYING: "PLAYING",
    STATE_FINISHED: "FINISHED",
}

SECONDARY_NAMES = {
    STATE2_NORMAL: "NONE",
    STATE2_TIMEOUT: "TIMEOUT",
    STATE2_DIRECT_FREEKICK: "DIRECT_FREEKICK",
    STATE2_INDIRECT_FREEKICK: "INDIRECT_FREEKICK",
    STATE2_PENALTYKICK: "PENALTY_KICK",
    STATE2_CORNER_KICK: "CORNER_KICK",
    STATE2_GOAL_KICK: "GOAL_KICK",
    STATE2_THROW_IN: "THROW_IN",
}

# 自动时间线的各阶段时长（秒），沿用早期假裁判实现（fake_referee.cpp）的历史时序
TIMELINE = [
    (STATE_INITIAL, 5, "INITIAL 备场"),
    (STATE_READY, 10, "READY 入场站位"),
    (STATE_SET, 5, "SET 静止待哨"),
    (STATE_PLAYING, 60, "PLAYING 比赛"),
    (STATE_FINISHED, 3, "FINISHED 结束"),
]

# 出界判罚的自动子状态时序（对齐 Brain gameSubStateMap）：0=STOP 1=GET_READY 2=SET。
# 出界 → STOP（摆球定位）→ GET_READY（双方走位）→ SET（静止待哨）→ 清空二级
# 状态恢复比赛，随后由 Brain 的 isFreekickKickingOff 窗口接管踢球。
# begin_freekick(wait_kicker=True) 时 GET_READY 不按固定 6s 推进：挂起等待
# note_kicker_ready() 的就位信号（发球方到位）后立即进入 SET；
# FREEKICK_READY_TIMEOUT 为兜底放行上限（None = 无限等待，防机器人故障永久挂起）。
FREEKICK_STAGES = [(0, 1.5), (1, 6.0), (2, 2.5)]
FREEKICK_READY_TIMEOUT = 30.0

# 进球重开仪式（对齐真实 GC：进球 → 失球方开球 → READY → SET → PLAYING；
# 时长与 TIMELINE 的 READY/SET 阶段一致）
GOAL_RESTART_STAGES = [(STATE_READY, 10.0), (STATE_SET, 5.0)]


# ---------------------------------------------------------------------------
# HL v12 结构体（ctypes 复刻；_pack_=1 与 C 布局逐字节一致）
# ---------------------------------------------------------------------------
class HlRobotInfo(ctypes.LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ("penalty", ctypes.c_uint8),             # 罚下状态
        ("secsTillUnpenalised", ctypes.c_uint8),  # 预计解罚剩余秒数
        ("numberOfWarnings", ctypes.c_uint8),
        ("yellowCardCount", ctypes.c_uint8),
        ("redCardCount", ctypes.c_uint8),
        ("goalKeeper", ctypes.c_uint8),
    ]


class HlTeamInfo(ctypes.LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ("teamNumber", ctypes.c_uint8),
        ("fieldPlayerColour", ctypes.c_uint8),
        ("score", ctypes.c_uint8),
        ("penaltyShot", ctypes.c_uint8),
        ("singleShots", ctypes.c_uint16),
        ("coachSequence", ctypes.c_uint8),
        ("coachMessage", ctypes.c_uint8 * SPL_COACH_MESSAGE_SIZE),
        ("coach", HlRobotInfo),
        ("players", HlRobotInfo * HL_MAX_NUM_PLAYERS),
    ]


class HlRoboCupGameControlData(ctypes.LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ("header", ctypes.c_char * 4),
        ("version", ctypes.c_uint16),
        ("packetNumber", ctypes.c_uint8),
        ("playersPerTeam", ctypes.c_uint8),
        ("gameType", ctypes.c_uint8),
        ("state", ctypes.c_uint8),
        ("firstHalf", ctypes.c_uint8),
        ("kickOffTeam", ctypes.c_uint8),
        ("secondaryState", ctypes.c_uint8),
        # 注意：C 侧是 char[4]（有符号）。brain 直接把 byte1 当索引取
        # {STOP, GET_READY, SET}，因此 byte1 必须保持 0..2，否则内存越界。
        ("secondaryStateInfo", ctypes.c_uint8 * 4),
        ("dropInTeam", ctypes.c_uint8),
        ("dropInTime", ctypes.c_uint16),
        ("secsRemaining", ctypes.c_uint16),
        ("secondaryTime", ctypes.c_uint16),
        ("teams", HlTeamInfo * 2),
    ]


def selfcheck() -> bool:
    """结构体尺寸与关键偏移自检，返回是否通过。"""
    expected = {
        "HlRobotInfo": (6, 6),
        "HlTeamInfo": (332, 332),
        "HlRoboCupGameControlData": (688, 688),
    }
    ok = True
    print("== 裁判机结构体自检（对齐 RoboCupGameControlData.h，无 pack 的 C 布局） ==")
    for name, (want, _) in expected.items():
        cls = {"HlRobotInfo": HlRobotInfo, "HlTeamInfo": HlTeamInfo,
               "HlRoboCupGameControlData": HlRoboCupGameControlData}[name]
        got = ctypes.sizeof(cls)
        flag = "OK " if got == want else "FAIL"
        if got != want:
            ok = False
        print(f"  [{flag}] sizeof({name}) = {got}（期望 {want}）")

    offsets = {
        "version": 4,
        "state": 9,
        "secondaryStateInfo": 13,
        "secsRemaining": 20,
        "teams": 24,
    }
    for field, want in offsets.items():
        got = getattr(HlRoboCupGameControlData, field).offset
        flag = "OK " if got == want else "FAIL"
        if got != want:
            ok = False
        print(f"  [{flag}] offsetof(HlRoboCupGameControlData, {field}) = {got}（期望 {want}）")

    # 构造一个真实数据包并检查包头、版本、总长
    data = HlRoboCupGameControlData()
    ctypes.memset(ctypes.byref(data), 0, ctypes.sizeof(data))
    data.header = GAMECONTROLLER_STRUCT_HEADER
    data.version = HL_GAMECONTROLLER_STRUCT_VERSION
    raw = bytes(data)
    checks = [
        ("包头 = RGme", raw[:4] == b"RGme"),
        ("版本 = 12", raw[4] | (raw[5] << 8) == 12),
        ("总长 = 688", len(raw) == 688),
    ]
    for label, cond in checks:
        flag = "OK " if cond else "FAIL"
        if not cond:
            ok = False
        print(f"  [{flag}] {label}")
    print("自检结果:", "通过" if ok else "失败")
    return ok


# ---------------------------------------------------------------------------
# 裁判机运行时
# ---------------------------------------------------------------------------
class RefereeSim:
    """裁判机状态 + 5 Hz UDP 发送线程；面板可在运行时修改。"""

    def __init__(self, target: str = "127.0.0.1", port: int = GAMECONTROLLER_PORT,
                 team_id: int = 29, opponent_team_id: int = 30,
                 player_id: int = 1, players_per_team: int = 4):
        # RLock：_run 持锁期间会再调用 build_packet()/snapshot()（均内部加锁），
        # 普通 Lock 会在第一轮发送时自死锁，导致快照接口一并阻塞。
        self._lock = threading.RLock()
        self.target = target
        self.port = port
        self.team_id = team_id
        self.opponent_team_id = opponent_team_id
        self.player_id = player_id
        self.players_per_team = players_per_team

        # 运行时可调状态（默认值：启用发送、手动 INITIAL）
        self.enabled = True
        self.auto = False
        self.state = STATE_INITIAL
        self.secs_remaining = 600
        self.secondary_state = STATE2_NORMAL
        self.secondary_sub = 0                    # 0..2 -> STOP / GET_READY / SET
        self.kick_off_team = team_id              # team_id / opponent / DROPBALL
        self.our_score = 0
        self.opp_score = 0
        # 逐球员罚下（1..players_per_team 可独立设置；其余槽位恒为 SUBSTITUTE）
        self.penalize_our_players = set()
        self.penalize_opp_players = set()

        # 自动时间线游标
        self._phase_index = 0
        self._phase_start = time.monotonic()

        # 任意球自动时序游标（None = 无进行中的出界判罚）
        self._fk_stage = None
        self._fk_stage_start = 0.0
        self._fk_wait_kicker = False    # True = GET_READY 挂起等发球方到位（begin_freekick 选用）
        self._fk_kicker_ready = False   # 就位信号（note_kicker_ready）已收到

        # 进球重开仪式游标（None = 无重开；auto 开启时改由时间线推进）
        self._gr_stage = None
        self._gr_stage_start = 0.0

        # 比赛时钟游标：secs_remaining 仅在 PLAYING 期间按真实时间递减
        self._clock_last = time.monotonic()
        self._clock_acc = 0.0

        self._packet_number = 0
        self._sent_count = 0
        self._last_send_ts = 0.0
        self._send_error_count = 0
        self._sock = None
        self._thread = None
        self._stop = threading.Event()

    # -- 生命周期 ----------------------------------------------------------
    def start(self) -> None:
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="referee-udp", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    # -- 面板交互 ----------------------------------------------------------
    def update(self, patch: dict) -> None:
        """应用面板提交的补丁；非法值忽略。"""
        with self._lock:
            # 面板手动介入比赛流程即接管任意球/重开仪式时序（清空自动推进游标）
            if any(k in patch for k in ("state", "secondary_state", "kick_off_team")):
                self._fk_stage = None
                self._fk_wait_kicker = False
                self._fk_kicker_ready = False
                self._gr_stage = None
            if "enabled" in patch:
                self.enabled = bool(patch["enabled"])
            if "auto" in patch:
                new_auto = bool(patch["auto"])
                if new_auto and not self.auto:
                    # 对齐到当前状态所在阶段重新计时；重开仪式改由时间线推进
                    self._phase_index = 0
                    for i, (st, _d, _n) in enumerate(TIMELINE):
                        if st == self.state:
                            self._phase_index = i
                            break
                    self._phase_start = time.monotonic()
                    self._gr_stage = None
                self.auto = new_auto
            if "state" in patch:
                try:
                    st = int(patch["state"])
                except (TypeError, ValueError):
                    st = -1
                if st in STATE_NAMES:
                    self.state = st
                    self.auto = False  # 手动指定状态即退出自动时间线
            if "secs_remaining" in patch:
                # 时钟与自动时间线解耦：手动应用数值不再切换状态循环
                try:
                    self.secs_remaining = max(0, min(65535, int(patch["secs_remaining"])))
                except (TypeError, ValueError):
                    pass
            if "secondary_state" in patch:
                try:
                    ss = int(patch["secondary_state"])
                except (TypeError, ValueError):
                    ss = -1
                if ss in SECONDARY_NAMES:
                    self.secondary_state = ss
                    self.secondary_sub = 0
            if "secondary_sub" in patch:
                try:
                    self.secondary_sub = max(0, min(2, int(patch["secondary_sub"])))
                except (TypeError, ValueError):
                    pass
            if "kick_off_team" in patch:
                try:
                    kt = int(patch["kick_off_team"])
                except (TypeError, ValueError):
                    kt = -1
                if kt in (self.team_id, self.opponent_team_id, DROPBALL):
                    self.kick_off_team = kt
            if "our_score" in patch:
                try:
                    self.our_score = max(0, min(255, int(patch["our_score"])))
                except (TypeError, ValueError):
                    pass
            if "opp_score" in patch:
                try:
                    self.opp_score = max(0, min(255, int(patch["opp_score"])))
                except (TypeError, ValueError):
                    pass
            # 逐球员罚下：列表形式为全量替换；兼容旧的布尔 penalize_our（映射 player_id）
            if "penalize_our" in patch:
                if bool(patch["penalize_our"]):
                    self.penalize_our_players.add(self.player_id)
                else:
                    self.penalize_our_players.discard(self.player_id)
            if "penalize_our_players" in patch:
                parsed = self._parse_players(patch["penalize_our_players"])
                if parsed is not None:
                    self.penalize_our_players = parsed
            if "penalize_opp_players" in patch:
                parsed = self._parse_players(patch["penalize_opp_players"])
                if parsed is not None:
                    self.penalize_opp_players = parsed

    # -- 任意球自动时序 ----------------------------------------------------
    def begin_freekick(self, secondary_state: int, team: int, wait_kicker: bool = False) -> None:
        """出界判罚入口：设置二级状态与球权方，并启动子状态自动时序。

        wait_kicker=True 时 GET_READY 不按固定时长推进，而是挂起等待
        note_kicker_ready() 的就位信号（发球方到位）后才进入 SET——
        "发任意球不按定时、球员到位再进入下一步"。
        """
        with self._lock:
            if secondary_state not in SECONDARY_NAMES or secondary_state < STATE2_DIRECT_FREEKICK:
                return
            self.secondary_state = secondary_state
            self.secondary_sub = 0
            if team in (self.team_id, self.opponent_team_id):
                self.kick_off_team = team
            self._fk_stage = 0
            self._fk_stage_start = time.monotonic()
            self._fk_wait_kicker = bool(wait_kicker)
            self._fk_kicker_ready = False

    def note_kicker_ready(self) -> None:
        """发球方已就位信号（外部判定方如 match_official，或面板操作员）:
        放行挂起的 GET_READY，立即进入 SET。"""
        with self._lock:
            self._fk_kicker_ready = True

    def cancel_freekick(self) -> None:
        """打断任意球时序并恢复常规比赛（进球/复位/手动介入/离开 PLAYING）。"""
        with self._lock:
            self._fk_stage = None
            self._fk_wait_kicker = False
            self._fk_kicker_ready = False
            if self.secondary_state >= STATE2_DIRECT_FREEKICK:
                self.secondary_state = STATE2_NORMAL
            self.secondary_sub = 0

    def _advance_freekick(self, now: float) -> None:
        """推进任意球子状态时序；调用方须持有锁。

        wait_kicker 模式：GET_READY（stage 1）不看时长——挂起直到收到就位信号
        （或兜底超时 FREEKICK_READY_TIMEOUT）后立即进入 SET；SET 仍保留完整时长。
        """
        if self._fk_stage is None:
            return
        if self.state != STATE_PLAYING or not self.enabled:
            self.cancel_freekick()
            return
        while self._fk_stage is not None:
            sub, dur = FREEKICK_STAGES[self._fk_stage]
            self.secondary_sub = sub
            if self._fk_wait_kicker and sub == 1:
                if not self._fk_kicker_ready:
                    if (FREEKICK_READY_TIMEOUT is not None
                            and now - self._fk_stage_start >= FREEKICK_READY_TIMEOUT):
                        self._fk_kicker_ready = True    # 兜底放行，防永久挂起比赛
                    else:
                        return                          # 挂起 GET_READY，等发球方到位
                # 到位即走：立即进入 SET，以当前时刻为阶段起点（SET 时长得完整）
                self._fk_stage += 1
                self._fk_stage_start = now
                if self._fk_stage >= len(FREEKICK_STAGES):
                    self._fk_stage = None
                    self.secondary_state = STATE2_NORMAL
                    self.secondary_sub = 0
                    self._fk_wait_kicker = False
                    self._fk_kicker_ready = False
                continue
            if now - self._fk_stage_start < dur:
                return
            self._fk_stage_start += dur
            self._fk_stage += 1
            if self._fk_stage >= len(FREEKICK_STAGES):
                # 时序结束：球已就位，清除二级状态恢复比赛
                self._fk_stage = None
                self.secondary_state = STATE2_NORMAL
                self.secondary_sub = 0
                self._fk_wait_kicker = False
                self._fk_kicker_ready = False

    # -- 进球结算 / 重开仪式 -----------------------------------------------
    def apply_goal(self, our_goals: int, opp_goals: int) -> None:
        """进球结算：比分累计 + 交换开球权（失球方开球）+ 重开仪式。

        启用且处于 PLAYING 时进入重开仪式（READY → SET → PLAYING）；其余
        状态仅结算比分与开球权（如仪式进行中再次进球，则继续原仪式）。
        """
        with self._lock:
            self.our_score = min(255, self.our_score + max(0, int(our_goals)))
            self.opp_score = min(255, self.opp_score + max(0, int(opp_goals)))
            # 交换开球权：我方进球 → 对方开球；对方进球 → 我方开球
            if our_goals:
                self.kick_off_team = self.opponent_team_id
            elif opp_goals:
                self.kick_off_team = self.team_id
            # 进球即结束进行中的出界判罚（回到常规重开流程）
            self.secondary_state = STATE2_NORMAL
            self.secondary_sub = 0
            self._fk_stage = None
            self._fk_wait_kicker = False
            self._fk_kicker_ready = False
            if self.enabled and self.state == STATE_PLAYING:
                self.begin_goal_restart()   # RLock 可重入

    def begin_goal_restart(self) -> None:
        """进球重开入口：清判罚、状态置 READY（失球方开球由调用方设置）。

        auto 开启：把自动时间线对齐到 READY 阶段里程碑（随后由 _advance_auto
        推进 READY → SET → PLAYING）；auto 关闭：由 _gr_stage 一次性序列推进。
        """
        with self._lock:
            self.secondary_state = STATE2_NORMAL
            self.secondary_sub = 0
            self._fk_stage = None
            self.state = STATE_READY
            if self.auto:
                self._gr_stage = None
                self._phase_index = 0
                for i, (st, _d, _n) in enumerate(TIMELINE):
                    if st == STATE_READY:
                        self._phase_index = i
                        break
                self._phase_start = time.monotonic()
            else:
                self._gr_stage = 0
                self._gr_stage_start = time.monotonic()

    def _advance_goal_restart(self, now: float) -> None:
        """推进进球重开仪式一次性序列（auto 关闭时启用）；调用方须持有锁。"""
        if self._gr_stage is None:
            return
        if not self.enabled:
            self._gr_stage = None   # 停发即操作员接管
            return
        while self._gr_stage is not None:
            st, dur = GOAL_RESTART_STAGES[self._gr_stage]
            self.state = st
            if now - self._gr_stage_start < dur:
                return
            self._gr_stage_start += dur
            self._gr_stage += 1
            if self._gr_stage >= len(GOAL_RESTART_STAGES):
                self._gr_stage = None
                self.state = STATE_PLAYING

    # -- 逐球员罚下 --------------------------------------------------------
    def _parse_players(self, values):
        """解析球员号列表为合法集合（1..players_per_team）；非法输入返回 None（忽略）。"""
        try:
            items = list(values)
        except TypeError:
            return None
        out = set()
        for v in items:
            try:
                n = int(v)
            except (TypeError, ValueError):
                continue
            if 1 <= n <= self.players_per_team:
                out.add(n)
        return out

    def _penalty_slots(self, penalized: set) -> list:
        """11 槽位 penalty 值：1..players_per_team 视罚下集合，未用槽位恒 SUBSTITUTE。

        未用槽位必须非 NONE：brain 侧 liveCount/gcAliveCount 遍历全部
        HL_MAX_NUM_PLAYERS 槽位统计 PENALTY_NONE，填充后才能得到期望人数。
        """
        slots = []
        for i in range(HL_MAX_NUM_PLAYERS):
            if i >= self.players_per_team:
                slots.append(PENALTY_SUBSTITUTE)
            elif (i + 1) in penalized:
                slots.append(PENALTY_MANUAL)
            else:
                slots.append(PENALTY_NONE)
        return slots

    # -- 数据包构造 --------------------------------------------------------
    def build_packet(self) -> bytes:
        """按当前状态构造 688 字节 HL v12 数据包。"""
        with self._lock:
            data = HlRoboCupGameControlData()
            ctypes.memset(ctypes.byref(data), 0, ctypes.sizeof(data))
            data.header = GAMECONTROLLER_STRUCT_HEADER
            data.version = HL_GAMECONTROLLER_STRUCT_VERSION
            data.packetNumber = self._packet_number & 0xFF
            data.playersPerTeam = self.players_per_team
            data.gameType = GAME_ADULT
            data.state = self.state
            data.firstHalf = 1
            data.kickOffTeam = self.kick_off_team
            data.secondaryState = self.secondary_state

            sub = self.secondary_sub if self.secondary_state >= STATE2_DIRECT_FREEKICK else 0
            sub = max(0, min(2, sub))
            info_team = self.kick_off_team if self.secondary_state >= STATE2_DIRECT_FREEKICK else 0
            for i, b in enumerate((info_team & 0xFF, sub & 0xFF, 0, 0)):
                data.secondaryStateInfo[i] = b

            data.dropInTeam = DROPBALL
            data.dropInTime = 0xFFFF
            data.secsRemaining = self.secs_remaining & 0xFFFF
            data.secondaryTime = 0

            our = data.teams[0]
            our.teamNumber = self.team_id
            our.fieldPlayerColour = TEAM_CYAN
            our.score = min(self.our_score, 255)
            for i, pen in enumerate(self._penalty_slots(self.penalize_our_players)):
                our.players[i].penalty = pen
                if pen == PENALTY_MANUAL:
                    our.players[i].secsTillUnpenalised = 40

            opp = data.teams[1]
            opp.teamNumber = self.opponent_team_id
            opp.fieldPlayerColour = TEAM_MAGENTA
            opp.score = min(self.opp_score, 255)
            for i, pen in enumerate(self._penalty_slots(self.penalize_opp_players)):
                opp.players[i].penalty = pen
                if pen == PENALTY_MANUAL:
                    opp.players[i].secsTillUnpenalised = 40

            return bytes(data)

    # -- 内部：比赛时钟 ----------------------------------------------------
    def _advance_clock(self, now: float) -> None:
        """比赛时钟：仅 PLAYING（且启用发送）期间按真实时间递减；归零判半场结束。

        与自动时间线解耦——无论 auto 开关如何，只要处于 PLAYING 就走时。
        调用方须持有锁。
        """
        dt = now - self._clock_last
        self._clock_last = now
        if self.state != STATE_PLAYING or not self.enabled or self.secs_remaining <= 0:
            self._clock_acc = 0.0
            return
        self._clock_acc += dt
        whole = int(self._clock_acc)
        if whole <= 0:
            return
        self._clock_acc -= whole
        self.secs_remaining = max(0, self.secs_remaining - whole)
        if self.secs_remaining == 0:
            # 时间到：半场结束，停止自动时间线等待操作员复位
            self.state = STATE_FINISHED
            self.auto = False

    # -- 内部：自动时间线 --------------------------------------------------
    def _advance_auto(self, now: float) -> None:
        """推进自动时间线（仅状态循环，不再改写比赛时钟）；调用方须持有锁。"""
        while True:
            idx = self._phase_index % len(TIMELINE)
            _state, dur, _name = TIMELINE[idx]
            if now - self._phase_start < dur:
                break
            self._phase_index = (idx + 1) % len(TIMELINE)
            self._phase_start += dur
            self.state = TIMELINE[self._phase_index][0]

    # -- 内部：发送线程 ----------------------------------------------------
    def _run(self) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        next_send = time.monotonic()
        while not self._stop.is_set():
            now = time.monotonic()
            with self._lock:
                if self.auto:
                    self._advance_auto(now)
                self._advance_freekick(now)
                self._advance_goal_restart(now)
                self._advance_clock(now)
                enabled = self.enabled
                self._packet_number = (self._packet_number + 1) & 0xFF
                packet = self.build_packet() if enabled else None
            if packet is not None:
                try:
                    self._sock.sendto(packet, (self.target, self.port))
                    self._sent_count += 1
                    self._last_send_ts = now
                except OSError:
                    self._send_error_count += 1
            next_send += 0.2
            sleep = next_send - time.monotonic()
            if sleep > 0:
                self._stop.wait(sleep)
            else:
                next_send = time.monotonic()

    # -- 快照 --------------------------------------------------------------
    def snapshot(self) -> dict:
        with self._lock:
            age = (time.monotonic() - self._last_send_ts) if self._last_send_ts else None
            return {
                "enabled": self.enabled,
                "auto": self.auto,
                "state": self.state,
                "state_name": STATE_NAMES.get(self.state, "?"),
                "secs_remaining": self.secs_remaining,
                "secondary_state": self.secondary_state,
                "secondary_name": SECONDARY_NAMES.get(self.secondary_state, "?"),
                "secondary_sub": self.secondary_sub,
                "freekick_stage": self._fk_stage,
                "waiting_kicker": bool(self._fk_wait_kicker and self._fk_stage == 1
                                       and not self._fk_kicker_ready),
                "kicker_ready": self._fk_kicker_ready,
                "goal_restart_stage": self._gr_stage,
                "kick_off_team": self.kick_off_team,
                "our_score": self.our_score,
                "opp_score": self.opp_score,
                "penalize_our": self.player_id in self.penalize_our_players,
                "penalize_our_players": sorted(self.penalize_our_players),
                "penalize_opp_players": sorted(self.penalize_opp_players),
                "players_per_team": self.players_per_team,
                "our_penalty_slots": self._penalty_slots(self.penalize_our_players),
                "opp_penalty_slots": self._penalty_slots(self.penalize_opp_players),
                "team_id": self.team_id,
                "opponent_team_id": self.opponent_team_id,
                "player_id": self.player_id,
                "target": f"{self.target}:{self.port}",
                "sent_count": self._sent_count,
                "send_error_count": self._send_error_count,
                "last_send_age": round(age, 2) if age is not None else None,
            }


if __name__ == "__main__":
    import argparse
    import sys

    parser = argparse.ArgumentParser(description="裁判机模拟器（referee_core）")
    parser.add_argument("--selfcheck", action="store_true", help="结构体尺寸自检后退出")
    parser.add_argument("--target", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=GAMECONTROLLER_PORT)
    parser.add_argument("--team-id", type=int, default=29)
    args = parser.parse_args()

    if args.selfcheck:
        sys.exit(0 if selfcheck() else 1)

    ref = RefereeSim(target=args.target, port=args.port, team_id=args.team_id)
    ref.update({"auto": True, "enabled": True})
    ref.start()
    print(f"裁判机模拟器已启动 -> {args.target}:{args.port}（自动时间线，Ctrl+C 退出）")
    try:
        while True:
            time.sleep(1)
            snap = ref.snapshot()
            print(f"  [{snap['state_name']:>8}] 剩余 {snap['secs_remaining']:>3}s "
                  f"二级 {snap['secondary_name']:<16} 比分 {snap['our_score']}:{snap['opp_score']} "
                  f"已发送 {snap['sent_count']}")
    except KeyboardInterrupt:
        ref.stop()
        print("\n已停止")
