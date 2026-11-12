#!/bin/bash
# 独立裁判机 UI 启停脚本（无 ROS 依赖；协议实现在本目录 ../app/referee_core.py，
# 不依赖 sim_console，整目录拷走即可运行）。
#
# 用法:
#   ./start_referee_ui.sh [run|fg|stop|status] [gc_ui.py 参数...]
#     run     后台启动（默认）：setsid + PID 文件
#     fg      前台启动（Ctrl+C 退出，便于看实时状态行）
#     stop    停止（SIGINT → 兜底 SIGKILL）
#     status  查看运行状态
#
# 透传参数示例:
#   ./start_referee_ui.sh run --http-port 8090 --target 127.0.0.1 --port 3838 \
#       --team-id 29 --opponent-team-id 30 --players-per-team 3
#
# 环境变量:
#   REFEREE_UI_LOG_DIR  覆盖日志/PID 目录（默认 runtime/referee_ui）
#
# 配套（另开终端，域 100，与 Brain 同域）:
#   ros2 launch game_controller launch.py
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
UI_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
REPO_ROOT="$(cd "$UI_ROOT/.." && pwd)"
ENTRY="$UI_ROOT/app/gc_ui.py"

# 日志目录：本仓统一落在 runtime/referee_ui（runtime/ 已 gitignore），
# 可用 REFEREE_UI_LOG_DIR 显式覆盖。
LOG_DIR="${REFEREE_UI_LOG_DIR:-$REPO_ROOT/runtime/referee_ui}"
LOG="$LOG_DIR/referee_ui.log"
PID_FILE="$LOG_DIR/referee_ui.pid"

# 日志轮转器（可选；缺失时降级为裸重定向）：优先随包副本，其次仓库 scripts/。
ROTATOR=""
for cand in "$UI_ROOT/scripts/log_rotate.py" "$REPO_ROOT/scripts/log_rotate.py"; do
  if [ -f "$cand" ]; then ROTATOR="$cand"; break; fi
done

MODE="${1:-run}"
case "$MODE" in
  run|fg|stop|status) shift ;;
  *) MODE=run ;;
esac

is_running() {  # 输出 "pid cmd"；未运行时退出码非 0
  local out
  out="$(pgrep -a -f '[g]c_ui.py' | head -1)"
  [ -n "$out" ] || return 1
  printf '%s\n' "$out"
}

case "$MODE" in
  stop)
    if [ -f "$PID_FILE" ]; then kill -INT "$(cat "$PID_FILE")" 2>/dev/null || true; fi
    pkill -INT -f '[g]c_ui.py' 2>/dev/null || true
    sleep 2
    pkill -9 -f '[g]c_ui.py' 2>/dev/null || true
    rm -f "$PID_FILE"
    echo "裁判机 UI 已停止"
    ;;
  status)
    RUNNING="$(is_running || true)"
    if [ -n "$RUNNING" ]; then
      echo "运行中: $RUNNING"
      grep -E "面板:" "$LOG" 2>/dev/null | tail -1 || true
    else
      echo "未运行"
    fi
    ;;
  fg)
    exec python3 "$ENTRY" "$@"
    ;;
  run)
    if is_running >/dev/null 2>&1; then
      echo "已在运行（先 stop，或用 status 查看）: $(is_running)"
      exit 0
    fi
    mkdir -p "$LOG_DIR"
    if [ -f "$ROTATOR" ]; then
      # 长驻日志统一经 log_rotate.py 按大小轮转（rotator 会转发 SIGINT/SIGTERM）。
      setsid nohup python3 "$ROTATOR" --log "$LOG" --max 8M --keep 2 -- \
        python3 "$ENTRY" "$@" >/dev/null 2>&1 &
    else
      setsid nohup python3 "$ENTRY" "$@" >"$LOG" 2>&1 &
    fi
    echo $! > "$PID_FILE"
    sleep 1.5
    echo "裁判机 UI 已启动（PID $(cat "$PID_FILE")），日志: $LOG"
    grep -E "面板:|UDP:|配合:" "$LOG" 2>/dev/null | tail -3 || true
    ;;
esac
