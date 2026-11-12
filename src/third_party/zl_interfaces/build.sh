#!/usr/bin/env bash
# 冻结接口包独立构建：robot_interfaces / yesense_interface
#
# 目的：根仓库未取 zl_robot 源码时，仍能让 robocup_demo/brain 编译通过。
# 关键：build/install 前缀都落在本目录内，绝不复用 zl_robot/install 或
#       robocup_demo/install，避免同名包的双份消息定义混入同一 AMENT 前缀。
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "[ERR] 未找到 /opt/ros/humble/setup.bash，无法构建接口包" >&2
  exit 1
fi

# shellcheck disable=SC1091
# ROS 的 setup.bash 引用未定义变量（AMENT_TRACE_SETUP_FILES 等），必须先关 -u
set +u
source /opt/ros/humble/setup.bash
set -u

colcon build \
  --base-paths "$HERE/src" \
  --build-base "$HERE/build" \
  --install-base "$HERE/install" \
  --symlink-install \
  "$@"

echo "[OK] 冻结接口包已构建，使用方式：source $HERE/install/setup.bash"

# 自检：快照 install 不得反向引用 zl_robot/install（否则 source 快照会把真实 zl 前缀
# 一起拉进来，造成 robot_interfaces/yesense_interface 同名包双份）。
if grep -rq "zl_robot/install" "$HERE/install" 2>/dev/null; then
  echo "[WARN] 快照 install 中出现了 zl_robot/install 引用，请在新 shell（未 source zl_robot）中重建：" >&2
  echo "[WARN]   rm -rf $HERE/build $HERE/install && bash $HERE/build.sh" >&2
fi
