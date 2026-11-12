#!/usr/bin/env bash
# 构建 robocup_sim 工作区。
#
# 依赖：ROS 2 Humble + rmw_cyclonedds_cpp + domain_bridge + MuJoCo/GLFW（随仓）。
# 注意：install/ 内嵌绝对路径，不可跨机复制；每台设备在本仓路径下各自构建。
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "[ERR] 未找到 /opt/ros/humble/setup.bash" >&2
  exit 1
fi

# ROS 的 setup.bash 引用未定义变量，先关 -u
set +u
source /opt/ros/humble/setup.bash
set -u

# ZL_ROOT 与旧名 ZL_ROBOT_ROOT 都可指定 zl_robot 工作区；未设置时使用本仓
# src/third_party/zl_interfaces 的冻结接口包，完全可以独立构建。
ZL_ROOT="${ZL_ROOT:-${ZL_ROBOT_ROOT:-}}"
if [ -n "$ZL_ROOT" ] && [ -f "$ZL_ROOT/install/setup.bash" ]; then
  echo "[INFO] 额外 source zl_robot: $ZL_ROOT/install/setup.bash" >&2
  set +u
  source "$ZL_ROOT/install/setup.bash"
  set -u
fi

exec colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release "$@"
