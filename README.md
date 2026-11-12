# robocup_sim

RoboCup 六机比赛独立仿真仓：MuJoCo 物理仿真 + 真值定位 + 比赛管理，与
`zl_robot`（运控）和 `robocup_demo`（brain/策略）三方独立启动。

> 本仓库只负责“仿真 + 真值 + 比赛管理”。机器人运控与策略来自两个外部 ROS 2
> 工作区（见[外部工作区](#外部工作区control--brain)）。只做模型、场景或话题开发时，
> 可以不准备它们，直接以 `--role sim` 启动。

## 设计目标

- **贴近实机**：仿真+zl 运控在域 10，brain/GC 在域 100，经 domain_bridge 白名单跨域；
  与实机 3588（域 10）↔ NX（域 100）的分工一致。
- **三方独立**：仿真机独占 MuJoCo；zl 控制栈与 brain 可按机器人子集分布到多台设备。
- **真值定位**：仿真按机器人发布标准 `/soccer/ground_truth_pose|ball/<robot>`，
  brain 的 `ground_truth.enable_pose/enable_ball` 直接消费；实机保持关闭。
- **名单可选**：`roster.yaml` 选择上场机器人与角色，默认 A 队 3 台，可扩到 6 台；
  未选中的机器人由仿真补丁冻结到场外。
- **完整比赛**：自带 game_controller、边裁/摆球、referee_ui Web 面板与完整比赛状态机。

## 环境要求

| 项 | 要求 |
|---|---|
| 操作系统 | Ubuntu 22.04（x86_64 或 aarch64） |
| ROS 2 | Humble（`/opt/ros/humble/setup.bash`） |
| 编译器 | 支持 C++17 的 g++；CMake ≥ 3.16 |
| 构建工具 | `colcon`（`python3-colcon-common-extensions`） |
| Python | Python 3.10 + PyYAML |
| 显示 | 窗口模式需要 OpenGL/X11 或 Wayland；无显示器用 `--headless` |
| 外部工作区 | `zl_robot`（仅 control 角色）、`robocup_demo`（仅 brain 角色） |

Ubuntu 22.04 上安装依赖：

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-rmw-cyclonedds-cpp \
  ros-humble-domain-bridge \
  ros-humble-eigen3-cmake-module \
  python3-colcon-common-extensions \
  python3-yaml \
  libeigen3-dev
```

MuJoCo 3.1.1 与 GLFW 已随仓放在 `src/robot_sim/mj_sim/thirdparty/`，无需另装；
Eigen 的随仓副本供 `mj_sim` 使用，`sim_common` 还依赖系统 `libeigen3-dev`（见上）。
如需补齐其它 ROS 依赖，可用 `rosdep install --from-paths src --ignore-src -r -y`。

## 仓库结构

```
robocup_sim                         CLI：渲染配置、设置 DDS/ROS 环境、按角色拉起进程
build.sh                            构建入口（colcon + 可选 source zl_robot）
roster.yaml                         上场名单与队号
inventory.yaml                      主机、角色、域号、DDS 模式、外部工作区路径
src/robot_sim/mj_sim/               MuJoCo 基类、GUI/渲染、随仓 MuJoCo/GLFW/Eigen
src/robot_sim/sim_common/           ROS 节点基类、消息交换、误差模型
src/robot_sim/sim_z01_std_robocup/  六机仿真节点（MuJoCo 模型加载与状态发布）
src/robot_assets/                   六机/单机模型、球场、网格与生成脚本
src/robocup_sim_bridge/             GT + 逐机踢球感知中继（sim_relay_node）
src/robocup_sim_bringup/            sim / relay launch
src/third_party/zl_interfaces/      robot_interfaces / yesense_interface 冻结快照
src/vendor_robocup_demo/            game_controller + 消息包
referee_ui/                         独立裁判 Web UI + match manager（边裁/摆球）
configs/                            sim / brain / DDS 配置与模板
tools/standup.py                    ready→reload→walk 立起机器人
docs/                               架构、话题契约、部署说明
runtime/                            运行时生成物（gitignore）
```

域桥配置没有静态模板：`domain_bridge.yaml` 在每次 `up` 时按 roster 渲染到
`runtime/<run_id>/bridge/`。

注意：`src/third_party/zl_interfaces/README.md` 与 `referee_ui` 的部分 docstring
保留了上游工作区的脚本路径（如 `scripts/joint_test/`、`scripts/ref/`），这些脚本
不在本仓；本仓对应功能已由 `robocup_sim`、`tools/standup.py` 和 `referee_ui/app/`
提供。

## 快速开始（单机、A 队 3 台）

```bash
git clone <repo-url> robocup_sim
cd robocup_sim

./build.sh                 # 首次构建几分钟；产物在 install/
./robocup_sim doctor       # 环境自检

./robocup_sim up --role all --profile local
# 浏览器打开 http://127.0.0.1:8090/，把比赛状态切到 PLAYING
./robocup_sim standup
./robocup_sim status
./robocup_sim down
```

无显示器/远程机器：

```bash
./robocup_sim up --role all --headless
```

6 台同机：

```bash
./robocup_sim up --role all \
  --robots "robot_01 robot_02 robot_03 robot_04 robot_05 robot_06"
```

> **单实例约束**：重复执行 `up` 会复用当前运行中的 run（已运行的角色直接跳过），
> 不会新建第二套进程；`down` 默认停止本机所有 run。`--run-id` 只能指向当前存活的
> run，否则会被拒绝。
>
> 8 核机器上 6 套 RL + 6 个 brain 会算力不足（实测站立数秒后失稳），推荐把
> control/brain 按 roster 分到多台设备（见 [docs/deployment.md](docs/deployment.md)）。

常用参数：

| 参数 | 说明 |
|---|---|
| `--role sim/control/brain/boundary/match/all` | 本机启动哪些角色 |
| `--profile local/unicast` | DDS 模式；默认取 `inventory.yaml` 的 `dds.mode` |
| `--robots "robot_01 ..."` | 覆盖 `roster.yaml` 的 `enabled` 名单 |
| `--headless` | 关闭 MuJoCo 窗口 |
| `--cameras robot_01,robot_02` | 按台开启深度相机（只影响渲染出的 `sim.yaml`） |
| `--host <name>` | 指定 inventory 主机；默认按本机 IP 匹配，匹配不到用第一个 host |
| `--no-boundary` / `--no-match` | `--role sim` 时不顺带启动 boundary / 比赛管理 |

## 外部工作区（control / brain）

| 变量 / 配置 | 默认 | 用途 |
|---|---|---|
| `ZL_ROOT` 或 `inventory.yaml: hosts.<host>.zl_root` | `~/zl_robot` | zl_robot 运控工作区；需要 `install/setup.bash` 与 `robot_bringup` 的 `zl_robot_multi.launch.py` |
| `BRAIN_WS` 或 `inventory.yaml: hosts.<host>.brain_ws` | `~/robocup_demo` | brain/策略工作区；需要 `install/setup.bash` 与 `brain` 包 |

优先级：环境变量 > `inventory.yaml` > 默认路径；路径支持 `~` 与 `$VAR`。
环境变量建议只在单机使用；多设备部署时请按主机在 `inventory.yaml` 中分别填写路径。
两个工作区都只需在本机构建好（`install/` 不可跨机拷贝）。

只做仿真/模型/话题开发时可以不准备外部工作区：

```bash
./robocup_sim up --role sim --no-boundary --no-match
```

该命令只启动 MuJoCo 与 GT 中继，不启动运控与 brain（机器人不会站立），适合调模型、
相机、场地和 `/sim/*` 话题。需要完整比赛时再准备两个外部工作区。

> `doctor` 会检查 inventory 中所有 control/brain 主机的工作区；只跑仿真时可以对
> 对应的 ERR 忽略不计，或把 inventory 临时改成只含 sim 角色。

## 配置速查

| 文件 | 作用 | 常改字段 |
|---|---|---|
| `roster.yaml` | 上场名单、队号、角色 | `robots[].enabled/team/player_id/role/control_host/brain_host`、`match.*` |
| `inventory.yaml` | 主机、角色、域号、DDS 模式、外部工作区路径 | `hosts.*.address/dds_iface/roles/zl_root/brain_ws`、`dds.mode` |
| `configs/sim/sim.yaml` | MuJoCo 仿真参数 | 出生点、`camera_period_us_list`、`errors.*`、`viewer.*` |
| `configs/brain/sim_profile.yaml` | brain 仿真 profile | GT 开关、多机通信、踢球视觉 |
| `configs/dds/cyclonedds_localhost.xml` | 单机 lo DDS 配置 | 一般不改 |

本机私有覆盖：把 `inventory.yaml` / `roster.yaml` 复制为 `inventory.local.yaml` /
`roster.local.yaml` 后修改，CLI 会优先读取这两个文件；它们已在 `.gitignore` 中，
适合保存本机路径、IP、调试名单，不会污染公开仓库的通用默认值。

运行产物（不提交）：

| 路径 | 内容 |
|---|---|
| `runtime/<run_id>/dds/dds.xml` | `--profile local` 复制 localhost 配置；`unicast` 按 inventory 渲染跨机 Peers |
| `runtime/<run_id>/bridge/domain_bridge.yaml` | 域 10 ↔ 100 逐机话题白名单 |
| `runtime/<run_id>/sim/sim.yaml`、`brain/sim_profile.yaml` | 叠加 CLI 参数后的渲染结果 |
| `runtime/<run_id>/logs/` | 各角色日志 |
| `runtime/<run_id>/state.json`、`runtime/active.json` | 进程组记录 / 当前 run |

`up` 只渲染到 `runtime/`，不会改 `configs/`；要改默认值请直接编辑上表文件。

## 代码导览

| 想改什么 | 从哪里开始 |
|---|---|
| 仿真节点 / 机器人状态发布 | `src/robot_sim/sim_z01_std_robocup/`（入口 `src/main.cpp`、`SimZ01StdRobocup.cpp`） |
| 物理/渲染/GUI 基类 | `src/robot_sim/mj_sim/`（`MujocoSimBase.cpp`、`simulate/`） |
| ROS 消息交换 / 误差模型 | `src/robot_sim/sim_common/` |
| GT、感知中继 | `src/robocup_sim_bridge/src/sim_relay_node.cpp` |
| 六机模型 / 球场 / 网格 | `src/robot_assets/`（生成器在 `utils/`） |
| launch | `src/robocup_sim_bringup/launch/` |
| 裁判面板 / 边裁 / 摆球 | `referee_ui/app/` |
| CLI、roster/inventory 解析 | `robocup_sim` |
| 话题契约 | [docs/topics.md](docs/topics.md) |
| 架构与部署 | [docs/architecture.md](docs/architecture.md)、[docs/deployment.md](docs/deployment.md) |

常用开发命令：

```bash
# 只构建某个包
./build.sh --packages-select robocup_sim_bridge

# 新增/修改 robot_assets 的 XML 后，强制重新 configure mj_sim
./build.sh --packages-select mj_sim --cmake-force-configure

# 生成并校验六机模型（改过机器人本体或球场后）
python3 src/robot_assets/utils/gen_robocup_multi_xml.py
python3 src/robot_assets/utils/gen_robocup_multi_xml.py --check

# 查看当前 run 的进程与日志
./robocup_sim status
tail -f runtime/<run_id>/logs/sim.log
```

`robocup_sim` 本身只做三件事：渲染 `runtime/<run_id>/` 下的配置、设置 DDS/ROS
环境变量、按角色拉起进程组并记录到 `state.json`。调试单个模块时，可展开成下面的
等效命令手动执行。本仓目前没有自动化测试；改动后用 `./robocup_sim doctor`、
`./robocup_sim status` 与 `ros2 topic echo` 做冒烟验证。

## 快捷指令与等效命令（调试用）

以下命令均在仓库根目录执行。注意 `ros2 launch` / `ros2 run` 会常驻阻塞，
实际调试时请分别开终端执行（或自行加 `&` 放后台）。

### 公共环境

每次 `up` 生成的 run 目录：

| 文件 | 内容 |
|---|---|
| `dds/dds.xml` | `--profile local` 复制 `configs/dds/cyclonedds_localhost.xml`；`unicast` 按 `inventory.yaml` 渲染域 10/100 的 Peers（`--profile` 默认取 `inventory.yaml` 的 `dds.mode`） |
| `sim/sim.yaml` | `configs/sim/sim.yaml` + 按 `--cameras` 生成的 `camera_period_us_list` |
| `bridge/domain_bridge.yaml` | 逐机器人的域 10 ↔ 100 话题白名单 |
| `brain/sim_profile.yaml` | `configs/brain/sim_profile.yaml` + `game_control_ip` |
| `state.json` / `active.json` / `logs/` | 进程组记录 / 当前 run / 各角色日志 |

手动启动前先设置环境（`RUN` 换成当前 run 目录）：

```bash
cd /path/to/robocup_sim
RUN=runtime/<run_id>                  # 例如 runtime/20261111_101500_ab12cd
export ROS_DOMAIN_ID=10               # sim/control/boundary/match_manager；brain/GC 用 100
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$PWD/$RUN/dds/dds.xml"
export ROS_HOME="$PWD/$RUN/ros_home"
export ROS_LOG_DIR="$PWD/$RUN/ros_log"
export TMPDIR="$PWD/$RUN/tmp"
source /opt/ros/humble/setup.bash
```

> 没有现成 run 时，可先跑一次 `./robocup_sim up --role sim --no-boundary --no-match`
> 生成配置，再手动调试其它模块。`./robocup_sim standup --dry-run` 会打印它将要执行的
> 完整命令。

### `./build.sh`

```bash
source /opt/ros/humble/setup.bash
# ZL_ROOT（旧名 ZL_ROBOT_ROOT）存在时 build.sh 会先额外 source 它的 install/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

额外参数原样透传，例如 `./build.sh --packages-select robocup_sim_bridge`。
未设置 `ZL_ROOT` 时，本仓 `src/third_party/zl_interfaces` 提供冻结的消息接口，
可以完全独立构建。

### `up --role control`

```bash
source /opt/ros/humble/setup.bash
export ZL_ROOT="${ZL_ROOT:-$HOME/zl_robot}"   # 或从 inventory.yaml 读取
source "$ZL_ROOT/install/setup.bash"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 \
       MKL_NUM_THREADS=1 NUMEXPR_NUM_THREADS=1
ros2 launch robot_bringup zl_robot_multi.launch.py \
  robots:="robot_01 robot_02 robot_03" \
  config_root:="$ZL_ROOT/config"
```

按 `roster.yaml` 的 `control_host` 选机器人；`--role all` 时忽略该分配，启动全部选中机器人。

### `up --role sim`

```bash
source install/setup.bash
ros2 launch robocup_sim_bringup sim.launch.py \
  headless:=false \
  sim_params_file:="$PWD/$RUN/sim/sim.yaml" \
  robot_active_mask:="1,1,1,0,0,0"    # 顺序 robot_01..06，实际由 roster/--robots 生成
ros2 launch robocup_sim_bringup relay.launch.py \
  robots:="robot_01 robot_02 robot_03"
```

`--headless` 对应 `headless:=true`；`--cameras` 只影响生成的 `sim.yaml`。

### `up --role boundary`

```bash
source install/setup.bash
ros2 run domain_bridge domain_bridge "$PWD/$RUN/bridge/domain_bridge.yaml"
```

`--role sim` 默认顺带启动 boundary，`--no-boundary` 可关闭。

### `up --role match`

```bash
source install/setup.bash
# GC：域 100
ROS_DOMAIN_ID=100 ros2 launch game_controller launch.py
# match manager：域 10，面板 :8092
ROS_DOMAIN_ID=10 python3 referee_ui/app/match_manager.py \
  --robots robot_01 robot_02 robot_03 --our-side a \
  --referee-url http://127.0.0.1:8090
# 裁判 Web UI：不依赖 ROS，页面 :8090
python3 referee_ui/app/gc_ui.py --target 127.0.0.1 \
  --team-id 29 --opponent-team-id 30 --players-per-team 3
```

`--role sim` 默认顺带启动 match，`--no-match` 可关闭；`match_manager.py` / `gc_ui.py`
不存在时对应进程会被跳过。

### `up --role brain`

```bash
export BRAIN_WS="${BRAIN_WS:-$HOME/robocup_demo}"   # 或从 inventory.yaml 读取
source "$BRAIN_WS/install/setup.bash"
export ROS_DOMAIN_ID=100
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 \
       MKL_NUM_THREADS=1 NUMEXPR_NUM_THREADS=1
# 每台机器人一条；team_id/player_id/role 来自 roster.yaml
ros2 launch brain launch.py sim:=true \
  robot_name:=robot_01 team_id:=29 player_id:=1 role:=goal_keeper \
  sim_profile:="$PWD/$RUN/brain/sim_profile.yaml"
```

队号规则：A=29、B 单队=30、A+B 同场时 B=39（`roster.yaml` 的 `match` 段可覆盖）。
按 `brain_host` 选机器人，`--role all` 时忽略该分配。

### `up --role all`

依次启动 control → sim → boundary → match → brain。重复 `up` 会复用当前 run；
已存活的进程组由 `spawn` 跳过，不会重复拉起。

### `standup`

```bash
export ROS_DOMAIN_ID=10
python3 tools/standup.py --robots robot_01 robot_02 robot_03 \
  --alt-threshold 0.5 --timeout 60
```

`tools/standup.py` 用 rclpy 直接交互，不走 `ros2` CLI：

- 订阅 `/<robot>/state`、`/<robot>/imu/nav_all`；
- 向 `/<robot>/input/intent` 发布 `RobotIntent`（`source=api`、`intent_type=status`、
  `priority=60`），命令为 `ready` / `walk` / `reinit`；
- 调服务 `/reload_<robot>`（`std_srvs/srv/Trigger`）；
- 判据：`state == RL_ACTIVE(6)` 且 `pos.altitude >= 0.5`。

手动复现单步：

```bash
ros2 topic echo /robot_01/state
ros2 topic echo /robot_01/imu/nav_all
ros2 topic pub --once /robot_01/input/intent robot_interfaces/msg/RobotIntent \
  "{source: api, intent_type: status, command: ready, action: '', priority: 60}"
ros2 service call /reload_robot_01 std_srvs/srv/Trigger "{}"
```

### `status` / `down` / `doctor`

`status` 只读文件，等价于：

```bash
cat runtime/active.json
cat runtime/<run_id>/state.json
kill -0 -- -<pgid> && echo alive || echo dead    # 逐个记录的进程组
ps -eo pid,pgid,stat,cmd | awk '$2 == <pgid>'    # 查看组内进程
```

`down` 对 `state.json` 里每个存活进程组整组发信号（`start_new_session=True` 拉起，
`pgid == pid`），随后删除 `active.json`（`down --run-id` 只在该 run 正好是 active 时删除）：

```bash
kill -INT -- -<pgid>      # 先 SIGINT
sleep 3
kill -KILL -- -<pgid>     # 仍未退出再 SIGKILL
rm -f runtime/active.json
```

`doctor` 是下列检查的集合：

- `/opt/ros/humble/setup.bash`
- roster 至少有一台 `enabled`
- 每台 control 机的 `$ZL_ROOT/install/setup.bash` 与
  `.../robot_bringup/share/robot_bringup/launch/zl_robot_multi.launch.py`
- 每台 brain 机的 `$BRAIN_WS/install/setup.bash`
- 本仓 `install/setup.bash`、`install/robocup_sim_bridge`、六机 MuJoCo XML、
  `configs/brain/sim_profile.yaml`

## 常见问题

**构建时报找不到 Eigen3 / eigen3_cmake_module**
安装 `libeigen3-dev` 与 `ros-humble-eigen3-cmake-module`，重新 `source /opt/ros/humble/setup.bash`
后构建。

**`ros2 run domain_bridge` 找不到可执行文件**
安装 `ros-humble-domain-bridge`；`--role sim` 默认会启动 boundary，只调仿真时可加
`--no-boundary`。

**窗口打开了但没有话题发布**
检查运行目录下的 `MUJOCO_LOG.TXT`。六机模型的 UI 面板超过 OpenGL 辅助缓冲高度上限时，
渲染线程会卡住；本仓已默认关闭 UI 面板，如手工改过 `viewer.ui0/ui1`，先改回 `false`，
或用 `--headless` 确认是否为渲染问题。更多细节见
[src/robot_sim/mj_sim/README.md](src/robot_sim/mj_sim/README.md)。

**机器人立不起来 / `standup` 超时**
先确认裁判面板状态已切到 `PLAYING`（GC 非 PLAYING 时运控不进入 RL），再执行
`./robocup_sim standup`。仍失败时看 `runtime/<run_id>/logs/control.log` 与
`brain_<robot>.log`，并用 `ros2 topic echo /robot_01/state` 确认状态是否到
`RL_ACTIVE(6)`。

**重复启动/残留进程**
`./robocup_sim status` 查看所有存活 run；`./robocup_sim down` 会停掉本机所有 run。
本机同一时刻只允许一个 run 是有意为之，避免两套 MuJoCo 抢同一批话题。

**跨机看不到对方**
确认每台机器的 `inventory.yaml` 中 `address` 是本机实际 IP、`roles` 正确，
`dds.mode: unicast`，并用 `./robocup_sim doctor` 检查；放行 UDP 3838/3939/3940
与 HTTP 8090/8092，各机 NTP 同步。渲染出的 Peers 在
`runtime/<run_id>/dds/dds.xml`，可直接检查。

**`install/` 能否拷贝到别的机器**
不能。`install/` 内嵌构建机绝对路径，每台设备都要在本仓路径下自行 `./build.sh`。

**8 核跑 6 台失稳**
算力不足，不是模型问题。减少到 3 台，或按 `roster.yaml` 把 control/brain 分到多台设备。

## 许可证

Apache-2.0，见 `LICENSE` 与 `NOTICE`。随仓第三方组件的许可证文件位于
`src/robot_sim/mj_sim/thirdparty/{mujoco,glfw,eigen3}/`。
