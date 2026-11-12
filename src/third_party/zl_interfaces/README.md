# zl_interfaces：zl_robot 接口包冻结快照

`robocup_demo/brain` 编译期只依赖 `zl_robot` 的两个纯消息包（`robot_interfaces`、
`yesense_interface`）。本目录是这两个包的**冻结副本**，使根仓库在未取 `zl_robot`
源码时也能自行编译通过；它不包含 zl 的控制栈、驱动、仿真等任何实现。


## 目录

```
third_party/zl_interfaces/
├── build.sh                     # 独立前缀构建（build/ 与 install/ 均在本目录内）
├── zl_interfaces.ref            # 冻结来源：zl_robot 的仓库、分支、提交
└── src/
    ├── robot_interfaces/                  # 17 msg + 21 srv；brain 的 RobotIntent/RobotState/HeadControl/RlManagerStatus/JointStates 在此
    └── robot_imu/yesense_interface/       # 23 msg；brain 的 NavAll 在此
```

> **本地补充**：`yesense_interface/msg/NavAllStamped.msg`（含 `std_msgs` 依赖）来自
> zl_robot 工作区，用于仿真新增的 `/robot_X/imu/nav_all_stamped` 话题；同步时该消息
> 尚未进入 zl_robot 提交，因此未包含在 `75ef424` 快照里。上游提交后可重导移除。

## 构建

```bash
bash third_party/zl_interfaces/build.sh
source third_party/zl_interfaces/install/setup.bash
```

`robocup_demo/scripts/build.sh` 已有三级门控，正常情况下无需手动执行上一步：

1. `../zl_robot/install/setup.bash` 存在 → 用它（实机/已取码环境，永远优先）；
2. 否则快照 `install/` 已构建 → 用它，并打印 WARN；
3. 否则快照源码在 → 自动构建快照再 source；
4. 都没有 → 报错并给出两条取码路径。

**不要同时 source 两份 install**：`robot_interfaces` / `yesense_interface` 是
同名包，两份不同版本的消息定义会导致运行期反序列化不兼容。build.sh 会在检测到
外部环境已含 `zl_robot/install` 时告警。

## 更新（zl_robot 跟版后必做）

```bash
bash scripts/ref/sync_zl_interfaces.sh --check   # 只比对：列出字段级差异，有差异返回 1
bash scripts/ref/sync_zl_interfaces.sh --apply   # 按 zl_robot 当前 HEAD 重导并刷新 ref
```

`--apply` 只覆盖本目录 `src/` 下两个包，不动 `build/`、`install/`；重导后需重跑
`build.sh`。差异若涉及**删除或改名字段**，brain 会编译失败，属预期闸门；仅新增
字段则编译通过，但语义是否兼容需人工确认。

## 限制（务必知悉）

- 本快照**只保证编译**。运行期仍需要 zl 节点（NX/板子提供）发布 `RobotState`、
  `RobotIntent` 等话题，接口语义对齐靠实机联调。
- 冻结副本不会自动跟随同事改动：新增字段编译期无感，因此 zl_robot 每次跟版后
  都要跑一次 `--check`，把差异当作契约变更评审。
- 若同事拆包/改名（例如 `robot_interfaces` 更名），需同步改
  `robocup_demo/src/brain/{package.xml,CMakeLists.txt}` 与本目录脚本。
