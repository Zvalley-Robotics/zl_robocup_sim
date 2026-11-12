# robot_interfaces

zl_robot 的 ROS 2 公共接口包。这里定义跨节点共享的消息和服务，README 应与 `CMakeLists.txt` 中注册的 `.msg`、`.srv` 文件保持一致。

## 消息

当前注册 26 个消息：

| 文件 | 用途 |
|---|---|
| `AudioRequest.msg` | 统一音频请求，供 `audio/request` 使用 |
| `BatteryInfo.msg` | 电池/BMS 状态 |
| `DriverStatus.msg` | 驱动状态、硬件使能和错误摘要 |
| `FaultReport.msg` | 节点内部上报的单条活动故障 |
| `FaultReportArray.msg` | 单个节点当前活动故障快照 |
| `FaultInfo.msg` | 补齐等级和描述后的标准故障信息 |
| `FaultInfoArray.msg` | 面向 App 的系统活动故障快照 |
| `FollowControl.msg` | 跟随控制输出 |
| `HeadControl.msg` | 头部控制命令 |
| `JointCommand.msg` | 单关节命令 |
| `JointCommands.msg` | 多关节命令帧 |
| `JointState.msg` | 单关节反馈 |
| `JointStates.msg` | 多关节反馈帧 |
| `RlManagerStatus.msg` | RL 管理器状态 |
| `RlModelList.msg` | 当前平台可用的 RL 模型能力清单 |
| `RlModelInfo.msg` | RL 模型信息 |
| `RobotIntent.msg` | 上层意图命令 |
| `RobotState.msg` | 系统状态机状态 |
| `SpeakStatus.msg` | 音频播放状态 |
| `SystemStatus.msg` | 系统资源监控状态 |
| `tool_control/ToolCmd.msg` | 灵巧手执行指令 |
| `tool_control/ToolControl.msg` | 多工具控制帧 |
| `tool_control/ToolState.msg` | 单个灵巧手状态 |
| `tool_control/ToolStates.msg` | 多工具状态帧 |
| `VersionInfo.msg` | 版本信息

## 服务

当前注册 21 个服务：

| 文件 | 用途 |
|---|---|
| `EncosGetMotorId.srv` | Encos 电机 ID 查询 |
| `EncosGetMotorParam.srv` | Encos 电机参数查询 |
| `EncosSetMotorId.srv` | Encos 电机 ID 修改 |
| `EncosSetMotorZero.srv` | Encos 电机零点设置 |
| `ListRlModels.srv` | 列出可用 RL 模型 |
| `LoadRlModel.srv` | 加载 RL 模型 |
| `MixQueryJoint.srv` | Mix 关节查询 |
| `MixQueryMotorId.srv` | Mix 电机 ID 查询 |
| `MixSetMotorId.srv` | Mix 电机 ID 修改 |
| `MixSetZero.srv` | Mix 零点设置 |
| `ResetFault.srv` | 故障复位 |
| `SetMute.srv` | 静音设置 |
| `SetRobotMode.srv` | 机器人模式设置 |
| `SetVolume.srv` | 音量设置 |
| `Speak.srv` | 语音服务接口定义，当前 `robot_audio_node` 未创建该服务 |
| `SwitchRlModel.srv` | 切换 RL 模型 |
| `TienkungModifyID.srv` | Tienkung 电机 ID 修改 |
| `TienkungQueryID.srv` | Tienkung 电机 ID 查询 |
| `TienkungQueryStatus.srv` | Tienkung 电机状态查询 |
| `TienkungResetID.srv` | Tienkung 电机 ID 复位 |
| `TienkungSetZero.srv` | Tienkung 电机零点设置 |

## 兼容性约束

- `.msg`、`.srv` 字段名、类型、常量和值属于公共接口，修改前需要同步更新所有调用方和 README。
- 新增接口后，应同步更新本 README、相关节点 README、launch/YAML 示例和测试。
- 删除或重命名接口属于破坏性变化，需要明确迁移方案。
- `tool_control/` 下的灵巧手消息仍是公共接口，字段语义保持不变。

`FaultReport.code` 和 `FaultInfo.code` 固定使用七位 `SSSNNNN` 字符串（例如
`A410001`）。`FaultInfo.system_name` 由前三位查询系统分类表获得，`level` 固定使用
`LOW`、`MEDIUM` 或 `HIGH`。`FaultReportArray.reporter_id` 仅用于隔离不同节点快照，
不作为面向用户的故障分类。

`FaultReport.occurred_at` 保存当前一轮故障首次发生的系统时间戳；同一故障持续期间
更新 detail 不改变该时间。`FaultInfo.occurred_at` 是 monitor 转换后的东八区可读时间，
固定格式为 `YYYY-MM-DD HH:MM:SS.mmm`。`FaultInfoArray.header.stamp` 仍表示快照发布时间。

## 构建

```bash
colcon build --packages-select robot_interfaces
```
