# 话题契约

`<r>` = robot_01 … robot_06。

## 仿真原生（域 10）

| 话题 | 类型 | 方向 |
|---|---|---|
| `/<r>/control/hardware_commands` | robot_interfaces/JointCommands | zl → sim |
| `/<r>/driver/joint_states` | robot_interfaces/JointStates | sim → zl |
| `/<r>/driver/status` | robot_interfaces/DriverStatus | sim → zl |
| `/<r>/imu/nav_all` | yesense_interface/NavAll | sim → zl |
| `/<r>/camera/{color,depth}/...` | sensor_msgs | sim（默认关闭） |
| `/sim/ball_pose`、`/sim/ball_twist` | geometry_msgs | sim |
| `/sim/goal_team_a`、`/sim/goal_team_b` | geometry_msgs/Point | sim |
| `/sim/set_ball_pose`、`/sim/set_ball_twist` | geometry_msgs | match → sim |
| `/reload_robot_0N`、`/reload_all_robot`、`/reset_robot_0N`、`/reset_ball` | std_srvs/Trigger | 工具 → sim |

## zl 控制栈（域 10）

| 话题 | 类型 | 方向 |
|---|---|---|
| `/<r>/state` | robot_interfaces/RobotState | zl → brain |
| `/<r>/rl_manager/status` | robot_interfaces/RlManagerStatus | zl → brain |
| `/<r>/router/active_intent` | robot_interfaces/RobotIntent | zl → brain |
| `/<r>/odometry/odom` | nav_msgs/Odometry | zl（可选） |
| `/<r>/recovery_state` | std_msgs/UInt8MultiArray | zl（可选） |
| `/<r>/cmd_vel` | geometry_msgs/Twist | brain → zl |
| `/<r>/input/intent` | robot_interfaces/RobotIntent | brain → zl |
| `/<r>/external/head_control` | robot_interfaces/HeadControl | brain → zl |
| `/robot/head_to_base/<r>` | geometry_msgs/Pose | zl → brain |

## 中继与 GT（域 10）

| 话题 | 类型 | 说明 |
|---|---|---|
| `/soccer/ground_truth_pose/<r>` | geometry_msgs/Pose2D | A 队 180° 翻转；brain 直接消费 |
| `/soccer/ground_truth_ball/<r>` | geometry_msgs/Point | 同上 |
| `/<r>/perception/ball` | robot_interfaces/Ball | 基座系球 |
| `/<r>/perception/target_point` | geometry_msgs/Point | 基座系球 |
| `/<r>/perception/target_destination` | geometry_msgs/Point | 基座系进攻球门 |
| `/<r>/perception/target_goal_normal` | geometry_msgs/Vector3 | 基座系进攻方向 |

## 比赛（域 100，经桥到 10）

| 话题 | 类型 |
|---|---|
| `/soccer/game_controller` | game_controller_interface/GameControlData |
| `/soccer/brain_strategy/<r>` | std_msgs/String |

## 桥接白名单

- 10 → 100：state、joint_states、driver/status、nav_all、rl_manager/status、
  router/active_intent、odometry/odom、recovery_state、GT pose/ball。
- 100 → 10：cmd_vel、input/intent、external/head_control、brain_strategy、GC（可选回读）。
- 不桥接 `/sim/*`、`hardware_commands`、`perception/*` 等内部话题。
