# 架构

## 三方边界

```
域 10（仿真/运控）                        域 100（策略）
┌──────────────────────────────┐          ┌──────────────────────────┐
│ sim 主机                      │          │ brain 主机                │
│  sim_z01_std_robocup (1 进程) │          │  brain_node × N           │
│  robocup_sim_bridge (GT/感知) │          │                          │
│  match_manager (边裁/摆球)    │  bridge  │  game_controller (UDP 3838)│
│  domain_bridge 10↔100         │ ───────▶ │  referee_ui (HTTP :8090)  │
│ control 主机                  │          │                          │
│  zl_robot_multi (每台 5~6 节点)│          │                          │
└──────────────────────────────┘          └──────────────────────────┘
```

- 域 10：MuJoCo、zl 控制栈、GT/感知中继、match manager、domain_bridge 的 10 侧。
- 域 100：brain、game_controller、referee_ui 的 UDP 目标。
- 与实机一致：板子（zl，域 10）↔ NX（brain/GC，域 100）。
- domain_bridge 默认与 sim 同机；可拆到独立 boundary 主机。

## 角色与进程

| 角色 | 进程 | 说明 |
|---|---|---|
| sim | `sim_z01_std_robocup` ×1 | 6 台机器人、球场/球/球门；`/robot_0X/*` + `/sim/*` |
| sim | `sim_relay_node` ×1 | nav_all + `/sim/*` → 逐机 GT 与 perception/target_* |
| control | `zl_robot_multi.launch.py` ×1 | 每台 5 节点：state/router/control/rl_manager/external（odometry 可选） |
| brain | `brain_node` ×N | `sim:=true`、`robot_name`、`sim_profile` |
| boundary | `domain_bridge` ×1 | 域 10 ↔ 100 白名单 |
| match | GC ×1、match_manager ×1、referee_ui ×1 | UDP 3838、边裁/摆球、Web 面板 :8090 |

默认 A 队 3 台：1 sim + 3×5 zl 节点 + 3 brain。6 台：1 sim + 6×5 zl 节点 + 6 brain。

## 启动顺序

1. control（zl 控制栈）
2. sim（延迟数秒）
3. GT/感知中继（随 sim）
4. boundary
5. match（GC + 边裁 + 裁判 UI）
6. brain
7. `standup`（ready → reload → walk，等 alt≥0.5）

`robocup_sim up --role all` 自动执行 1~6；`standup` 单独执行。

## 机器人选择

- 仿真始终保留 6 台模型；`roster.yaml` 的 `enabled` / `--robots` 决定上场子集。
- 未上场机器人由补丁 `robot_active_mask` 冻结到场地下方，不发布状态/相机、不接收控制。
- 默认 A 队 3 台；显式 `--robots "robot_01 ... robot_06"` 为 6 台。
- 6 台同机在 8 核机器上算力不足（实测站立数秒后失稳）；推荐把 brain/control 分到多台设备，
  或使用 `--robots` 保持 3 台。
