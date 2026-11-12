# 部署与启动

## 单机开发（默认 A 队 3 台）

```bash
cd /path/to/robocup_sim
./build.sh
./robocup_sim doctor

# 窗口模式（默认）；无显示器用 --headless
./robocup_sim up --role all
./robocup_sim status

# 在裁判面板把状态切到 PLAYING：
#   http://127.0.0.1:8090/  （或 POST /api/command {"cmd":"patch","patch":{"state":3}}）
./robocup_sim standup
./robocup_sim down
```

- 日志：`runtime/<run_id>/logs/`，状态：`runtime/<run_id>/state.json`。
- 面板：裁判 UI `:8090`，match manager `:8092`。

## 多设备

1. 编辑 `inventory.yaml`：按实际 IP / 网卡增加主机并设置 `roles`，填写
   `zl_root` / `brain_ws`，把 `dds.mode` 改为 `unicast`。例如：

   ```yaml
   hosts:
     sim-host:
       address: 192.168.1.10
       dds_iface: auto
       roles: [sim, boundary, match]
     control-2:
       address: 192.168.1.11
       dds_iface: auto
       roles: [control]
       zl_root: ~/zl_robot
     brain-2:
       address: 192.168.1.21
       dds_iface: auto
       roles: [brain]
       brain_ws: ~/robocup_demo
   ```

   control 机的 `zl_root` 必须包含
   `robot_bringup/share/robot_bringup/launch/zl_robot_multi.launch.py`
   （`./robocup_sim doctor` 会检查）。该文件是 zl_robot 侧为多机仿真新增的
   launch，干净 checkout 里没有；升级/换机时先确认。
   `zl_root` / `brain_ws` 也可用环境变量 `ZL_ROOT` / `BRAIN_WS` 覆盖。
   本机实际部署时建议把 `inventory.yaml` 复制为 `inventory.local.yaml` 再改，
   该文件已 gitignore，不会被提交。
2. 编辑 `roster.yaml`（或本机私有的 `roster.local.yaml`）：每台机器人的
   `control_host` / `brain_host` 填上述主机名。未分配的机器人不会在该主机启动。
3. 每台设备本机构建（禁止拷贝 install）：
   - sim 机：`./build.sh`
   - control 机：构建 `zl_robot`
   - brain 机：构建 `robocup_demo`（含 `third_party/zl_interfaces`）
4. 各设备按角色启动：

```bash
# sim 机
./robocup_sim up --role sim --host sim-host
# control 机
./robocup_sim up --role control --host control-2
# brain 机
./robocup_sim up --role brain --host brain-2
```

- 同一台设备可承载多个角色/多台机器人；`--role all` 仅用于单机联调（忽略 host 分配）。
- 每台设备各自渲染 `runtime/<run_id>/dds/dds.xml` 与 `bridge/domain_bridge.yaml`。
- `unicast` 下 Peers 由 inventory 中的 `address` 自动渲染，检查
  `runtime/<run_id>/dds/dds.xml` 确认包含各主机 IP。
- 时钟：各设备 NTP 同步；UDP 3838/3939/3940 与 HTTP 8090/8092 按需放行。

## 端口

| 端口 | 用途 |
|---|---|
| 3838/udp | referee_ui → game_controller |
| 3939/udp | brain 心跳 → referee_ui robot_watch |
| 3940/udp | 实机位姿上报（仿真不使用） |
| 8090/tcp | referee_ui Web |
| 8092/tcp | match manager Web |

## 排障

- 重复 `up` 会复用当前存活 run，已运行的角色跳过；本机同一时刻只允许一个 run，
  `up --run-id` 指向其它 run 会被拒绝。
- `./robocup_sim down` 默认停止本机所有存活 run；`down --run-id` 只停指定 run。
- `./robocup_sim status`：列出所有存活 run 及其进程。
- `./robocup_sim doctor`：依赖、install、资产、roster/inventory。
- 域 10：`ROS_DOMAIN_ID=10 ros2 topic hz /sim/ball_pose`。
- 域 100：`ROS_DOMAIN_ID=100 ros2 topic echo --once /robot_01/state`。
- 机器人瘫倒：确认 GC 为 PLAYING 后重跑 `./robocup_sim standup`。
- 6 台同机失稳：减少到 3 台，或把 control/brain 分到其它设备。
