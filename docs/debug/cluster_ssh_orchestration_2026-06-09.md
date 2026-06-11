# 2026-06-09 11 板 SSH 一键启动编排

本文记录 11 块 RK3588 开发板使用 SSH 一键启动 PX4 SITL、MicroXRCEAgent、ROS2 bridge 和集群控制节点的方案。

## 1. 目标

之前两块板调试时，每块板都要手动执行：

```bash
./scripts/start_micro_xrce_agent.sh
PX4_INSTANCE_START=... ./scripts/start_px4_multi_sitl.sh ...
ros2 launch swarm_bringup swarm_px4_uxrce.launch.py ...
```

当开发板数量增加到 11 块后，逐条命令启动效率太低，也容易把 `PX4_INSTANCE_START`、`instance_start`、`spawn_origin` 写错。

本次新增：

```text
config/cluster_boards.yaml
scripts/swarm_cluster.sh
```

在 A 板 `192.168.1.40` 上执行一个命令，即可通过 SSH 并发控制 `192.168.1.40` 到 `192.168.1.50`。

## 2. 固定规模

默认配置：

```text
11 块板
每板 4 架 PX4 SITL
总计 44 架
```

编号规则：

| IP | PX4_INSTANCE_START | UAV | PX4 topic | MAV_SYS_ID | spawn_x |
|---|---:|---|---|---|---:|
| `192.168.1.40` | 1 | `uav_1` ~ `uav_4` | `px4_1` ~ `px4_4` | 2 ~ 5 | 0 |
| `192.168.1.41` | 5 | `uav_5` ~ `uav_8` | `px4_5` ~ `px4_8` | 6 ~ 9 | 40 |
| `192.168.1.42` | 9 | `uav_9` ~ `uav_12` | `px4_9` ~ `px4_12` | 10 ~ 13 | 80 |
| `192.168.1.43` | 13 | `uav_13` ~ `uav_16` | `px4_13` ~ `px4_16` | 14 ~ 17 | 120 |
| `192.168.1.44` | 17 | `uav_17` ~ `uav_20` | `px4_17` ~ `px4_20` | 18 ~ 21 | 160 |
| `192.168.1.45` | 21 | `uav_21` ~ `uav_24` | `px4_21` ~ `px4_24` | 22 ~ 25 | 200 |
| `192.168.1.46` | 25 | `uav_25` ~ `uav_28` | `px4_25` ~ `px4_28` | 26 ~ 29 | 240 |
| `192.168.1.47` | 29 | `uav_29` ~ `uav_32` | `px4_29` ~ `px4_32` | 30 ~ 33 | 280 |
| `192.168.1.48` | 33 | `uav_33` ~ `uav_36` | `px4_33` ~ `px4_36` | 34 ~ 37 | 320 |
| `192.168.1.49` | 37 | `uav_37` ~ `uav_40` | `px4_37` ~ `px4_40` | 38 ~ 41 | 360 |
| `192.168.1.50` | 41 | `uav_41` ~ `uav_44` | `px4_41` ~ `px4_44` | 42 ~ 45 | 400 |

关键原则仍然不变：

```text
PX4_INSTANCE_START == ROS2 instance_start
PX4_SPAWN_X/Y == ROS2 spawn_origin_x/y
PX4_SPAWN_X/Y_STEP == ROS2 spawn_spacing_x/y
```

## 3. 使用方法

先看 dry-run：

```bash
cd /home/jie/uav_swarm_system
./scripts/swarm_cluster.sh dry-run
```

检查环境：

```bash
./scripts/swarm_cluster.sh check
```

先启动两块板验证：

```bash
./scripts/swarm_cluster.sh start --limit 2
./scripts/swarm_cluster.sh status --limit 2
```

完整启动 11 块板：

```bash
./scripts/swarm_cluster.sh start
./scripts/swarm_cluster.sh status
```

批量起飞：

```bash
./scripts/swarm_arm_takeoff.sh --count 44
```

停止：

```bash
./scripts/swarm_cluster.sh stop
```

## 4. 启动顺序

`start` 的固定顺序：

1. 读取 `config/cluster_boards.yaml` 并生成板卡映射。
2. SSH 检查每块板项目路径、PX4、ROS2、MicroXRCEAgent。
3. 并发停止旧的 PX4、Gazebo、MicroXRCEAgent、swarm launch。
4. 并发启动每块板本机 MicroXRCEAgent。
5. 并发启动每块板本机 PX4/Gazebo，默认每板 4 架。
6. 等待 `PX4_WAIT_SEC`，默认 25 秒。
7. 并发启动每块板本机 `px4_bridge_uxrce` launch。
8. 等待 `BRIDGE_WAIT_SEC`，默认 8 秒。
9. 只在 A 板启动一份 `/swarm/manager` 和 `/swarm/formation_controller`。

每块板只负责自己的 PX4 和 bridge：

```text
worker board:
  MicroXRCEAgent
  PX4 SITL / Gazebo
  /uav_N/px4_bridge
```

A 板额外负责全局集群节点：

```text
controller board:
  /swarm/manager
  /swarm/formation_controller
```

## 5. 每块板实际启动命令

PX4/Gazebo：

```bash
PX4_HOME_LAT=34.566096 PX4_HOME_LON=110.092301 PX4_HOME_ALT=350 \
PX4_INSTANCE_START=<instance_start> \
PX4_SPAWN_X=<spawn_x> PX4_SPAWN_Y=3 \
PX4_SPAWN_X_STEP=10 PX4_SPAWN_Y_STEP=0 \
HEADLESS=1 \
  ./scripts/start_px4_multi_sitl.sh 4 iris
```

bridge：

```bash
ros2 launch swarm_bringup swarm_px4_uxrce.launch.py \
  instance_start:=<instance_start> vehicle_count:=4 \
  enable_swarm_nodes:=false \
  spawn_origin_x:=<spawn_x> spawn_origin_y:=3 \
  spawn_spacing_x:=10 spawn_spacing_y:=0 \
  swarm_config_file:=/home/jie/uav_swarm_system/config/swarm.yaml \
  formations_config_file:=/home/jie/uav_swarm_system/config/formations.yaml \
  waypoints_config_file:=/home/jie/uav_swarm_system/config/waypoints.yaml
```

A 板 swarm 节点：

```bash
ros2 launch swarm_bringup swarm_px4_uxrce.launch.py \
  instance_start:=1 vehicle_count:=44 \
  enable_bridges:=false enable_swarm_nodes:=true \
  spawn_origin_x:=0 spawn_origin_y:=3 \
  spawn_spacing_x:=10 spawn_spacing_y:=0 \
  swarm_config_file:=/home/jie/uav_swarm_system/config/swarm.yaml \
  formations_config_file:=/home/jie/uav_swarm_system/config/formations.yaml \
  waypoints_config_file:=/home/jie/uav_swarm_system/config/waypoints.yaml
```

这里显式传入 `config/*.yaml`，避免再次出现“源码配置改了，但 launch 读 install 目录旧配置”的问题。

## 6. 日志

每块板本机日志目录：

```text
/home/jie/uav_swarm_system/logs/cluster/<run_id>/
```

主要文件：

| 文件 | 说明 |
|---|---|
| `micro_xrce_agent.log` | MicroXRCEAgent 输出 |
| `px4_sitl.log` | PX4/Gazebo 启动脚本输出 |
| `bridge.launch.log` | 本机 bridge launch 输出 |
| `swarm_nodes.launch.log` | A 板全局 swarm 节点输出 |
| `stop_before_start.log` | 启动前清理旧进程的输出 |

## 7. 常见问题

### SSH 失败

检查：

```bash
ssh jie@192.168.1.41 'hostname'
```

需要配置免密 SSH。脚本使用 `BatchMode=yes`，不会交互式等待密码。

### QGC 图标闪烁

优先看 dry-run 输出里的 `MAV_SYS_ID` 是否重复。

正确情况下 44 架应该是：

```text
MAV_SYS_ID 2 ~ 45
```

### ROS2 控制不到某块板

检查该板 bridge 是否启动：

```bash
./scripts/swarm_cluster.sh status --limit 2
```

在 A 板看 ROS2 节点：

```bash
source scripts/setup_env.sh
ros2 node list | grep px4_bridge
```

如果某块板的 `/uav_N/px4_bridge` 不存在，优先看对应板的：

```text
logs/cluster/<run_id>/bridge.launch.log
```

### 飞机位置整体偏移

检查同一块板上的参数是否对齐：

```text
PX4_SPAWN_X == spawn_origin_x
PX4_SPAWN_X_STEP == spawn_spacing_x
```

本脚本默认统一使用：

```text
spawn_y = 3
vehicle_spacing_x = 10
vehicle_spacing_y = 0
```

### 网络压力较高

44 架飞机同时向 Windows QGC 发送 MAVLink，会明显增加以太网接收流量。

需要时降低 PX4 的 QGC MAVLink stream 频率：

```text
/home/jie/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink
```

详见：

```text
docs/debug/daily_review_2026-06-04.md
```

## 8. 验收清单

两块板验证：

```bash
./scripts/swarm_cluster.sh dry-run --limit 2
./scripts/swarm_cluster.sh check --limit 2
./scripts/swarm_cluster.sh start --limit 2
./scripts/swarm_cluster.sh status --limit 2
```

预期：

```text
/uav_1/px4_bridge ~ /uav_8/px4_bridge
/swarm/manager 只有一份
/swarm/formation_controller 只有一份
```

完整 11 板验证：

```bash
./scripts/swarm_cluster.sh start
./scripts/swarm_cluster.sh status
./scripts/swarm_arm_takeoff.sh --count 44
```

预期：

```text
/uav_1/px4_bridge ~ /uav_44/px4_bridge
QGC 中 MAV_SYS_ID 2 ~ 45 不重复
/swarm/state 中 44 架飞机状态持续更新
```
