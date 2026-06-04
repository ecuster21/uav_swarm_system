# UAV Swarm System

面向无人机集群飞行的 `ROS2 + PX4` 工程。当前主线是 RK3588 开发板上的 PX4 多机 SITL、MicroXRCE-DDS、ROS2 状态管理和 leader-follower 编队控制。

当前已验证：

- RK3588 / Ubuntu 22.04.4 arm64
- PX4 Autopilot v1.14.4
- ROS2 Humble
- Gazebo Classic 11.10.2 headless SITL
- Micro-XRCE-DDS-Agent 2.4.1
- 单板 3 架 `iris` PX4 SITL OFFBOARD 编队
- 两块 RK3588 分布式运行 PX4 SITL，并统一接入 Windows QGroundControl

详细原理和调试记录放在 `docs/`，README 只保留日常最常用入口。

## 固定环境

不要自动升级或切换以下版本。

| 项目 | 固定值 |
|---|---|
| 主项目 | `/home/jie/uav_swarm_system` |
| PX4 | `/home/jie/PX4-Autopilot`, `v1.14.4` |
| ROS2 | Humble |
| Gazebo | Gazebo Classic `11.10.2` |
| MicroXRCEAgent | `/usr/local/bin/MicroXRCEAgent`, `2.4.1` |
| QGroundControl | Windows 主机运行 |
| Windows GCS IP | `192.168.1.20` |
| RK3588 A | `192.168.1.40` |
| RK3588 B | `192.168.1.41` |

约束：

- 不切换 PX4 main、PX4 v1.15/v1.16、ROS2 Jazzy 或 Gazebo Garden/Harmonic/Ignition。
- 主线不依赖 ROS1、`roscore`、`catkin_make` 或 ROS1 MAVROS。
- PX4 负责飞控、姿态/位置控制、failsafe 和底层飞行安全。
- RK3588 侧负责 ROS2、编队、任务、通信、感知和日志。
- 所有真实飞行相关功能必须先在 SITL 验证。

## 目录概览

```text
uav_swarm_system/
├── config/
│   ├── swarm.yaml
│   ├── formations.yaml
│   └── waypoints.yaml
├── docs/
├── ros2_ws/src/
│   ├── swarm_msgs
│   ├── px4_msgs
│   ├── px4_bridge              # mock
│   ├── px4_bridge_uxrce        # real PX4 uXRCE-DDS bridge
│   ├── swarm_manager
│   ├── formation_controller
│   └── swarm_bringup
└── scripts/
    ├── setup_env.sh
    ├── start_micro_xrce_agent.sh
    ├── start_px4_multi_sitl.sh
    ├── stop_sitl_stack.sh
    ├── swarm_arm_takeoff.sh
    ├── swarm_land_all.sh
    └── swarm_status_once.sh
```

## 环境加载

每个新终端先执行：

```bash
cd /home/jie/uav_swarm_system
source scripts/setup_env.sh
```

修改 ROS2 代码后重新构建：

```bash
cd /home/jie/uav_swarm_system/ros2_ws
colcon build --cmake-args \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DPYTHON_INCLUDE_DIR=/usr/include/python3.10 \
  -DPYTHON_LIBRARY=/usr/lib/$(dpkg-architecture -qDEB_HOST_MULTIARCH)/libpython3.10.so

cd /home/jie/uav_swarm_system
source scripts/setup_env.sh
```

## 快速 Mock 验证

不需要 PX4、Gazebo 或 MicroXRCEAgent：

```bash
cd /home/jie/uav_swarm_system
source scripts/setup_env.sh
ros2 launch swarm_bringup swarm_mock.launch.py formation_type:=triangle
```

可选：

```text
formation_type:=triangle
formation_type:=line
formation_type:=column
```

## 单板 PX4 SITL

建议先跑 3 架，不要一开始直接跑大规模压力测试。

终端 1：MicroXRCEAgent

```bash
cd /home/jie/uav_swarm_system
./scripts/start_micro_xrce_agent.sh
```

终端 2：PX4/Gazebo

```bash
cd /home/jie/uav_swarm_system
./scripts/start_px4_multi_sitl.sh 3 iris
```

默认 `HEADLESS=1`，只启动 `gzserver`，不启动 `gzclient`。

终端 3：ROS2 bridge + swarm nodes

```bash
cd /home/jie/uav_swarm_system
source scripts/setup_env.sh
ros2 launch swarm_bringup swarm_px4_uxrce.launch.py \
  formation_type:=triangle vehicle_count:=3
```

起飞：

```bash
cd /home/jie/uav_swarm_system
source scripts/setup_env.sh
./scripts/swarm_arm_takeoff.sh --count 3
```

查看状态：

```bash
./scripts/swarm_status_once.sh
```

降落和停止：

```bash
./scripts/swarm_land_all.sh --count 3
./scripts/stop_sitl_stack.sh
```

## 两块板分布式 SITL

核心原则：

```text
每架飞机一份 /uav_N/px4_bridge
整个集群一份 /swarm/manager
整个集群一份 /swarm/formation_controller
```

固定映射：

```text
板子   IP             PX4_INSTANCE_START   ROS2 instance_start   ROS2 ID   PX4话题   QGC身份
A      192.168.1.40   1                    1                     uav_1    px4_1    MAV_SYS_ID 2
B      192.168.1.41   2                    2                     uav_2    px4_2    MAV_SYS_ID 3
```

两块板都先启动本机 MicroXRCEAgent：

```bash
cd /home/jie/uav_swarm_system
./scripts/start_micro_xrce_agent.sh
```

### A 板

PX4/Gazebo：

```bash
PX4_INSTANCE_START=1 PX4_SPAWN_X=0 PX4_SPAWN_Y=3 \
  ./scripts/start_px4_multi_sitl.sh 1 iris
```

bridge：

```bash
source scripts/setup_env.sh
ros2 launch swarm_bringup swarm_px4_uxrce.launch.py \
  instance_start:=1 vehicle_count:=1 \
  enable_swarm_nodes:=false \
  spawn_origin_x:=0 spawn_origin_y:=3
```

### B 板

PX4/Gazebo：

```bash
PX4_INSTANCE_START=2 PX4_SPAWN_X=30 PX4_SPAWN_Y=3 \
  ./scripts/start_px4_multi_sitl.sh 1 iris
```

bridge：

```bash
source scripts/setup_env.sh
ros2 launch swarm_bringup swarm_px4_uxrce.launch.py \
  instance_start:=2 vehicle_count:=1 \
  enable_swarm_nodes:=false \
  spawn_origin_x:=30 spawn_origin_y:=3
```

### 只在 A 板启动集群节点

```bash
source scripts/setup_env.sh
ros2 launch swarm_bringup swarm_px4_uxrce.launch.py \
  instance_start:=1 vehicle_count:=2 \
  enable_bridges:=false enable_swarm_nodes:=true \
  spawn_origin_x:=0 spawn_origin_y:=3 \
  spawn_spacing_x:=30 spawn_spacing_y:=0
```

起飞：

```bash
./scripts/swarm_arm_takeoff.sh uav_1 uav_2
```

降落：

```bash
./scripts/swarm_land_all.sh uav_1 uav_2
```

## 多机 spawn 对齐规则

`PX4_SPAWN_X/Y` 和 ROS2 launch 的 `spawn_origin_x/y` 必须一致。

多机时 step 也要一致：

```text
PX4_INSTANCE_START      == ROS2 instance_start
PX4 启动数量            == ROS2 vehicle_count
PX4_SPAWN_X/Y           == ROS2 spawn_origin_x/y
PX4_SPAWN_X/Y_STEP      == ROS2 spawn_spacing_x/y
```

例如 B 板从 instance 2 开始启动 3 架：

```bash
PX4_INSTANCE_START=2 \
PX4_SPAWN_X=20 PX4_SPAWN_Y=3 \
PX4_SPAWN_X_STEP=10 PX4_SPAWN_Y_STEP=0 \
  ./scripts/start_px4_multi_sitl.sh 3 iris
```

对应 bridge：

```bash
ros2 launch swarm_bringup swarm_px4_uxrce.launch.py \
  instance_start:=2 vehicle_count:=3 \
  enable_swarm_nodes:=false \
  spawn_origin_x:=20 spawn_origin_y:=3 \
  spawn_spacing_x:=10 spawn_spacing_y:=0
```

这会生成：

```text
uav_2 -> px4_2 -> initial/spawn x=20
uav_3 -> px4_3 -> initial/spawn x=30
uav_4 -> px4_4 -> initial/spawn x=40
```

## QGC 地图位置

如果要让 QGC 中的飞机出现在指定经纬高位置，设置 PX4 全球 home：

```bash
PX4_HOME_LAT=31.230400 PX4_HOME_LON=121.473700 PX4_HOME_ALT=5 \
PX4_INSTANCE_START=1 PX4_SPAWN_X=0 PX4_SPAWN_Y=3 \
  ./scripts/start_px4_multi_sitl.sh 1 iris
```

多块板要显示在同一片地图区域时，使用同一组：

```text
PX4_HOME_LAT
PX4_HOME_LON
PX4_HOME_ALT
```

再用不同 `PX4_SPAWN_X/Y` 拉开本地距离。

## 常用检查

节点：

```bash
ros2 node list | sort
```

分布式两板期望只有一份：

```text
/swarm/manager
/swarm/formation_controller
```

每架飞机各有一份：

```text
/uav_1/px4_bridge
/uav_2/px4_bridge
```

PX4 DDS 话题：

```bash
ros2 topic list | grep '/px4_'
```

状态和目标：

```bash
ros2 topic echo /swarm/state --once
ros2 topic echo /uav_1/state --once
ros2 topic echo /uav_2/state --once
ros2 topic echo /uav_1/formation_target --once
ros2 topic echo /uav_2/formation_target --once
```

如果目标是：

```text
active: false
source: formation_controller.hold:px4_state_unhealthy
```

优先检查是否重复启动了 `/swarm/manager` 或 `/swarm/formation_controller`，以及 `PX4_INSTANCE_START` 和 ROS2 `instance_start` 是否对齐。

## 核心话题和服务

| Topic/Service | 说明 |
|---|---|
| `/px4_N/fmu/out/*` | PX4 uXRCE-DDS 输出 |
| `/px4_N/fmu/in/*` | PX4 uXRCE-DDS 输入 |
| `/uav_N/state` | 单机状态 |
| `/swarm/state` | 集群状态 |
| `/uav_N/formation_target` | 编队目标 |
| `/uav_N/arm` | 解锁服务 |
| `/uav_N/takeoff` | 起飞服务 |
| `/uav_N/land` | 降落服务 |
| `/uav_N/hold` | hold/loiter 服务 |
| `/uav_N/rtl` | 返航服务 |

## 详细文档

优先看这几份：

- `docs/debug/daily_review_2026-06-03.md`
- `docs/debug/px4_ros2_dds_runtime_concepts.md`
- `docs/debug/waypoints_initial_position_and_rates.md`
- `docs/debug/control_flow_and_node_interfaces.md`
- `docs/debug/rk3588_distributed_sitl_debug_2026-06-02.md`

其他文档：

- `docs/rk3588_full_sitl_install.md`
- `docs/debug/rk3588_install_log_2026-05-31.md`
- `docs/architecture.md`
- `docs/roadmap.md`
- `docs/reference_analysis.md`
- `AGENTS.md`

## 清理

停止 PX4/Gazebo：

```bash
./scripts/stop_sitl_stack.sh
```

检查残留进程：

```bash
pgrep -af 'px4|gzserver|gzclient|gazebo|MicroXRCEAgent'
```

不要删除：

```text
/home/jie/PX4-Autopilot
```

PX4 SITL 可执行文件、Gazebo 插件、模型、world 和启动脚本仍在该目录中。
