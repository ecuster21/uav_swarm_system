# UAV Swarm System

面向无人机集群飞行的 `ROS2 + PX4` 工程。当前主线是三机 PX4 SITL 仿真、ROS2 状态管理、leader-follower 编队控制，后续部署目标是 RK3588 伴随计算机。

## 当前状态

已完成：

- ROS2 工作空间基础框架。
- 三机 mock 仿真链路。
- PX4 v1.14.4 多机 SITL 接入。
- uXRCE-DDS / `px4_msgs` C++ bridge。
- C++ leader-follower 编队控制器。
- `triangle`、`line`、`column` 三种队形。

当前推荐验证路径：

- 首选：`px4_bridge_uxrce`，C++ uXRCE-DDS / Offboard 主线。
- 保留：`px4_bridge` 的 mock 和 Python MAVSDK backend，仅用于低频验证和对照，不作为长期高频飞控核心。

## 固定环境

不要自动升级或切换以下版本：

| 项目 | 固定值 |
|---|---|
| 主项目 | `/home/jie/uav_swarm_system` |
| PX4 | `/home/jie/PX4-Autopilot`, `v1.14.4` |
| ROS2 | Humble |
| Gazebo | Gazebo Classic `11.10.2` |
| MicroXRCEAgent | `/home/jie/Micro-XRCE-DDS-Agent`, `2.4.1` |
| 系统 | WSL2 Ubuntu 22.04 |
| QGroundControl | Windows 主机 |

约束：

- 不使用 ROS1 作为主工程运行依赖。
- 不切换 PX4 main、PX4 v1.15/v1.16、ROS2 Jazzy 或新 Gazebo。
- 不把 PX4、MicroXRCEAgent、`px4_msgs` 复制进本项目。
- 所有真实飞行相关功能必须先在 SITL 验证。

## 目录结构

```text
uav_swarm_system/
├── config/
│   ├── swarm.yaml          # 三机、namespace、system_id、backend 参数
│   ├── formations.yaml     # 队形 offset、控制频率、安全限制
│   └── waypoints.yaml      # leader 航点
├── docs/                   # 架构、路线图、参考项目分析
├── ros2_ws/src/
│   ├── swarm_msgs          # DroneState / SwarmState / FormationTarget
│   ├── px4_bridge          # Python mock / MAVSDK 验证层
│   ├── px4_bridge_uxrce    # C++ PX4 uXRCE-DDS bridge
│   ├── swarm_manager       # Python 低频集群状态汇总
│   └── formation_controller # C++ leader-follower 编队控制
└── scripts/
    ├── setup_env.sh
    ├── start_micro_xrce_agent.sh
    ├── start_px4_multi_sitl.sh
    ├── stop_sitl_stack.sh
    ├── swarm_arm_takeoff.sh
    ├── swarm_land_all.sh
    └── swarm_status_once.sh
```

## 构建

每个新终端建议先使用项目环境脚本，避免 Anaconda 的 `python3` 干扰 ROS2 Humble：

```bash
cd /home/jie/uav_swarm_system
source scripts/setup_env.sh
```

构建工作空间：

```bash
cd /home/jie/uav_swarm_system/ros2_ws
colcon build --cmake-args \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DPYTHON_INCLUDE_DIR=/usr/include/python3.10 \
  -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.10.so
```

构建后重新 source：

```bash
cd /home/jie/uav_swarm_system
source scripts/setup_env.sh
```

## 快速运行 Mock

不需要 PX4、Gazebo 或 MicroXRCEAgent：

```bash
cd /home/jie/uav_swarm_system
source scripts/setup_env.sh
ros2 launch formation_controller swarm_mock.launch.py formation_type:=triangle
```

可选队形：

```bash
formation_type:=triangle
formation_type:=line
formation_type:=column
```

## 运行 PX4 SITL uXRCE-DDS

这是当前推荐主线。建议三个终端分别运行。

关键前提：

- 每个 ROS2 终端都必须 `source scripts/setup_env.sh`，不要只 source `/opt/ros/humble/setup.bash` 和 `ros2_ws/install/setup.bash`。uXRCE bridge 还需要 `/home/jie/px4_ros_com_ws/install` 中的 `px4_msgs` 运行库。
- `ros2 launch formation_controller swarm_px4_uxrce.launch.py` 只启动状态桥接和编队目标发布，不会自动 arm/takeoff。飞机真正运动前必须显式执行 `./scripts/swarm_arm_takeoff.sh`。

终端 1：MicroXRCEAgent

```bash
cd /home/jie/uav_swarm_system
source scripts/setup_env.sh
./scripts/start_micro_xrce_agent.sh
```

终端 2：PX4 多机 SITL

```bash
cd /home/jie/uav_swarm_system
./scripts/start_px4_multi_sitl.sh 3 iris
```

终端 3：ROS2 控制节点

```bash
cd /home/jie/uav_swarm_system
source scripts/setup_env.sh
ros2 launch formation_controller swarm_px4_uxrce.launch.py formation_type:=triangle
```

起飞：

```bash
cd /home/jie/uav_swarm_system
source scripts/setup_env.sh
./scripts/swarm_arm_takeoff.sh
```

查看状态：

```bash
./scripts/swarm_status_once.sh
```

降落：

```bash
./scripts/swarm_land_all.sh
```

注意：

- 不要同时启动多套 `swarm_mock.launch.py`、`swarm_px4_uxrce.launch.py` 或 `swarm_px4_sitl.launch.py`，否则同名节点会互相干扰。
- `swarm_land_all.sh` 会先停止 autonomous `formation_controller`，再发布 inactive target，最后调用 `/land`。
- 如果 Gazebo 或 PX4 端口残留，先运行 `./scripts/stop_sitl_stack.sh`。

## MAVSDK Backend

MAVSDK backend 仍保留，但只作为 Python 低频验证层：

```bash
cd /home/jie/uav_swarm_system
source scripts/setup_env.sh
ros2 launch formation_controller swarm_px4_sitl.launch.py backend_type:=mavsdk formation_type:=triangle
```

如果使用 MAVSDK，需要系统 Python 能导入 `mavsdk`：

```bash
/usr/bin/python3 -m pip install --user mavsdk
```

长期真实 PX4 通信和 Offboard 控制优先走 C++ `px4_bridge_uxrce`。

## 核心话题

| Topic | Type | 说明 |
|---|---|---|
| `/uav_N/state` | `swarm_msgs/msg/DroneState` | 单机状态，N 为 1/2/3 |
| `/swarm/state` | `swarm_msgs/msg/SwarmState` | 集群状态汇总 |
| `/uav_N/formation_target` | `swarm_msgs/msg/FormationTarget` | 编队控制目标 |
| `/px4_N/fmu/out/*` | `px4_msgs/msg/*` | PX4 uXRCE-DDS 输出 |
| `/px4_N/fmu/in/*` | `px4_msgs/msg/*` | PX4 uXRCE-DDS 输入 |

常用 service：

```bash
ros2 service call /uav_1/arm std_srvs/srv/Trigger {}
ros2 service call /uav_1/takeoff std_srvs/srv/Trigger {}
ros2 service call /uav_1/land std_srvs/srv/Trigger {}
ros2 service call /uav_1/hold std_srvs/srv/Trigger {}
ros2 service call /uav_1/rtl std_srvs/srv/Trigger {}
```

## 编队控制

`formation_controller` 是 C++ `rclcpp` 节点。

行为：

- `uav_1` 是 leader。
- `uav_2`、`uav_3` 是 follower。
- leader 按 `config/waypoints.yaml` 中的航点循环飞行。
- follower 根据 leader 位置和 `config/formations.yaml` 中的 offset 生成目标点。
- 当前 offset 是固定 `local_enu` 世界坐标偏移，暂不随 leader yaw 旋转。

安全限制：

- 最大速度限制。
- 最大/最小高度限制。
- 最小机间距检查。
- leader 暂时丢失时，健康飞机保持当前位置 active hold setpoint。
- 本机 PX4 状态丢失或不健康时，停止发送 active target。

坐标系：

- 上层 ROS2 编队控制统一使用 `local_enu`：`x=east`、`y=north`、`z=up`。
- PX4 本地位置使用 `local_ned`：`x=north`、`y=east`、`z=down`。
- bridge 层负责转换：`ENU.x=NED.y`，`ENU.y=NED.x`，`ENU.z=-NED.z`。

## 配置文件

`config/swarm.yaml`：

- 无人机数量、namespace、role。
- PX4 `system_id`。
- MAVSDK UDP / server port。
- uXRCE-DDS topic prefix。
- 初始 ENU 锚点。

`config/formations.yaml`：

- `triangle`、`line`、`column` offset。
- `control_rate_hz`，当前默认 `2.0`。
- 高度、速度、最小间距安全限制。

`config/waypoints.yaml`：

- leader 航点。
- 坐标系固定为 `local_enu`。
- 当前默认方形航线，高度约 `2m`。

## QGroundControl

QGroundControl 运行在 Windows 主机，不在 WSL2 内部。PX4 SITL、ROS2 和 MicroXRCEAgent 运行在 WSL2，因此 QGC 连接需要考虑 WSL2 与 Windows 主机之间的 UDP 通信。

本文件不硬编码 Windows 主机 IP。若 QGC 无法发现飞机，需要单独确认：

- Windows 防火墙。
- WSL2 网络地址。
- PX4 MAVLink GCS 端口。
- 是否需要 UDP 转发。

## 开发规则

- `swarm_manager` 这类低频状态管理可用 Python。
- mock、MAVSDK 快速验证可用 Python。
- uXRCE-DDS、`px4_msgs`、Offboard setpoint、编队控制核心、安全监控优先 C++。
- 当前 `formation_controller` 已迁移为 C++，不要再回退到 Python 编队核心。

更多背景见：

- `AGENTS.md`
- `docs/architecture.md`
- `docs/roadmap.md`
- `docs/reference_analysis.md`
