# AGENTS.md

## 项目目标

本项目目标是实现一个面向无人机集群飞行的 ROS2 + PX4 系统，支持：

1. PX4 多机 SITL 仿真。
2. ROS2 多机状态管理。
3. leader-follower 编队控制。
4. 任务分配。
5. 区域覆盖搜索。
6. 后续部署到 RK3588 伴随计算机。
7. 后续接入目标检测、雷达感知、地面站和大模型任务规划。

核心原则：

- PX4 负责飞控、姿态控制、位置控制、failsafe 和底层飞行安全。
- RK3588 伴随计算机负责 ROS2、感知、任务分配、编队控制、通信和日志。
- 所有真实飞行相关功能必须先在 SITL 中验证。

---

## 当前固定开发环境

请严格按照以下环境设计和实现，不要自动升级或替换版本：

- 主项目路径：`/home/jie/uav_swarm_system`
- PX4 源码路径：`/home/jie/PX4-Autopilot`
- PX4 版本：`v1.14.4`
- Micro-XRCE-DDS-Agent 源码路径：`/home/jie/Micro-XRCE-DDS-Agent`
- MicroXRCEAgent 版本：`2.4.1`
- ROS2 版本：`Humble`
- 系统环境：`WSL2 Ubuntu 22.04`
- Gazebo：`Gazebo Classic 11.10.2`
- QGroundControl：运行在 Windows 主机上，不在 WSL2 内部

---

## 版本约束

1. 不要使用 PX4 main 分支。
2. 不要切换到 PX4 v1.15 或 v1.16。
3. 不要切换到 ROS2 Jazzy。
4. 不要切换到 Gazebo Garden、Harmonic 或 Ignition。
5. 不要升级 MicroXRCEAgent 到 3.x。
6. 不要引入 ROS1 作为主工程运行依赖。
7. 如果参考项目使用 ROS1，只能学习其架构和算法，不允许直接迁移 ROS1 运行方式。
8. 所有代码优先兼容 PX4 v1.14.4 + ROS2 Humble + Gazebo Classic 11.10.2 + MicroXRCEAgent 2.4.1。

---

## WSL2 / Windows 网络约束

当前运行方式：

- PX4 SITL 在 WSL2 Ubuntu 22.04 中运行。
- ROS2 节点在 WSL2 Ubuntu 22.04 中运行。
- MicroXRCEAgent 在 WSL2 Ubuntu 22.04 中运行。
- QGroundControl 在 Windows 主机上运行。

因此：

1. 所有 MAVLink / DDS / UDP 端口配置必须考虑 WSL2 和 Windows 主机之间的通信。
2. 不要假设 QGC 与 PX4 在同一个 Linux 网络命名空间内。
3. 不要默认 QGC 运行在 WSL2 中。
4. 如果需要 QGC 连接 PX4 SITL，需要明确说明 UDP 端口、广播地址或转发方式。
5. 如果需要可视化，优先考虑 headless Gazebo 或明确说明 GUI 依赖。
6. 真机部署时，WSL2 环境和 RK3588 环境不同，部署脚本要单独设计。

---

## ROS1 策略

本项目主线禁止依赖 ROS1。

禁止：

- roscore
- rospy
- roscpp
- catkin_make
- ROS1 launch 文件
- ROS1 mavros 作为主运行依赖

允许：

- 阅读 ROS1 项目作为参考。
- 借鉴 ROS1 项目的算法思想、话题设计、系统架构。
- 将有价值的思想重新实现为 ROS2 节点。

---

## 参考项目使用规则

`references/` 目录下的项目只作为学习和参考，不作为主工程直接依赖。

重点学习：

1. `navigation2`
   - 学习 ROS2 工程架构、行为树、插件化、生命周期管理。
   - 不作为无人机主导航系统。

2. `mavsdk_drone_show`
   - 学习 MAVSDK 多无人机控制、任务下发、地面站设计、多机任务组织。
   - 可作为 MAVSDK 后端参考。

3. `PythonRobotics`
   - 学习路径规划、覆盖搜索、控制算法、算法原型。
   - 可参考算法思想，不要直接大段复制。

4. `swarmSim`
   - 学习多机器人/多无人机接口设计。
   - 它是 ROS1 项目，不作为主线。

5. `PX4_Swarm_Controller`
   - 重点学习 ROS2 + PX4 多机仿真、Offboard、leader-follower、邻居拓扑、编队控制。
   - 可作为第一阶段仿真架构的重要参考。

6. `aerial-autonomy-stack`
   - 重点学习 ROS2 + PX4/ArduPilot + Gazebo + 感知 + 多机通信 + 部署的全栈架构。
   - 注意它偏 Jetson/Orin，不要直接照搬 NVIDIA/TensorRT/DeepStream 依赖到 RK3588。

---

## 外部依赖使用规则

不要把以下外部项目复制进主项目：

- `/home/jie/PX4-Autopilot`
- `/home/jie/Micro-XRCE-DDS-Agent`

如果需要调用它们，请通过环境变量或配置路径引用：

- `PX4_DIR=/home/jie/PX4-Autopilot`
- `MICRO_XRCE_DDS_AGENT_DIR=/home/jie/Micro-XRCE-DDS-Agent`

---

## 推荐目录结构

```text
uav_swarm_system/
├── AGENTS.md
├── README.md
├── config/
│   └── env.yaml
├── docs/
│   ├── environment.md
│   ├── reference_analysis.md
│   ├── architecture.md
│   ├── roadmap.md
│   ├── px4_sitl.md
│   └── wsl2_network.md
├── references/
│   ├── navigation2/
│   ├── mavsdk_drone_show/
│   ├── PythonRobotics/
│   ├── swarmSim/
│   ├── PX4_Swarm_Controller/
│   └── aerial-autonomy-stack/
├── ros2_ws/
│   └── src/
│       ├── swarm_msgs/
│       ├── px4_bridge/
│       ├── swarm_manager/
│       ├── formation_controller/
│       ├── task_allocator/
│       ├── coverage_planner/
│       ├── collision_avoidance/
│       └── mission_manager/
├── simulation/
│   ├── launch/
│   ├── worlds/
│   └── models/
├── scripts/
│   ├── setup_env.sh
│   ├── start_px4_single_sitl.sh
│   ├── start_px4_multi_sitl.sh
│   └── start_micro_xrce_agent.sh
├── onboard/
│   ├── rk3588_setup/
│   └── systemd/
└── ground_station/
