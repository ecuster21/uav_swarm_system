# UAV Swarm System

本项目用于实现基于 ROS2 + PX4 的无人机集群飞行系统。

当前固定环境：

- WSL2 Ubuntu 22.04
- ROS2 Humble
- PX4 v1.14.4
- Gazebo Classic 11.10.2
- MicroXRCEAgent 2.4.1
- QGroundControl on Windows host

第一阶段目标：

1. PX4 多机 SITL 仿真。
2. ROS2 多机状态管理。
3. leader-follower 编队控制。
4. 后续迁移到 RK3588 伴随计算机。
