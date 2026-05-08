# 当前开发环境

## 固定版本

- 操作系统：WSL2 Ubuntu 22.04
- ROS2：Humble
- PX4：v1.14.4
- Gazebo：Gazebo Classic 11.10.2
- MicroXRCEAgent：2.4.1
- QGroundControl：运行在 Windows 主机上
- 主项目路径：`/home/jie/uav_swarm_system`
- PX4 源码路径：`/home/jie/PX4-Autopilot`
- Micro-XRCE-DDS-Agent 源码路径：`/home/jie/Micro-XRCE-DDS-Agent`

## 重要约束

1. 不要升级 PX4 到 main、v1.15、v1.16。
2. 不要切换 ROS2 Jazzy。
3. 不要切换 Gazebo Garden、Harmonic、Ignition。
4. 不要升级 MicroXRCEAgent 到 3.x。
5. 不要把 ROS1 作为主工程依赖。
6. 不要把 PX4-Autopilot 复制进主项目。
7. 不要把 Micro-XRCE-DDS-Agent 复制进主项目。
8. 主项目只保存我们自己的 ROS2 节点、配置、launch、脚本和文档。

## WSL2 网络说明

当前运行方式：

- PX4 SITL 在 WSL2 Ubuntu 22.04 中运行
- ROS2 节点在 WSL2 Ubuntu 22.04 中运行
- MicroXRCEAgent 在 WSL2 Ubuntu 22.04 中运行
- QGroundControl 在 Windows 主机上运行

因此，所有 MAVLink、DDS、UDP 端口设计都必须考虑 WSL2 和 Windows 主机之间的网络通信。
