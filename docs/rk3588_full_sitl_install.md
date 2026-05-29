# RK3588 Full SITL Install

本文档用于在 RK3588 开发板上安装“全套仿真验收环境”：

- ROS2 Humble
- Micro-XRCE-DDS-Agent 2.4.1
- PX4-Autopilot v1.14.4
- Gazebo Classic 11
- `px4_msgs` `release/1.14`
- 本项目 `ros2_ws`

这套环境用于板端 SITL、领导验收和算法调试。真实飞行时不要运行 Gazebo 或 PX4 SITL；真实飞行模式只运行 ROS2、MicroXRCEAgent、任务/编队/感知/日志节点，PX4 固件运行在飞控硬件上。

## 给开发板 Codex 的执行指令

在 RK3588 上打开终端，确认本项目位于：

```bash
/home/jie/uav_swarm_system
```

然后执行：

```bash
cd /home/jie/uav_swarm_system
chmod +x scripts/rk3588_install_full_sitl.sh
./scripts/rk3588_install_full_sitl.sh
```

如果只安装后续真机飞行所需的轻量环境，不安装 PX4 SITL 和 Gazebo：

```bash
cd /home/jie/uav_swarm_system
./scripts/rk3588_install_full_sitl.sh flight-only
```

## 目标系统

推荐：

```text
硬件：RK3588 / RK3588S
系统：Ubuntu 22.04 arm64
ROS2：Humble
PX4：v1.14.4
Gazebo：Gazebo Classic 11
MicroXRCEAgent：2.4.1
QGroundControl：Windows 电脑上运行，不在 RK3588 上运行
```

不要切换到：

```text
PX4 main / v1.15 / v1.16
ROS2 Jazzy
Gazebo Garden / Harmonic / Ignition
MicroXRCEAgent 3.x
ROS1 / roscore / catkin_make
```

## 安装后检查

新开一个终端，执行：

```bash
cd /home/jie/uav_swarm_system
source scripts/setup_env.sh

ros2 --version
MicroXRCEAgent --version
gazebo --version

cd /home/jie/PX4-Autopilot
git describe --tags --exact-match
```

期望：

```text
ROS2 为 Humble
MicroXRCEAgent 为 2.4.1
PX4 tag 为 v1.14.4
Gazebo 为 Classic 11.x
```

## 单机 SITL 冒烟测试

终端 1：

```bash
cd /home/jie/uav_swarm_system
source scripts/setup_env.sh
./scripts/start_micro_xrce_agent.sh
```

终端 2：

```bash
cd /home/jie/uav_swarm_system
source scripts/setup_env.sh
HEADLESS=1 ./scripts/start_px4_single_sitl.sh
```

终端 3：

```bash
cd /home/jie/uav_swarm_system
source scripts/setup_env.sh
ros2 topic list | grep fmu
```

如果 `HEADLESS=1` 仍然尝试打开 GUI，说明当前 PX4/Gazebo 启动方式需要单独调整为 headless。先把单机跑通，再扩展多机。

## 多机 SITL 冒烟测试

先从 3 架开始，不要一上来跑 40 架：

```bash
cd /home/jie/uav_swarm_system
source scripts/setup_env.sh
./scripts/start_px4_multi_sitl.sh 3 iris
```

另一个终端启动 ROS2 控制节点：

```bash
cd /home/jie/uav_swarm_system
source scripts/setup_env.sh
ros2 launch formation_controller swarm_px4_uxrce.launch.py formation_type:=triangle vehicle_count:=3
```

起飞前先 dry run：

```bash
./scripts/swarm_arm_takeoff.sh --dry-run --count 3
```

确认目标无人机正确后再起飞：

```bash
./scripts/swarm_arm_takeoff.sh --count 3
```

降落：

```bash
./scripts/swarm_land_all.sh --count 3
```

## Windows QGroundControl 连接 RK3588 SITL

运行方式：

```text
RK3588：PX4 SITL + Gazebo + ROS2 + MicroXRCEAgent
Windows：QGroundControl
```

网络要求：

1. RK3588 和 Windows 在同一个局域网。
2. Windows 防火墙允许 QGroundControl 使用 UDP。
3. QGroundControl 监听 UDP `14550`。
4. 如果 QGC 没有自动发现，先确认 RK3588 能 ping 通 Windows，Windows 也能 ping 通 RK3588。

常见端口：

```text
QGC MAVLink UDP：14550
PX4 SITL MAVLink instance ports：14540 起
MicroXRCEAgent UDP：8888
```

如果 QGC 仍然无法连接，需要根据现场网络单独配置 PX4 MAVLink remote host 或做 UDP 转发。不要默认把 QGC 当成运行在 RK3588 本机。

## 真机飞行模式

真机飞行时 RK3588 只启动：

```bash
cd /home/jie/uav_swarm_system
source scripts/setup_env.sh
MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600
```

或者 UDP：

```bash
MicroXRCEAgent udp4 -p 8888
```

真机飞行时不要启动：

```text
Gazebo
PX4 SITL
jmavsim
任何仿真 world
```

## 风险记录

RK3588 可以安装全套环境，但需要明确这些风险：

1. Gazebo Classic 在 ARM64 板端的 GUI 和 OpenGL 兼容性不如 x86 PC 稳定。
2. PX4 SITL + Gazebo 会占用较多 CPU，可能影响 ROS2 控制节点实时性。
3. 多机 SITL 应从 1 架、3 架逐步增加，不要直接跑大规模集群。
4. 板端全套环境只用于仿真和验收，不作为真实飞行运行架构。
