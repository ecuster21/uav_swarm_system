# 2026-06-04 调试复盘

本文记录 2026-06-04 围绕 PX4 SITL、ROS2 集群控制、QGroundControl 网络占用和 MAVLink stream 做的现场排查。

当天主要解决两类问题：

- 修改飞行高度、速度后，仿真中没有生效。
- 两块开发板同时连接 Windows QGC 后，Windows 网卡接收流量较高。

## 1. 高度和速度修改没有生效

### 1.1 现象

修改了项目根目录下的配置：

```text
config/waypoints.yaml
config/formations.yaml
```

期望：

- leader 航点高度升高。
- formation_controller 发布目标时速度上限变大。

但仿真仍然表现为：

- 飞机高度还是接近旧的 10 m。
- 速度仍然接近旧的 2.5 m/s。

### 1.2 现场检查命令

先看当前运行进程：

```bash
ps -ef | rg 'swarm_px4_uxrce|formation_controller|swarm_manager|start_px4_multi|px4|MicroXRCE|gzserver'
```

当时看到：

```text
./scripts/start_px4_multi_sitl.sh 5 iris
px4 -i 1
px4 -i 2
px4 -i 3
px4 -i 4
px4 -i 5
ros2 launch swarm_bringup swarm_px4_uxrce.launch.py instance_start:=1 vehicle_count:=5 enable_swarm_nodes:=false ...
ros2 launch swarm_bringup swarm_px4_uxrce.launch.py instance_start:=1 vehicle_count:=15 enable_bridges:=false enable_swarm_nodes:=true ...
```

再看 ROS2 节点：

```bash
source /home/jie/uav_swarm_system/scripts/setup_env.sh
ros2 node list
```

关键节点：

```text
/swarm/manager
/swarm/formation_controller
/uav_1/px4_bridge
/uav_2/px4_bridge
...
```

继续确认 formation_controller 实际加载的配置文件：

```bash
source /home/jie/uav_swarm_system/scripts/setup_env.sh
ros2 param get /swarm/formation_controller waypoints_config_file
ros2 param get /swarm/formation_controller formations_config_file
ros2 param get /swarm/formation_controller swarm_config_file
```

结果：

```text
waypoints_config_file:
/home/jie/uav_swarm_system/ros2_ws/install/swarm_bringup/share/swarm_bringup/config/waypoints.yaml

formations_config_file:
/home/jie/uav_swarm_system/ros2_ws/install/swarm_bringup/share/swarm_bringup/config/formations.yaml

swarm_config_file:
/tmp/uav_swarm_runtime_xxxxx.yaml
```

这说明当前 controller 没有读取项目根目录的 `config/*.yaml`，而是读取了 install-space 里的配置。

### 1.3 源配置和 install 配置不一致

对比配置：

```bash
sed -n '1,80p' config/waypoints.yaml
sed -n '1,80p' ros2_ws/install/swarm_bringup/share/swarm_bringup/config/waypoints.yaml

sed -n '30,45p' config/formations.yaml
sed -n '30,45p' ros2_ws/install/swarm_bringup/share/swarm_bringup/config/formations.yaml
```

项目根目录配置已经是新值：

```yaml
leader_waypoints:
  points:
    - [0.0, 0.0, 80.0]
    - [100.0, 0.0, 80.0]
    - [100.0, 100.0, 80.0]
    - [0.0, 100.0, 80.0]

safety:
  max_speed_m_s: 5
  max_altitude_m: 150.0
```

install 目录配置仍是旧值：

```yaml
leader_waypoints:
  points:
    - [0.0, 0.0, 10.0]
    - [100.0, 0.0, 10.0]
    - [100.0, 100.0, 10.0]
    - [0.0, 100.0, 10.0]

safety:
  max_speed_m_s: 2.5
  max_altitude_m: 15.0
```

### 1.4 通过 topic 验证控制输出

查看 formation_controller 实际发布的目标：

```bash
source /home/jie/uav_swarm_system/scripts/setup_env.sh
ros2 topic echo /uav_1/formation_target --once
```

当时看到：

```text
position.z: 8.58
velocity.x: 2.49
source: formation_controller.leader_waypoints
```

再看状态：

```bash
ros2 topic echo /uav_1/state --once
```

当时看到：

```text
flight_mode: OFFBOARD
position.z: 8.57
velocity.x: 2.20
healthy: true
```

这说明 PX4、uXRCE-DDS、bridge、Offboard 链路是通的，问题不是飞机不听指令，而是 formation_controller 仍在使用旧的航点高度和旧的速度限制。

### 1.5 根因

根因有两个叠加：

1. `swarm_bringup` launch 默认配置路径来自 package share：

```text
ros2_ws/install/swarm_bringup/share/swarm_bringup/config/*.yaml
```

2. `formation_controller` 启动时只读一次 YAML：

```cpp
swarm_config_ = YAML::LoadFile(swarm_config_file);
formations_config_ = YAML::LoadFile(formations_config_file);
waypoints_config_ = YAML::LoadFile(waypoints_config_file);
```

运行中直接修改 YAML 不会热加载。

### 1.6 正确处理方式

如果只想快速让修改生效，不重建 ROS2，可以重启 swarm 节点，并显式指定项目根目录配置：

```bash
ros2 launch swarm_bringup swarm_px4_uxrce.launch.py \
  instance_start:=1 vehicle_count:=5 \
  enable_bridges:=false enable_swarm_nodes:=true \
  swarm_config_file:=/home/jie/uav_swarm_system/config/swarm.yaml \
  formations_config_file:=/home/jie/uav_swarm_system/config/formations.yaml \
  waypoints_config_file:=/home/jie/uav_swarm_system/config/waypoints.yaml \
  spawn_origin_x:=0 spawn_origin_y:=3 \
  spawn_spacing_x:=30 spawn_spacing_y:=0
```

如果希望默认 launch 也用更新后的 install 配置，需要重新构建 ROS2 工作区并重启 launch：

```bash
cd /home/jie/uav_swarm_system/ros2_ws
colcon build --symlink-install
source /home/jie/uav_swarm_system/scripts/setup_env.sh
```

注意：

```text
只改 yaml，不重启 formation_controller，不会生效。
```

## 2. QGC 网络占用高

### 2.1 现象

Windows 上 QGroundControl 连接两块开发板后，Windows 任务管理器中以太网接收大约：

```text
4.0 Mbps
```

需要确认：

- 是谁在发。
- 发给谁。
- 发的是什么数据。

### 2.2 先确认当前有几架 PX4 在发

检查进程：

```bash
ps -ef | rg 'start_px4_multi|px4 -i|gzserver|MicroXRCE|swarm_px4_uxrce'
```

当时 A 板上有：

```text
./scripts/start_px4_multi_sitl.sh 5 iris
px4 -i 1
px4 -i 2
px4 -i 3
px4 -i 4
px4 -i 5
```

也就是说单板不是 1 架，而是 5 个 PX4 实例都在运行。

### 2.3 检查 UDP 端口

查看 UDP socket：

```bash
ss -uapn | rg '145|185|8888|px4|gzserver|MicroXRCE|udp'
```

关键端口：

```text
0.0.0.0:18571 users:(("px4",...))
0.0.0.0:18572 users:(("px4",...))
0.0.0.0:18573 users:(("px4",...))
0.0.0.0:18574 users:(("px4",...))
0.0.0.0:18575 users:(("px4",...))
```

这些是 PX4 SITL 的 GCS MAVLink 本地端口，规律为：

```text
px4 instance 1 -> 18571
px4 instance 2 -> 18572
px4 instance 3 -> 18573
px4 instance 4 -> 18574
px4 instance 5 -> 18575
```

所有这些端口最终都向 Windows QGC 的 `14550` 发送 MAVLink。

### 2.4 检查网卡流量

板子上没有安装 `tcpdump`，所以用网卡计数估算：

```bash
ip -s link show eth0
sleep 3
ip -s link show eth0
```

当时 3 秒内 `TX bytes` 增加约 804 KB：

```text
804 KB * 8 / 3 ~= 2.1 Mbps
```

A 板单板约 2 Mbps，两块板同时向 Windows QGC 发，Windows 看到约 4 Mbps 属于合理量级。

### 2.5 查到 PX4 MAVLink 启动配置

搜索 PX4 MAVLink 启动脚本：

```bash
rg -n "mavlink|14550|18570|stream|rate|gcs" \
  /home/jie/PX4-Autopilot/ROMFS \
  /home/jie/PX4-Autopilot/Tools/simulation/gazebo-classic
```

关键文件：

```text
/home/jie/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink
```

查看内容：

```bash
nl -ba /home/jie/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink | sed -n '1,60p'
```

关键配置：

```sh
mavlink start -x -u $udp_gcs_port_local -o 14550 -t 192.168.1.20 -r 4000000 -f
mavlink stream -r 50 -s POSITION_TARGET_LOCAL_NED -u $udp_gcs_port_local
mavlink stream -r 50 -s LOCAL_POSITION_NED -u $udp_gcs_port_local
mavlink stream -r 50 -s GLOBAL_POSITION_INT -u $udp_gcs_port_local
mavlink stream -r 50 -s ATTITUDE -u $udp_gcs_port_local
mavlink stream -r 50 -s ATTITUDE_QUATERNION -u $udp_gcs_port_local
mavlink stream -r 50 -s ATTITUDE_TARGET -u $udp_gcs_port_local
mavlink stream -r 50 -s SERVO_OUTPUT_RAW_0 -u $udp_gcs_port_local
mavlink stream -r 20 -s RC_CHANNELS -u $udp_gcs_port_local
mavlink stream -r 10 -s OPTICAL_FLOW_RAD -u $udp_gcs_port_local
```

含义：

```text
每个 PX4 都主动向 Windows 192.168.1.20:14550 发送 MAVLink。
每条 GCS 链路的最大发送速率设置为 4000000 bit/s。
多条遥测消息被设置为 50 Hz。
```

### 2.6 向 QGC 传了什么

主要是 MAVLink 遥测流：

| MAVLink 消息 | 当前频率 | 用途 |
|---|---:|---|
| `POSITION_TARGET_LOCAL_NED` | 50 Hz | PX4 当前位置控制目标 |
| `LOCAL_POSITION_NED` | 50 Hz | 本地位置、速度 |
| `GLOBAL_POSITION_INT` | 50 Hz | 经纬高位置 |
| `ATTITUDE` | 50 Hz | 欧拉角姿态 |
| `ATTITUDE_QUATERNION` | 50 Hz | 四元数姿态 |
| `ATTITUDE_TARGET` | 50 Hz | 姿态控制目标 |
| `SERVO_OUTPUT_RAW_0` | 50 Hz | 电机/舵机输出 |
| `RC_CHANNELS` | 20 Hz | 遥控通道 |
| `OPTICAL_FLOW_RAD` | 10 Hz | 光流数据 |

除此之外，PX4 和 QGC 之间还会有默认的 heartbeat、系统状态、电池、GPS、参数、mission 等消息。

### 2.7 根因

QGC 网络占用高的核心原因：

```text
多架 PX4 同时向 Windows QGC 的 14550 端口发送高频 MAVLink 遥测。
```

当前量级估算：

```text
A 板 5 架 PX4 ~= 2 Mbps
B 板 5 架 PX4 ~= 2 Mbps
Windows QGC 总接收 ~= 4 Mbps
```

这和 Windows 任务管理器截图吻合。

## 3. 是否需要重新编译 PX4

### 3.1 PX4 SITL 实际读取哪个启动脚本

当前 PX4 进程类似：

```text
px4 -i 1 -d /home/jie/PX4-Autopilot/build/px4_sitl_default/etc
```

所以运行时实际读取的是：

```text
/home/jie/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/px4-rc.mavlink
```

源码模板位于：

```text
/home/jie/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink
```

### 3.2 三种修改方式

长期推荐：

```bash
vim /home/jie/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink

cd /home/jie/PX4-Autopilot
DONT_RUN=1 HEADLESS=1 make px4_sitl gazebo-classic
```

然后重启 PX4 SITL。

这种方式不是大量重新编译 C++，通常主要是把 ROMFS 启动脚本同步到 build 目录。

快速测试：

```bash
vim /home/jie/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/px4-rc.mavlink
```

然后重启 PX4 SITL。

这种方式不需要 make，但下次 PX4 构建可能被覆盖。

正在运行时：

```text
只改文件，不重启 PX4，不会生效。
```

因为 `mavlink stream -r ...` 是 PX4 启动时执行的命令。

## 4. 降低 QGC 网络占用的建议

QGC 日常看图标、姿态、位置，不需要所有消息都 50 Hz。

建议先把关键流降频：

```sh
mavlink stream -r 10 -s LOCAL_POSITION_NED -u $udp_gcs_port_local
mavlink stream -r 10 -s GLOBAL_POSITION_INT -u $udp_gcs_port_local
mavlink stream -r 10 -s ATTITUDE -u $udp_gcs_port_local
mavlink stream -r 5 -s POSITION_TARGET_LOCAL_NED -u $udp_gcs_port_local
```

这些可以先注释掉，除非确实需要在 QGC 里高频观察：

```sh
# mavlink stream -r 50 -s ATTITUDE_QUATERNION -u $udp_gcs_port_local
# mavlink stream -r 50 -s ATTITUDE_TARGET -u $udp_gcs_port_local
# mavlink stream -r 50 -s SERVO_OUTPUT_RAW_0 -u $udp_gcs_port_local
# mavlink stream -r 10 -s OPTICAL_FLOW_RAD -u $udp_gcs_port_local
```

需要注意：

- 这些是 PX4 到 QGC 的 MAVLink 遥测，不是 ROS2 DDS 控制链路。
- 降低 QGC MAVLink stream 频率，不会直接影响 ROS2 `/uav_N/formation_target` 到 PX4 的 Offboard 控制频率。
- Offboard 控制仍由 `px4_bridge_uxrce` 持续发布 `OffboardControlMode` 和 `TrajectorySetpoint`。

## 5. 今天形成的排查方法

以后遇到类似网络或配置不生效问题，可以按这个顺序查：

```bash
# 1. 看进程，确认到底跑了几架 PX4、几份 launch
ps -ef | rg 'px4|MicroXRCE|gzserver|ros2 launch'

# 2. 看 UDP 端口
ss -uapn | rg 'px4|145|185|8888|179'

# 3. 看网卡流量
ip -s link show eth0
sleep 3
ip -s link show eth0

# 4. 看 ROS2 节点
source /home/jie/uav_swarm_system/scripts/setup_env.sh
ros2 node list

# 5. 看 formation_controller 实际加载的配置
ros2 param get /swarm/formation_controller waypoints_config_file
ros2 param get /swarm/formation_controller formations_config_file
ros2 param get /swarm/formation_controller swarm_config_file

# 6. 看控制器实际发布的目标
ros2 topic echo /uav_1/formation_target --once

# 7. 看 PX4 bridge 汇总出来的飞机状态
ros2 topic echo /uav_1/state --once

# 8. 查 PX4 MAVLink 配置
nl -ba /home/jie/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink | sed -n '1,60p'

# 9. 看 PX4 源码是否有未提交修改
git -C /home/jie/PX4-Autopilot diff -- ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink
```

核心判断原则：

```text
先看实际运行进程，再看实际端口，再看实际加载参数，最后再判断应该改哪个配置文件。
```

