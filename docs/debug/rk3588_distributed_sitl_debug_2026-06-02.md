# RK3588 双板分布式 SITL 调试记录 2026-06-02

## 目标

在两块 RK3588 开发板上分别运行完整本地仿真栈，并统一接入 Windows 主机上的 QGroundControl：

- A 板：`192.168.1.40`
- B 板：`192.168.1.41`
- Windows / QGC：`192.168.1.20`
- 两块板各自运行 `PX4-Autopilot + Gazebo Classic + MicroXRCEAgent + ROS2`
- QGC 同时显示两架飞机
- ROS2 能同时控制两架飞机进入 Offboard 编队

本次调试基于固定环境：

- PX4 v1.14.4
- ROS2 Humble
- Gazebo Classic 11.10.2
- MicroXRCEAgent 2.4.1
- 项目目录：`/home/jie/uav_swarm_system`

## 现象和排查过程

### 1. QGC 图标交替闪烁

初始操作是在 A、B 两块板都运行：

```bash
cd /home/jie/uav_swarm_system
./scripts/start_px4_multi_sitl.sh 1 iris
```

QGC 能收到飞机，但图标交替闪烁。

排查 PX4 v1.14.4 启动脚本后确认：

```sh
param set MAV_SYS_ID $((px4_instance+1))
param set UXRCE_DDS_KEY $((px4_instance+1))
```

两块板都从 PX4 instance 1 启动时：

```text
A 板：px4_instance=1 -> MAV_SYS_ID=2
B 板：px4_instance=1 -> MAV_SYS_ID=2
```

QGC 通过 MAVLink `MAV_SYS_ID` 区分飞机。两个不同 IP 的 PX4 使用相同 `MAV_SYS_ID=2` 时，QGC 会把它们当成同一架飞机的两条链路，表现为图标闪烁或状态交替覆盖。

处理方式：

- A 板固定使用 `PX4_INSTANCE_START=1`
- B 板固定使用 `PX4_INSTANCE_START=2`

对应关系：

```text
A 板：PX4 instance 1 -> MAV_SYS_ID 2
B 板：PX4 instance 2 -> MAV_SYS_ID 3
```

### 2. QGC 中两架飞机位置重叠

区分 `MAV_SYS_ID` 后，QGC 可以稳定显示两架飞机，但地图位置重叠。

原因：

- 两块板各自运行独立 Gazebo world。
- 默认 GPS/本地原点相同。
- 默认模型出生点相同。
- QGC 看到的是两架不同飞机，但两架上报的地理位置几乎一致。

处理方式是在启动 PX4 SITL 时给不同板设置不同 Gazebo 出生点：

```bash
# A 板
PX4_INSTANCE_START=1 PX4_SPAWN_X=0 PX4_SPAWN_Y=3 \
  ./scripts/start_px4_multi_sitl.sh 1 iris

# B 板
PX4_INSTANCE_START=2 PX4_SPAWN_X=30 PX4_SPAWN_Y=3 \
  ./scripts/start_px4_multi_sitl.sh 1 iris
```

说明：

- `PX4_SPAWN_X/Y` 是该板本地 Gazebo 世界坐标，单位按米理解。
- 该设置会改变各自 Gazebo 中飞机的出生位置。
- 两块板仍然不是同一个 Gazebo 物理世界；这只是让 QGC 和 ROS2 统一显示时不重叠。

### 3. PX4 初始经纬高需要跟随地图切换

后续加载其他地区的地图时，只改 `PX4_SPAWN_X/Y` 不够。`PX4_SPAWN_X/Y` 只是 Gazebo 本地米级坐标，QGC 地图上的经纬高来自 PX4/Gazebo GPS 仿真的全球 home。

本项目现在在 `scripts/start_px4_multi_sitl.sh` 中支持：

```bash
PX4_HOME_LAT
PX4_HOME_LON
PX4_HOME_ALT
```

示例：

```bash
PX4_HOME_LAT=31.230400 PX4_HOME_LON=121.473700 PX4_HOME_ALT=5 \
PX4_INSTANCE_START=1 PX4_SPAWN_X=0 PX4_SPAWN_Y=3 \
  ./scripts/start_px4_multi_sitl.sh 1 iris
```

含义：

- `PX4_HOME_LAT/LON/ALT`：PX4 GPS/groundtruth/气压计等 Gazebo Classic 插件使用的全球原点。
- `PX4_SPAWN_X/Y`：飞机相对该全球原点的本地 Gazebo 出生偏移。
- 两块板要显示在同一片地图区域时，使用同一组 `PX4_HOME_LAT/LON/ALT`。
- 两块板要避免 QGC 图标重叠时，再使用不同的 `PX4_SPAWN_X/Y`。
- `PX4_HOME_LAT` 和 `PX4_HOME_LON` 必须成对设置；`PX4_HOME_ALT` 单位是 m。

两块板示例：

```bash
# A 板
PX4_HOME_LAT=31.230400 PX4_HOME_LON=121.473700 PX4_HOME_ALT=5 \
PX4_INSTANCE_START=1 PX4_SPAWN_X=0 PX4_SPAWN_Y=3 \
  ./scripts/start_px4_multi_sitl.sh 1 iris

# B 板
PX4_HOME_LAT=31.230400 PX4_HOME_LON=121.473700 PX4_HOME_ALT=5 \
PX4_INSTANCE_START=2 PX4_SPAWN_X=30 PX4_SPAWN_Y=3 \
  ./scripts/start_px4_multi_sitl.sh 1 iris
```

### 4. QGC 中两架都显示，但 ROS2 控制时只有一架飞

QGC 侧正常后，继续用 ROS2 控制发现只有一架飞机响应，另一架原地不动。

根因是 PX4 instance 和 ROS2 launch 生成的 bridge 没有对齐。

PX4 uXRCE-DDS topic 规则：

```text
PX4 instance 1 -> /px4_1/fmu/*
PX4 instance 2 -> /px4_2/fmu/*
```

项目内部 ROS2 namespace 规则：

```text
uav_1 -> /uav_1/arm, /uav_1/state, /uav_1/formation_target
uav_2 -> /uav_2/arm, /uav_2/state, /uav_2/formation_target
```

bridge 负责把项目内部服务转成 PX4 DDS topic：

```text
/uav_1/px4_bridge -> /px4_1/fmu/in/*
/uav_2/px4_bridge -> /px4_2/fmu/in/*
```

如果 B 板 PX4 已经是 instance 2，但 ROS2 launch 仍默认生成 `uav_1 / px4_1 / system_id 2`，那么 B 板 ROS2 命令会发到错误 topic，无法控制 B 板的 `px4_2`。

处理方式是在 `swarm_px4_uxrce.launch.py` 增加 `instance_start`：

```bash
# A 板 bridge
ros2 launch swarm_bringup swarm_px4_uxrce.launch.py \
  instance_start:=1 vehicle_count:=1

# B 板 bridge
ros2 launch swarm_bringup swarm_px4_uxrce.launch.py \
  instance_start:=2 vehicle_count:=1
```

最终固定映射：

```text
板子   PX4_INSTANCE_START   ROS2 instance_start   ROS2无人机ID   PX4话题   QGC身份
A      1                    1                     uav_1        px4_1    MAV_SYS_ID 2
B      2                    2                     uav_2        px4_2    MAV_SYS_ID 3
```

### 5. 飞机已 arm/offboard，但不继续按航点飞

进一步调试时，`/uav_1/state` 和 `/uav_2/state` 显示飞机已经：

```text
armed: true
flight_mode: OFFBOARD
healthy: true
```

但飞机仍不继续飞航点。

检查编队目标：

```bash
ros2 topic echo /uav_1/formation_target --once
ros2 topic echo /uav_2/formation_target --once
```

看到：

```text
active: false
source: formation_controller.hold:px4_state_unhealthy
```

继续检查 ROS2 graph：

```bash
ros2 node list | sort
```

发现同时存在两份同名 swarm 节点：

```text
/swarm/formation_controller
/swarm/formation_controller
/swarm/manager
/swarm/manager
/uav_1/px4_bridge
/uav_2/px4_bridge
```

原因：

- A、B 两块板都完整启动了 `swarm_px4_uxrce.launch.py`。
- 两边都会创建 `/swarm/manager` 和 `/swarm/formation_controller`。
- 两个 manager 都发布 `/swarm/state`，但各自只掌握本机无人机状态。
- formation controller 会收到不完整或过期的集群状态，判断目标机状态不健康，于是发布 inactive hold target。

处理方式是在 launch 中增加两个开关：

```text
enable_bridges
enable_swarm_nodes
```

分布式运行时：

- 每块板只跑自己的 `px4_bridge`。
- `/swarm/manager` 和 `/swarm/formation_controller` 全网只跑一份。

最终启动方式：

```bash
# A 板：只跑 uav_1 bridge
ros2 launch swarm_bringup swarm_px4_uxrce.launch.py \
  instance_start:=1 vehicle_count:=1 enable_swarm_nodes:=false

# B 板：只跑 uav_2 bridge
ros2 launch swarm_bringup swarm_px4_uxrce.launch.py \
  instance_start:=2 vehicle_count:=1 enable_swarm_nodes:=false

# 只在 A 板：跑一份集群管理和编队控制，不再启动 bridge
ros2 launch swarm_bringup swarm_px4_uxrce.launch.py \
  instance_start:=1 vehicle_count:=2 \
  enable_bridges:=false enable_swarm_nodes:=true \
  spawn_origin_x:=0 spawn_origin_y:=3 spawn_spacing_x:=30 spawn_spacing_y:=0
```

正确 ROS2 graph：

```text
/swarm/formation_controller
/swarm/manager
/uav_1/px4_bridge
/uav_2/px4_bridge
```

起飞命令明确指定无人机 ID：

```bash
./scripts/swarm_arm_takeoff.sh uav_1 uav_2
```

不建议在分布式场景下依赖无参数自动发现，因为不同板上的本地 launch 和 DDS graph 可能让脚本误判控制对象。

## 本次代码和文档改动

### `scripts/start_px4_multi_sitl.sh`

新增长期参数：

```bash
PX4_INSTANCE_START
PX4_SPAWN_X
PX4_SPAWN_Y
PX4_SPAWN_X_STEP
PX4_SPAWN_Y_STEP
```

作用：

- 允许不同开发板从不同 PX4 instance 启动，避免 `MAV_SYS_ID` 冲突。
- 允许设置 Gazebo 出生点，避免 QGC 显示位置重叠。
- 保留 headless 默认运行方式，适合 RK3588 板端。

### `ros2_ws/src/swarm_bringup/launch/swarm_px4_uxrce.launch.py`

新增 launch 参数：

```text
instance_start
enable_bridges
enable_swarm_nodes
```

作用：

- `instance_start`：让 ROS2 生成的 `uav_N / px4_N / system_id` 与 PX4 instance 对齐。
- `enable_bridges`：控制是否启动每机 `px4_bridge_uxrce`。
- `enable_swarm_nodes`：控制是否启动 `/swarm/manager` 和 `/swarm/formation_controller`。

### `README.md`

新增“两块开发板分布式 SITL”章节，记录：

- A/B 板固定映射表。
- 启动顺序。
- ROS2 分层启动方式。
- 起飞和降落命令。
- 常见问题排查。

## 最终验证结果

已验证：

- A、B 两块 RK3588 开发板分别运行 PX4 SITL。
- 两架飞机以不同 `MAV_SYS_ID` 稳定显示在 Windows QGC。
- QGC 中两架飞机位置可通过出生点偏移错开。
- ROS2 graph 中只保留一份 `/swarm/manager` 和 `/swarm/formation_controller`。
- `/uav_1/px4_bridge` 控制 `/px4_1/fmu/in/*`。
- `/uav_2/px4_bridge` 控制 `/px4_2/fmu/in/*`。
- `./scripts/swarm_arm_takeoff.sh uav_1 uav_2` 可以让两架飞机进入可控状态。
- 修正重复 swarm 节点后，编队目标不再因为 `px4_state_unhealthy` 被置为 inactive。

## 后续注意事项

1. `PX4_INSTANCE_START` 必须和 ROS2 `instance_start` 对齐。
2. 分布式两块板时，每块板跑自己的 bridge，但 `/swarm/manager` 和 `/swarm/formation_controller` 只能跑一份。
3. 两块板各自运行独立 Gazebo world，不具备同一物理世界中的碰撞和传感器交互。
4. QGC 只参与 MAVLink 地面站显示和基础飞控监控，不是 ROS2 集群控制的一部分。
5. 需要统一控制多块板时，必须确保 ROS2 DDS 域、网络、防火墙和节点 namespace 都一致且无重复 swarm 节点。
6. 分布式场景下起飞/降落脚本优先显式写 `uav_1 uav_2`，不要依赖自动发现。

## 常用检查命令

检查节点是否重复：

```bash
ros2 node list | sort
```

检查 PX4 DDS topic：

```bash
ros2 topic list | grep '/px4_'
```

检查单机状态：

```bash
ros2 topic echo /uav_1/state --once
ros2 topic echo /uav_2/state --once
```

检查集群状态：

```bash
ros2 topic echo /swarm/state --once
```

检查编队目标：

```bash
ros2 topic echo /uav_1/formation_target --once
ros2 topic echo /uav_2/formation_target --once
```

检查 QGC 相关 MAVLink 日志：

```bash
rg -n "mavlink|14550|remote port" /home/jie/PX4-Autopilot/build/px4_sitl_default/rootfs/*/out.log
```
