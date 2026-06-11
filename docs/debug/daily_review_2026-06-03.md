# 2026-06-03 调试复盘

本文复盘 2026-06-03 围绕两块 RK3588 开发板、PX4 SITL、MicroXRCE-DDS、ROS2 bridge、QGC 和编队控制做的整理、验证和文档沉淀。

当天的核心目标不是新增复杂算法，而是把已经跑通的分布式仿真链路讲清楚、固定下来，并把容易混淆的坐标、命名空间、DDS 发现、频率参数和航点偏移关系写成可复用文档。

## 1. 当天解决的主要问题

### 1.1 PX4 初始经纬高可配置

需求：

```text
加载其他地方的地图时，希望飞机在 QGC 地图上出现在指定经纬高位置。
```

处理：

- 在 `scripts/start_px4_multi_sitl.sh` 中支持：

```bash
PX4_HOME_LAT
PX4_HOME_LON
PX4_HOME_ALT
```

结论：

- `PX4_HOME_LAT/LON/ALT` 影响 PX4/Gazebo Classic GPS、groundtruth、气压计等插件使用的全球 home。
- QGC 地图上的经纬高会跟着这组参数变化。
- `PX4_HOME_LAT` 和 `PX4_HOME_LON` 必须成对设置。
- `PX4_HOME_ALT` 是海拔高度，单位 m。

示例：

```bash
PX4_HOME_LAT=34.566096 PX4_HOME_LON=110.092301 PX4_HOME_ALT=350 \
PX4_INSTANCE_START=1 PX4_SPAWN_X=0 PX4_SPAWN_Y=3 \
  ./scripts/start_px4_multi_sitl.sh 1 iris
```

### 1.2 全局经纬高、Gazebo 出生点、ROS2 initial_position 分清

当天明确了四层位置概念：

| 层级 | 参数/文件 | 作用 |
|---|---|---|
| PX4 全球 home | `PX4_HOME_LAT/LON/ALT` | QGC 地图经纬高、PX4 传感器仿真全球原点 |
| Gazebo 出生点 | `PX4_SPAWN_X/Y` 和 step | 模型在 Gazebo 本地世界中的出生位置 |
| ROS2 初始锚点 | launch `spawn_origin/spawn_spacing` 生成的 `initial_position` | bridge 做 ENU/NED 转换时使用 |
| 编队 offset | `config/formations.yaml` | follower 相对 leader 的目标位置 |

关键原则：

```text
PX4_SPAWN_X/Y 必须和 bridge 的 initial_position 对齐。
```

否则物理目标会整体偏移。

### 1.3 解释 B 板飞机从 20 飞到 120 的现象

当天把这个现象归纳成公式：

```text
实际物理目标 = PX4_SPAWN_X + (waypoint - initial_position.x)
```

如果：

```text
PX4_SPAWN_X = 20
initial_position.x = 0
waypoint = 100
```

那么：

```text
实际物理目标 = 20 + (100 - 0) = 120
```

这不是航点自动变成了 `[20, 120]`，而是 Gazebo 出生点和 bridge 的 `initial_position` 没有对齐。

正确对齐时：

```text
PX4_SPAWN_X = 20
initial_position.x = 20
waypoint = 100

实际物理目标 = 20 + (100 - 20) = 100
```

### 1.4 多机启动时 origin 和 step 的对应关系

如果一块板上一次启动多架飞机，锚点只设置一次，后续用 step/spacing 自动排开。

PX4/Gazebo：

```bash
PX4_HOME_LAT=34.566096 PX4_HOME_LON=110.092301 PX4_HOME_ALT=350 \
PX4_INSTANCE_START=2 \
PX4_SPAWN_X=20 PX4_SPAWN_Y=3 \
PX4_SPAWN_X_STEP=10 PX4_SPAWN_Y_STEP=0 \
  ./scripts/start_px4_multi_sitl.sh 3 iris
```

对应 ROS2 bridge：

```bash
ros2 launch swarm_bringup swarm_px4_uxrce.launch.py \
  instance_start:=2 vehicle_count:=3 \
  enable_swarm_nodes:=false \
  spawn_origin_x:=20 spawn_origin_y:=3 \
  spawn_spacing_x:=10 spawn_spacing_y:=0
```

对应关系：

```text
instance 2 -> uav_2 -> px4_2 -> spawn/initial x=20
instance 3 -> uav_3 -> px4_3 -> spawn/initial x=30
instance 4 -> uav_4 -> px4_4 -> spawn/initial x=40
```

必须保持：

```text
PX4_INSTANCE_START      == ROS2 instance_start
PX4 启动数量            == ROS2 vehicle_count
PX4_SPAWN_X/Y           == ROS2 spawn_origin_x/y
PX4_SPAWN_X/Y_STEP      == ROS2 spawn_spacing_x/y
```

## 2. 当天澄清的 PX4 / ROS2 / DDS 概念

### 2.1 PX4 rootfs

`rootfs` 不是开发板 Ubuntu 系统，也不是板子镜像。

它是每个 PX4 SITL 实例自己的运行目录：

```text
/home/jie/PX4-Autopilot/build/px4_sitl_default/rootfs/<instance_id>
```

作用：

- 保存该 PX4 实例自己的日志。
- 保存参数、dataman 等运行状态。
- 避免多实例互相覆盖运行文件。

### 2.2 MAV_SYS_ID 和 UXRCE_DDS_KEY

PX4 v1.14.4 多机 SITL 规则：

```text
PX4 instance N -> MAV_SYS_ID N+1
PX4 instance N -> UXRCE_DDS_KEY N+1
```

`MAV_SYS_ID`：

- 是 MAVLink 包里的飞机 ID。
- QGC 靠它区分飞机。
- 它不是端口号。

`UXRCE_DDS_KEY`：

- 是 PX4 uXRCE-DDS client 的客户端标识。
- 用于让 MicroXRCEAgent / DDS 侧区分不同 PX4 client。
- 它不是 ROS2 namespace，也不是 MAVLink 端口。

### 2.3 `/px4_N/fmu/*` 和 `/uav_N/px4_bridge`

当天明确：

```text
/px4_N/fmu/*        由 PX4 uXRCE-DDS client 生成
/uav_N/px4_bridge   由本项目 launch 生成
```

PX4 的 `rcS` 会按 instance 启动：

```text
uxrce_dds_client -n px4_1
uxrce_dds_client -n px4_2
```

所以出现：

```text
/px4_1/fmu/*
/px4_2/fmu/*
```

本项目 launch 创建：

```text
/uav_1/px4_bridge
/uav_2/px4_bridge
```

并给 bridge 传：

```text
uav_1 -> px4_topic_prefix=px4_1
uav_2 -> px4_topic_prefix=px4_2
```

### 2.4 为什么 Offboard setpoint 必须持续发送

PX4 Offboard 不是“发一次航点让 PX4 自己飞完整航线”。

Offboard 语义是：

```text
外部 companion computer 正在实时控制，并且持续在线。
```

所以 bridge 需要持续发布：

```text
/px4_N/fmu/in/offboard_control_mode
/px4_N/fmu/in/trajectory_setpoint
```

作用：

- 给 PX4 当前目标。
- 告诉 PX4 外部控制链路还活着。

如果只发一次，可能导致：

- PX4 拒绝进入 Offboard。
- 已进入 Offboard 后退出 Offboard。
- 触发 hold/failsafe。

### 2.5 `/swarm/manager` 和 `/swarm/formation_controller` 只跑一份

ROS2 使用 DDS。

同一 DDS domain 中，只要：

```text
ROS_DOMAIN_ID 相同
topic 名字相同
消息类型相同
QoS 匹配
网络互通
```

发布者和订阅者就会通过 DDS discovery 自动发现。

所以 A 板上的 `/swarm/manager` 可以订阅 B 板的 `/uav_2/state`，不需要在代码里写 B 板 IP。

但 `/swarm/manager` 和 `/swarm/formation_controller` 必须全局只跑一份：

```text
每架飞机一份 /uav_N/px4_bridge
整个集群一份 /swarm/manager
整个集群一份 /swarm/formation_controller
```

如果两块板各跑一份 manager/controller，会出现：

- 两个 `/swarm/manager` 同时发布 `/swarm/state`。
- 两个 `/swarm/formation_controller` 同时发布 `/uav_N/formation_target`。
- DDS 不会报错，但控制目标和集群状态会混乱。

## 3. 当天澄清的航点和 leader/follower 行为

### 3.1 当前不是多个 leader 同时跑同一套航点

正确分布式模式下：

```text
uav_1 = leader
uav_2 = follower
```

只有 leader 直接飞 `config/waypoints.yaml`：

```text
/uav_1/formation_target
source: formation_controller.leader_waypoints
```

follower 飞：

```text
leader_position + formation_offset
```

如果看到 B 板的飞机也像 leader 一样跑 waypoint，优先怀疑：

```text
B 板也启动了自己的 /swarm/formation_controller
```

### 3.2 航点是项目 local_enu 坐标

`config/waypoints.yaml`：

```yaml
leader_waypoints:
  frame: local_enu
  points:
    - [0.0, 0.0, 10.0]
    - [100.0, 0.0, 10.0]
    - [100.0, 100.0, 10.0]
    - [0.0, 100.0, 10.0]
```

含义：

```text
x = east
y = north
z = up
单位 = m
```

它不是经纬度，也不是 PX4 原生 NED 坐标。

## 4. 当天澄清的频率参数

频率单位：

```text
1 Hz = 每秒 1 次
10 Hz = 每秒 10 次
20 Hz = 每秒 20 次
```

当前主要频率：

| 参数 | 默认 | 谁使用 | 作用 |
|---|---:|---|---|
| `swarm.publish_rate_hz` | 5Hz | `swarm_manager` | 发布 `/swarm/state` |
| `uxrce_backend.state_rate_hz` | 10Hz | 每个 `px4_bridge_uxrce` | 发布 `/uav_N/state` |
| `uxrce_backend.offboard_rate_hz` | 20Hz | 每个 `px4_bridge_uxrce` | 持续发布 PX4 Offboard setpoint |
| `formation_controller.control_rate_hz` | 10Hz | `formation_controller` | 计算 `/uav_N/formation_target` |
| `mavlink stream -r` | 视配置 | PX4 MAVLink | 发给 QGC 的显示数据刷新频率 |

频率链路：

```text
PX4 /px4_N/fmu/out/*
  -> bridge 以 state_rate_hz 发布 /uav_N/state
  -> manager 以 publish_rate_hz 汇总 /swarm/state
  -> controller 以 control_rate_hz 发布 /uav_N/formation_target
  -> bridge 以 offboard_rate_hz 持续喂 PX4 setpoint
```

重要结论：

```text
formation_controller 可以 10Hz
offboard_timer 仍然要 20Hz 持续发送
```

因为上层目标不需要每秒几十次更新，但 PX4 Offboard 需要稳定 setpoint 流。

## 5. 当天整理的 PX4 飞行模式

当前 bridge 会把 PX4 `VehicleStatus.nav_state` 转成：

```text
MANUAL
ALTCTL
POSCTL
AUTO_MISSION
AUTO_LOITER
AUTO_RTL
OFFBOARD
AUTO_TAKEOFF
AUTO_LAND
```

本项目最相关：

| 模式 | 当前作用 |
|---|---|
| `AUTO_TAKEOFF` | `/uav_N/takeoff` 后 PX4 执行自动起飞 |
| `OFFBOARD` | ROS2 bridge 持续给 PX4 setpoint，编队主模式 |
| `AUTO_LAND` | `/uav_N/land` 后 PX4 自动降落 |
| `AUTO_RTL` | `/uav_N/rtl` 后 PX4 自动返航 |
| `AUTO_LOITER` | `/uav_N/hold` 后 PX4 悬停/等待 |
| `AUTO_MISSION` | PX4 自己执行上传任务航线，不是当前编队主线 |

核心区别：

```text
AUTO_MISSION = 任务先上传给 PX4，PX4 自己飞。
OFFBOARD = ROS2 companion computer 持续实时给 setpoint。
```

## 6. 当天判断的算力消耗

当前 SITL/ROS2 编队链路里，最吃算力的是：

```text
Gazebo Classic + PX4 SITL
```

原因：

- Gazebo 跑物理仿真和传感器插件。
- 每架飞机是一个 PX4 SITL 进程。
- PX4 内部有 EKF、控制器、MAVLink、uXRCE-DDS client、日志等。

ROS2 节点相对轻：

```text
px4_bridge_uxrce：每机一份，中等偏轻，但实时性重要
formation_controller：当前小规模很轻
swarm_manager：很轻，只做状态汇总
MicroXRCEAgent：中等，主要取决于 px4_msgs 流量
```

后续接入感知后，算力大头会转移到：

```text
目标检测 / 图像推理 / 点云处理 / 建图
```

## 7. 推荐的两板分布式启动模板

### A 板 PX4

```bash
PX4_HOME_LAT=34.566096 PX4_HOME_LON=110.092301 PX4_HOME_ALT=350 \
PX4_INSTANCE_START=1 \
PX4_SPAWN_X=0 PX4_SPAWN_Y=3 \
  ./scripts/start_px4_multi_sitl.sh 1 iris
```

### B 板 PX4

```bash
PX4_HOME_LAT=34.566096 PX4_HOME_LON=110.092301 PX4_HOME_ALT=350 \
PX4_INSTANCE_START=2 \
PX4_SPAWN_X=30 PX4_SPAWN_Y=3 \
  ./scripts/start_px4_multi_sitl.sh 1 iris
```

### A 板 bridge

```bash
ros2 launch swarm_bringup swarm_px4_uxrce.launch.py \
  instance_start:=1 vehicle_count:=1 \
  enable_swarm_nodes:=false \
  spawn_origin_x:=0 spawn_origin_y:=3
```

### B 板 bridge

```bash
ros2 launch swarm_bringup swarm_px4_uxrce.launch.py \
  instance_start:=2 vehicle_count:=1 \
  enable_swarm_nodes:=false \
  spawn_origin_x:=30 spawn_origin_y:=3
```

### 只在 A 板启动集群节点

```bash
ros2 launch swarm_bringup swarm_px4_uxrce.launch.py \
  instance_start:=1 vehicle_count:=2 \
  enable_bridges:=false enable_swarm_nodes:=true \
  spawn_origin_x:=0 spawn_origin_y:=3 \
  spawn_spacing_x:=30 spawn_spacing_y:=0
```

## 8. 当天新增或更新的文档

新增/更新内容主要分散在以下文件：

```text
docs/debug/px4_ros2_dds_runtime_concepts.md
docs/debug/waypoints_initial_position_and_rates.md
docs/debug/control_flow_and_node_interfaces.md
docs/debug/rk3588_distributed_sitl_debug_2026-06-02.md
README.md
```

其中：

- `px4_ros2_dds_runtime_concepts.md`：解释 PX4 rootfs、UXRCE_DDS_KEY、MAV_SYS_ID、DDS 发现、QGC 端口、飞行模式、算力、Mermaid 总图。
- `waypoints_initial_position_and_rates.md`：解释航点、出生偏移、`initial_position`、频率参数和 B 板 20 到 120 问题。
- `control_flow_and_node_interfaces.md`：补充全球 home 与局部 ENU 控制坐标的关系。
- `rk3588_distributed_sitl_debug_2026-06-02.md`：补充 PX4 初始经纬高跟随地图切换的调试记录。
- `README.md`：挂载相关文档，并补充可复制运行示例。

## 9. 当天形成的排查清单

### QGC 图标闪烁

优先检查：

```text
两块板是否用了相同 PX4_INSTANCE_START
是否产生了相同 MAV_SYS_ID
```

### 飞机位置整体偏移

优先检查：

```text
PX4_SPAWN_X/Y 是否和 ROS2 spawn_origin_x/y 对齐
PX4_SPAWN_X/Y_STEP 是否和 ROS2 spawn_spacing_x/y 对齐
```

起飞前看：

```bash
ros2 topic echo /uav_2/state --once
```

如果 B 板 `PX4_SPAWN_X=30`，起飞前 `/uav_2/state.position.x` 应接近 `30`。

### 飞机已 arm/offboard 但不飞

优先检查：

```bash
ros2 topic echo /uav_1/formation_target --once
ros2 topic echo /uav_2/formation_target --once
```

如果看到：

```text
active: false
source: formation_controller.hold:px4_state_unhealthy
```

继续检查：

```bash
ros2 node list | sort
```

确保只有一份：

```text
/swarm/manager
/swarm/formation_controller
```

### DDS 跨板不通

检查：

```text
两块板 ROS_DOMAIN_ID 是否相同
网络是否互通
防火墙是否阻断 DDS discovery
topic 名字、类型、QoS 是否匹配
```

## 10. 后续建议

短期建议：

- 把 `ROS_DOMAIN_ID` 显式固定到 `setup_env.sh` 或单独环境配置里，避免以后接入其他 ROS2 系统时串域。
- 继续保持 `HEADLESS=1`，板端不启动 `gzclient`。
- 每次多机启动都成对检查 PX4 spawn 参数和 ROS2 spawn 参数。
- 分布式运行时严格保持“一机一 bridge，全局一 manager/controller”。

中期建议：

- 给 `swarm_bringup` 增加一个可打印 runtime config 的调试选项，方便直接看到每架无人机生成的 `initial_position`、`system_id`、`px4_topic_prefix`。
- 增加一个脚本检查当前 ROS2 graph 中是否存在重复 `/swarm/manager` 或 `/swarm/formation_controller`。
- 增加一个脚本对比起飞前 `/uav_N/state.position` 与预期 `spawn_origin`，提前发现 20 到 120 这类偏移。

真机前建议：

- 保持 Offboard 主链路 C++ 实现。
- 感知/目标检测接入后单独做算力评估。
- 所有真实飞行逻辑继续先在 SITL 中验证。
