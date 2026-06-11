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
│   ├── cluster_boards.yaml
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
    ├── swarm_cluster.sh
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

## 首次构建

### 新开发板完整安装

全新 RK3588 开发板优先使用安装脚本。它会安装 ROS2 Humble、MicroXRCEAgent 2.4.1、PX4 v1.14.4、Gazebo Classic 11、`px4_msgs release/1.14`，并构建本项目 `ros2_ws`。

```bash
cd /home/jie/uav_swarm_system
./scripts/rk3588_install_full_sitl.sh full-sitl
```

如果只部署伴随计算机飞行环境，不在该板运行 PX4/Gazebo SITL：

```bash
./scripts/rk3588_install_full_sitl.sh flight-only
```

详细安装记录见 `docs/rk3588_full_sitl_install.md`。

### 已有环境下构建 ROS2 工作空间

如果 ROS2、PX4、MicroXRCEAgent、Gazebo 已经安装好，只需要构建本项目：

```bash
cd /home/jie/uav_swarm_system/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DPYTHON_INCLUDE_DIR=/usr/include/python3.10 \
  -DPYTHON_LIBRARY=/usr/lib/$(dpkg-architecture -qDEB_HOST_MULTIARCH)/libpython3.10.so
```

构建成功后重新加载项目环境：

```bash
cd /home/jie/uav_swarm_system
source scripts/setup_env.sh
```

快速确认包已安装到 ROS2 环境：

```bash
ros2 pkg list | grep -E 'swarm_bringup|px4_bridge_uxrce|formation_controller'
ros2 launch swarm_bringup swarm_px4_uxrce.launch.py --show-args
```

### 什么时候需要重新构建

修改这些内容后需要重新构建：

- `ros2_ws/src/*` 下的 C++ 或 Python 包代码。
- `swarm_msgs` 消息定义。
- 新增或删除 ROS2 package。
- 修改 `CMakeLists.txt` 或 `package.xml`。

只修改这些内容通常不需要重新构建：

- `config/*.yaml`
- `docs/*`
- `README.md`
- `scripts/*.sh`，除非脚本依赖新安装的包或新生成的接口。

## PX4-Autopilot 构建和机型参数

PX4 源码不在本项目里，固定路径是：

```text
/home/jie/PX4-Autopilot
```

### 构建 PX4 SITL

首次安装脚本会自动构建 PX4。手动重新构建时执行：

```bash
cd /home/jie/PX4-Autopilot
git checkout v1.14.4
DONT_RUN=1 HEADLESS=1 make px4_sitl gazebo-classic
```

`DONT_RUN=1` 表示只编译，不自动启动仿真。

构建产物检查：

```bash
ls -lh /home/jie/PX4-Autopilot/build/px4_sitl_default/bin/px4
ls /home/jie/PX4-Autopilot/build/px4_sitl_default/build_gazebo-classic/*.so
```

### 改机型

`start_px4_multi_sitl.sh` 的第二个参数是 Gazebo Classic model：

```bash
./scripts/start_px4_multi_sitl.sh <数量> <模型>
```

默认是：

```bash
./scripts/start_px4_multi_sitl.sh 3 iris
```

例如换成 `typhoon_h480`：

```bash
PX4_HOME_LAT=34.566096 PX4_HOME_LON=110.092301 PX4_HOME_ALT=350 \
PX4_INSTANCE_START=1 PX4_SPAWN_X=0 PX4_SPAWN_Y=3 \
  ./scripts/start_px4_multi_sitl.sh 1 typhoon_h480
```

模型必须同时满足两点：

```text
1. 有 Gazebo model:
   /home/jie/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/<model>

2. 有 PX4 airframe:
   /home/jie/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/*_gazebo-classic_<model>
```

检查可用模型：

```bash
ls /home/jie/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models
ls /home/jie/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/*gazebo-classic*
```

注意：当前编队控制和起飞脚本主要按多旋翼 `iris` 验证。固定翼、车、船、VTOL 等模型即使能启动，也不一定适配当前 leader-follower OFFBOARD 编队逻辑。

### 改 world

Gazebo world 用 `GAZEBO_WORLD` 指定，不带 `.world` 后缀：

```bash
GAZEBO_WORLD=ksql_airport \
PX4_HOME_LAT=34.566096 PX4_HOME_LON=110.092301 PX4_HOME_ALT=350 \
PX4_INSTANCE_START=1 PX4_SPAWN_X=0 PX4_SPAWN_Y=3 \
  ./scripts/start_px4_multi_sitl.sh 1 iris
```

可用 world：

```bash
ls /home/jie/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds
```

### 改启动参数但不重编 PX4

这些参数通常只需要重启 SITL，不需要重新编译 PX4：

| 参数 | 作用 |
|---|---|
| `PX4_HOME_LAT/LON/ALT` | QGC 地图全球 home 经纬高 |
| `PX4_INSTANCE_START` | PX4 instance 起始编号 |
| `PX4_SPAWN_X/Y` | 第一架 Gazebo 出生位置 |
| `PX4_SPAWN_X/Y_STEP` | 多机出生间距 |
| `GAZEBO_WORLD` | Gazebo world |
| `HEADLESS` | 是否启动 `gzclient` |
| `PX4_TARGET` | 使用哪个 PX4 build 目录，默认 `px4_sitl_default` |

### 修改 PX4 文件后什么时候重编

需要重新构建 PX4：

- 修改 PX4 C/C++ 飞控模块。
- 修改 PX4 Gazebo 插件源码。
- 修改 `ROMFS/px4fmu_common/init.d-posix/airframes/*`。
- 修改 `ROMFS/px4fmu_common/init.d-posix/px4-rc.*` 后希望同步到 build 目录。
- 新增 airframe，或者让新模型通过 `PX4_SIM_MODEL=gazebo-classic_<model>` 自动匹配。

通常只重启即可：

- 修改 world 文件。
- 修改 model 的 `.sdf.jinja`。
- 修改本项目 `config/*.yaml`。
- 调整 `PX4_HOME_*`、`PX4_SPAWN_*`、`GAZEBO_WORLD` 等启动环境变量。

临时改 PX4 参数可以通过 QGC 或 PX4 shell 的 `param set`，但这类修改可能只保存在对应 instance 的 `rootfs` 参数文件里。需要可复现运行时，优先写进启动脚本、airframe 或项目配置，并记录到 docs。

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
PX4_HOME_LAT=34.566096 PX4_HOME_LON=110.092301 PX4_HOME_ALT=350 \
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
PX4_HOME_LAT=34.566096 PX4_HOME_LON=110.092301 PX4_HOME_ALT=350 \
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
PX4_HOME_LAT=34.566096 PX4_HOME_LON=110.092301 PX4_HOME_ALT=350 \
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

## 11 块板 SSH 一键启动

当开发板镜像一致、项目路径一致、只有 IP 不同时，推荐使用集中编排脚本：

```bash
cd /home/jie/uav_swarm_system
./scripts/swarm_cluster.sh dry-run
./scripts/swarm_cluster.sh check
./scripts/swarm_cluster.sh start
```

默认配置文件：

```text
config/cluster_boards.yaml
```

默认规模：

```text
IP: 192.168.1.40 ~ 192.168.1.50
SSH user: jie
每板: 4 架 PX4 SITL
总数: 44 架
主控板: 192.168.1.40
```

编号规则：

```text
192.168.1.40 -> uav_1  ~ uav_4,  px4_1  ~ px4_4,  MAV_SYS_ID 2  ~ 5,  spawn_x=0
192.168.1.41 -> uav_5  ~ uav_8,  px4_5  ~ px4_8,  MAV_SYS_ID 6  ~ 9,  spawn_x=40
...
192.168.1.50 -> uav_41 ~ uav_44, px4_41 ~ px4_44, MAV_SYS_ID 42 ~ 45, spawn_x=400
```

脚本动作：

| 命令 | 作用 |
|---|---|
| `./scripts/swarm_cluster.sh dry-run` | 打印 11 块板映射和远程命令，不启动 |
| `./scripts/swarm_cluster.sh check` | 检查 SSH、项目路径、PX4、ROS2、MicroXRCEAgent |
| `./scripts/swarm_cluster.sh start` | 并发启动所有板子的 Agent、PX4/Gazebo、bridge，并在 A 板启动 swarm 节点 |
| `./scripts/swarm_cluster.sh status` | 查看各板进程状态 |
| `./scripts/swarm_cluster.sh stop` | 停止各板 PX4/Gazebo、MicroXRCEAgent 和 swarm launch |

先用两块板验证：

```bash
./scripts/swarm_cluster.sh start --limit 2
./scripts/swarm_cluster.sh status --limit 2
```

确认无误后启动完整 11 板：

```bash
./scripts/swarm_cluster.sh start
./scripts/swarm_cluster.sh status
```

起飞：

```bash
./scripts/swarm_arm_takeoff.sh --count 44
```

日志位于每块板本机：

```text
/home/jie/uav_swarm_system/logs/cluster/<run_id>/
```

使用前提：

- A 板到其他 10 块板已经配置 `jie` 用户免密 SSH。
- 所有板项目路径都是 `/home/jie/uav_swarm_system`。
- 所有板 ROS2 DDS 网络互通，`ROS_DOMAIN_ID` 保持一致。
- 只在 A 板启动一份 `/swarm/manager` 和 `/swarm/formation_controller`。

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
PX4_HOME_LAT=34.566096 PX4_HOME_LON=110.092301 PX4_HOME_ALT=350 \
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
PX4_HOME_LAT=34.566096 PX4_HOME_LON=110.092301 PX4_HOME_ALT=350 \
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
- `docs/debug/cluster_ssh_orchestration_2026-06-09.md`
- `docs/debug/px4_ros2_dds_runtime_concepts.md`
- `docs/debug/waypoints_initial_position_and_rates.md`
- `docs/debug/control_flow_and_node_interfaces.md`
- `docs/debug/rk3588_distributed_sitl_debug_2026-06-02.md`

其他文档：

- `docs/rk3588_full_sitl_install.md`
- `docs/debug/rk3588_full_sitl_install_issues.md`
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
