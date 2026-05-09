# MY Log

## 使用规则

本文件用于记录阶段性进度。每次任务收尾时，至少补充以下内容：

- 这次做了什么
- 改了哪些文件
- 下一步做什么

建议按时间倒序追加，最新记录放在最上方。

---

## 2026-05-09（README 初始位置与队形 offset 对比说明）

### 本次做了什么

- 按用户要求在 README 中新增对比表，区分 Gazebo 出生位置、`swarm.yaml initial_position` 和 `formations.yaml offsets`。
- 补充说明三者分别在哪个阶段生效、由谁使用、坐标含义以及与飞行编队的关系。
- 顺手清理 README 中残留的旧后端说明，保持 uXRCE-DDS 主线清晰。

### 改了哪些文件

- `README.md`
- `docs/for_codex/MY_Log.md`

### 验证结果

- 已检查 README 中不再残留旧后端入口和旧 launch 名称。

### 下一步做什么

- 后续如果调整 PX4 spawn 位置，应同步核对 `config/swarm.yaml` 的 `initial_position`，避免 ROS2 侧状态坐标和 Gazebo 出生位置不一致。

---

## 2026-05-09（配置文件中文注释补充）

### 本次做了什么

- 按用户要求为项目根目录 `config/` 下的配置文件补充中文注释。
- 注释覆盖配置文件整体作用和关键字段含义。
- 保持原配置值不变，只补充说明性注释。

### 改了哪些文件

- `config/env.yaml`
- `config/swarm.yaml`
- `config/formations.yaml`
- `config/waypoints.yaml`
- `docs/for_codex/MY_Log.md`

### 验证结果

- 已使用 `yaml.safe_load` 验证 4 个 YAML 文件均可正常解析。
- 已验证 `swarm_mock.launch.py --show-args` 和 `swarm_px4_uxrce.launch.py --show-args` 正常。

### 下一步做什么

- 后续新增配置字段时，同步补充中文注释，避免配置含义只存在于代码里。

---

## 2026-05-08（清理旧 PX4 Python 后端，收敛到 uXRCE-DDS 主线）

### 本次做了什么

- 按用户要求保留 mock，移除项目主工程中的旧 PX4 Python 通信后端。
- 删除旧真实 PX4 Python 后端源码和对应 SITL launch。
- 将 `px4_bridge` 收窄为纯 mock 包，仅用于无 PX4/Gazebo 的快速 smoke test。
- 保留 `px4_bridge_uxrce` 作为唯一真实 PX4 通信与 Offboard 主线。
- 清理 `config/swarm.yaml` 中旧后端端口、参数和 supported type。
- 清理 README、AGENTS、architecture 和脚本中的旧后端说明。
- 压缩本日志，移除已经废弃的旧后端历史流水，避免后续阅读被误导。

### 改了哪些文件

- `AGENTS.md`
- `README.md`
- `config/swarm.yaml`
- `docs/architecture.md`
- `docs/for_codex/MY_Log.md`
- `ros2_ws/src/formation_controller/launch/swarm_mock.launch.py`
- `ros2_ws/src/px4_bridge/package.xml`
- `ros2_ws/src/px4_bridge/setup.py`
- `ros2_ws/src/px4_bridge/px4_bridge/px4_bridge_node.py`
- `scripts/stop_sitl_stack.sh`
- 删除旧真实 PX4 Python launch 文件
- 删除旧真实 PX4 Python 后端源码文件

### 验证结果

- 已清理 `build/px4_bridge`、`install/px4_bridge`、`build/formation_controller`、`install/formation_controller` 中的旧安装产物并重建。
- `colcon build --packages-select swarm_msgs px4_bridge px4_bridge_uxrce swarm_manager formation_controller --symlink-install` 成功。
- `colcon build --packages-select px4_bridge swarm_manager --symlink-install` 成功。
- `ros2 pkg executables px4_bridge` 只剩 `px4_bridge` 和 `mock_px4_bridge`。
- `find ros2_ws/src ros2_ws/install -name '*sitl.launch.py' -o -name '*mav*'` 无输出。
- `rg --hidden --no-ignore` 确认除 `references/` 和 `docs/reference_analysis.md` 外，项目内无旧后端关键字残留。
- `ros2 launch formation_controller swarm_mock.launch.py --show-args` 正常。
- `ros2 launch formation_controller swarm_px4_uxrce.launch.py --show-args` 正常。
- 短跑 `swarm_mock.launch.py formation_type:=line` 成功，mock bridge、`swarm_manager`、C++ `formation_controller` 均可启动并干净退出。
- `git diff --check` 通过。

### 下一步做什么

- 默认开发、SITL 和后续真机部署都走 `px4_bridge_uxrce`。
- mock 只用于快速验证消息、状态管理、编队逻辑和配置。
- 如果后续需要参考外部旧通信实现，只在 `references/` 或 `docs/reference_analysis.md` 中保留分析，不进入主工程。

---

## 2026-05-08（uXRCE 三机实飞闭环排查）

### 本次做了什么

- 实际启动并检查完整链路：MicroXRCEAgent、PX4 v1.14.4 三机 Gazebo Classic SITL、ROS2 uXRCE bridge、`swarm_manager`、C++ `formation_controller`。
- 发现一次启动失败的直接原因：ROS2 launch 终端只 source 了本项目 install，没有 source `scripts/setup_env.sh`，导致 `px4_bridge_uxrce` 找不到 `px4_msgs` 运行库。
- 按固定环境重新 source `/home/jie/px4_ros_com_ws/install` 和本项目 install，并重建相关包。
- 验证 `/swarm/state` 中三机 telemetry 正常，起飞前状态是 `armed=false`、`healthy=true`，说明 launch 只发布目标，不会自动解锁起飞。
- 执行 `./scripts/swarm_arm_takeoff.sh`，三机成功进入 `armed=true`、`flight_mode=OFFBOARD`，leader 按航点运动，follower 保持三角偏移。
- 执行 `./scripts/swarm_land_all.sh` 并停止 PX4/Gazebo/ROS2/MicroXRCEAgent，避免残留进程占端口。
- 修复 `swarm_manager` 在 Ctrl-C 停止时重复 `rclpy.shutdown()` 导致 traceback 的问题。
- 更新 README，明确 uXRCE 版必须用 `scripts/setup_env.sh`，并明确 launch 后还需要显式 arm/takeoff。

### 改了哪些文件

- `README.md`
- `ros2_ws/src/swarm_manager/swarm_manager/swarm_manager_node.py`
- `docs/for_codex/MY_Log.md`

### 验证结果

- `colcon build --packages-select swarm_msgs px4_bridge px4_bridge_uxrce swarm_manager formation_controller --symlink-install` 成功。
- `colcon build --packages-select swarm_manager --symlink-install` 成功。
- `timeout -s SIGINT 3s ros2 run swarm_manager swarm_manager ...` 无 traceback，验证 Ctrl-C 退出修正有效。
- 实测三机起飞后连续采样 `/swarm/state`：
- 三机均为 `armed=true`、`flight_mode=OFFBOARD`、`healthy=true`。
- leader 从约 `(2.35, -0.05, 1.69)` 移动到 `(4.03, 3.09, 2.06)`，随后继续沿方形航线移动。
- follower 保持三角队形，例如 leader 约 `(-0.04, 1.63, 2.01)` 时，uav_2 约 `(-2.02, 0.94, 2.06)`，uav_3 约 `(-2.03, 3.92, 2.07)`。
- 降落后已清理 PX4/Gazebo/ROS2/MicroXRCEAgent 进程。

### 下一步做什么

- 后续运行 uXRCE 主线时，每个 ROS2 终端统一使用 `source scripts/setup_env.sh`。
- 如果还想减少误操作，可以新增一个 `scripts/start_ros2_uxrce_swarm.sh` 包装脚本，把 source 环境和 launch 命令固定下来。
- 继续测试 `formation_type:=line` 和 `formation_type:=column`，确认三种队形都能在 SITL 中稳定切换。

---

## 2026-05-08（清理 formation_controller Python 残留）

### 本次做了什么

- 检查 `formation_controller` 从 Python 包迁移到 C++ 包后的残留问题。
- 删除迁移后遗留的空目录。
- 确认当前 `formation_controller` 包结构为标准 C++ ROS2 包：`package.xml`、`CMakeLists.txt`、`src/formation_controller_node.cpp`、`launch/`。

### 改了哪些文件

- 删除空目录 `ros2_ws/src/formation_controller/formation_controller`
- 删除空目录 `ros2_ws/src/formation_controller/resource`
- 更新 `docs/for_codex/MY_Log.md`

### 验证结果

- `rg` 检查 `ros2_ws/src/formation_controller` 中已无 `ament_python`、`rclpy`、`setup.py`、`setup.cfg`、`console_scripts`、旧 Python 节点入口。
- `ros2 pkg executables formation_controller` 仍显示 `formation_controller formation_controller`。
- 安装产物确认为 ELF C++ executable。

### 下一步做什么

- 如果后续再从 Python 包迁移为 C++ 包，应同步清理源码、build、install 中的旧入口；必要时执行 `rm -rf build/<pkg> install/<pkg>` 后重新 `colcon build`。

---

## 2026-05-08（简化 Codex 协作文档）

### 本次做了什么

- 按用户要求删除 `docs/for_codex/MY_README.md` 和 `docs/for_codex/MY_Memory.md`。
- 后续不再维护这两个文件。
- 后续协作信息只更新项目根目录 `README.md` 和 `docs/for_codex/MY_Log.md`。

### 改了哪些文件

- 删除 `docs/for_codex/MY_README.md`
- 删除 `docs/for_codex/MY_Memory.md`
- 更新 `docs/for_codex/MY_Log.md`

### 下一步做什么

- 后续任务收尾只更新 `README.md` 和 `docs/for_codex/MY_Log.md` 中确有必要的内容。

---

## 2026-05-08（formation_controller C++ 重构）

### 本次做了什么

- 按 `AGENTS.md` 的 Python / C++ 语言规则重新检查当前 ROS2 包。
- 判断 `swarm_manager` 属于低频状态管理，继续使用 Python 合规。
- 将 `formation_controller` 从 Python prototype 重构为 C++ `rclcpp` 节点，因为它已经承担 leader-follower 编队控制核心逻辑。
- 保持原 executable 名称 `formation_controller`、原 launch 文件、原 topic、原参数和 YAML 配置语义不变。
- 移除旧 Python package 入口，避免后续误用 Python 编队控制核心。

### 改了哪些文件

- `README.md`
- `docs/for_codex/MY_Log.md`
- `ros2_ws/src/formation_controller/CMakeLists.txt`
- `ros2_ws/src/formation_controller/package.xml`
- `ros2_ws/src/formation_controller/src/formation_controller_node.cpp`
- 删除旧 Python 入口

### 验证结果

- `colcon build --packages-select formation_controller` 成功。
- 全工作空间 `colcon build` 成功，5 个包全部通过。
- 安装产物确认是 ELF C++ executable。
- `ros2 launch formation_controller swarm_mock.launch.py --show-args` 正常。
- 使用独立 `ROS_DOMAIN_ID=89` 短跑 `swarm_mock.launch.py formation_type:=column` 成功，C++ 控制器启动并推进 leader 航点。

### 下一步做什么

- 后续真实 PX4 通信、Offboard 控制和高频安全逻辑继续优先使用 C++。

---

## 2026-05-08（leader-follower 编队控制 v1）

### 本次做了什么

- 实现 `formation_controller` 第一版三机 leader-follower 编队控制。
- 支持 `formation_type=triangle|line|column`，队形 offset 从 `config/formations.yaml` 读取。
- 新增 `config/waypoints.yaml`，leader 从该文件读取航点，按顺序飞行并在到达后切换下一个航点。
- follower 根据 leader 的 `DroneState.position` 加固定 ENU offset 生成 `FormationTarget`。
- 增加安全门控：最大速度限制、最大高度/最小高度限制、最小机间距检查、leader 状态丢失全队 hold、单机状态不健康或超时则对应飞机 hold。
- 明确上层控制坐标统一为 `local_enu`，PX4 `local_ned` 转换由 bridge 层负责。

### 改了哪些文件

- `README.md`
- `config/formations.yaml`
- `config/waypoints.yaml`
- `docs/for_codex/MY_Log.md`
- `ros2_ws/src/formation_controller/launch/swarm_mock.launch.py`
- `ros2_ws/src/formation_controller/launch/swarm_px4_uxrce.launch.py`
- `ros2_ws/src/formation_controller/src/formation_controller_node.cpp`

### 验证结果

- `colcon build` 通过。
- `ros2 launch ... --show-args` 验证 launch 参数正常。
- mock 短跑通过。

### 下一步做什么

- 继续用 uXRCE-DDS backend 验证 `triangle`、`line`、`column` 三种队形。

---

## 2026-05-08（uXRCE-DDS C++ backend 骨架）

### 本次做了什么

- 新增 C++ 包 `px4_bridge_uxrce`，作为 uXRCE-DDS / `px4_msgs` 后端骨架。
- `px4_bridge_uxrce` 订阅 PX4 `/px4_N/fmu/out/vehicle_local_position`、`vehicle_status`、`timesync_status`。
- 将 PX4 NED 状态转换为项目内部 local ENU `DroneState`，并发布到 `/uav_N/state`。
- 提供 `connect/arm/takeoff/land/goto/hold/rtl` service。
- 为 Offboard 控制发布 `OffboardControlMode` 和 `TrajectorySetpoint`。
- 新增 `swarm_px4_uxrce.launch.py`，可启动三机 uXRCE-DDS bridge、`swarm_manager` 和 `formation_controller`。
- 更新 `setup_env.sh`，自动 source `/home/jie/px4_ros_com_ws/install` 以引入 `px4_msgs`，不复制外部依赖。

### 改了哪些文件

- `README.md`
- `config/swarm.yaml`
- `scripts/setup_env.sh`
- `docs/for_codex/MY_Log.md`
- `ros2_ws/src/px4_bridge_uxrce/package.xml`
- `ros2_ws/src/px4_bridge_uxrce/CMakeLists.txt`
- `ros2_ws/src/px4_bridge_uxrce/resource/px4_bridge_uxrce`
- `ros2_ws/src/px4_bridge_uxrce/src/px4_uxrce_bridge_node.cpp`
- `ros2_ws/src/formation_controller/package.xml`
- `ros2_ws/src/formation_controller/launch/swarm_px4_uxrce.launch.py`

### 验证结果

- source `scripts/setup_env.sh` 后运行 `colcon build`，相关包构建成功。
- 启动 `MicroXRCEAgent udp4 -p 8888` 后，ROS2 能看到 `/px4_1/fmu/out/*`、`/px4_2/fmu/out/*`、`/px4_3/fmu/out/*`。
- 验证 `/px4_1/fmu/out/vehicle_local_position --once` 可读。
- 临时运行单机 `px4_bridge_uxrce`，验证状态话题能输出健康 `DroneState`。

### 下一步做什么

- 切换到 uXRCE-DDS launch 时，应先启动 `MicroXRCEAgent`，再启动 PX4 SITL，再启动 `swarm_px4_uxrce.launch.py`。
- 重点继续验证 uXRCE-DDS `arm/takeoff/land/goto/offboard` 时序，再扩展到三机编队。

---

## 2026-05-08（Phase 1 ROS2 mock 工程骨架）

### 本次做了什么

- 创建了 `ros2_ws/src/swarm_msgs` ROS2 消息包。
- 创建了 `ros2_ws/src/px4_bridge`，当前保留 mock backend，不连接 PX4 真机。
- 创建了 `ros2_ws/src/swarm_manager`，读取无人机配置并周期发布 `/swarm/state`。
- 创建了 `ros2_ws/src/formation_controller`，实现最小 leader-follower 三机三角队形控制。
- 新增默认配置和 mock launch。
- 保持 Phase 1 边界：只实现 mock 架构，不依赖 PX4 真机、不引入 ROS1、不复制 references 代码。

### 改了哪些文件

- `README.md`
- `config/swarm.yaml`
- `config/formations.yaml`
- `ros2_ws/src/swarm_msgs`
- `ros2_ws/src/px4_bridge`
- `ros2_ws/src/swarm_manager`
- `ros2_ws/src/formation_controller`
- `docs/for_codex/MY_Log.md`

### 验证结果

- `colcon build` 成功。
- 短跑 `ros2 launch formation_controller swarm_mock.launch.py`，节点可启动。
- `/swarm/state` 能输出 3 架无人机状态。
- 构建时发现当前 WSL2 默认 `python3` 指向 Anaconda，会干扰 ROS2 Humble 消息生成；后续统一使用 `/usr/bin/python3`。

### 下一步做什么

- 进入真实 PX4 前，先完成单机 SITL 通信验证。

---

## 2026-05-08（参考分析与架构文档）

### 本次做了什么

- 重新阅读并确认仓库内 `AGENTS.md` 的固定环境、版本、ROS2、PX4、WSL2、QGC 和安全约束。
- 阅读 `references/` 下参考项目的顶层说明与关键包结构。
- 新增 `docs/reference_analysis.md`，系统分析每个参考项目的定位、价值、可借鉴架构和不适合直接使用的部分。
- 新增 `docs/architecture.md`，设计项目 `ROS2 + PX4` 主线架构，覆盖 namespace、多机状态共享、leader-follower、任务分配、覆盖搜索、感知扩展、地面站边界和 WSL2/QGC 网络边界。
- 新增 `docs/roadmap.md`，按 `Phase 0` 到 `Phase 7` 给出分阶段开发路线、交付物、验收标准和风险点。

### 改了哪些文件

- `docs/reference_analysis.md`
- `docs/architecture.md`
- `docs/roadmap.md`
- `docs/for_codex/MY_Log.md`

### 下一步做什么

- 后续实现时，优先从单机通信、状态归一化、多机 namespace、`swarm_manager` 状态汇总这些主线能力开始。
