# MY Log

## 使用规则

本文件用于记录阶段性进度。每次任务收尾时，至少补充以下内容：

- 这次做了什么
- 改了哪些文件
- 下一步做什么

建议按时间倒序追加，最新记录放在最上方。

---

## 2026-05-08（uXRCE 三机实飞闭环排查）

### 本次做了什么

- 按用户要求实际启动并检查完整链路：MicroXRCEAgent、PX4 v1.14.4 三机 Gazebo Classic SITL、ROS2 uXRCE bridge、`swarm_manager`、C++ `formation_controller`。
- 发现一次启动失败的直接原因：ROS2 launch 终端只 source 了本项目 install，没有 source `scripts/setup_env.sh`，导致 `px4_bridge_uxrce` 找不到 `libpx4_msgs__rosidl_typesupport_cpp.so`。
- 按固定环境重新 source `/home/jie/px4_ros_com_ws/install` 和本项目 install，并重建 `px4_bridge_uxrce` 等包。
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

- 回答并检查 `formation_controller` 从 Python 包迁移到 C++ 包后的残留问题。
- 确认源码层面不应保留旧 Python 包入口，包括 `setup.py`、`setup.cfg`、Python module、ament_python resource。
- 删除两个迁移后遗留的空目录：
  - `ros2_ws/src/formation_controller/formation_controller`
  - `ros2_ws/src/formation_controller/resource`
- 确认当前 `formation_controller` 包结构为标准 C++ ROS2 包：`package.xml`、`CMakeLists.txt`、`src/formation_controller_node.cpp`、`launch/`。

### 改了哪些文件

- 删除空目录 `ros2_ws/src/formation_controller/formation_controller`
- 删除空目录 `ros2_ws/src/formation_controller/resource`
- 更新 `docs/for_codex/MY_Log.md`

### 验证结果

- `rg` 检查 `ros2_ws/src/formation_controller` 中已无 `ament_python`、`rclpy`、`setup.py`、`setup.cfg`、`console_scripts`、旧 Python 节点入口。
- `ros2 pkg executables formation_controller` 仍显示 `formation_controller formation_controller`。
- `file ros2_ws/install/formation_controller/lib/formation_controller/formation_controller` 确认为 ELF C++ executable。
- `git diff --check` 通过。

### 下一步做什么

- 如果后续再从 Python 包迁移为 C++ 包，应同步清理源码、build、install 中的旧入口；必要时执行 `rm -rf build/<pkg> install/<pkg>` 后重新 `colcon build`。

---

## 2026-05-08（删除指定语言检查文档）

### 本次做了什么

- 按用户要求删除指定语言检查文档。
- 清理 `README.md` 中对该文件的引用。
- 记录新约定：以后不要再创建或恢复该文档。

### 改了哪些文件

- `README.md`
- 删除指定语言检查文档
- `docs/for_codex/MY_Log.md`

### 下一步做什么

- 后续语言规则相关内容只遵守 `AGENTS.md`，必要摘要写入 `README.md` 或本日志，不再单独创建独立语言检查文档。

---

## 2026-05-08（简化 Codex 协作文档）

### 本次做了什么

- 按用户要求删除 `docs/for_codex/MY_README.md` 和 `docs/for_codex/MY_Memory.md`。
- 后续不再维护这两个文件。
- 后续协作信息只更新：
  - 项目根目录 `README.md`
  - `docs/for_codex/MY_Log.md`

### 改了哪些文件

- 删除 `docs/for_codex/MY_README.md`
- 删除 `docs/for_codex/MY_Memory.md`
- 更新 `docs/for_codex/MY_Log.md`

### 下一步做什么

- 后续任务收尾只更新 `README.md` 和 `docs/for_codex/MY_Log.md` 中确有必要的内容。

---

## 2026-05-08（README 规范化整理）

### 本次做了什么

- 按用户要求重写根目录 `README.md`，删除历史排错流水、重复说明和影响阅读的临时细节。
- 将 README 调整为项目入口文档，按当前状态、固定环境、目录结构、构建、mock 运行、PX4 uXRCE-DDS 运行、MAVSDK 验证层、核心话题、编队控制、配置文件、QGC 和开发规则组织。
- 保留必要的安全提醒：不要重复启动多套 swarm launch、降落使用 `swarm_land_all.sh`、真实 PX4 主线优先 C++ uXRCE-DDS。

### 改了哪些文件

- `README.md`
- `docs/for_codex/MY_Log.md`

### 验证结果

- `git diff --check` 通过。
- 手动检查 README 标题结构和关键命令，确认没有继续保留旧的长篇排错过程。

### 下一步做什么

- 如果后续新增 `docs/wsl2_network.md` 或正式 QGC 转发脚本，再在 README 中加一条链接，不把长篇网络排查塞回 README。

---

## 2026-05-08（formation_controller C++ 重构）

### 本次做了什么

- 按 `AGENTS.md` 的 Python / C++ 语言规则重新检查当前 ROS2 包。
- 判断 `swarm_manager` 属于低频状态管理，继续使用 Python 合规。
- 判断 Python `px4_bridge` 仍只作为 mock / MAVSDK 快速验证层保留；真实 PX4 主线继续使用 C++ `px4_bridge_uxrce`。
- 将 `formation_controller` 从 Python prototype 重构为 C++ `rclcpp` 节点，因为它已经承担 leader-follower 编队控制核心逻辑。
- 保持原 executable 名称 `formation_controller`、原 launch 文件、原 topic、原参数和 YAML 配置语义不变。
- 移除旧 Python package 入口，避免后续误用 Python 编队控制核心。

### 改了哪些文件

- `README.md`
- `docs/for_codex/MY_README.md`
- `docs/for_codex/MY_Memory.md`
- `docs/for_codex/MY_Log.md`
- `ros2_ws/src/formation_controller/CMakeLists.txt`
- `ros2_ws/src/formation_controller/package.xml`
- `ros2_ws/src/formation_controller/src/formation_controller_node.cpp`
- 删除旧 Python 入口：
  - `ros2_ws/src/formation_controller/setup.py`
  - `ros2_ws/src/formation_controller/setup.cfg`
  - `ros2_ws/src/formation_controller/resource/formation_controller`
  - `ros2_ws/src/formation_controller/formation_controller/__init__.py`
  - `ros2_ws/src/formation_controller/formation_controller/config.py`
  - `ros2_ws/src/formation_controller/formation_controller/formation_controller_node.py`

### 验证结果

- `colcon build --packages-select formation_controller` 成功。
- 全工作空间 `colcon build` 成功，5 个包全部通过。
- `file ros2_ws/install/formation_controller/lib/formation_controller/formation_controller` 确认安装产物是 ELF C++ executable。
- `ros2 launch formation_controller swarm_mock.launch.py --show-args` 正常。
- 使用独立 `ROS_DOMAIN_ID=89` 短跑 `swarm_mock.launch.py formation_type:=column` 成功，C++ 控制器启动并推进 leader 航点。

### 下一步做什么

- 后续如果要把 MAVSDK backend 从验证层提升为长期通信后端，需要另做 C++ MAVSDK backend 设计，并确认本机是否具备 MAVSDK C++ SDK。
- 可继续补 `stop_ros2_swarm.sh`，避免重复 ROS2 控制节点残留。

---

## 2026-05-08（编队仿真排查与稳定性修复）

### 本次做了什么

- 按用户反馈接管当前仿真，排查“飞机没有动静”。
- 发现最初有两类问题：
  - 残留了重复的 ROS2 控制节点，同名 `/swarm/formation_controller`、`/swarm/manager` 和 `/uav_N/px4_bridge` 会互相干扰。
  - 三机初始 `healthy=true` 但 `armed=false`，只启动 launch 不会自动起飞，必须显式调用 arm/takeoff。
- 清理重复节点后，启动干净的 `swarm_px4_uxrce.launch.py formation_type:=triangle` 并执行三机 arm/takeoff。
- 确认 uXRCE-DDS 链路可进入 `OFFBOARD`，`TrajectorySetpoint` 正常发布。
- 发现第一版 `control_rate_hz=10Hz` 会让速度限幅后的 setpoint 每次只前移约 `0.15m`，PX4 位置控制表现为低幅慢动。
- 将 `config/formations.yaml` 中 `control_rate_hz` 调为 `2Hz`，保持 `max_speed_m_s=1.5` 的同时让 setpoint 前视距离约 `0.75m`。
- 发现安全 hold 原先发布 `active=false`，会导致 PX4 `offboard_control_signal_lost` 并触发 failsafe。
- 修改 `formation_controller`：限距/leader 丢失等普通 hold 在本机状态健康时发布当前位置 active hold setpoint；只有本机 PX4 状态不健康/丢失时才发布 inactive target。
- 修正 `scripts/swarm_land_all.sh`：降落前先停止 autonomous `formation_controller`，发布 inactive target，再调用 `/land` service。
- 重新启动 PX4/Gazebo、ROS2 uXRCE launch 并测试三角编队。

### 改了哪些文件

- `README.md`
- `config/formations.yaml`
- `docs/for_codex/MY_Log.md`
- `docs/for_codex/MY_Memory.md`
- `ros2_ws/src/formation_controller/formation_controller/formation_controller_node.py`
- `scripts/swarm_land_all.sh`

### 验证结果

- `colcon build --packages-select formation_controller` 成功。
- `bash -n scripts/swarm_land_all.sh` 成功。
- 干净重启后，三机成功进入 `armed=true`、`flight_mode=OFFBOARD`、`healthy=true`。
- 采样显示 leader 沿方形航点运动，两个 follower 以三角 offset 跟随：
  - 示例 1：leader 约 `(4.00, 3.80, 2.11)`，uav_2 约 `(2.06, 1.11, 2.24)`，uav_3 约 `(2.08, 4.19, 2.27)`。
  - 示例 2：leader 约 `(0.33, 0.05, 2.00)`，uav_2 约 `(-1.93, -0.98, 2.02)`，uav_3 约 `(-1.94, 1.97, 2.01)`。
- 连续 6 次采样中三机均保持 `OFFBOARD` 和 `healthy=true`，未再出现 `offboard_control_signal_lost`。
- 测试结束后已停止 ROS2 控制节点和 PX4/Gazebo SITL，当前只保留 MicroXRCEAgent。

### 下一步做什么

- 可继续增加一个正式的 `stop_ros2_swarm.sh`，用于清理重复 ROS2 控制节点。
- 后续应完善编队 safety monitor，区分“保持 Offboard 的位置 hold”和“真正停止控制目标”的状态机。
- 可以继续测试 `formation_type:=line` 和 `formation_type:=column`。

---

## 2026-05-08（leader-follower 编队控制 v1）

### 本次做了什么

- 实现 `formation_controller` 第一版三机 leader-follower 编队控制。
- 支持 `formation_type=triangle|line|column`，队形 offset 从 `config/formations.yaml` 读取。
- 新增 `config/waypoints.yaml`，leader 从该文件读取航点，按顺序飞行并在到达后切换下一个航点。
- follower 根据 leader 的 `DroneState.position` 加固定 ENU offset 生成 `FormationTarget`。
- 增加安全门控：最大速度限制、最大高度/最小高度限制、最小机间距检查、leader 状态丢失全队 hold、单机状态不健康或超时则对应飞机 hold。
- 明确上层控制坐标统一为 `local_enu`，PX4 `local_ned` 转换由 bridge 层负责。
- 更新三个 launch，使 mock、MAVSDK 和 uXRCE-DDS 都支持 `formation_type` 和 `waypoints_config_file` 参数。
- 更新 README，补充队形切换、坐标系约定和仿真运行说明。

### 改了哪些文件

- `README.md`
- `config/formations.yaml`
- `config/waypoints.yaml`
- `docs/for_codex/MY_README.md`
- `docs/for_codex/MY_Memory.md`
- `docs/for_codex/MY_Log.md`
- `ros2_ws/src/formation_controller/setup.py`
- `ros2_ws/src/formation_controller/formation_controller/formation_controller_node.py`
- `ros2_ws/src/formation_controller/launch/swarm_mock.launch.py`
- `ros2_ws/src/formation_controller/launch/swarm_px4_sitl.launch.py`
- `ros2_ws/src/formation_controller/launch/swarm_px4_uxrce.launch.py`

### 验证结果

- `/usr/bin/python3 -m compileall -q ros2_ws/src/formation_controller/formation_controller` 通过。
- `colcon build` 通过，5 个包全部构建成功。
- `ros2 launch ... --show-args` 验证三个 launch 都包含 `formation_type` 和 `waypoints_config_file`。
- 使用独立 `ROS_DOMAIN_ID=77` 短跑 `swarm_mock.launch.py formation_type:=line`，节点启动成功，控制器先 hold 等待状态，收到 `/swarm/state` 后恢复目标发布并切换 leader 航点。

### 下一步做什么

- 若继续飞行验证，建议优先用 uXRCE-DDS backend 单机确认 Offboard 状态机，再三机测试 `triangle`、`line`、`column`。
- 后续可补 C++ 版高频编队控制核心、yaw 旋转 offset、服务结果确认和更完整的 safety monitor。

---

## 2026-05-08（uXRCE-DDS 仿真测试修复）

### 本次做了什么

- 接管用户当前的 uXRCE-DDS 仿真环境，检查 MicroXRCEAgent、Gazebo、PX4、uXRCE bridge、ROS2 节点和话题。
- 确认 PX4 三机、MicroXRCEAgent 和 `/px4_N/fmu/out/*` telemetry 均正常。
- 发现 `/swarm/state` 中三机 `healthy=true` 但 `armed=false`，且 `ros2 service call` 一直等待。
- 定位根因：`px4_bridge_uxrce` 中 C++ `create_service(...)` 返回值没有保存，service server 被析构。
- 修复 C++ service 生命周期，将所有 `Trigger` service 保存到 `services_` 成员容器。
- 重新构建 `px4_bridge_uxrce` 和 `formation_controller`。
- 重启 ROS2 uXRCE launch，不重启 PX4/Gazebo/MicroXRCE。
- 验证 `/uav_1`、`/uav_2`、`/uav_3` 的 `connect/arm/takeoff/land/goto/hold/rtl` service 全部注册成功。
- 连续采样 `/swarm/state`，确认三机 `armed=true`、`flight_mode=OFFBOARD`、速度非零、位置持续变化。

### 改了哪些文件

- `ros2_ws/src/px4_bridge_uxrce/src/px4_uxrce_bridge_node.cpp`
- `docs/for_codex/MY_Memory.md`
- `docs/for_codex/MY_Log.md`

### 验证结果

- `colcon build --packages-select px4_bridge_uxrce formation_controller` 成功。
- `ros2 service list -t` 显示三机所有 uXRCE bridge service。
- `/swarm/state` 连续采样显示三机处于 `OFFBOARD`，且位置明显变化。

### 下一步做什么

- 如果需要停止当前仿真飞行，运行 `./scripts/swarm_land_all.sh`。
- 后续继续完善 uXRCE-DDS offboard 控制的安全状态机和 service 结果确认，不只返回“命令已发布”。

---

## 2026-05-08（uXRCE-DDS C++ backend 骨架）

### 本次做了什么

- 判断当前阶段需要补充 uXRCE-DDS，但不应直接替代 MAVSDK 演示链路。
- 新增 C++ 包 `px4_bridge_uxrce`，作为 uXRCE-DDS / `px4_msgs` 后端骨架。
- `px4_bridge_uxrce` 订阅 PX4 `/px4_N/fmu/out/vehicle_local_position`、`vehicle_status`、`timesync_status`。
- 将 PX4 NED 状态转换为项目内部 local ENU `DroneState`，并发布到 `/uav_N/state`。
- 提供 `connect/arm/takeoff/land/goto/hold/rtl` service，与 MAVSDK bridge 的上层接口对齐。
- 为 Offboard 控制发布 `OffboardControlMode` 和 `TrajectorySetpoint`，但保留为后续严格验证的 C++ 主线骨架。
- 新增 `swarm_px4_uxrce.launch.py`，可启动三机 uXRCE-DDS bridge、`swarm_manager` 和 `formation_controller`。
- 更新 `setup_env.sh`，自动 source `/home/jie/px4_ros_com_ws/install` 以引入 `px4_msgs`，不复制外部依赖。

### 改了哪些文件

- `README.md`
- `config/swarm.yaml`
- `scripts/setup_env.sh`
- `docs/for_codex/MY_README.md`
- `docs/for_codex/MY_Memory.md`
- `docs/for_codex/MY_Log.md`
- `ros2_ws/src/px4_bridge_uxrce/package.xml`
- `ros2_ws/src/px4_bridge_uxrce/CMakeLists.txt`
- `ros2_ws/src/px4_bridge_uxrce/resource/px4_bridge_uxrce`
- `ros2_ws/src/px4_bridge_uxrce/src/px4_uxrce_bridge_node.cpp`
- `ros2_ws/src/formation_controller/package.xml`
- `ros2_ws/src/formation_controller/setup.py`
- `ros2_ws/src/formation_controller/launch/swarm_px4_uxrce.launch.py`

### 验证结果

- 已 source `scripts/setup_env.sh` 后运行 `colcon build`，5 个包全部构建成功：
  - `swarm_msgs`
  - `px4_bridge`
  - `px4_bridge_uxrce`
  - `swarm_manager`
  - `formation_controller`
- 启动 `MicroXRCEAgent udp4 -p 8888` 后，ROS2 能看到 `/px4_1/fmu/out/*`、`/px4_2/fmu/out/*`、`/px4_3/fmu/out/*`。
- 已验证 `/px4_1/fmu/out/vehicle_local_position --once` 可读。
- 已临时运行单机 `px4_bridge_uxrce`，验证 `/uav_1_uxrce_test/state` 能输出健康 `DroneState`。

### 下一步做什么

- 后续若要切换到 uXRCE-DDS launch，应先启动 `MicroXRCEAgent`，再启动 PX4 SITL，再启动 `swarm_px4_uxrce.launch.py`。
- 下一步重点是单机验证 uXRCE-DDS `arm/takeoff/land/goto/offboard` 时序，再扩展到三机编队。
- MAVSDK 仍保留为当前阶段的快速演示和对照 backend。

---

## 2026-05-08（MAVSDK 三机起飞测试）

### 本次做了什么

- 在用户当前运行的 PX4 SITL + ROS2 launch 环境中检查进程、端口、ROS2 节点和话题。
- 确认 Gazebo、3 个 PX4 实例、3 个 MAVSDK server 均正常运行。
- 读取 `/swarm/state`，确认三机 health 已变为 `true`，但初始 `armed=false`，因此不会自动移动。
- 手动调用三架机的 `/arm` 和 `/takeoff` service，全部返回成功。
- 再次读取 `/swarm/state`，确认三机 `armed=true`，位置和速度持续变化，MAVSDK 控制链路可用。
- 增加便捷脚本，减少后续手动 service 调用。
- 改进 bridge 逻辑：MAVSDK backend 在未 armed 时不消费自主编队目标，只低频提示等待。
- 便捷脚本避免在 source ROS2 setup 前启用 `set -u`，防止 `AMENT_TRACE_SETUP_FILES` 未定义导致脚本退出。

### 改了哪些文件

- `README.md`
- `docs/for_codex/MY_Log.md`
- `ros2_ws/src/px4_bridge/px4_bridge/px4_bridge_node.py`
- `scripts/swarm_arm_takeoff.sh`
- `scripts/swarm_land_all.sh`
- `scripts/swarm_status_once.sh`

### 验证结果

- 已验证三机 `arm` 和 `takeoff` service 成功。
- 已验证 `/swarm/state` 中三机位置发生变化，说明无人机已在 SITL 中运动。

### 下一步做什么

- 重新 `colcon build` 使 bridge 逻辑更新进入 install。
- 以后启动 MAVSDK launch 后，使用 `./scripts/swarm_arm_takeoff.sh` 显式开始飞行测试。
- 需要停止时使用 `./scripts/swarm_land_all.sh`，需要查看状态时使用 `./scripts/swarm_status_once.sh`。

---

## 2026-05-08（Gazebo master 端口占用修正）

### 本次做了什么

- 根据用户运行日志确认 `Unable to start server[bind: Address already in use]` 是旧 `gzserver` 占用 Gazebo master 端口 `11345`。
- 强化 `scripts/stop_sitl_stack.sh`，除常规 `pkill` 外会检查并清理 `11345` 端口的 Gazebo owner。
- 如果清理后端口仍被占用，脚本会明确报错并退出，避免继续 spawn 到旧 Gazebo 中形成半启动状态。
- 更新 `scripts/start_px4_multi_sitl.sh`，启动前统一调用 `stop_sitl_stack.sh`。
- 更新 `README.md` 和 `MY_Memory.md`，记录端口占用和 `connection closed by client` 的处理方式。

### 改了哪些文件

- `README.md`
- `scripts/stop_sitl_stack.sh`
- `scripts/start_px4_multi_sitl.sh`
- `docs/for_codex/MY_Memory.md`
- `docs/for_codex/MY_Log.md`

### 下一步做什么

- 用户先停止当前混合状态的 PX4/Gazebo/ROS2 窗口，再运行 `./scripts/stop_sitl_stack.sh`。
- 重新运行 `./scripts/start_px4_multi_sitl.sh 3 iris`，确认不再出现 `Address already in use`。

---

## 2026-05-08（PX4 SITL 首次运行问题修正）

### 本次做了什么

- 根据实际运行日志确认 MAVSDK backend 已能连接 PX4 SITL。
- 修正 MAVSDK 连接字符串，将 deprecated 的 `udp://` 改为 `udpin://0.0.0.0:<port>`。
- 修正多机 MAVSDK embedded server 端口冲突：每架无人机使用独立 `mavsdk_server_port`。
- MAVSDK connect timeout 后会尝试清理本 backend 启动的 embedded `mavsdk_server`，避免残留进程继续占端口。
- 增加 MAVSDK telemetry rate 配置，默认 `5Hz`，降低 `User callback queue slow` 风险。
- 在 PX4 health 未 ready 时，`px4_bridge` 不再尝试消费编队目标，只低频提示等待状态，避免目标命令反复失败。
- 改进 `DroneState.status_text`，在 health 未 ready 时显示 `global/home` 具体状态。
- 更新 `scripts/setup_env.sh`，确保新终端优先使用 `/usr/bin/python3`，并 source ROS2 Humble 与当前 `ros2_ws/install`。
- 新增 `scripts/stop_sitl_stack.sh`，用于清理 PX4/Gazebo/MAVSDK 残留进程。
- 在 `README.md` 添加 `swarm_msgs/msg/DroneState is invalid` 的排查方法。

### 改了哪些文件

- `README.md`
- `config/swarm.yaml`
- `scripts/setup_env.sh`
- `scripts/start_px4_multi_sitl.sh`
- `scripts/stop_sitl_stack.sh`
- `docs/for_codex/MY_Memory.md`
- `docs/for_codex/MY_Log.md`
- `ros2_ws/src/px4_bridge/px4_bridge/mavsdk_backend.py`
- `ros2_ws/src/px4_bridge/px4_bridge/px4_bridge_node.py`
- `ros2_ws/src/formation_controller/launch/swarm_px4_sitl.launch.py`

### 验证结果

- 已重新 `colcon build`，4 个包构建成功。
- 已执行 `source scripts/setup_env.sh` 后验证：
  - `python3=/usr/bin/python3`
  - `ros2 interface show swarm_msgs/msg/DroneState` 正常
  - `import swarm_msgs.msg` 正常
- 已再次构建验证新增 `mavsdk_server_port` 参数与 launch 安装逻辑，构建通过。

### 下一步做什么

- 用户重新开一个终端后先执行 `source /home/jie/uav_swarm_system/scripts/setup_env.sh`，再运行 `ros2 topic echo`。
- 如果 PX4 health 长时间未 ready，下一步检查 PX4 SITL GPS/home position、Gazebo 是否正常运行，以及 MAVSDK telemetry 中 health 标志。

---

## 2026-05-08（PX4 SITL MAVSDK backend 接入）

### 本次做了什么

- 在保留 `mock` backend 的基础上，为 `px4_bridge` 新增统一 backend 接口和 `mavsdk` backend。
- MAVSDK backend 支持多 PX4 SITL 实例配置，提供 `connect`、`arm`、`takeoff`、`land`、`goto`、`hold`、`rtl`。
- 将 MAVSDK telemetry 转换为 `DroneState`，并将 `FormationTarget` 的 local ENU 目标转换为 MAVSDK `goto_location`。
- 增加控制前检查：未连接、MAVSDK 缺失、health 未 ready 或 PX4 拒绝命令时返回失败。
- 为 MAVSDK 目标下发增加节流和目标去重，避免 `formation_controller` 10Hz 目标导致重复 `goto_location` 或失败死循环。
- 更新三机配置，写入 namespace、system_id、MAVSDK UDP port、initial_position 和 backend 参数。
- 新增 PX4 SITL launch：`swarm_px4_sitl.launch.py`。
- 更新 `README.md`，写清 MAVSDK 依赖、MicroXRCEAgent、PX4 多机 SITL、ROS2 launch、QGC on Windows 注意事项和后续 uXRCE-DDS 方向。

### 改了哪些文件

- `README.md`
- `config/swarm.yaml`
- `config/formations.yaml`
- `docs/for_codex/MY_README.md`
- `docs/for_codex/MY_Memory.md`
- `docs/for_codex/MY_Log.md`
- `ros2_ws/src/px4_bridge/package.xml`
- `ros2_ws/src/px4_bridge/setup.py`
- `ros2_ws/src/px4_bridge/px4_bridge/backend_base.py`
- `ros2_ws/src/px4_bridge/px4_bridge/mock_backend.py`
- `ros2_ws/src/px4_bridge/px4_bridge/mavsdk_backend.py`
- `ros2_ws/src/px4_bridge/px4_bridge/mock_px4_bridge_node.py`
- `ros2_ws/src/px4_bridge/px4_bridge/px4_bridge_node.py`
- `ros2_ws/src/swarm_manager/swarm_manager/swarm_manager_node.py`
- `ros2_ws/src/formation_controller/package.xml`
- `ros2_ws/src/formation_controller/setup.py`
- `ros2_ws/src/formation_controller/formation_controller/formation_controller_node.py`
- `ros2_ws/src/formation_controller/launch/swarm_mock.launch.py`
- `ros2_ws/src/formation_controller/launch/swarm_px4_sitl.launch.py`

### 验证结果

- 已通过 `colcon build`，4 个 ROS2 包构建成功。
- 已短跑 `ros2 launch formation_controller swarm_mock.launch.py`，mock 多机节点可启动。
- 已短跑 `ros2 launch formation_controller swarm_px4_sitl.launch.py backend_type:=mavsdk`；当前 `/usr/bin/python3` 未安装 `mavsdk`，节点按预期提示缺少依赖并保持运行，不会因为缺包导致构建失败。
- MAVSDK 缺失场景下，目标失败告警已被节流，不再 10Hz 刷屏或死循环发命令。

### 下一步做什么

- 安装并确认 `/usr/bin/python3` 可导入 `mavsdk`。
- 启动 PX4 v1.14.4 三机 Gazebo Classic SITL，按 `14541/14542/14543` 验证 MAVSDK connect 和 telemetry。
- 逐步验证单机 `arm`、`takeoff`、`land`、`goto`，再验证三机 leader-follower。
- 设计 C++ uXRCE-DDS / `px4_msgs` backend，避免 Python MAVSDK 验证层变成长期高频飞控核心。

---

## 2026-05-08（编码语言规则检查）

### 本次做了什么

- 对照 `AGENTS.md` 中的 `Python / C++ 使用规则` 检查了 Phase 1 ROS2 mock 工程骨架。
- 确认当前 Python 实现只属于 Phase 1 mock/prototype 范围，不连接 PX4 真机、不发布真实 Offboard setpoint、不使用 `px4_msgs` 或 uXRCE-DDS。
- 将 mock 状态生成默认频率从 `20Hz` 收紧到 `10Hz`，更贴合低频 mock 验证范围。
- 更新 `README.md`、`MY_README.md` 和 `MY_Memory.md`，记录语言选择边界。

### 改了哪些文件

- `config/swarm.yaml`
- `ros2_ws/src/px4_bridge/px4_bridge/mock_px4_bridge_node.py`
- `README.md`
- `docs/for_codex/MY_README.md`
- `docs/for_codex/MY_Memory.md`
- `docs/for_codex/MY_Log.md`

### 当前判断

- Phase 1 当前使用 Python 符合 `AGENTS.md` 中“第一版可以用 Python 快速实现 mock 和算法原型”的规则。
- 需要严格保持边界：后续真实 PX4 通信、uXRCE-DDS、`px4_msgs`、高频 Offboard、真实编队闭环和真机部署应优先使用 C++。

### 下一步做什么

- 重新运行 `colcon build` 和 mock launch smoke test，确认频率调整和文档补充后工程仍正常。
- 后续进入真实 PX4 前，先设计 C++ backend 边界，不要直接把当前 Python mock 改成真实飞行链路。

---

## 2026-05-08（Phase 1 ROS2 mock 工程骨架）

### 本次做了什么

- 创建了 `ros2_ws/src/swarm_msgs` ROS2 消息包。
- 创建了 `ros2_ws/src/px4_bridge`，当前实现 mock backend，不连接 PX4 真机。
- 创建了 `ros2_ws/src/swarm_manager`，读取无人机配置并周期发布 `/swarm/state`。
- 创建了 `ros2_ws/src/formation_controller`，实现最小 leader-follower 三机三角队形控制。
- 新增默认配置：
  - `config/swarm.yaml`
  - `config/formations.yaml`
- 新增默认 launch：
  - `ros2_ws/src/formation_controller/launch/swarm_mock.launch.py`
- 更新根目录 `README.md`，补充 build、source、launch、话题说明和后续接 PX4 的路径。
- 保持 Phase 1 边界：只实现 mock 架构，不依赖 PX4 真机、不引入 ROS1、不复制 references 代码。

### 改了哪些文件

- `README.md`
- `config/swarm.yaml`
- `config/formations.yaml`
- `ros2_ws/src/swarm_msgs/package.xml`
- `ros2_ws/src/swarm_msgs/CMakeLists.txt`
- `ros2_ws/src/swarm_msgs/msg/DroneState.msg`
- `ros2_ws/src/swarm_msgs/msg/SwarmState.msg`
- `ros2_ws/src/swarm_msgs/msg/FormationTarget.msg`
- `ros2_ws/src/px4_bridge/package.xml`
- `ros2_ws/src/px4_bridge/setup.py`
- `ros2_ws/src/px4_bridge/setup.cfg`
- `ros2_ws/src/px4_bridge/resource/px4_bridge`
- `ros2_ws/src/px4_bridge/px4_bridge/__init__.py`
- `ros2_ws/src/px4_bridge/px4_bridge/mock_backend.py`
- `ros2_ws/src/px4_bridge/px4_bridge/mock_px4_bridge_node.py`
- `ros2_ws/src/swarm_manager/package.xml`
- `ros2_ws/src/swarm_manager/setup.py`
- `ros2_ws/src/swarm_manager/setup.cfg`
- `ros2_ws/src/swarm_manager/resource/swarm_manager`
- `ros2_ws/src/swarm_manager/swarm_manager/__init__.py`
- `ros2_ws/src/swarm_manager/swarm_manager/config.py`
- `ros2_ws/src/swarm_manager/swarm_manager/swarm_manager_node.py`
- `ros2_ws/src/formation_controller/package.xml`
- `ros2_ws/src/formation_controller/setup.py`
- `ros2_ws/src/formation_controller/setup.cfg`
- `ros2_ws/src/formation_controller/resource/formation_controller`
- `ros2_ws/src/formation_controller/formation_controller/__init__.py`
- `ros2_ws/src/formation_controller/formation_controller/config.py`
- `ros2_ws/src/formation_controller/formation_controller/formation_controller_node.py`
- `ros2_ws/src/formation_controller/launch/swarm_mock.launch.py`
- `docs/for_codex/MY_README.md`
- `docs/for_codex/MY_Log.md`
- `docs/for_codex/MY_Memory.md`

### 验证结果

- 已通过 `colcon build`，4 个包全部构建成功：
  - `swarm_msgs`
  - `px4_bridge`
  - `swarm_manager`
  - `formation_controller`
- 已短跑 `ros2 launch formation_controller swarm_mock.launch.py`，节点可启动。
- 已通过 `ros2 topic echo --once /swarm/state` 验证集群状态输出，包含 3 架无人机，leader 为 `uav_1`，两个 follower 健康状态正常。
- 构建时发现当前 WSL2 默认 `python3` 指向 Anaconda，会干扰 ROS2 Humble 消息生成；已在 README 和 `MY_Memory.md` 记录使用 `/usr/bin/python3` 的构建方式。

### 下一步做什么

- Phase 1 后续可继续补：
  - 单机 mock bridge 与未来 uXRCE-DDS backend 的接口边界文档
  - `/swarm/state` 的测试脚本或 launch smoke test
  - `px4_bridge` 的真实 PX4 v1.14.4 uXRCE-DDS 后端设计
- 进入真实 PX4 前，应先完成单机 SITL 通信验证。

---

## 2026-05-08（参考分析与架构文档）

### 本次做了什么

- 重新阅读并确认了仓库内 `AGENTS.md` 的固定环境、版本、ROS2、PX4、WSL2、QGC 和安全约束。
- 阅读了 `references/` 下以下参考项目的顶层说明与关键包结构：
  - `navigation2`
  - `mavsdk_drone_show`
  - `PythonRobotics`
  - `swarmSim`
  - `PX4_Swarm_Controller`
  - `aerial-autonomy-stack`
- 新增 `docs/reference_analysis.md`，系统分析了每个参考项目的定位、价值、可借鉴架构和不适合直接使用的部分。
- 新增 `docs/architecture.md`，设计了我们自己的 `ROS2 + PX4` 主线架构，覆盖 namespace、多机状态共享、leader-follower、任务分配、覆盖搜索、感知扩展、地面站边界和 WSL2/QGC 网络边界。
- 新增 `docs/roadmap.md`，按 `Phase 0` 到 `Phase 7` 给出了分阶段开发路线、交付物、验收标准和风险点。
- 本次严格只完成文档工作，没有继续实现代码。

### 改了哪些文件

- `docs/reference_analysis.md`
- `docs/architecture.md`
- `docs/roadmap.md`
- `docs/for_codex/MY_README.md`
- `docs/for_codex/MY_Log.md`

### 当前判断

- 参考项目的吸收优先级已经更清晰：
  - `PX4_Swarm_Controller` 最适合当前阶段的多机 SITL 与编队主线
  - `navigation2` 最适合借 ROS2 工程组织
  - `aerial-autonomy-stack` 最适合借全栈分层
  - `mavsdk_drone_show` 最适合借任务系统和地面站思路
  - `PythonRobotics` 最适合借覆盖搜索算法原型
  - `swarmSim` 只保留 ROS1 设计启发
- 项目主线已经明确为：
  - `ROS2 Humble + PX4 v1.14.4 + Gazebo Classic 11.10.2 + MicroXRCEAgent 2.4.1`
  - `WSL2 Ubuntu 22.04`
  - `QGroundControl on Windows Host`
- 在进入编码前，文档层面的主架构和路线图已经具备比较稳定的基础。

### 下一步做什么

- 等待用户确认这三份主文档是否符合预期。
- 若确认通过，优先进入 `Phase 0` 到 `Phase 1` 的环境验证与单机链路设计。
- 后续实现时，优先从：
  - 单机 `px4_bridge`
  - 单机状态归一化
  - 多机 namespace 规划
  - `swarm_manager` 基础状态汇总
  这些主线能力开始。

---

## 2026-05-08

### 本次做了什么

- 创建了 `docs/for_codex/` 目录。
- 创建了 `MY_README.md`，用于汇总项目要点、固定环境、核心约束和协作方式。
- 创建了 `MY_Log.md`，用于后续持续记录每次任务的进展。
- 创建了 `MY_Memory.md`，用于保存长期有效的规则和协作约定。

### 改了哪些文件

- `docs/for_codex/MY_README.md`
- `docs/for_codex/MY_Log.md`
- `docs/for_codex/MY_Memory.md`

### 当前判断

- 项目当前仍以环境约束明确、文档整理和主线架构搭建为主。
- `docs/for_codex/` 已经建立，可以作为后续 Codex 协作入口。

### 下一步做什么

- 后续每次任务结束时，持续更新本文件。
- 当项目目录、目标拆解或阶段重点发生变化时，更新 `MY_README.md`。
- 当形成新的长期规则时，更新 `MY_Memory.md`。
