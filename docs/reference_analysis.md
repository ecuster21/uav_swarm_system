# 参考项目分析

## 目的与范围

本文档用于分析 `references/` 目录中的 6 个参考项目，并明确它们对本项目
`ROS2 + PX4` 无人机集群系统的参考价值与边界。

分析结论必须服从本项目固定约束：

- 主线必须是 `ROS2 Humble + PX4 v1.14.4 + Gazebo Classic 11.10.2 + MicroXRCEAgent 2.4.1`
- 主项目禁止依赖 ROS1 运行时
- QGroundControl 运行在 Windows 主机，不在 WSL2 内
- 后续主要部署目标是 `RK3588`，不应强依赖 `Jetson / TensorRT / DeepStream`

## 总览结论

| 项目 | 主要定位 | ROS 依赖 | 对 PX4 多机仿真适配度 | 对 RK3588 适配度 | 对真机部署参考价值 | 建议用途 |
|---|---|---|---|---|---|---|
| `navigation2` | ROS2 导航框架 | ROS2 | 间接适合 | 中 | 中 | 学架构，不直接搬导航栈 |
| `mavsdk_drone_show` | PX4 多机/GCS/任务系统 | 非 ROS 主线 | 高 | 中 | 高 | 学多机任务组织、GCS、任务生命周期 |
| `PythonRobotics` | 机器人算法样例库 | 无 ROS 依赖 | 间接适合 | 中 | 中 | 学规划/覆盖/控制算法原型 |
| `swarmSim` | ROS1 集群仿真平台 | ROS1 | 低 | 低 | 低 | 只学接口和交互思路 |
| `PX4_Swarm_Controller` | ROS2 + PX4 多机编队控制 | ROS2 | 很高 | 中 | 中 | 第一阶段最重要参考 |
| `aerial-autonomy-stack` | ROS2 多机全栈仿真与部署 | ROS2 | 高，但与当前版本不兼容 | 低到中 | 高 | 学分层架构、部署分层、状态共享 |

## 1. navigation2

### 它主要做什么

`navigation2` 是 ROS2 机器人导航框架，核心关注点不是无人机集群，而是：

- 生命周期节点管理
- 行为树任务编排
- 插件化导航服务器
- 路径规划、控制、平滑、行为恢复
- waypoint 跟随与任务执行器

从仓库结构看，它是一个大型 ROS2 多包系统，包含：

- `nav2_lifecycle_manager`
- `nav2_bt_navigator`
- `nav2_core`
- `nav2_waypoint_follower`
- 各类 planner / controller / behavior / costmap 包

### 对我们项目的价值

它对本项目的价值主要不是算法复用，而是工程方法复用：

- 可学习 `ROS2` 大型多包工程如何分层
- 可学习生命周期节点如何组织启动、暂停、恢复、关闭
- 可学习行为树如何管理复杂任务流
- 可学习插件接口如何隔离“系统框架”和“具体算法”
- 可学习 waypoint 任务如何从“导航能力”提升到“任务执行能力”

### 可借鉴的架构

- 使用 `Lifecycle Node` 管理关键节点启动顺序
- 使用“任务服务器 + 插件接口”分离框架与算法
- 使用行为树或任务状态机协调复杂任务
- 使用 action/service/topic 明确长时任务、短时请求和流式状态的边界
- 使用独立 bringup、参数文件和测试组织方式管理系统复杂度

### 不适合直接使用的代码或依赖

- `navigation2` 不是 PX4 多机无人机框架，不能直接作为飞行集群主控
- `nav2_bringup` 的仿真依赖偏向 `ros_gz_sim` / `ros_gz_bridge`，并非 `Gazebo Classic 11.10.2`
- 其中大量组件默认围绕地面机器人、地图、costmap、AMCL 展开
- 其 planner/controller 语义与多旋翼 `NED/ENU`、offboard setpoint 流不一致

### ROS1 还是 ROS2

- `ROS2`

### 是否适合 PX4 多机仿真

- 不直接适合
- 但非常适合借鉴“多节点系统如何组织”的方法

### 是否适合 RK3588 部署

- 作为架构思想适合
- 作为整套依赖不适合直接搬到 RK3588 上

### 是否适合真机部署

- 可借鉴其生命周期管理和任务组织方式
- 不能直接作为 PX4 集群飞行控制栈使用

### 结论

`navigation2` 不是我们的飞控或集群控制参考主线，但它是本项目最重要的
`ROS2 工程架构参考` 之一。

---

## 2. mavsdk_drone_show

### 它主要做什么

`mavsdk_drone_show` 是围绕 `PX4 + MAVSDK` 构建的多机任务系统，仓库中同时包含：

- 无人机侧运行逻辑
- GCS / backend 服务
- React 仪表盘
- 多机 SITL 工具
- Smart Swarm 编队运行模式
- QuickScout 搜索任务模式

从仓库内容看，它已经覆盖：

- 多机 SITL
- leader-follower 风格的 Smart Swarm
- 区域覆盖搜索任务
- 地面站可视化与任务操作
- 日志与任务状态管理

### 对我们项目的价值

它对我们的价值很高，特别适合借鉴：

- 多无人机任务组织方式
- 任务生命周期与任务状态持久化
- 地面站到无人机的任务下发模型
- 区域覆盖搜索任务的操作流
- 编队、任务、日志、监控如何统一到一个任务系统里

### 可借鉴的架构

- 将“任务规划”“任务下发”“任务执行监控”分成独立层次
- 使用统一命令生命周期跟踪长时任务
- 将编队模式与覆盖搜索模式建模成不同 mission mode
- 将地面站视图、任务存储、执行状态拆分为独立模块
- 将 leader 状态传输、任务计划、进度反馈做成明确的数据通道

### 不适合直接使用的代码或依赖

- 它不是 `ROS2` 主线，而是 `MAVSDK + Python + FastAPI + React` 主线
- `pyproject.toml` 要求 `Python >= 3.11`，而 `ROS2 Humble` 常见系统 Python 为 `3.10`
- 它的系统边界是“PX4 + MAVSDK 应用平台”，不是“ROS2 + PX4 机载自主系统”
- 其 GCS 与 web 后端体量较大，不适合作为当前阶段最小主线依赖
- 不能替代我们计划中的 `px4_bridge / swarm_manager / formation_controller`

### ROS1 还是 ROS2

- 不依赖 ROS1
- 也不是以 ROS2 作为主运行框架

### 是否适合 PX4 多机仿真

- 很适合
- 它本身就是 PX4 多机仿真与任务运行的重要参考

### 是否适合 RK3588 部署

- 部分适合
- 其多机任务组织思想、日志与任务模型值得借鉴
- 但整套后端/GCS 架构不应直接成为 RK3588 机载主线

### 是否适合真机部署

- 适合借鉴
- 但真实部署前必须重新对齐我们的 `ROS2 + PX4` 主线、网络方式与安全逻辑

### 结论

`mavsdk_drone_show` 最值得学习的是：

- 多机任务系统设计
- GCS 与无人机运行逻辑的分层
- 编队与搜索任务的任务化表达

不应直接把它当作本项目的主工程框架。

---

## 3. PythonRobotics

### 它主要做什么

`PythonRobotics` 是一个机器人算法样例库和教材，覆盖：

- 路径规划
- 覆盖搜索
- 轨迹跟踪
- 控制算法
- SLAM / Mapping / Localization
- 少量 aerial navigation 示例

其特点是：

- 代码易读
- 依赖少
- 适合算法原型验证

### 对我们项目的价值

它对本项目最大的价值在算法层：

- 区域覆盖搜索算法原型
- 路径规划与航迹生成
- 控制算法快速验证
- 从二维任务规划快速过渡到多机任务划分原型

### 可借鉴的架构

它不提供完整系统架构，但可借鉴：

- 算法模块独立组织
- 输入输出简单清晰
- 原型算法先独立验证，再嵌入系统

### 不适合直接使用的代码或依赖

- 它不是 ROS 项目，也不是 PX4 项目
- 大量算法是教学/演示实现，不是可直接上真机的工程实现
- 很多算法默认二维地面机器人语义，需要改造成三维空域、NED 坐标和无人机约束
- 不能直接作为多机任务系统或飞行控制系统

### ROS1 还是 ROS2

- 都不是

### 是否适合 PX4 多机仿真

- 不直接适合
- 但很适合为 `coverage_planner` 和 `task_allocator` 提供算法原型来源

### 是否适合 RK3588 部署

- 算法思想适合
- 教学代码本身不适合作为直接部署代码

### 是否适合真机部署

- 算法思想适合做前置研究
- 需要重新工程化、参数化和安全验证后才能进入真机链路

### 结论

`PythonRobotics` 是 `算法参考库`，不是 `系统参考库`。它对 `Phase 5`
的区域覆盖搜索和任务规划价值很高。

---

## 4. swarmSim

### 它主要做什么

`swarmSim` 是一个轻量级集群仿真平台，仓库内容显示其包含：

- `crazyswarm_ros`
- `swarm_brain`
- `swarm_view`
- `wheelswarm_ros`

支持对象既有无人机，也有轮式机器人，偏重：

- 集群交互
- 高级控制指令
- 仿真可视化
- ROS 话题接口设计

### 对我们项目的价值

它对我们的价值主要在“接口设计和交互方式”层面：

- 多机器人 namespace / topic 组织方式
- 高级控制指令的抽象方式
- 集群仿真展示和 demo 组织方式

### 可借鉴的架构

- 将“控制脑”和“仿真接口”分离
- 多机器人命名空间按个体区分
- 高级命令与低级控制命令分层表达

### 不适合直接使用的代码或依赖

- 它明确依赖 `ROS1 Noetic`
- `package.xml` 中使用了 `catkin`、`roscpp`、`rospy`
- 启动方式依赖 `roscore`、`roslaunch`、`catkin_make`
- 仿真对象偏 `Crazyflie / wheel swarm`，不是 `PX4 + Gazebo Classic`
- 与本项目主线的 `ROS2 Humble + PX4 v1.14.4` 不兼容

### ROS1 还是 ROS2

- `ROS1`

### 是否适合 PX4 多机仿真

- 不适合直接使用
- 只能作为“集群接口设计”参考

### 是否适合 RK3588 部署

- 不适合直接使用

### 是否适合真机部署

- 不适合作为真机主线依赖

### 结论

`swarmSim` 的参考方式必须严格限制为：

- 学 topic / command 设计
- 学 demo 组织
- 不迁移 ROS1 运行方式

---

## 5. PX4_Swarm_Controller

### 它主要做什么

`PX4_Swarm_Controller` 是本批参考项目中与我们当前目标最贴近的项目。

它直接面向：

- `ROS2 Humble`
- `PX4`
- 多机 SITL
- leader-follower 编队控制
- 邻居拓扑
- offboard 控制

从仓库内容看，它包含：

- `launch/launch_simulation.py`
- `swarm_config.json`
- `control_config.json`
- `custom_msgs`
- `WeightedTopologyNeighbors`
- `WeightedTopologyController`
- `arming` 与 `waypoint` 节点

### 对我们项目的价值

它对我们的价值非常高，尤其在前几个阶段：

- 单机到多机 PX4 SITL 的组织方式
- 多机 namespace 设计
- leader/follower 的角色拆分
- 邻居拓扑与编队控制节点的拆分
- PX4 offboard 控制启动顺序
- ROS2 多节点如何围绕 `px4_msgs` 组织

### 可借鉴的架构

- 每架无人机独立 namespace
- 将 leader 航迹源与 follower 控制器解耦
- 单独设置邻居计算节点
- 使用配置文件描述 swarm 拓扑、初始位置、leader 标记
- 将仿真启动、arming、waypoint、controller 明确拆成多个 ROS2 节点

### 不适合直接使用的代码或依赖

- README 中采用“覆盖 PX4 原始 `sitl_multiple_run.sh`”的方式，不适合直接照搬
- 其 README 默认按外部仓库使用方式 clone 和改动 PX4 工具，不符合我们“主项目不复制外部依赖”的规则
- 控制器当前主要围绕论文中的 weighted topology 编队控制，功能面较窄
- 缺少任务分配、覆盖搜索、感知、部署、地面站等更大系统能力
- 仓库更偏“仿真控制实验平台”，不是全栈工程

### ROS1 还是 ROS2

- `ROS2`

### 是否适合 PX4 多机仿真

- 非常适合
- 是我们 `Phase 1` 到 `Phase 4` 的首要参考项目

### 是否适合 RK3588 部署

- 部分适合
- 架构和控制节点拆分方式可以迁移
- 但还缺少部署脚本、资源管理、设备驱动和机载运行约束

### 是否适合真机部署

- 具有参考价值
- 但必须补齐安全状态机、丢链保护、日志、任务管理与真机通信策略后才能进入真机阶段

### 结论

`PX4_Swarm_Controller` 是本项目当前阶段最重要的直接参考项目。

本项目应重点吸收它的：

- 多机 ROS2 + PX4 架构方式
- namespace 模式
- 编队控制角色分离
- 邻居拓扑和配置驱动思想

同时避免直接复用其“修改 PX4 外部仓库脚本”的做法。

---

## 6. aerial-autonomy-stack

### 它主要做什么

`aerial-autonomy-stack` 是一个更完整的 ROS2 多机空中自主系统，覆盖：

- PX4 / ArduPilot 双自驾仪支持
- 多机仿真
- Ground / Aircraft / Simulation 分层
- YOLO 与 LiDAR 感知
- state sharing
- ground system
- Docker 化仿真与部署
- Jetson / Orin 真机部署

### 对我们项目的价值

它对我们的价值非常大，但主要体现在“系统分层”与“全栈边界设计”：

- 仿真层、机载层、地面层分离
- autopilot interface 抽象
- mission orchestrator 设计
- state sharing 单独成包
- 感知、控制、任务、通信模块分层
- WSL / 容器 / 真机部署的组织方式

### 可借鉴的架构

- `simulation / aircraft / ground` 三层拆分
- 将 autopilot interface 抽成统一控制接口
- 将 mission、offboard_control、state_sharing 分离
- 将感知模块与飞控接口之间保持清晰边界
- 将地面系统与机载系统解耦

### 不适合直接使用的代码或依赖

- 依赖版本与我们主线差异很大：
  - `PX4 1.16.2`
  - `Gazebo Sim Harmonic`
  - `Ubuntu 24.04/22.04`
- 强依赖 NVIDIA 生态：
  - `CUDA`
  - `cuDNN`
  - `JetPack`
  - `DeepStream`
  - `nvidia-driver-580`
- 同时兼容 `ArduPilot` 与 `MAVROS`，而我们主线应保持 `ROS2 + PX4 + MicroXRCEAgent`
- 其仿真基础是新 Gazebo，而不是 `Gazebo Classic 11.10.2`

### ROS1 还是 ROS2

- `ROS2`

### 是否适合 PX4 多机仿真

- 适合借鉴
- 但不适合直接在我们当前固定版本上照搬

### 是否适合 RK3588 部署

- 结构上适合
- 依赖上不适合
- 尤其不能直接照搬其 `Jetson / DeepStream / TensorRT` 路径

### 是否适合真机部署

- 很适合提供部署分层思路
- 但必须改造成适合 `RK3588 + PX4 v1.14.4 + ROS2 Humble` 的版本

### 结论

`aerial-autonomy-stack` 应作为我们的 `全栈分层架构参考`，而不是版本和依赖参考。

最值得吸收的是：

- simulation / aircraft / ground 分层
- autopilot_interface / mission / state_sharing 拆分
- 真机部署时机载与地面系统边界

最需要避免的是：

- 新 Gazebo 版本依赖
- Jetson/NVIDIA 专有软件栈绑定
- ArduPilot/MAVROS 双主线引入

## 综合建议

### 直接参考优先级

1. `PX4_Swarm_Controller`
   - 用于 `Phase 1` 到 `Phase 4`
   - 重点学多机 SITL、offboard、namespace、leader-follower

2. `navigation2`
   - 用于系统架构设计
   - 重点学生命周期、行为树、插件化、action 组织

3. `aerial-autonomy-stack`
   - 用于全栈分层和部署架构设计
   - 重点学 aircraft/ground/simulation 分层与状态共享

4. `mavsdk_drone_show`
   - 用于地面站、多机任务组织、任务状态跟踪、覆盖搜索业务流设计

5. `PythonRobotics`
   - 用于覆盖搜索与路径规划算法原型

6. `swarmSim`
   - 仅作 ROS1 参考，不进入主线

### 不应直接迁移的内容

- 任何 ROS1 运行方式
- 任何要求升级到 `PX4 v1.15/v1.16` 的代码路径
- 任何要求切换到 `Gazebo Harmonic/Ignition/Garden` 的仿真路径
- 任何强依赖 `Jetson/TensorRT/DeepStream` 的感知部署链
- 任何直接覆盖 `PX4-Autopilot` 外部仓库脚本的开发方式

### 我们自己的参考吸收策略

- `PX4_Swarm_Controller` 提供近场架构参考
- `navigation2` 提供 ROS2 工程组织参考
- `aerial-autonomy-stack` 提供系统分层参考
- `mavsdk_drone_show` 提供任务系统与地面站参考
- `PythonRobotics` 提供算法原型参考
- `swarmSim` 仅提供 ROS1 设计启发，不进入主线
