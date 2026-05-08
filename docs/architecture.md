# 系统架构设计

## 1. 设计目标

本项目主线架构必须满足以下目标：

- 主线采用 `ROS2 Humble + PX4 v1.14.4`
- 仿真环境采用 `Gazebo Classic 11.10.2`
- PX4 与 ROS2 通信采用 `MicroXRCEAgent 2.4.1`
- 支持 PX4 多机 SITL
- 每架无人机使用独立 namespace
- 支持多机状态共享
- 支持 leader-follower 编队控制
- 为任务分配、区域覆盖搜索、目标检测、雷达感知和地面站预留扩展位
- 主线不依赖 ROS1
- 不强依赖 `Jetson / TensorRT / DeepStream`
- 后续可迁移到 `RK3588` 伴随计算机

## 2. 基本原则

### 2.1 职责边界

- `PX4` 负责：
  - 姿态控制
  - 位置控制
  - 模式管理
  - failsafe
  - 底层飞行安全

- `ROS2` 负责：
  - 多机状态管理
  - 编队控制
  - 任务分配
  - 覆盖搜索规划
  - 感知与目标信息处理
  - 地面站和任务系统对接

### 2.2 安全原则

- ROS2 只发送高层 setpoint、航点或任务指令，不替代 PX4 的底层安全逻辑
- 所有真机功能必须先在 SITL 验证
- 编队、任务、感知逻辑失效时，优先降级到 PX4 可处理的安全模式

### 2.3 工程原则

- 每架无人机独立 namespace
- 高层任务流与低层 PX4 接口解耦
- 单机控制逻辑与集群协同逻辑解耦
- 仿真层、机载层、地面层解耦
- 统一使用 ROS2 topic / service / action 表达不同类型交互

## 3. 总体分层架构

```text
Windows Host
└── QGroundControl
    └── 通过 MAVLink UDP 与 WSL2 中 PX4 SITL 通信

WSL2 Ubuntu 22.04
├── PX4 SITL x N
├── Gazebo Classic 11.10.2
├── MicroXRCEAgent 2.4.1
└── ROS2 Humble
    ├── 单机接口层
    │   └── px4_bridge (每机一个实例，独立 namespace)
    ├── 集群状态层
    │   └── swarm_manager
    ├── 任务编排层
    │   └── mission_manager
    ├── 编队控制层
    │   └── formation_controller
    ├── 任务分配层
    │   └── task_allocator
    ├── 规划层
    │   └── coverage_planner
    ├── 安全增强层
    │   └── collision_avoidance
    └── 扩展能力层
        ├── target_detection
        ├── radar_perception
        └── ground_station_bridge
```

## 4. 三个运行域

## 4.1 仿真域

仿真域用于 SITL 与系统联调，包含：

- `PX4 SITL`
- `Gazebo Classic`
- `MicroXRCEAgent`
- ROS2 集群节点

目标：

- 在不引入真机风险的情况下验证多机控制、任务调度和状态共享
- 验证 WSL2 内部 ROS2/PX4 通信与 Windows 侧 QGC 联通

## 4.2 机载域

机载域是未来部署到 `RK3588` 的运行域，包含：

- 每架无人机本机的 `px4_bridge`
- 本机状态发布
- 本机任务执行器
- 本机感知节点
- 本机日志与健康监控

目标：

- 保持单机可独立工作
- 在机间链路不稳定时仍保留本机安全控制与有限自主能力

## 4.3 地面域

地面域包含：

- `QGroundControl`
- 后续自研地面站或任务可视化系统

目标：

- QGC 负责 PX4 级监控、模式查看、基础调试
- 后续地面站负责群体任务、编队状态、搜索任务、人机交互

## 5. ROS2 包级架构

结合当前推荐目录，主线建议如下：

## 5.1 `swarm_msgs`

职责：

- 定义项目内部统一消息、服务、动作接口
- 隔离 `px4_msgs` 与上层任务逻辑

建议承载的接口类型：

- 单机归一化状态消息
- 集群状态消息
- 编队配置消息
- 任务分配消息
- 覆盖搜索任务消息
- 长时任务 action

作用：

- 避免上层模块直接耦合 PX4 原始话题细节

## 5.2 `px4_bridge`

职责：

- 每架无人机一个桥接实例
- 订阅 PX4 `fmu/out` 侧状态
- 发布 PX4 `fmu/in` 侧控制命令
- 将 PX4 原始话题转换为项目统一状态
- 维护 offboard 进入前的心跳和命令时序

典型输入输出：

- 输入：
  - `px4_msgs` 状态类话题
  - 上层任务层/编队层的目标指令
- 输出：
  - 统一 `UavState`
  - `OffboardControlMode`
  - `TrajectorySetpoint`
  - `VehicleCommand`

设计要求：

- 完全 namespace 化
- 不在该层实现集群逻辑
- 不绕过 PX4 的模式管理和 failsafe

## 5.3 `swarm_manager`

职责：

- 汇总所有无人机状态
- 管理无人机注册表、健康状态和角色信息
- 对外发布集群级状态视图

核心能力：

- 无人机列表管理
- leader / follower 角色标记
- 在线/离线/失联状态判断
- 电量、定位、模式、任务状态汇总

它是“集群状态源”，不直接输出飞行控制指令。

## 5.4 `mission_manager`

职责：

- 管理单机与集群级任务状态机
- 统一调度 takeoff、land、goto、formation、coverage 等任务
- 负责不同控制模式间的互斥与切换

建议采用：

- ROS2 action 作为长时任务接口
- 生命周期管理或显式状态机管理任务切换

关键设计原则：

- 每架无人机同一时刻只允许一个主控任务源生效
- 编队控制、覆盖搜索、手动航点等模式必须由 `mission_manager` 仲裁

## 5.5 `formation_controller`

职责：

- 负责 leader-follower 编队控制
- 消费 leader 状态、follower 状态和编队偏置配置
- 为 follower 生成相对目标或 setpoint

建议拆分：

- 拓扑/邻居计算
- 编队几何描述
- 控制律实现
- 编队状态监测

支持能力：

- 单 leader 多 follower
- 多 cluster 编队
- leader 丢失降级
- 编队暂停与恢复

## 5.6 `task_allocator`

职责：

- 为多机任务分配无人机和任务片段
- 综合考虑无人机状态、位置、剩余电量、任务优先级

典型场景：

- 覆盖搜索区域切分
- 多目标巡查分配
- leader 或 relay 角色调整

注意：

- 该层输出“任务分配结果”，不直接输出 PX4 控制话题

## 5.7 `coverage_planner`

职责：

- 根据区域、多边形或目标带生成覆盖搜索路径
- 将覆盖路径划分为可分配的任务片段

算法来源：

- 优先参考 `PythonRobotics`
- 工程化后再接入 `task_allocator` 与 `mission_manager`

要求：

- 输出应与具体飞控解耦
- 先输出标准化航点/轨迹，再由下层转换为 PX4 可执行命令

## 5.8 `collision_avoidance`

职责：

- 作为上层任务与 PX4 bridge 之间的安全增强层
- 对 setpoint 做限幅、隔离距离校验和冲突过滤

定位：

- 它不是 PX4 failsafe 的替代品
- 它是 ROS2 侧的额外保护层

## 5.9 后续扩展包

建议预留：

- `target_detection`
- `radar_perception`
- `ground_station_bridge`

这些模块都不应直接侵入 `px4_bridge`，而应通过统一消息接口与
`mission_manager`、`swarm_manager` 交互。

## 6. namespace 与标识体系

## 6.1 namespace 规则

每架无人机使用独立 namespace，建议统一格式：

- `/uav_1`
- `/uav_2`
- `/uav_3`

每个 namespace 内至少包含：

- 本机 `px4_bridge`
- 本机状态输出
- 本机任务执行输入

集群级节点建议放在：

- `/swarm`

例如：

- `/uav_1/state`
- `/uav_1/command`
- `/swarm/fleet_state`
- `/swarm/formation_status`

## 6.2 标识字段

建议区分以下概念：

- `uav_id`：项目内部逻辑编号
- `px4_instance`：PX4 SITL 实例编号
- `mavlink_sysid`：MAVLink 系统 ID
- `vehicle_role`：leader / follower / standby

目的：

- 不把 namespace、PX4 实例号和任务角色混为一谈

## 7. 多机状态共享设计

多机状态共享采用“两层状态模型”：

## 7.1 单机状态层

由每个 `px4_bridge` 发布本机归一化状态，至少包含：

- 位置
- 速度
- 姿态
- 飞行模式
- 是否 armed
- 电量
- 任务状态
- 时间戳

## 7.2 集群状态层

由 `swarm_manager` 汇总所有单机状态，生成：

- `fleet_state`
- `leader_state`
- `formation_state`
- `health_summary`

这样做的好处：

- 上层模块不需要直接订阅大量 PX4 原始话题
- 后续真机部署时可替换集群状态传输方式，而不改任务逻辑

## 8. leader-follower 编队控制链路

建议控制链路如下：

1. `mission_manager` 设定当前集群进入 `FORMATION` 模式
2. leader 的目标轨迹来自：
   - 航点任务
   - 手动指定目标
   - 后续搜索/巡逻任务
3. leader 的 `px4_bridge` 将目标转为 PX4 可执行指令
4. `formation_controller` 订阅 leader 与 follower 状态
5. `formation_controller` 根据相对偏置和控制律生成 follower setpoint
6. `collision_avoidance` 对输出做约束检查
7. follower 的 `px4_bridge` 将 setpoint 发送给 PX4

关键要求：

- follower 不直接依赖 QGC
- leader 状态丢失时必须进入降级策略
- 编队控制只负责相对控制，不接管 PX4 底层稳定控制

## 9. 任务分配与覆盖搜索链路

建议链路如下：

1. 地面域或上层任务系统提交区域搜索任务
2. `coverage_planner` 生成区域覆盖路径
3. `task_allocator` 将路径分块分配给多架无人机
4. `mission_manager` 为每架无人机生成可执行任务
5. 各机 `px4_bridge` 负责转换为航点或 offboard 指令
6. `swarm_manager` 统一收集执行进度

这样可以保证：

- 算法层和执行层解耦
- 区域规划与具体飞控接口解耦
- 后续替换算法不会影响底层桥接

## 10. 感知与目标处理扩展

后续目标检测、雷达感知不应直接嵌入飞控链路，而应采用旁路接入：

- 感知模块发布目标、障碍物、区域语义信息
- `mission_manager` 或 `collision_avoidance` 决定是否调整任务
- 必要时再由 `px4_bridge` 转为新的飞行目标

针对 `RK3588`，建议：

- 模型推理框架保持可替换
- 不把 TensorRT / DeepStream 作为唯一前提
- 预留 `ONNX Runtime / RKNN / CPU fallback` 的适配可能

## 11. WSL2 / Windows / QGC 网络架构

当前仿真环境下，网络边界必须明确：

- PX4 SITL 在 WSL2
- ROS2 在 WSL2
- MicroXRCEAgent 在 WSL2
- QGC 在 Windows Host

因此本项目架构约定：

- `QGC <-> PX4` 走 MAVLink UDP
- `ROS2 <-> PX4` 走 Micro XRCE-DDS
- QGC 不是 ROS2 网络的一部分
- QGC 不承担集群算法职责

后续需要单独文档明确：

- 多机 MAVLink 端口分配
- WSL2 到 Windows 主机的地址选择
- QGC 如何连接单机/多机 SITL

## 12. SITL 与 RK3588 的一致性策略

为了保证仿真到部署迁移平滑，主线应遵守：

- 单机和多机都使用相同 ROS2 包结构
- SITL 与真机尽量保持相同消息接口
- 差异仅体现在启动文件、设备驱动和网络参数

建议：

- `simulation/` 放 SITL 启动逻辑
- `scripts/` 放 WSL2 环境脚本
- `onboard/` 放 RK3588 安装和 systemd 方案

## 13. 推荐的控制与任务状态机

每架无人机建议具备如下高层状态：

- `IDLE`
- `READY`
- `ARMING`
- `TAKEOFF`
- `HOLD`
- `WAYPOINT`
- `FORMATION`
- `SEARCH`
- `RTL`
- `LAND`
- `FAULT`

说明：

- `mission_manager` 负责高层状态切换
- `px4_bridge` 负责将状态目标转化为 PX4 命令序列
- `swarm_manager` 负责状态汇总与外部可视化

## 14. 参考项目吸收映射

本架构主要吸收以下参考项目思想：

- `PX4_Swarm_Controller`
  - 多机 namespace
  - leader/follower 角色拆分
  - 邻居与控制器分离

- `navigation2`
  - 生命周期管理
  - 插件式扩展
  - action 驱动长时任务

- `aerial-autonomy-stack`
  - simulation / aircraft / ground 分层
  - state sharing 与 mission 分层

- `mavsdk_drone_show`
  - 任务生命周期
  - 搜索任务与地面站组织方式

## 15. 架构结论

本项目推荐采用：

- `PX4` 负责飞行安全和底层控制
- `ROS2` 负责群体智能和任务系统
- `px4_bridge` 负责单机接口标准化
- `swarm_manager + mission_manager` 负责系统主控
- `formation_controller + task_allocator + coverage_planner` 负责群体智能能力
- `QGC` 作为标准飞控地面站保留
- `RK3588` 作为未来机载计算平台，但不绑定 NVIDIA 专有栈

这是一个适合从 `SITL -> 多机仿真 -> 机载部署 -> 小规模真机测试`
逐步演进的架构。
