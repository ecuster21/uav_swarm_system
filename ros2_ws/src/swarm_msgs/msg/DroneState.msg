# 单架无人机归一化状态，由各自 bridge 发布到 /uav_N/state。
# header.frame_id 当前约定为 local_enu，position/velocity/yaw 都使用该项目内部坐标系。

uint8 ROLE_UNKNOWN=0
uint8 ROLE_LEADER=1
uint8 ROLE_FOLLOWER=2

std_msgs/Header header

# 项目内部无人机 ID，例如 uav_1；用于状态表、队形 offset 和日志索引。
string drone_id

# ROS2 namespace，不带前导斜杠，例如 uav_1；对应 /uav_1/state。
string drone_namespace

# 集群逻辑角色，取上面的 ROLE_* 常量。
uint8 role

# PX4 或 mock 后端报告的解锁状态和飞行模式。
bool armed
string flight_mode

# local_enu 坐标：x=east, y=north, z=up，单位 m；yaw 单位 rad。
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
float64 yaw

# healthy=false 表示该状态不可用于闭环控制，status_text 给出原因。
bool healthy
string status_text
