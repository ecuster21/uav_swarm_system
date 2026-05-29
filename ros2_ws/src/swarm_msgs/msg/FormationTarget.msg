# 发给单架无人机的编队/航点目标，由 formation_controller 发布到 /uav_N/formation_target。
# 坐标使用 local_enu：x=east, y=north, z=up；bridge 负责转换到 PX4 local_ned。

std_msgs/Header header

# 目标无人机 ID；为空时允许接收端自行决定是否接受，常规路径中应填写 uav_N。
string drone_id

# 目标来源标签，例如 formation_controller.triangle 或 formation_controller.hold:reason。
string source

# 位置目标和可选速度前馈，单位分别为 m 和 m/s。
geometry_msgs/Point position
geometry_msgs/Vector3 velocity

# 目标 yaw，单位 rad，local_enu 下绕 z 轴。
float64 yaw

# true 表示 bridge 同时发送 velocity 前馈；false 表示只发送位置目标。
bool use_velocity

# false 表示该目标不应继续驱动 Offboard 运动；真实 bridge 会按安全逻辑处理。
bool active
