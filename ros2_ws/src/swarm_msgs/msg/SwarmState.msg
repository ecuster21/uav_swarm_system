# 集群状态快照，由 swarm_manager 发布到 /swarm/state。
# 它只汇总各机状态，不直接表达控制目标。

std_msgs/Header header

# drones 数组长度的显式副本，便于地面站或日志快速读取。
uint32 drone_count

# 当前 leader 的项目内部无人机 ID，例如 uav_1。
string leader_id

# 按 swarm.yaml 中 drones 顺序汇总的单机状态。
DroneState[] drones
