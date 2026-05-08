uint8 ROLE_UNKNOWN=0
uint8 ROLE_LEADER=1
uint8 ROLE_FOLLOWER=2

std_msgs/Header header
string drone_id
string drone_namespace
uint8 role
bool armed
string flight_mode
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
float64 yaw
bool healthy
string status_text
