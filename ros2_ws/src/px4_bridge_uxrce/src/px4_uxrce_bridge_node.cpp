#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/timesync_status.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "swarm_msgs/msg/drone_state.hpp"
#include "swarm_msgs/msg/formation_target.hpp"

using namespace std::chrono_literals;

namespace px4_bridge_uxrce
{
namespace
{
using Trigger = std_srvs::srv::Trigger;
using TriggerRequest = std::shared_ptr<Trigger::Request>;
using TriggerResponse = std::shared_ptr<Trigger::Response>;

constexpr float kMavModeFlagCustomModeEnabled = 1.0F;
constexpr float kPx4CustomMainModeAuto = 4.0F;
constexpr float kPx4CustomSubModeAutoLoiter = 3.0F;
constexpr float kPx4CustomMainModeOffboard = 6.0F;
constexpr uint8_t kTargetComponentAutopilot = 1;
constexpr uint8_t kSourceSystemCompanion = 1;
constexpr uint8_t kSourceComponentCompanion = 191;
constexpr double kPi = 3.14159265358979323846;

// 将配置中的 PX4 话题前缀统一成绝对 ROS2 topic 前缀。
// 例如 "px4_1" 会变成 "/px4_1"，后续拼接为 /px4_1/fmu/in/...。
std::string normalize_prefix(std::string prefix)
{
  if (prefix.empty()) {
    return "";
  }
  if (prefix.front() != '/') {
    prefix = "/" + prefix;
  }
  while (prefix.size() > 1 && prefix.back() == '/') {
    prefix.pop_back();
  }
  return prefix;
}

uint8_t role_to_constant(const std::string & role)
{
  if (role == "leader") {
    return swarm_msgs::msg::DroneState::ROLE_LEADER;
  }
  if (role == "follower") {
    return swarm_msgs::msg::DroneState::ROLE_FOLLOWER;
  }
  return swarm_msgs::msg::DroneState::ROLE_UNKNOWN;
}

std::string nav_state_to_string(uint8_t nav_state)
{
  using VehicleStatus = px4_msgs::msg::VehicleStatus;
  switch (nav_state) {
    case VehicleStatus::NAVIGATION_STATE_MANUAL:
      return "MANUAL";
    case VehicleStatus::NAVIGATION_STATE_ALTCTL:
      return "ALTCTL";
    case VehicleStatus::NAVIGATION_STATE_POSCTL:
      return "POSCTL";
    case VehicleStatus::NAVIGATION_STATE_AUTO_MISSION:
      return "AUTO_MISSION";
    case VehicleStatus::NAVIGATION_STATE_AUTO_LOITER:
      return "AUTO_LOITER";
    case VehicleStatus::NAVIGATION_STATE_AUTO_RTL:
      return "AUTO_RTL";
    case VehicleStatus::NAVIGATION_STATE_OFFBOARD:
      return "OFFBOARD";
    case VehicleStatus::NAVIGATION_STATE_AUTO_TAKEOFF:
      return "AUTO_TAKEOFF";
    case VehicleStatus::NAVIGATION_STATE_AUTO_LAND:
      return "AUTO_LAND";
    default:
      return "NAV_STATE_" + std::to_string(nav_state);
  }
}

double normalize_angle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

// PX4 local frame 使用 NED：x=north, y=east, z=down。
// 项目内部统一使用 ENU：x=east, y=north, z=up。
double ned_yaw_to_enu(double yaw_ned)
{
  return normalize_angle(kPi / 2.0 - yaw_ned);
}

double enu_yaw_to_ned(double yaw_enu)
{
  return normalize_angle(kPi / 2.0 - yaw_enu);
}
}  // namespace

class Px4UxrceBridge : public rclcpp::Node
{
public:
  Px4UxrceBridge()
  : Node("px4_uxrce_bridge")
  {
    // 单机身份参数由 launch 从 config/swarm.yaml 注入。
    // ROS2 namespace 负责项目内隔离，例如 /uav_1/state；
    // px4_topic_prefix 负责连接对应 PX4 SITL 实例，例如 /px4_1/fmu/out/...
    drone_id_ = declare_parameter<std::string>("drone_id", "uav_1");
    drone_namespace_ = declare_parameter<std::string>("drone_namespace", "uav_1");
    role_ = declare_parameter<std::string>("role", "unknown");
    frame_id_ = declare_parameter<std::string>("frame_id", "local_enu");
    px4_topic_prefix_ = normalize_prefix(declare_parameter<std::string>("px4_topic_prefix", "px4_1"));
    system_id_ = static_cast<uint8_t>(declare_parameter<int>("system_id", 2));
    takeoff_altitude_m_ = declare_parameter<double>("takeoff_altitude_m", 2.5);
    state_rate_hz_ = declare_parameter<double>("state_rate_hz", 10.0);
    offboard_rate_hz_ = declare_parameter<double>("offboard_rate_hz", 20.0);
    enable_offboard_from_target_ = declare_parameter<bool>("enable_offboard_from_target", false);
    offboard_warmup_cycles_ = declare_parameter<int>("offboard_warmup_cycles", 10);
    initial_position_ = declare_parameter<std::vector<double>>(
      "initial_position", std::vector<double>{0.0, 0.0, 0.0});
    if (initial_position_.size() != 3) {
      throw std::runtime_error("initial_position must contain exactly three values");
    }

    // 本节点在 /uav_N namespace 下，相对 topic "state" 最终会变成 /uav_N/state。
    state_pub_ = create_publisher<swarm_msgs::msg::DroneState>("state", 10);

    // 对 PX4 发布控制消息。这里使用 px4_topic_prefix 选择具体 PX4 SITL 实例，
    // 不使用 /uav_N namespace，避免把项目内部命名和 PX4 原生 DDS 命名混在一起。
    command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
      px4_topic_prefix_ + "/fmu/in/vehicle_command", 10);
    offboard_control_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
      px4_topic_prefix_ + "/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      px4_topic_prefix_ + "/fmu/in/trajectory_setpoint", 10);

    // 从对应 PX4 实例读取遥测；SensorDataQoS 更适合高频数据，允许丢弃旧帧以降低延迟。
    local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      px4_topic_prefix_ + "/fmu/out/vehicle_local_position", rclcpp::SensorDataQoS(),
      [this](px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        latest_local_position_ = msg;
      });
    vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      px4_topic_prefix_ + "/fmu/out/vehicle_status", rclcpp::SensorDataQoS(),
      [this](px4_msgs::msg::VehicleStatus::SharedPtr msg) {
        latest_vehicle_status_ = msg;
      });
    timesync_sub_ = create_subscription<px4_msgs::msg::TimesyncStatus>(
      px4_topic_prefix_ + "/fmu/out/timesync_status", rclcpp::SensorDataQoS(),
      [this](px4_msgs::msg::TimesyncStatus::SharedPtr msg) {
        latest_px4_timestamp_us_ = msg->timestamp;
      });
    target_sub_ = create_subscription<swarm_msgs::msg::FormationTarget>(
      "formation_target", 10,
      [this](swarm_msgs::msg::FormationTarget::SharedPtr msg) {
        handle_target(msg);
      });

    // 对每架无人机暴露相同服务，例如 /uav_1/arm；服务只负责翻译成 PX4 VehicleCommand。
    services_.push_back(create_service<Trigger>("connect", [this](TriggerRequest, TriggerResponse response) {
        response->success = latest_local_position_ != nullptr || latest_vehicle_status_ != nullptr;
        response->message = response->success ? "uXRCE-DDS telemetry is available" :
          "waiting for uXRCE-DDS telemetry";
      }));
    services_.push_back(create_service<Trigger>("arm", [this](TriggerRequest, TriggerResponse response) {
        publish_vehicle_command(
          px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0F);
        fill_success(response, "arm command published");
      }));
    services_.push_back(create_service<Trigger>("takeoff", [this](TriggerRequest, TriggerResponse response) {
        publish_vehicle_command(
          px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF,
          0.0F, 0.0F, 0.0F, std::numeric_limits<float>::quiet_NaN(),
          std::numeric_limits<double>::quiet_NaN(),
          std::numeric_limits<double>::quiet_NaN(),
          static_cast<float>(takeoff_altitude_m_));
        fill_success(response, "takeoff command published");
      }));
    services_.push_back(create_service<Trigger>("land", [this](TriggerRequest, TriggerResponse response) {
        offboard_requested_ = false;
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
        fill_success(response, "land command published");
      }));
    services_.push_back(create_service<Trigger>("hold", [this](TriggerRequest, TriggerResponse response) {
        offboard_requested_ = false;
        publish_vehicle_command(
          px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
          kMavModeFlagCustomModeEnabled, kPx4CustomMainModeAuto, kPx4CustomSubModeAutoLoiter);
        fill_success(response, "hold/loiter command published");
      }));
    services_.push_back(create_service<Trigger>("rtl", [this](TriggerRequest, TriggerResponse response) {
        offboard_requested_ = false;
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH);
        fill_success(response, "RTL command published");
      }));
    services_.push_back(create_service<Trigger>("goto", [this](TriggerRequest, TriggerResponse response) {
        if (!latest_target_) {
          response->success = false;
          response->message = "no FormationTarget has been received";
          return;
        }
        offboard_requested_ = true;
        offboard_warmup_count_ = 0;
        fill_success(response, "uXRCE-DDS offboard goto requested");
      }));

    state_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(state_rate_hz_, 1.0)),
      [this]() { publish_state(); });

    // Offboard setpoint 必须持续发送；PX4 依赖稳定的数据流判断伴随计算机是否在线。
    offboard_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(offboard_rate_hz_, 2.0)),
      [this]() { publish_offboard_if_requested(); });

    RCLCPP_INFO(
      get_logger(),
      "uXRCE-DDS bridge ready for %s: px4_topic_prefix=%s, system_id=%u",
      drone_id_.c_str(), px4_topic_prefix_.c_str(), system_id_);
  }

private:
  void fill_success(const TriggerResponse & response, const std::string & message) const
  {
    response->success = true;
    response->message = message;
  }

  uint64_t px4_timestamp_us() const
  {
    // 优先使用 PX4 timesync 提供的时间戳；还未收到 timesync 时退回 ROS 当前时间。
    if (latest_px4_timestamp_us_ > 0) {
      return latest_px4_timestamp_us_;
    }
    return static_cast<uint64_t>(now().nanoseconds() / 1000ULL);
  }

  void publish_vehicle_command(
    uint32_t command,
    float param1 = 0.0F,
    float param2 = 0.0F,
    float param3 = 0.0F,
    float param4 = 0.0F,
    double param5 = 0.0,
    double param6 = 0.0,
    float param7 = 0.0F)
  {
    // MAVLink VehicleCommand 的 target_system 必须和当前 PX4 实例的 system_id 对齐。
    // 多机时这是避免命令打到错误飞机上的关键字段。
    px4_msgs::msg::VehicleCommand msg{};
    msg.timestamp = px4_timestamp_us();
    msg.command = command;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.param3 = param3;
    msg.param4 = param4;
    msg.param5 = param5;
    msg.param6 = param6;
    msg.param7 = param7;
    msg.target_system = system_id_;
    msg.target_component = kTargetComponentAutopilot;
    msg.source_system = kSourceSystemCompanion;
    msg.source_component = kSourceComponentCompanion;
    msg.from_external = true;
    command_pub_->publish(msg);
  }

  void handle_target(const swarm_msgs::msg::FormationTarget::SharedPtr & msg)
  {
    // formation_controller 会给每架机发布目标；如果消息显式写了其他 drone_id，则忽略。
    if (!msg->drone_id.empty() && msg->drone_id != drone_id_) {
      return;
    }
    if (!msg->active) {
      // 飞机已解锁且位置有效时，收到 inactive target 不直接停掉 Offboard 流。
      // 改为发布当前位置保持目标，降低 PX4 因 setpoint 中断退出 Offboard 的风险。
      if (is_armed() && local_position_valid()) {
        latest_target_ = make_current_position_hold_target(*msg);
        offboard_requested_ = true;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Received inactive target while armed; keeping offboard stream with current-position hold");
      } else {
        latest_target_ = msg;
        offboard_requested_ = false;
      }
      return;
    }

    latest_target_ = msg;
    // enable_offboard_from_target=true 时，收到有效 FormationTarget 后自动开始 setpoint 流。
    if (enable_offboard_from_target_ && is_armed() && local_position_valid()) {
      offboard_requested_ = true;
    }
  }

  swarm_msgs::msg::FormationTarget::SharedPtr make_current_position_hold_target(
    const swarm_msgs::msg::FormationTarget & inactive_msg) const
  {
    auto hold = std::make_shared<swarm_msgs::msg::FormationTarget>(inactive_msg);
    hold->header.stamp = now();
    hold->header.frame_id = frame_id_;
    hold->drone_id = drone_id_;
    hold->source = inactive_msg.source + ".bridge_current_position_hold";
    hold->position = current_enu_position();
    hold->yaw = latest_local_position_ ? ned_yaw_to_enu(latest_local_position_->heading) :
      inactive_msg.yaw;
    hold->use_velocity = false;
    hold->active = true;
    return hold;
  }

  geometry_msgs::msg::Point current_enu_position() const
  {
    geometry_msgs::msg::Point point{};
    if (!latest_local_position_) {
      return point;
    }
    // PX4 local_position 是相对本机 EKF 原点的 NED 坐标。
    // initial_position 是 Gazebo 出生点在项目 ENU 坐标中的锚点。
    point.x = initial_position_[0] + latest_local_position_->y;
    point.y = initial_position_[1] + latest_local_position_->x;
    point.z = initial_position_[2] - latest_local_position_->z;
    return point;
  }

  void publish_state()
  {
    swarm_msgs::msg::DroneState msg{};
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.drone_id = drone_id_;
    msg.drone_namespace = drone_namespace_;
    msg.role = role_to_constant(role_);
    msg.armed = is_armed();
    msg.flight_mode = latest_vehicle_status_ ? nav_state_to_string(latest_vehicle_status_->nav_state) :
      "UNKNOWN";
    msg.healthy = local_position_valid() && (!latest_vehicle_status_ || !latest_vehicle_status_->failsafe);
    msg.status_text = status_text();

    if (latest_local_position_) {
      msg.position = current_enu_position();
      msg.velocity.x = latest_local_position_->vy;
      msg.velocity.y = latest_local_position_->vx;
      msg.velocity.z = -latest_local_position_->vz;
      msg.yaw = ned_yaw_to_enu(latest_local_position_->heading);
    }
    state_pub_->publish(msg);
  }

  bool is_armed() const
  {
    return latest_vehicle_status_ &&
           latest_vehicle_status_->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
  }

  bool local_position_valid() const
  {
    return latest_local_position_ && latest_local_position_->xy_valid && latest_local_position_->z_valid;
  }

  std::string status_text() const
  {
    if (!latest_local_position_ && !latest_vehicle_status_) {
      return "waiting for uXRCE-DDS telemetry";
    }
    if (!latest_local_position_) {
      return "waiting for vehicle_local_position";
    }
    if (!local_position_valid()) {
      return "local position is not valid";
    }
    if (latest_vehicle_status_ && latest_vehicle_status_->failsafe) {
      return "PX4 reports failsafe";
    }
    if (offboard_requested_) {
      return "uXRCE-DDS offboard setpoint streaming";
    }
    return "uXRCE-DDS telemetry ready";
  }

  void publish_offboard_if_requested()
  {
    // 只有已经收到 active FormationTarget 且明确请求 Offboard 时才发送 setpoint。
    if (!offboard_requested_ || !latest_target_ || !latest_target_->active) {
      return;
    }
    if (!local_position_valid()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Cannot stream offboard setpoint: local position invalid");
      return;
    }
    if (!is_armed()) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000, "Waiting to stream offboard setpoint: vehicle is not armed");
      return;
    }

    px4_msgs::msg::OffboardControlMode control_mode{};
    control_mode.timestamp = px4_timestamp_us();
    control_mode.position = true;
    control_mode.velocity = latest_target_->use_velocity;
    offboard_control_mode_pub_->publish(control_mode);

    // FormationTarget 使用项目 ENU 坐标；PX4 TrajectorySetpoint 需要本机 local NED。
    // 这里先减去 initial_position，再做 ENU -> NED 轴变换。
    px4_msgs::msg::TrajectorySetpoint setpoint{};
    setpoint.timestamp = control_mode.timestamp;
    setpoint.position[0] = static_cast<float>(latest_target_->position.y - initial_position_[1]);
    setpoint.position[1] = static_cast<float>(latest_target_->position.x - initial_position_[0]);
    setpoint.position[2] = static_cast<float>(-(latest_target_->position.z - initial_position_[2]));
    const float nan = std::numeric_limits<float>::quiet_NaN();
    if (latest_target_->use_velocity) {
      setpoint.velocity[0] = static_cast<float>(latest_target_->velocity.y);
      setpoint.velocity[1] = static_cast<float>(latest_target_->velocity.x);
      setpoint.velocity[2] = static_cast<float>(-latest_target_->velocity.z);
    } else {
      setpoint.velocity = {nan, nan, nan};
    }
    setpoint.acceleration = {nan, nan, nan};
    setpoint.jerk = {nan, nan, nan};
    setpoint.yaw = static_cast<float>(enu_yaw_to_ned(latest_target_->yaw));
    setpoint.yawspeed = nan;
    trajectory_setpoint_pub_->publish(setpoint);

    // PX4 进入 Offboard 前需要先收到一段连续 setpoint。
    // warmup 完成后再发送 DO_SET_MODE，避免刚切模式就因为 setpoint 不足被拒绝。
    if (offboard_warmup_count_ < offboard_warmup_cycles_) {
      ++offboard_warmup_count_;
      return;
    }
    if (!latest_vehicle_status_ ||
      latest_vehicle_status_->nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD)
    {
      publish_vehicle_command(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
        kMavModeFlagCustomModeEnabled, kPx4CustomMainModeOffboard);
    }
  }

  std::string drone_id_;
  std::string drone_namespace_;
  std::string role_;
  std::string frame_id_;
  std::string px4_topic_prefix_;
  uint8_t system_id_{2};
  double takeoff_altitude_m_{2.5};
  double state_rate_hz_{10.0};
  double offboard_rate_hz_{20.0};
  bool enable_offboard_from_target_{false};
  int offboard_warmup_cycles_{10};
  int offboard_warmup_count_{0};
  bool offboard_requested_{false};
  uint64_t latest_px4_timestamp_us_{0};
  std::vector<double> initial_position_;

  rclcpp::Publisher<swarm_msgs::msg::DroneState>::SharedPtr state_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr timesync_sub_;
  rclcpp::Subscription<swarm_msgs::msg::FormationTarget>::SharedPtr target_sub_;
  std::vector<rclcpp::Service<Trigger>::SharedPtr> services_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::TimerBase::SharedPtr offboard_timer_;

  px4_msgs::msg::VehicleLocalPosition::SharedPtr latest_local_position_;
  px4_msgs::msg::VehicleStatus::SharedPtr latest_vehicle_status_;
  swarm_msgs::msg::FormationTarget::SharedPtr latest_target_;
};
}  // namespace px4_bridge_uxrce

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<px4_bridge_uxrce::Px4UxrceBridge>());
  rclcpp::shutdown();
  return 0;
}
