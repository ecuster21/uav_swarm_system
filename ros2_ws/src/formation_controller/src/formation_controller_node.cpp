#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <swarm_msgs/msg/drone_state.hpp>
#include <swarm_msgs/msg/formation_target.hpp>
#include <swarm_msgs/msg/swarm_state.hpp>
#include <yaml-cpp/yaml.h>

namespace
{
using DroneState = swarm_msgs::msg::DroneState;
using FormationTarget = swarm_msgs::msg::FormationTarget;
using Point = geometry_msgs::msg::Point;
using SwarmState = swarm_msgs::msg::SwarmState;
using Vector3 = geometry_msgs::msg::Vector3;

struct DroneConfig
{
  std::string drone_id;
  std::string drone_namespace;
  std::string role;
};

struct FormationSpacing
{
  double longitudinal_spacing_m{2.0};
  double lateral_spacing_m{1.5};
  double spacing_m{2.0};
  double z_offset_m{0.0};
};

struct SafetyConfig
{
  double max_speed_m_s{1.5};
  double max_altitude_m{5.0};
  double min_altitude_m{0.5};
  double min_inter_drone_distance_m{1.2};
};

struct ControlGains
{
  double kp_position{0.8};
  double kd_velocity{0.35};
};

struct SmoothingConfig
{
  double max_acceleration_m_s2{1.5};
  double max_jerk_m_s3{3.0};
  double low_pass_alpha{0.35};
};

struct ControlHistory
{
  Vector3 filtered_velocity{};
  Vector3 acceleration{};
  bool has_velocity{false};
  bool has_acceleration{false};
};

std::string trim_slashes(const std::string & value)
{
  const auto first = value.find_first_not_of('/');
  if (first == std::string::npos) {
    return "";
  }
  const auto last = value.find_last_not_of('/');
  return value.substr(first, last - first + 1);
}

Point point_from_yaml(const YAML::Node & node)
{
  if (!node.IsSequence() || node.size() != 3) {
    throw std::runtime_error("Point YAML value must be a 3-item sequence");
  }
  Point point{};
  point.x = node[0].as<double>();
  point.y = node[1].as<double>();
  point.z = node[2].as<double>();
  return point;
}

Point copy_point(const Point & point)
{
  Point copy{};
  copy.x = point.x;
  copy.y = point.y;
  copy.z = point.z;
  return copy;
}

double distance(const Point & a, const Point & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double norm(const Vector3 & vector)
{
  return std::sqrt(
    vector.x * vector.x + vector.y * vector.y + vector.z * vector.z);
}

double clamp(double value, double low, double high)
{
  return std::max(low, std::min(high, value));
}

double clamp01(double value)
{
  return clamp(value, 0.0, 1.0);
}

Vector3 make_vector(double x, double y, double z)
{
  Vector3 vector{};
  vector.x = x;
  vector.y = y;
  vector.z = z;
  return vector;
}

Vector3 vector_between(const Point & from, const Point & to)
{
  return make_vector(to.x - from.x, to.y - from.y, to.z - from.z);
}

Vector3 add_vectors(const Vector3 & a, const Vector3 & b)
{
  return make_vector(a.x + b.x, a.y + b.y, a.z + b.z);
}

Vector3 subtract_vectors(const Vector3 & a, const Vector3 & b)
{
  return make_vector(a.x - b.x, a.y - b.y, a.z - b.z);
}

Vector3 scale_vector(const Vector3 & vector, double scale)
{
  return make_vector(vector.x * scale, vector.y * scale, vector.z * scale);
}

Vector3 limit_vector(const Vector3 & vector, double max_norm)
{
  const auto vector_norm = norm(vector);
  if (vector_norm <= max_norm || vector_norm <= 1e-9 || max_norm <= 0.0) {
    return vector;
  }
  return scale_vector(vector, max_norm / vector_norm);
}

Point translate_point(const Point & point, const Vector3 & vector)
{
  Point translated{};
  translated.x = point.x + vector.x;
  translated.y = point.y + vector.y;
  translated.z = point.z + vector.z;
  return translated;
}

double yaml_double(const YAML::Node & node, const std::string & key, double default_value)
{
  return node && node[key] ? node[key].as<double>() : default_value;
}

bool yaml_bool(const YAML::Node & node, const std::string & key, bool default_value)
{
  return node && node[key] ? node[key].as<bool>() : default_value;
}

std::string yaml_string(
  const YAML::Node & node, const std::string & key, const std::string & default_value)
{
  return node && node[key] ? node[key].as<std::string>() : default_value;
}

std::array<double, 3> offset_from_yaml(const YAML::Node & values, const std::string & drone_id)
{
  if (!values.IsSequence() || values.size() != 3) {
    throw std::runtime_error("Offset for " + drone_id + " must have exactly 3 values");
  }
  return {values[0].as<double>(), values[1].as<double>(), values[2].as<double>()};
}

}  // namespace

class FormationController : public rclcpp::Node
{
public:
  FormationController()
  : Node("formation_controller")
  {
    // launch 参数优先级高于 YAML，便于同一套配置快速切换队形和调参。
    declare_parameter<std::string>("swarm_config_file", "");
    declare_parameter<std::string>("formations_config_file", "");
    declare_parameter<std::string>("waypoints_config_file", "");
    declare_parameter<std::string>("formation_type", "");
    declare_parameter<std::string>("formation_name", "");
    declare_parameter<double>("control_rate_hz", 0.0);
    declare_parameter<double>("waypoint_acceptance_radius", 0.0);
    declare_parameter<double>("leader_state_timeout_sec", 0.0);
    declare_parameter<double>("max_speed_m_s", 0.0);
    declare_parameter<double>("max_altitude_m", 0.0);
    declare_parameter<double>("min_altitude_m", 0.0);
    declare_parameter<double>("min_inter_drone_distance_m", 0.0);
    declare_parameter<std::string>("control_mode", "");
    declare_parameter<std::string>("offset_frame", "");
    declare_parameter<std::string>("yaw_mode", "");
    declare_parameter<double>("kp_position", 0.0);
    declare_parameter<double>("kd_velocity", 0.0);
    declare_parameter<double>("max_acceleration_m_s2", 0.0);
    declare_parameter<double>("max_jerk_m_s3", 0.0);
    declare_parameter<double>("low_pass_alpha", -1.0);
    declare_parameter<double>("leader_prediction_horizon_sec", 0.0);
    declare_parameter<bool>("distributed_followers", false);

    const auto swarm_config_file = get_parameter("swarm_config_file").as_string();
    const auto formations_config_file = get_parameter("formations_config_file").as_string();
    const auto waypoints_config_file = get_parameter("waypoints_config_file").as_string();
    if (swarm_config_file.empty() || formations_config_file.empty() ||
      waypoints_config_file.empty())
    {
      throw std::runtime_error(
        "swarm_config_file, formations_config_file, and waypoints_config_file are required");
    }

    swarm_config_ = YAML::LoadFile(swarm_config_file);
    formations_config_ = YAML::LoadFile(formations_config_file);
    waypoints_config_ = YAML::LoadFile(waypoints_config_file);

    validate_frame_config();

    // 三个配置文件共同确定控制对象、队形几何和 leader 航线。
    const auto swarm_section = swarm_config_["swarm"];
    if (!swarm_section) {
      throw std::runtime_error("swarm_config_file must contain swarm section");
    }
    const auto controller_section = formations_config_["controller"];
    drone_configs_ = load_drone_configs(swarm_section);
    leader_id_ = swarm_section["leader_id"].as<std::string>();
    for (const auto & drone : drone_configs_) {
      if (drone.drone_id != leader_id_) {
        follower_ids_.push_back(drone.drone_id);
      }
    }

    formation_type_ = resolve_formation_type(controller_section);
    const auto formation_section = formations_config_["formations"][formation_type_];
    if (!formation_section) {
      throw std::runtime_error("Unknown formation_type: " + formation_type_);
    }

    offsets_ = load_or_generate_offsets(formation_section);
    load_waypoints();
    frame_id_ = "local_enu";
    distributed_followers_ = get_parameter("distributed_followers").as_bool();

    control_rate_hz_ = resolve_double(
      "control_rate_hz", yaml_double(controller_section, "control_rate_hz", 10.0), 1.0);
    acceptance_radius_ = resolve_double(
      "waypoint_acceptance_radius",
      yaml_double(controller_section, "waypoint_acceptance_radius_m", 0.45), 0.05);
    leader_state_timeout_sec_ = resolve_double(
      "leader_state_timeout_sec",
      yaml_double(controller_section, "leader_state_timeout_sec", 2.0), 0.1);
    safety_ = load_safety_config();
    load_control_config(controller_section);

    // formation_controller 运行在 /swarm 下，但目标必须发到每架飞机自己的 namespace。
    for (const auto & drone : drone_configs_) {
      target_publishers_[drone.drone_id] = create_publisher<FormationTarget>(
        "/" + drone.drone_namespace + "/formation_target", 10);
    }
    leader_reference_pub_ = create_publisher<FormationTarget>("/swarm/leader_reference", 10);

    swarm_sub_ = create_subscription<SwarmState>(
      "/swarm/state", 10,
      [this](const SwarmState::SharedPtr msg) {
        swarm_state_callback(msg);
      });

    const auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {
        timer_callback();
      });

    RCLCPP_INFO(
      get_logger(),
      "Formation controller ready: formation_type=%s, leader=%s, frame=%s, mode=%s, offset_frame=%s, yaw_mode=%s, rate=%.1f Hz, distributed_followers=%s",
      formation_type_.c_str(), leader_id_.c_str(), frame_id_.c_str(), control_mode_.c_str(),
      offset_frame_.c_str(), yaw_mode_.c_str(), control_rate_hz_,
      distributed_followers_ ? "true" : "false");
  }

private:
  std::string resolve_formation_type(const YAML::Node & controller_section)
  {
    auto formation_type = get_parameter("formation_type").as_string();
    const auto legacy_name = get_parameter("formation_name").as_string();
    if (formation_type.empty() && !legacy_name.empty()) {
      formation_type = legacy_name;
    }
    if (formation_type.empty()) {
      formation_type = yaml_string(controller_section, "default_formation_type", "triangle");
    }

    const std::set<std::string> allowed{"triangle", "line", "column"};
    if (allowed.count(formation_type) == 0) {
      throw std::runtime_error(
        "formation_type must be one of: triangle, line, column; got " + formation_type);
    }
    return formation_type;
  }

  double resolve_double(
    const std::string & parameter_name, double config_value, double min_value) const
  {
    const double parameter_value = get_parameter(parameter_name).as_double();
    const double value = parameter_value > 0.0 ? parameter_value : config_value;
    if (value < min_value) {
      throw std::runtime_error(
        parameter_name + " must be >= " + std::to_string(min_value) + ", got " +
        std::to_string(value));
    }
    return value;
  }

  void validate_frame_config() const
  {
    const auto frame_section = formations_config_["frame"];
    const auto command_frame = yaml_string(frame_section, "command_frame", "local_enu");
    const auto px4_frame = yaml_string(frame_section, "px4_internal_frame", "local_ned");
    if (command_frame != "local_enu") {
      throw std::runtime_error("formation_controller currently publishes local_enu targets only");
    }
    if (px4_frame != "local_ned") {
      throw std::runtime_error("PX4 bridge contract expects PX4 local_ned internally");
    }
  }

  std::vector<DroneConfig> load_drone_configs(const YAML::Node & swarm_section) const
  {
    std::vector<DroneConfig> drones;
    const auto drone_section = swarm_section["drones"];
    if (!drone_section || !drone_section.IsSequence()) {
      throw std::runtime_error("swarm.drones must contain at least one drone");
    }

    for (const auto & item : drone_section) {
      const auto drone_id = item["id"].as<std::string>();
      const auto drone_namespace = trim_slashes(
        yaml_string(item, "namespace", drone_id));
      drones.push_back(
        DroneConfig{
          drone_id,
          drone_namespace.empty() ? drone_id : drone_namespace,
          yaml_string(item, "role", "unknown")});
    }

    if (drones.empty()) {
      throw std::runtime_error("swarm.drones must contain at least one drone");
    }
    return drones;
  }

  std::map<std::string, std::array<double, 3>> load_or_generate_offsets(
    const YAML::Node & formation_section) const
  {
    std::map<std::string, std::array<double, 3>> offsets;

    // generator 负责按无人机数量自动扩展队形；显式 offsets 用于局部覆盖。
    const auto generator_section = formation_section["generator"];
    if (generator_section) {
      offsets = generate_offsets(generator_section);
    }

    const auto explicit_offsets = load_explicit_offsets(formation_section);
    offsets.insert_or_assign(leader_id_, std::array<double, 3>{0.0, 0.0, 0.0});
    for (const auto & [drone_id, offset] : explicit_offsets) {
      offsets[drone_id] = offset;
    }

    if (!generator_section && explicit_offsets.empty()) {
      throw std::runtime_error("formation must define generator or offsets");
    }

    validate_offsets(offsets);
    return offsets;
  }

  std::map<std::string, std::array<double, 3>> load_explicit_offsets(
    const YAML::Node & formation_section) const
  {
    std::map<std::string, std::array<double, 3>> offsets;
    const auto offsets_section = formation_section["offsets"];
    if (!offsets_section) {
      return offsets;
    }
    if (!offsets_section.IsMap()) {
      throw std::runtime_error("formation offsets must be a mapping");
    }

    for (const auto & item : offsets_section) {
      offsets[item.first.as<std::string>()] =
        offset_from_yaml(item.second, item.first.as<std::string>());
    }
    return offsets;
  }

  std::map<std::string, std::array<double, 3>> generate_offsets(
    const YAML::Node & generator_section) const
  {
    if (!generator_section.IsMap()) {
      throw std::runtime_error("formation generator must be a mapping");
    }

    const auto generator_type = yaml_string(generator_section, "type", formation_type_);
    const auto spacing = load_formation_spacing(generator_section);

    std::map<std::string, std::array<double, 3>> offsets;
    offsets[leader_id_] = {0.0, 0.0, 0.0};

    std::size_t follower_index = 0;
    for (const auto & drone : drone_configs_) {
      if (drone.drone_id == leader_id_) {
        continue;
      }

      if (generator_type == "triangle" || generator_type == "wedge") {
        offsets[drone.drone_id] = generate_wedge_offset(follower_index, spacing);
      } else if (generator_type == "line") {
        offsets[drone.drone_id] = generate_line_offset(follower_index, spacing);
      } else if (generator_type == "column") {
        offsets[drone.drone_id] = generate_column_offset(follower_index, spacing);
      } else {
        throw std::runtime_error(
          "formation generator type must be one of: triangle, wedge, line, column; got " +
          generator_type);
      }
      ++follower_index;
    }

    return offsets;
  }

  FormationSpacing load_formation_spacing(const YAML::Node & generator_section) const
  {
    FormationSpacing spacing{};
    spacing.spacing_m = yaml_double(generator_section, "spacing_m", spacing.spacing_m);
    spacing.longitudinal_spacing_m = yaml_double(
      generator_section, "longitudinal_spacing_m", spacing.spacing_m);
    spacing.lateral_spacing_m = yaml_double(
      generator_section, "lateral_spacing_m", spacing.spacing_m);
    spacing.z_offset_m = yaml_double(generator_section, "z_offset_m", 0.0);

    if (spacing.spacing_m <= 0.0 || spacing.longitudinal_spacing_m <= 0.0 ||
      spacing.lateral_spacing_m <= 0.0)
    {
      throw std::runtime_error("formation generator spacing values must be positive");
    }
    return spacing;
  }

  std::array<double, 3> generate_wedge_offset(
    std::size_t follower_index, const FormationSpacing & spacing) const
  {
    const auto pair_index = follower_index / 2 + 1;
    const auto side = follower_index % 2 == 0 ? -1.0 : 1.0;
    return {
      -spacing.longitudinal_spacing_m * static_cast<double>(pair_index),
      side * spacing.lateral_spacing_m * static_cast<double>(pair_index),
      spacing.z_offset_m};
  }

  std::array<double, 3> generate_line_offset(
    std::size_t follower_index, const FormationSpacing & spacing) const
  {
    const auto pair_index = follower_index / 2 + 1;
    const auto side = follower_index % 2 == 0 ? -1.0 : 1.0;
    return {
      0.0,
      side * spacing.spacing_m * static_cast<double>(pair_index),
      spacing.z_offset_m};
  }

  std::array<double, 3> generate_column_offset(
    std::size_t follower_index, const FormationSpacing & spacing) const
  {
    return {
      -spacing.spacing_m * static_cast<double>(follower_index + 1),
      0.0,
      spacing.z_offset_m};
  }

  void validate_offsets(const std::map<std::string, std::array<double, 3>> & offsets) const
  {
    std::vector<std::string> missing_followers;
    for (const auto & follower_id : follower_ids_) {
      if (offsets.count(follower_id) == 0) {
        missing_followers.push_back(follower_id);
      }
    }
    if (!missing_followers.empty()) {
      throw std::runtime_error("Formation " + formation_type_ + " is missing follower offsets");
    }
  }

  void load_waypoints()
  {
    const auto section = waypoints_config_["leader_waypoints"];
    if (!section) {
      throw std::runtime_error("waypoints_config_file must contain leader_waypoints");
    }

    const auto frame = yaml_string(section, "frame", "local_enu");
    if (frame != "local_enu") {
      throw std::runtime_error("leader_waypoints.frame must be local_enu");
    }

    const auto points = section["points"];
    if (!points || !points.IsSequence() || points.size() == 0) {
      throw std::runtime_error("leader_waypoints.points must contain at least one waypoint");
    }
    for (const auto & point_node : points) {
      route_.push_back(point_from_yaml(point_node));
    }
    loop_route_ = yaml_bool(section, "loop", true);
    leader_yaw_ = yaml_double(section, "yaw", 0.0);
  }

  SafetyConfig load_safety_config() const
  {
    const auto safety_section = formations_config_["safety"];
    SafetyConfig safety{};
    safety.max_speed_m_s = resolve_double(
      "max_speed_m_s", yaml_double(safety_section, "max_speed_m_s", 1.5), 0.1);
    safety.max_altitude_m = resolve_double(
      "max_altitude_m", yaml_double(safety_section, "max_altitude_m", 5.0), 0.1);
    safety.min_altitude_m = resolve_double(
      "min_altitude_m", yaml_double(safety_section, "min_altitude_m", 0.5), 0.0);
    safety.min_inter_drone_distance_m = resolve_double(
      "min_inter_drone_distance_m",
      yaml_double(safety_section, "min_inter_drone_distance_m", 1.2), 0.0);

    if (safety.min_altitude_m > safety.max_altitude_m) {
      throw std::runtime_error("min_altitude_m must be <= max_altitude_m");
    }
    return safety;
  }

  void load_control_config(const YAML::Node & controller_section)
  {
    control_mode_ = get_parameter("control_mode").as_string();
    if (control_mode_.empty()) {
      control_mode_ = yaml_string(controller_section, "control_mode", "position_velocity");
    }
    if (control_mode_ != "position_only" && control_mode_ != "position_velocity") {
      throw std::runtime_error(
        "control_mode must be position_only or position_velocity; got " + control_mode_);
    }
    use_velocity_targets_ = control_mode_ == "position_velocity";

    offset_frame_ = get_parameter("offset_frame").as_string();
    if (offset_frame_.empty()) {
      offset_frame_ = yaml_string(controller_section, "offset_frame", "body_forward_right_up");
    }
    if (offset_frame_ != "local_enu" && offset_frame_ != "body_forward_right_up") {
      throw std::runtime_error(
        "offset_frame must be local_enu or body_forward_right_up; got " + offset_frame_);
    }

    yaw_mode_ = get_parameter("yaw_mode").as_string();
    if (yaw_mode_.empty()) {
      yaw_mode_ = yaml_string(controller_section, "yaw_mode", "fixed");
    }
    if (yaw_mode_ != "fixed" && yaw_mode_ != "face_velocity") {
      throw std::runtime_error(
        "yaw_mode must be fixed or face_velocity; got " + yaw_mode_);
    }

    const auto gains_section = controller_section["gains"];
    gains_.kp_position = resolve_configurable_double(
      "kp_position", yaml_double(gains_section, "kp_position", gains_.kp_position), 0.0);
    gains_.kd_velocity = resolve_configurable_double(
      "kd_velocity", yaml_double(gains_section, "kd_velocity", gains_.kd_velocity), 0.0);

    const auto smoothing_section = controller_section["smoothing"];
    smoothing_.max_acceleration_m_s2 = resolve_configurable_double(
      "max_acceleration_m_s2",
      yaml_double(
        smoothing_section, "max_acceleration_m_s2", smoothing_.max_acceleration_m_s2),
      0.1);
    smoothing_.max_jerk_m_s3 = resolve_configurable_double(
      "max_jerk_m_s3",
      yaml_double(smoothing_section, "max_jerk_m_s3", smoothing_.max_jerk_m_s3), 0.1);

    const auto parameter_alpha = get_parameter("low_pass_alpha").as_double();
    smoothing_.low_pass_alpha = parameter_alpha >= 0.0 ? parameter_alpha :
      yaml_double(smoothing_section, "low_pass_alpha", smoothing_.low_pass_alpha);
    smoothing_.low_pass_alpha = clamp01(smoothing_.low_pass_alpha);

    leader_prediction_horizon_sec_ = resolve_configurable_double(
      "leader_prediction_horizon_sec",
      yaml_double(controller_section, "leader_prediction_horizon_sec", 1.0), 0.0);
  }

  double resolve_configurable_double(
    const std::string & parameter_name, double config_value, double min_value) const
  {
    const double parameter_value = get_parameter(parameter_name).as_double();
    const double value = parameter_value > 0.0 ? parameter_value : config_value;
    if (value < min_value) {
      throw std::runtime_error(
        parameter_name + " must be >= " + std::to_string(min_value) + ", got " +
        std::to_string(value));
    }
    return value;
  }

  void swarm_state_callback(const SwarmState::SharedPtr msg)
  {
    state_by_id_.clear();
    for (const auto & state : msg->drones) {
      state_by_id_[state.drone_id] = state;
    }
  }

  void timer_callback()
  {
    // leader 状态不可用时，全队进入 hold，避免 follower 追逐过期目标。
    const auto * leader_state = find_state(leader_id_);
    if (!state_is_usable(leader_state)) {
      publish_inactive_leader_reference("leader_state_lost");
      publish_leader_lost_hold();
      return;
    }

    std::map<std::string, Point> planned_targets;
    publish_leader_target(*leader_state, planned_targets);
    publish_leader_reference(*leader_state);
    if (!distributed_followers_) {
      publish_follower_targets(*leader_state, planned_targets);
    }
  }

  void publish_leader_target(
    const DroneState & leader_state, std::map<std::string, Point> & planned_targets)
  {
    auto target = route_.at(waypoint_index_);
    if (distance(leader_state.position, target) <= acceptance_radius_) {
      advance_waypoint();
      target = route_.at(waypoint_index_);
    }

    target = prepare_target_point(leader_state.position, target);
    const auto reason = safety_violation(leader_id_, target, planned_targets);
    if (!reason.empty()) {
      publish_hold(leader_id_, &leader_state, reason);
      return;
    }

    planned_targets[leader_id_] = target;
    auto velocity = use_velocity_targets_ ?
      velocity_toward_target(leader_state.position, target) : Vector3{};
    if (use_velocity_targets_) {
      velocity = apply_soft_separation_velocity(leader_id_, leader_state.position, velocity);
    }
    publish_target(
      leader_id_, target, velocity, use_velocity_targets_,
      "formation_controller.leader_waypoints", true, yaw_for_velocity(velocity));
  }

  void publish_leader_reference(const DroneState & leader_state)
  {
    FormationTarget msg{};
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.drone_id = leader_id_;
    msg.source = "formation_controller.leader_reference";
    msg.position = predict_position(leader_state);
    msg.velocity = leader_state.velocity;
    msg.yaw = leader_state.yaw;
    msg.use_velocity = true;
    msg.active = true;
    leader_reference_pub_->publish(msg);
  }

  void publish_inactive_leader_reference(const std::string & reason)
  {
    FormationTarget msg{};
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.drone_id = leader_id_;
    msg.source = "formation_controller.leader_reference_inactive:" + reason;
    msg.yaw = leader_yaw_;
    msg.use_velocity = false;
    msg.active = false;
    leader_reference_pub_->publish(msg);
  }

  void publish_follower_targets(
    const DroneState & leader_state, std::map<std::string, Point> & planned_targets)
  {
    for (const auto & drone : drone_configs_) {
      if (drone.drone_id == leader_id_) {
        continue;
      }

      const auto * follower_state = find_state(drone.drone_id);
      if (!state_is_usable(follower_state)) {
        publish_inactive_hold(drone.drone_id, follower_state, "px4_state_unhealthy");
        continue;
      }

      // follower 使用轻量常速度预测补偿状态延迟，减少编队在转弯和加速时的滞后。
      const auto predicted_leader_position = predict_position(leader_state);
      const auto desired = target_from_leader(
        predicted_leader_position, leader_state.yaw, drone.drone_id);
      const auto target = use_velocity_targets_ ? clamp_altitude(desired) :
        prepare_target_point(follower_state->position, desired);
      const auto reason = safety_violation(drone.drone_id, target, planned_targets);
      if (!reason.empty()) {
        publish_hold(drone.drone_id, follower_state, reason);
        continue;
      }

      const auto raw_velocity = use_velocity_targets_ ?
        raw_follower_velocity_command(drone.drone_id, *follower_state, leader_state, target) :
        Vector3{};
      const auto velocity = use_velocity_targets_ ?
        smooth_velocity_command(drone.drone_id, raw_velocity) :
        Vector3{};
      planned_targets[drone.drone_id] = target;
      publish_target(
        drone.drone_id, target, velocity, use_velocity_targets_,
        "formation_controller." + formation_type_, true,
        yaw_for_velocity(raw_velocity, leader_state.yaw));
    }
  }

  void advance_waypoint()
  {
    const auto previous_index = waypoint_index_;
    if (waypoint_index_ + 1 < route_.size()) {
      ++waypoint_index_;
    } else if (loop_route_) {
      waypoint_index_ = 0;
    }

    if (waypoint_index_ != previous_index) {
      RCLCPP_INFO(get_logger(), "Leader waypoint advanced: index=%zu", waypoint_index_);
    }
  }

  Vector3 velocity_toward_target(const Point & current, const Point & target) const
  {
    const auto delta = vector_between(current, target);
    const auto target_distance = norm(delta);
    if (target_distance <= 1e-6) {
      return Vector3{};
    }
    return limit_vector(scale_vector(delta, control_rate_hz_), safety_.max_speed_m_s);
  }

  Point predict_position(const DroneState & state) const
  {
    const auto age = std::min(state_age_sec(state), leader_prediction_horizon_sec_);
    return translate_point(state.position, scale_vector(state.velocity, age));
  }

  Point target_from_leader(
    const Point & leader_position, double leader_yaw, const std::string & drone_id) const
  {
    const auto offset = offsets_.at(drone_id);
    const auto offset_vector = offset_vector_for_frame(offset, leader_yaw);
    Point target{};
    target.x = leader_position.x + offset_vector.x;
    target.y = leader_position.y + offset_vector.y;
    target.z = leader_position.z + offset_vector.z;
    return target;
  }

  Vector3 offset_vector_for_frame(
    const std::array<double, 3> & offset, double leader_yaw) const
  {
    if (offset_frame_ != "body_forward_right_up") {
      return make_vector(offset[0], offset[1], offset[2]);
    }

    // body_forward_right_up 下，offset[0] 是 leader 前向，offset[1] 是右向。
    // 这里按 leader yaw 旋转到 local_enu 世界坐标。
    const auto forward = offset[0];
    const auto right = offset[1];
    const auto cos_yaw = std::cos(leader_yaw);
    const auto sin_yaw = std::sin(leader_yaw);
    return make_vector(
      forward * cos_yaw + right * sin_yaw,
      forward * sin_yaw - right * cos_yaw,
      offset[2]);
  }

  Vector3 raw_follower_velocity_command(
    const std::string & drone_id, const DroneState & follower_state,
    const DroneState & leader_state, const Point & target) const
  {
    // 速度前馈取 leader 当前速度，PD 项负责消除 follower 相对目标的误差。
    const auto position_error = vector_between(follower_state.position, target);
    const auto velocity_error = subtract_vectors(leader_state.velocity, follower_state.velocity);
    auto command = add_vectors(
      leader_state.velocity,
      add_vectors(
        scale_vector(position_error, gains_.kp_position),
        scale_vector(velocity_error, gains_.kd_velocity)));
    command = apply_soft_separation_velocity(drone_id, follower_state.position, command);
    return command;
  }

  Vector3 apply_soft_separation_velocity(
    const std::string & drone_id, const Point & current_position, const Vector3 & command) const
  {
    // 这是编队层的软分离修正，不替代 PX4 failsafe 或后续独立避障模块。
    const auto soft_radius = std::max(
      safety_.min_inter_drone_distance_m * 2.0,
      safety_.min_inter_drone_distance_m + 1.0);
    Vector3 correction{};

    for (const auto & [other_id, state] : state_by_id_) {
      if (other_id == drone_id || !state_is_usable(&state)) {
        continue;
      }

      const auto away = vector_between(state.position, current_position);
      const auto separation = norm(away);
      if (separation <= 1e-6 || separation >= soft_radius) {
        continue;
      }

      auto strength = ((soft_radius - separation) / soft_radius) * safety_.max_speed_m_s;
      if (separation < safety_.min_inter_drone_distance_m) {
        strength = std::max(strength, safety_.max_speed_m_s * 0.8);
      }
      correction = add_vectors(correction, scale_vector(away, strength / separation));
    }

    return limit_vector(add_vectors(command, correction), safety_.max_speed_m_s);
  }

  Vector3 smooth_velocity_command(const std::string & drone_id, const Vector3 & raw_command)
  {
    // 每架 follower 独立维护速度历史，用加速度、jerk 和低通限制压住 setpoint 突变。
    const auto dt = 1.0 / control_rate_hz_;
    auto command = limit_vector(raw_command, safety_.max_speed_m_s);
    auto & history = control_history_by_drone_[drone_id];

    if (history.has_velocity) {
      auto acceleration = scale_vector(
        subtract_vectors(command, history.filtered_velocity), 1.0 / dt);
      acceleration = limit_vector(acceleration, smoothing_.max_acceleration_m_s2);

      if (history.has_acceleration) {
        auto jerk_step = subtract_vectors(acceleration, history.acceleration);
        jerk_step = limit_vector(jerk_step, smoothing_.max_jerk_m_s3 * dt);
        acceleration = add_vectors(history.acceleration, jerk_step);
      }

      command = add_vectors(history.filtered_velocity, scale_vector(acceleration, dt));
      const auto alpha = smoothing_.low_pass_alpha;
      command = add_vectors(
        scale_vector(command, alpha),
        scale_vector(history.filtered_velocity, 1.0 - alpha));
      command = limit_vector(command, safety_.max_speed_m_s);

      history.acceleration = scale_vector(
        subtract_vectors(command, history.filtered_velocity), 1.0 / dt);
      history.has_acceleration = true;
    } else {
      history.acceleration = Vector3{};
      history.has_acceleration = true;
    }

    history.filtered_velocity = command;
    history.has_velocity = true;
    return command;
  }

  double yaw_for_velocity(const Vector3 & velocity) const
  {
    return yaw_for_velocity(velocity, leader_yaw_);
  }

  double yaw_for_velocity(const Vector3 & velocity, double fallback_yaw) const
  {
    if (yaw_mode_ != "face_velocity") {
      return leader_yaw_;
    }
    const auto horizontal_speed = std::hypot(velocity.x, velocity.y);
    if (horizontal_speed < 0.15) {
      return fallback_yaw;
    }
    return std::atan2(velocity.y, velocity.x);
  }

  Point prepare_target_point(const Point & current_position, const Point & desired) const
  {
    return limit_step(current_position, clamp_altitude(desired));
  }

  Point clamp_altitude(const Point & point) const
  {
    auto target = copy_point(point);
    target.z = clamp(target.z, safety_.min_altitude_m, safety_.max_altitude_m);
    return target;
  }

  Point limit_step(const Point & current, const Point & desired) const
  {
    const auto total_distance = distance(current, desired);
    const auto max_step = safety_.max_speed_m_s / control_rate_hz_;
    if (total_distance <= max_step || total_distance == 0.0) {
      return desired;
    }

    const auto scale = max_step / total_distance;
    Point target{};
    target.x = current.x + (desired.x - current.x) * scale;
    target.y = current.y + (desired.y - current.y) * scale;
    target.z = current.z + (desired.z - current.z) * scale;
    return target;
  }

  std::string safety_violation(
    const std::string & drone_id, const Point & target,
    const std::map<std::string, Point> & planned_targets) const
  {
    if (target.z > safety_.max_altitude_m) {
      return "target_above_max_altitude";
    }
    if (target.z < safety_.min_altitude_m) {
      return "target_below_min_altitude";
    }

    for (const auto & [other_id, other_target] : planned_targets) {
      if (other_id != drone_id &&
        distance(target, other_target) < safety_.min_inter_drone_distance_m)
      {
        return "planned_spacing_too_small:" + other_id;
      }
    }

    return "";
  }

  bool state_is_usable(const DroneState * state) const
  {
    if (state == nullptr || !state->healthy) {
      return false;
    }
    return state_age_sec(*state) <= leader_state_timeout_sec_;
  }

  double state_age_sec(const DroneState & state) const
  {
    const rclcpp::Time stamp(state.header.stamp);
    const auto age = (now() - stamp).seconds();
    return std::max(0.0, age);
  }

  const DroneState * find_state(const std::string & drone_id) const
  {
    const auto item = state_by_id_.find(drone_id);
    if (item == state_by_id_.end()) {
      return nullptr;
    }
    return &item->second;
  }

  void publish_leader_lost_hold()
  {
    for (const auto & drone : drone_configs_) {
      if (distributed_followers_ && drone.drone_id != leader_id_) {
        continue;
      }
      const auto * state = find_state(drone.drone_id);
      if (state_is_usable(state)) {
        publish_hold(drone.drone_id, state, "leader_state_lost");
      } else {
        publish_inactive_hold(drone.drone_id, state, "px4_state_unhealthy");
      }
    }
  }

  void publish_hold(
    const std::string & drone_id, const DroneState * state, const std::string & reason)
  {
    publish_hold_target(drone_id, state, reason, true);
  }

  void publish_inactive_hold(
    const std::string & drone_id, const DroneState * state, const std::string & reason)
  {
    publish_hold_target(drone_id, state, reason, false);
  }

  void publish_hold_target(
    const std::string & drone_id, const DroneState * state, const std::string & reason,
    bool active)
  {
    // hold 会清掉该机速度滤波历史，恢复控制时从当前状态重新平滑起步。
    control_history_by_drone_.erase(drone_id);
    const auto position = state ? copy_point(state->position) : Point{};
    const auto yaw = state ? state->yaw : leader_yaw_;
    log_hold_once(drone_id, reason);
    publish_target(drone_id, position, "formation_controller.hold:" + reason, active, yaw);
  }

  void log_hold_once(const std::string & drone_id, const std::string & reason)
  {
    if (last_hold_reason_by_drone_[drone_id] == reason) {
      return;
    }
    last_hold_reason_by_drone_[drone_id] = reason;
    RCLCPP_WARN(get_logger(), "Holding %s: %s", drone_id.c_str(), reason.c_str());
  }

  void publish_target(
    const std::string & drone_id, const Point & position, const std::string & source,
    bool active)
  {
    publish_target(drone_id, position, source, active, leader_yaw_);
  }

  void publish_target(
    const std::string & drone_id, const Point & position, const std::string & source,
    bool active, double yaw)
  {
    publish_target(drone_id, position, Vector3{}, false, source, active, yaw);
  }

  void publish_target(
    const std::string & drone_id, const Point & position, const Vector3 & velocity,
    bool use_velocity, const std::string & source, bool active)
  {
    publish_target(drone_id, position, velocity, use_velocity, source, active, leader_yaw_);
  }

  void publish_target(
    const std::string & drone_id, const Point & position, const Vector3 & velocity,
    bool use_velocity, const std::string & source, bool active, double yaw)
  {
    const auto publisher = target_publishers_.find(drone_id);
    if (publisher == target_publishers_.end()) {
      return;
    }

    if (active && last_hold_reason_by_drone_.erase(drone_id) > 0) {
      RCLCPP_INFO(get_logger(), "Resuming target publication for %s", drone_id.c_str());
    }

    FormationTarget msg{};
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.drone_id = drone_id;
    msg.source = source;
    msg.position = copy_point(position);
    msg.velocity = velocity;
    msg.yaw = yaw;
    msg.use_velocity = use_velocity;
    msg.active = active;
    publisher->second->publish(msg);
  }

  YAML::Node swarm_config_;
  YAML::Node formations_config_;
  YAML::Node waypoints_config_;
  std::vector<DroneConfig> drone_configs_;
  std::vector<std::string> follower_ids_;
  std::string leader_id_;
  std::string formation_type_;
  std::string frame_id_;
  std::map<std::string, std::array<double, 3>> offsets_;
  std::vector<Point> route_;
  bool loop_route_{true};
  double leader_yaw_{0.0};
  double control_rate_hz_{10.0};
  double acceptance_radius_{0.45};
  double leader_state_timeout_sec_{2.0};
  double leader_prediction_horizon_sec_{1.0};
  bool distributed_followers_{false};
  std::string control_mode_{"position_velocity"};
  std::string offset_frame_{"body_forward_right_up"};
  std::string yaw_mode_{"fixed"};
  bool use_velocity_targets_{true};
  SafetyConfig safety_;
  ControlGains gains_;
  SmoothingConfig smoothing_;
  std::map<std::string, DroneState> state_by_id_;
  std::size_t waypoint_index_{0};
  std::map<std::string, ControlHistory> control_history_by_drone_;
  std::map<std::string, std::string> last_hold_reason_by_drone_;
  std::map<std::string, rclcpp::Publisher<FormationTarget>::SharedPtr> target_publishers_;
  rclcpp::Publisher<FormationTarget>::SharedPtr leader_reference_pub_;
  rclcpp::Subscription<SwarmState>::SharedPtr swarm_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<FormationController>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("formation_controller"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
