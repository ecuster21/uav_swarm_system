#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
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
using SwarmState = swarm_msgs::msg::SwarmState;
using Point = geometry_msgs::msg::Point;
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

double clamp(double value, double low, double high)
{
  return std::max(low, std::min(high, value));
}

double clamp01(double value)
{
  return clamp(value, 0.0, 1.0);
}

double yaml_double(const YAML::Node & node, const std::string & key, double default_value)
{
  return node && node[key] ? node[key].as<double>() : default_value;
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

Vector3 make_vector(double x, double y, double z)
{
  Vector3 vector{};
  vector.x = x;
  vector.y = y;
  vector.z = z;
  return vector;
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

Vector3 vector_between(const Point & from, const Point & to)
{
  return make_vector(to.x - from.x, to.y - from.y, to.z - from.z);
}

double norm(const Vector3 & vector)
{
  return std::sqrt(
    vector.x * vector.x + vector.y * vector.y + vector.z * vector.z);
}

Vector3 limit_vector(const Vector3 & vector, double max_norm)
{
  const auto vector_norm = norm(vector);
  if (vector_norm <= max_norm || vector_norm <= 1e-9 || max_norm <= 0.0) {
    return vector;
  }
  return scale_vector(vector, max_norm / vector_norm);
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

}  // namespace

class LocalFollowerController : public rclcpp::Node
{
public:
  LocalFollowerController()
  : Node("local_follower_controller")
  {
    // 本节点运行在 /uav_N namespace 下，只计算并发布本机 formation_target。
    declare_parameter<std::string>("drone_id", "");
    declare_parameter<std::string>("leader_id", "");
    declare_parameter<std::string>("swarm_config_file", "");
    declare_parameter<std::string>("formations_config_file", "");
    declare_parameter<std::string>("formation_type", "");
    declare_parameter<double>("control_rate_hz", 0.0);
    declare_parameter<double>("state_timeout_sec", 0.0);
    declare_parameter<double>("leader_reference_timeout_sec", 0.0);
    declare_parameter<double>("max_speed_m_s", 0.0);
    declare_parameter<double>("max_altitude_m", 0.0);
    declare_parameter<double>("min_altitude_m", 0.0);
    declare_parameter<std::string>("control_mode", "");
    declare_parameter<std::string>("offset_frame", "");
    declare_parameter<std::string>("yaw_mode", "");
    declare_parameter<double>("kp_position", 0.0);
    declare_parameter<double>("kd_velocity", 0.0);
    declare_parameter<double>("max_acceleration_m_s2", 0.0);
    declare_parameter<double>("max_jerk_m_s3", 0.0);
    declare_parameter<double>("low_pass_alpha", -1.0);

    drone_id_ = get_parameter("drone_id").as_string();
    const auto swarm_config_file = get_parameter("swarm_config_file").as_string();
    const auto formations_config_file = get_parameter("formations_config_file").as_string();
    if (drone_id_.empty() || swarm_config_file.empty() || formations_config_file.empty()) {
      throw std::runtime_error(
        "drone_id, swarm_config_file, and formations_config_file are required");
    }

    swarm_config_ = YAML::LoadFile(swarm_config_file);
    formations_config_ = YAML::LoadFile(formations_config_file);
    validate_frame_config();

    const auto swarm_section = swarm_config_["swarm"];
    if (!swarm_section) {
      throw std::runtime_error("swarm_config_file must contain swarm section");
    }
    leader_id_ = get_parameter("leader_id").as_string();
    if (leader_id_.empty()) {
      leader_id_ = swarm_section["leader_id"].as<std::string>();
    }
    if (drone_id_ == leader_id_) {
      throw std::runtime_error("local_follower_controller must not run for leader " + drone_id_);
    }

    drone_configs_ = load_drone_configs(swarm_section);
    const auto controller_section = formations_config_["controller"];
    formation_type_ = resolve_formation_type(controller_section);
    const auto formation_section = formations_config_["formations"][formation_type_];
    if (!formation_section) {
      throw std::runtime_error("Unknown formation_type: " + formation_type_);
    }
    offsets_ = load_or_generate_offsets(formation_section);
    if (offsets_.count(drone_id_) == 0) {
      throw std::runtime_error("Formation " + formation_type_ + " has no offset for " + drone_id_);
    }

    control_rate_hz_ = resolve_double(
      "control_rate_hz", yaml_double(controller_section, "control_rate_hz", 10.0), 1.0);
    state_timeout_sec_ = resolve_double(
      "state_timeout_sec", yaml_double(swarm_section, "state_timeout_sec", 2.0), 0.1);
    leader_reference_timeout_sec_ = resolve_double(
      "leader_reference_timeout_sec",
      yaml_double(controller_section, "leader_state_timeout_sec", 2.0), 0.1);
    safety_ = load_safety_config();
    load_control_config(controller_section);

    state_sub_ = create_subscription<DroneState>(
      "state", 10,
      [this](const DroneState::SharedPtr msg) {
        if (msg->drone_id == drone_id_) {
          latest_state_ = msg;
          latest_state_received_ns_ = now().nanoseconds();
        }
      });
    leader_reference_sub_ = create_subscription<FormationTarget>(
      "/swarm/leader_reference", 10,
      [this](const FormationTarget::SharedPtr msg) {
        if (msg->drone_id.empty() || msg->drone_id == leader_id_) {
          latest_leader_reference_ = msg;
          latest_leader_reference_received_ns_ = now().nanoseconds();
        }
      });
    swarm_state_sub_ = create_subscription<SwarmState>(
      "/swarm/state", 10,
      [this](const SwarmState::SharedPtr msg) {
        if (!msg->leader_id.empty() && msg->leader_id != leader_id_) {
          return;
        }
        for (const auto & drone : msg->drones) {
          if (drone.drone_id != leader_id_) {
            continue;
          }
          latest_swarm_leader_reference_ = leader_reference_from_state(drone);
          latest_swarm_leader_reference_received_ns_ = now().nanoseconds();
          return;
        }
      });
    target_pub_ = create_publisher<FormationTarget>("formation_target", 10);

    const auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {
        timer_callback();
      });

    RCLCPP_INFO(
      get_logger(),
      "Local follower controller ready: drone=%s, leader=%s, formation_type=%s, mode=%s, offset_frame=%s, yaw_mode=%s, rate=%.1f Hz, state_timeout=%.1f s, leader_timeout=%.1f s",
      drone_id_.c_str(), leader_id_.c_str(), formation_type_.c_str(), control_mode_.c_str(),
      offset_frame_.c_str(), yaw_mode_.c_str(), control_rate_hz_, state_timeout_sec_,
      leader_reference_timeout_sec_);
  }

private:
  void validate_frame_config() const
  {
    const auto frame_section = formations_config_["frame"];
    const auto command_frame = yaml_string(frame_section, "command_frame", "local_enu");
    const auto px4_frame = yaml_string(frame_section, "px4_internal_frame", "local_ned");
    if (command_frame != "local_enu") {
      throw std::runtime_error("local_follower_controller currently publishes local_enu targets only");
    }
    if (px4_frame != "local_ned") {
      throw std::runtime_error("PX4 bridge contract expects PX4 local_ned internally");
    }
  }

  std::string resolve_formation_type(const YAML::Node & controller_section)
  {
    auto formation_type = get_parameter("formation_type").as_string();
    if (formation_type.empty()) {
      formation_type = yaml_string(controller_section, "default_formation_type", "triangle");
    }

    const std::vector<std::string> allowed{"triangle", "line", "column"};
    if (std::find(allowed.begin(), allowed.end(), formation_type) == allowed.end()) {
      throw std::runtime_error(
        "formation_type must be one of: triangle, line, column; got " + formation_type);
    }
    return formation_type;
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
      const auto drone_namespace = trim_slashes(yaml_string(item, "namespace", drone_id));
      drones.push_back(
        DroneConfig{
          drone_id,
          drone_namespace.empty() ? drone_id : drone_namespace,
          yaml_string(item, "role", "unknown")});
    }
    return drones;
  }

  std::map<std::string, std::array<double, 3>> load_or_generate_offsets(
    const YAML::Node & formation_section) const
  {
    std::map<std::string, std::array<double, 3>> offsets;
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
      throw std::runtime_error("yaw_mode must be fixed or face_velocity; got " + yaw_mode_);
    }

    const auto gains_section = controller_section["gains"];
    gains_.kp_position = resolve_configurable_double(
      "kp_position", yaml_double(gains_section, "kp_position", gains_.kp_position), 0.0);
    gains_.kd_velocity = resolve_configurable_double(
      "kd_velocity", yaml_double(gains_section, "kd_velocity", gains_.kd_velocity), 0.0);

    const auto smoothing_section = controller_section["smoothing"];
    smoothing_.max_acceleration_m_s2 = resolve_configurable_double(
      "max_acceleration_m_s2",
      yaml_double(smoothing_section, "max_acceleration_m_s2", smoothing_.max_acceleration_m_s2),
      0.1);
    smoothing_.max_jerk_m_s3 = resolve_configurable_double(
      "max_jerk_m_s3",
      yaml_double(smoothing_section, "max_jerk_m_s3", smoothing_.max_jerk_m_s3), 0.1);
    const auto parameter_alpha = get_parameter("low_pass_alpha").as_double();
    smoothing_.low_pass_alpha = parameter_alpha >= 0.0 ? parameter_alpha :
      yaml_double(smoothing_section, "low_pass_alpha", smoothing_.low_pass_alpha);
    smoothing_.low_pass_alpha = clamp01(smoothing_.low_pass_alpha);
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

  void timer_callback()
  {
    if (!state_is_usable(latest_state_)) {
      publish_inactive_hold("own_state_unhealthy");
      return;
    }

    const auto leader_reference = usable_leader_reference();
    if (!leader_reference) {
      publish_hold(leader_reference_loss_reason());
      return;
    }

    const auto desired = target_from_leader_reference(*leader_reference);
    const auto target = use_velocity_targets_ ? clamp_altitude(desired) :
      prepare_target_point(latest_state_->position, desired);

    const auto raw_velocity = use_velocity_targets_ ?
      raw_velocity_command(*latest_state_, *leader_reference, target) :
      Vector3{};
    const auto velocity = use_velocity_targets_ ? smooth_velocity_command(raw_velocity) :
      Vector3{};

    publish_target(
      target, velocity, use_velocity_targets_,
      "local_follower_controller." + formation_type_ + leader_reference_source_suffix(*leader_reference),
      true, yaw_for_velocity(raw_velocity, leader_reference->yaw));
  }

  bool state_is_usable(const DroneState::SharedPtr & state) const
  {
    if (!state || !state->healthy) {
      return false;
    }
    return received_age_sec(latest_state_received_ns_) <= state_timeout_sec_;
  }

  bool reference_is_usable(
    const FormationTarget::SharedPtr & reference, int64_t received_time_ns) const
  {
    if (!reference || !reference->active) {
      return false;
    }
    return received_age_sec(received_time_ns) <= leader_reference_timeout_sec_;
  }

  FormationTarget::SharedPtr usable_leader_reference() const
  {
    if (reference_is_usable(latest_leader_reference_, latest_leader_reference_received_ns_)) {
      return latest_leader_reference_;
    }
    if (reference_is_usable(
        latest_swarm_leader_reference_, latest_swarm_leader_reference_received_ns_))
    {
      return latest_swarm_leader_reference_;
    }
    return nullptr;
  }

  std::string leader_reference_loss_reason() const
  {
    if (!latest_leader_reference_ && !latest_swarm_leader_reference_) {
      return "leader_reference_missing";
    }
    if ((latest_leader_reference_ && !latest_leader_reference_->active) ||
      (latest_swarm_leader_reference_ && !latest_swarm_leader_reference_->active))
    {
      return "leader_reference_inactive";
    }
    return "leader_reference_stale";
  }

  FormationTarget::SharedPtr leader_reference_from_state(const DroneState & state) const
  {
    auto reference = std::make_shared<FormationTarget>();
    reference->header = state.header;
    reference->header.frame_id = state.header.frame_id.empty() ? "local_enu" : state.header.frame_id;
    reference->drone_id = state.drone_id;
    reference->source = "local_follower_controller.leader_reference_from_swarm_state";
    reference->position = state.position;
    reference->velocity = state.velocity;
    reference->yaw = state.yaw;
    reference->use_velocity = true;
    reference->active = state.healthy;
    return reference;
  }

  std::string leader_reference_source_suffix(const FormationTarget & reference) const
  {
    if (reference.source == "local_follower_controller.leader_reference_from_swarm_state") {
      return ":swarm_state_fallback";
    }
    return "";
  }

  double received_age_sec(int64_t received_time_ns) const
  {
    if (received_time_ns <= 0) {
      return std::numeric_limits<double>::infinity();
    }
    const auto age_ns = now().nanoseconds() - received_time_ns;
    return std::max(0.0, static_cast<double>(age_ns) / 1e9);
  }

  Point target_from_leader_reference(const FormationTarget & leader_reference) const
  {
    const auto offset = offsets_.at(drone_id_);
    const auto offset_vector = offset_vector_for_frame(offset, leader_reference.yaw);
    Point target{};
    target.x = leader_reference.position.x + offset_vector.x;
    target.y = leader_reference.position.y + offset_vector.y;
    target.z = leader_reference.position.z + offset_vector.z;
    return target;
  }

  Vector3 offset_vector_for_frame(
    const std::array<double, 3> & offset, double leader_yaw) const
  {
    if (offset_frame_ != "body_forward_right_up") {
      return make_vector(offset[0], offset[1], offset[2]);
    }
    const auto forward = offset[0];
    const auto right = offset[1];
    const auto cos_yaw = std::cos(leader_yaw);
    const auto sin_yaw = std::sin(leader_yaw);
    return make_vector(
      forward * cos_yaw + right * sin_yaw,
      forward * sin_yaw - right * cos_yaw,
      offset[2]);
  }

  Vector3 raw_velocity_command(
    const DroneState & follower_state, const FormationTarget & leader_reference,
    const Point & target) const
  {
    const auto position_error = vector_between(follower_state.position, target);
    const auto velocity_error = subtract_vectors(leader_reference.velocity, follower_state.velocity);
    auto command = add_vectors(
      leader_reference.velocity,
      add_vectors(
        scale_vector(position_error, gains_.kp_position),
        scale_vector(velocity_error, gains_.kd_velocity)));
    return limit_vector(command, safety_.max_speed_m_s);
  }

  Vector3 smooth_velocity_command(const Vector3 & raw_command)
  {
    const auto dt = 1.0 / control_rate_hz_;
    auto command = limit_vector(raw_command, safety_.max_speed_m_s);

    if (control_history_.has_velocity) {
      auto acceleration = scale_vector(
        subtract_vectors(command, control_history_.filtered_velocity), 1.0 / dt);
      acceleration = limit_vector(acceleration, smoothing_.max_acceleration_m_s2);

      if (control_history_.has_acceleration) {
        auto jerk_step = subtract_vectors(acceleration, control_history_.acceleration);
        jerk_step = limit_vector(jerk_step, smoothing_.max_jerk_m_s3 * dt);
        acceleration = add_vectors(control_history_.acceleration, jerk_step);
      }

      command = add_vectors(control_history_.filtered_velocity, scale_vector(acceleration, dt));
      const auto alpha = smoothing_.low_pass_alpha;
      command = add_vectors(
        scale_vector(command, alpha),
        scale_vector(control_history_.filtered_velocity, 1.0 - alpha));
      command = limit_vector(command, safety_.max_speed_m_s);

      control_history_.acceleration = scale_vector(
        subtract_vectors(command, control_history_.filtered_velocity), 1.0 / dt);
      control_history_.has_acceleration = true;
    } else {
      control_history_.acceleration = Vector3{};
      control_history_.has_acceleration = true;
    }

    control_history_.filtered_velocity = command;
    control_history_.has_velocity = true;
    return command;
  }

  Point clamp_altitude(const Point & point) const
  {
    auto target = copy_point(point);
    target.z = clamp(target.z, safety_.min_altitude_m, safety_.max_altitude_m);
    return target;
  }

  Point prepare_target_point(const Point & current_position, const Point & desired) const
  {
    return limit_step(current_position, clamp_altitude(desired));
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

  double yaw_for_velocity(const Vector3 & velocity, double fallback_yaw) const
  {
    if (yaw_mode_ != "face_velocity") {
      return fallback_yaw;
    }
    const auto horizontal_speed = std::hypot(velocity.x, velocity.y);
    if (horizontal_speed < 0.15) {
      return fallback_yaw;
    }
    return std::atan2(velocity.y, velocity.x);
  }

  void publish_hold(const std::string & reason)
  {
    control_history_ = ControlHistory{};
    const auto yaw = latest_state_ ? latest_state_->yaw : 0.0;
    const auto position = latest_state_ ? copy_point(latest_state_->position) : Point{};
    publish_target(
      position, Vector3{}, false, "local_follower_controller.hold:" + reason, true, yaw);
  }

  void publish_inactive_hold(const std::string & reason)
  {
    control_history_ = ControlHistory{};
    const auto yaw = latest_state_ ? latest_state_->yaw : 0.0;
    const auto position = latest_state_ ? copy_point(latest_state_->position) : Point{};
    publish_target(
      position, Vector3{}, false, "local_follower_controller.hold:" + reason, false, yaw);
  }

  void publish_target(
    const Point & position, const Vector3 & velocity, bool use_velocity,
    const std::string & source, bool active, double yaw)
  {
    FormationTarget msg{};
    msg.header.stamp = now();
    msg.header.frame_id = "local_enu";
    msg.drone_id = drone_id_;
    msg.source = source;
    msg.position = copy_point(position);
    msg.velocity = velocity;
    msg.yaw = yaw;
    msg.use_velocity = use_velocity;
    msg.active = active;
    target_pub_->publish(msg);
  }

  YAML::Node swarm_config_;
  YAML::Node formations_config_;
  std::vector<DroneConfig> drone_configs_;
  std::string drone_id_;
  std::string leader_id_;
  std::string formation_type_;
  std::map<std::string, std::array<double, 3>> offsets_;
  double control_rate_hz_{10.0};
  double state_timeout_sec_{2.0};
  double leader_reference_timeout_sec_{2.0};
  std::string control_mode_{"position_velocity"};
  std::string offset_frame_{"body_forward_right_up"};
  std::string yaw_mode_{"fixed"};
  bool use_velocity_targets_{true};
  SafetyConfig safety_;
  ControlGains gains_;
  SmoothingConfig smoothing_;
  ControlHistory control_history_;

  DroneState::SharedPtr latest_state_;
  FormationTarget::SharedPtr latest_leader_reference_;
  FormationTarget::SharedPtr latest_swarm_leader_reference_;
  int64_t latest_state_received_ns_{0};
  int64_t latest_leader_reference_received_ns_{0};
  int64_t latest_swarm_leader_reference_received_ns_{0};
  rclcpp::Subscription<DroneState>::SharedPtr state_sub_;
  rclcpp::Subscription<FormationTarget>::SharedPtr leader_reference_sub_;
  rclcpp::Subscription<SwarmState>::SharedPtr swarm_state_sub_;
  rclcpp::Publisher<FormationTarget>::SharedPtr target_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<LocalFollowerController>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("local_follower_controller"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
