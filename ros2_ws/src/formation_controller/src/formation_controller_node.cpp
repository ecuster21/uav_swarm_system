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

double clamp(double value, double low, double high)
{
  return std::max(low, std::min(high, value));
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

    control_rate_hz_ = resolve_double(
      "control_rate_hz", yaml_double(controller_section, "control_rate_hz", 10.0), 1.0);
    acceptance_radius_ = resolve_double(
      "waypoint_acceptance_radius",
      yaml_double(controller_section, "waypoint_acceptance_radius_m", 0.45), 0.05);
    leader_state_timeout_sec_ = resolve_double(
      "leader_state_timeout_sec",
      yaml_double(controller_section, "leader_state_timeout_sec", 2.0), 0.1);
    safety_ = load_safety_config();

    for (const auto & drone : drone_configs_) {
      target_publishers_[drone.drone_id] = create_publisher<FormationTarget>(
        "/" + drone.drone_namespace + "/formation_target", 10);
    }

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
      "Formation controller ready: formation_type=%s, leader=%s, frame=%s, rate=%.1f Hz",
      formation_type_.c_str(), leader_id_.c_str(), frame_id_.c_str(), control_rate_hz_);
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

  void swarm_state_callback(const SwarmState::SharedPtr msg)
  {
    state_by_id_.clear();
    for (const auto & state : msg->drones) {
      state_by_id_[state.drone_id] = state;
    }
  }

  void timer_callback()
  {
    const auto * leader_state = find_state(leader_id_);
    if (!state_is_usable(leader_state)) {
      publish_leader_lost_hold();
      return;
    }

    std::map<std::string, Point> planned_targets;
    publish_leader_target(*leader_state, planned_targets);
    publish_follower_targets(*leader_state, planned_targets);
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
    publish_target(leader_id_, target, "formation_controller.leader_waypoints", true);
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

      const auto desired = target_from_leader(leader_state.position, drone.drone_id);
      const auto target = prepare_target_point(follower_state->position, desired);
      const auto reason = safety_violation(drone.drone_id, target, planned_targets);
      if (!reason.empty()) {
        publish_hold(drone.drone_id, follower_state, reason);
        continue;
      }

      planned_targets[drone.drone_id] = target;
      publish_target(
        drone.drone_id, target, "formation_controller." + formation_type_, true);
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

  Point target_from_leader(const Point & leader_position, const std::string & drone_id) const
  {
    const auto offset = offsets_.at(drone_id);
    Point target{};
    target.x = leader_position.x + offset[0];
    target.y = leader_position.y + offset[1];
    target.z = leader_position.z + offset[2];
    return target;
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

    for (const auto & [other_id, state] : state_by_id_) {
      if (other_id == drone_id || !state_is_usable(&state)) {
        continue;
      }
      if (distance(target, state.position) < safety_.min_inter_drone_distance_m) {
        return "spacing_too_small:" + other_id;
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
    msg.yaw = yaw;
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
  SafetyConfig safety_;
  std::map<std::string, DroneState> state_by_id_;
  std::size_t waypoint_index_{0};
  std::map<std::string, std::string> last_hold_reason_by_drone_;
  std::map<std::string, rclcpp::Publisher<FormationTarget>::SharedPtr> target_publishers_;
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
