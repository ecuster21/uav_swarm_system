from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from swarm_msgs.msg import DroneState, SwarmState
from swarm_manager.config import load_yaml, normalize_namespace


@dataclass(frozen=True)
class DroneConfig:
    drone_id: str
    namespace: str
    role: str


def _role_to_constant(role_name: str) -> int:
    normalized = role_name.lower().strip()
    if normalized == "leader":
        return DroneState.ROLE_LEADER
    if normalized == "follower":
        return DroneState.ROLE_FOLLOWER
    return DroneState.ROLE_UNKNOWN


class SwarmManager(Node):
    def __init__(self) -> None:
        super().__init__("swarm_manager")
        self.declare_parameter("swarm_config_file", "")
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("state_timeout_sec", 2.0)
        self.declare_parameter("frame_id", "local_enu")

        config_file = self.get_parameter("swarm_config_file").value
        if not config_file:
            raise ValueError("swarm_config_file parameter is required")

        config = load_yaml(config_file)
        swarm_config = config.get("swarm", {})
        self.leader_id = str(swarm_config.get("leader_id", ""))
        self.drone_configs = self._load_drone_configs(swarm_config)
        self.publish_rate_hz = float(
            swarm_config.get("publish_rate_hz", self.get_parameter("publish_rate_hz").value)
        )
        self.state_timeout_sec = float(
            swarm_config.get(
                "state_timeout_sec", self.get_parameter("state_timeout_sec").value
            )
        )
        self.frame_id = str(
            swarm_config.get("frame_id", self.get_parameter("frame_id").value)
        )

        self.states: dict[str, DroneState] = {}
        self.last_seen: dict[str, Time] = {}
        self.swarm_pub = self.create_publisher(SwarmState, "/swarm/state", 10)
        self.state_subscriptions = []

        for drone in self.drone_configs:
            topic = f"/{drone.namespace}/state"
            self.state_subscriptions.append(
                self.create_subscription(
                    DroneState,
                    topic,
                    lambda msg, drone_id=drone.drone_id: self._state_callback(
                        drone_id, msg
                    ),
                    10,
                )
            )
            self.get_logger().info(f"Listening to {topic} for {drone.drone_id}")

        period = 1.0 / max(self.publish_rate_hz, 1.0)
        self.timer = self.create_timer(period, self._publish_swarm_state)
        self.get_logger().info(
            f"Swarm manager ready for {len(self.drone_configs)} drones; "
            f"leader={self.leader_id}"
        )

    def _load_drone_configs(self, swarm_config: dict) -> list[DroneConfig]:
        drones = []
        for item in swarm_config.get("drones", []):
            drone_id = str(item["id"])
            namespace = normalize_namespace(str(item.get("namespace", drone_id)))
            role = str(item.get("role", "unknown"))
            drones.append(DroneConfig(drone_id=drone_id, namespace=namespace, role=role))

        if not drones:
            raise ValueError("swarm.drones must contain at least one drone")

        if not self.leader_id:
            for drone in drones:
                if drone.role.lower() == "leader":
                    self.leader_id = drone.drone_id
                    break

        return drones

    def _state_callback(self, drone_id: str, msg: DroneState) -> None:
        self.states[drone_id] = msg
        self.last_seen[drone_id] = self.get_clock().now()

    def _publish_swarm_state(self) -> None:
        now = self.get_clock().now()
        msg = SwarmState()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id
        msg.leader_id = self.leader_id

        drones = []
        for drone in self.drone_configs:
            state = self.states.get(drone.drone_id)
            if state is None:
                state = self._make_missing_state(drone)
            else:
                last_seen = self.last_seen.get(drone.drone_id, now)
                age_sec = (now - last_seen).nanoseconds / 1e9
                if age_sec > self.state_timeout_sec:
                    state.healthy = False
                    state.status_text = f"state timeout: {age_sec:.2f}s"
            drones.append(state)

        msg.drones = drones
        msg.drone_count = len(drones)
        self.swarm_pub.publish(msg)

    def _make_missing_state(self, drone: DroneConfig) -> DroneState:
        msg = DroneState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.drone_id = drone.drone_id
        msg.drone_namespace = drone.namespace
        msg.role = _role_to_constant(drone.role)
        msg.armed = False
        msg.flight_mode = "UNKNOWN"
        msg.healthy = False
        msg.status_text = "waiting for state"
        return msg


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SwarmManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
