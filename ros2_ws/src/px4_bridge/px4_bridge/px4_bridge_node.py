from math import sqrt
from typing import Sequence

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from swarm_msgs.msg import DroneState, FormationTarget

from px4_bridge.backend_base import CommandResult
from px4_bridge.mavsdk_backend import MavsdkBackend
from px4_bridge.mock_backend import MockBackend


def _role_to_constant(role_name: str) -> int:
    normalized = role_name.lower().strip()
    if normalized == "leader":
        return DroneState.ROLE_LEADER
    if normalized == "follower":
        return DroneState.ROLE_FOLLOWER
    return DroneState.ROLE_UNKNOWN


def _as_three_floats(value: Sequence[float]) -> list[float]:
    values = [float(item) for item in value]
    if len(values) != 3:
        raise ValueError("initial_position must contain exactly three values")
    return values


class Px4Bridge(Node):
    def __init__(self) -> None:
        super().__init__("px4_bridge")

        self.declare_parameter("drone_id", "uav_1")
        self.declare_parameter("drone_namespace", "uav_1")
        self.declare_parameter("role", "unknown")
        self.declare_parameter("backend_type", "mock")
        self.declare_parameter("frame_id", "local_enu")
        self.declare_parameter("system_id", 1)
        self.declare_parameter("mavlink_udp_port", 14540)
        self.declare_parameter("mavsdk_server_port", 50051)
        self.declare_parameter("initial_position", [0.0, 0.0, 0.0])
        self.declare_parameter("state_rate_hz", 10.0)
        self.declare_parameter("max_speed_m_s", 1.5)
        self.declare_parameter("auto_connect", True)
        self.declare_parameter("connection_timeout_sec", 10.0)
        self.declare_parameter("command_timeout_sec", 8.0)
        self.declare_parameter("health_required", True)
        self.declare_parameter("takeoff_altitude_m", 2.5)
        self.declare_parameter("telemetry_rate_hz", 5.0)
        self.declare_parameter(
            "connection_url_template", "udpin://0.0.0.0:{mavlink_udp_port}"
        )
        self.declare_parameter("goto_min_interval_sec", 2.0)
        self.declare_parameter("goto_position_tolerance_m", 0.5)
        self.declare_parameter("target_wait_log_interval_sec", 5.0)

        self.drone_id = self.get_parameter("drone_id").value
        self.drone_namespace = self.get_parameter("drone_namespace").value
        self.role_name = self.get_parameter("role").value
        self.backend_type = str(self.get_parameter("backend_type").value).lower()
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.state_rate_hz = float(self.get_parameter("state_rate_hz").value)
        self.goto_min_interval_sec = float(
            self.get_parameter("goto_min_interval_sec").value
        )
        self.goto_position_tolerance_m = float(
            self.get_parameter("goto_position_tolerance_m").value
        )
        self.target_wait_log_interval_sec = float(
            self.get_parameter("target_wait_log_interval_sec").value
        )
        self.last_target: FormationTarget | None = None
        self.last_command_target: FormationTarget | None = None
        self.last_command_time = None
        self.last_target_wait_log_time = None

        initial_position = _as_three_floats(self.get_parameter("initial_position").value)
        self.backend = self._create_backend(initial_position)

        self.state_pub = self.create_publisher(DroneState, "state", 10)
        self.control_target_pub = self.create_publisher(
            FormationTarget, f"{self.backend_type}/control_target", 10
        )
        self.target_sub = self.create_subscription(
            FormationTarget, "formation_target", self._target_callback, 10
        )

        self._create_command_services()

        if bool(self.get_parameter("auto_connect").value):
            result = self.backend.connect()
            self._log_result("auto_connect", result)

        period = 1.0 / max(self.state_rate_hz, 1.0)
        self.timer = self.create_timer(period, self._timer_callback)
        self.get_logger().info(
            f"PX4 bridge ready for {self.drone_id}: backend={self.backend_type}, "
            f"namespace={self.get_namespace()}"
        )

    def _create_backend(self, initial_position: list[float]):
        if self.backend_type == "mock":
            return MockBackend(
                initial_position=initial_position,
                max_speed_m_s=float(self.get_parameter("max_speed_m_s").value),
                takeoff_altitude_m=float(self.get_parameter("takeoff_altitude_m").value),
            )
        if self.backend_type == "mavsdk":
            return MavsdkBackend(
                mavlink_udp_port=int(self.get_parameter("mavlink_udp_port").value),
                mavsdk_server_port=int(
                    self.get_parameter("mavsdk_server_port").value
                ),
                system_id=int(self.get_parameter("system_id").value),
                initial_position=initial_position,
                connection_timeout_sec=float(
                    self.get_parameter("connection_timeout_sec").value
                ),
                command_timeout_sec=float(self.get_parameter("command_timeout_sec").value),
                health_required=bool(self.get_parameter("health_required").value),
                takeoff_altitude_m=float(self.get_parameter("takeoff_altitude_m").value),
                telemetry_rate_hz=float(self.get_parameter("telemetry_rate_hz").value),
                connection_url_template=str(
                    self.get_parameter("connection_url_template").value
                ),
            )
        self.get_logger().warning(
            f"Unsupported backend_type '{self.backend_type}', falling back to mock"
        )
        self.backend_type = "mock"
        return MockBackend(
            initial_position=initial_position,
            max_speed_m_s=float(self.get_parameter("max_speed_m_s").value),
            takeoff_altitude_m=float(self.get_parameter("takeoff_altitude_m").value),
        )

    def _create_command_services(self) -> None:
        self.create_service(Trigger, "connect", self._connect_service)
        self.create_service(Trigger, "arm", self._arm_service)
        self.create_service(Trigger, "takeoff", self._takeoff_service)
        self.create_service(Trigger, "land", self._land_service)
        self.create_service(Trigger, "goto", self._goto_service)
        self.create_service(Trigger, "hold", self._hold_service)
        self.create_service(Trigger, "rtl", self._rtl_service)

    def _target_callback(self, msg: FormationTarget) -> None:
        if msg.drone_id and msg.drone_id != self.drone_id:
            return
        self.last_target = msg
        self.control_target_pub.publish(msg)
        if not self._ready_for_autonomous_target():
            return
        if not self._should_send_target(msg):
            return

        self.last_command_target = msg
        self.last_command_time = self.get_clock().now()
        result = self.backend.goto_formation_target(msg)
        if not result.success:
            self.get_logger().warning(
                f"Rejected formation target for {self.drone_id}: {result.message}"
            )

    def _timer_callback(self) -> None:
        dt = 1.0 / max(self.state_rate_hz, 1.0)
        state = self.backend.read_state(dt)

        msg = DroneState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.drone_id = self.drone_id
        msg.drone_namespace = self.drone_namespace
        msg.role = _role_to_constant(self.role_name)
        msg.armed = state.armed
        msg.flight_mode = state.flight_mode
        msg.position = state.position
        msg.velocity = state.velocity
        msg.yaw = state.yaw
        msg.healthy = state.healthy
        msg.status_text = state.status_text
        self.state_pub.publish(msg)

    def _connect_service(self, request, response):
        return self._fill_trigger_response(response, self.backend.connect())

    def _arm_service(self, request, response):
        return self._fill_trigger_response(response, self.backend.arm())

    def _takeoff_service(self, request, response):
        return self._fill_trigger_response(response, self.backend.takeoff())

    def _land_service(self, request, response):
        return self._fill_trigger_response(response, self.backend.land())

    def _goto_service(self, request, response):
        if self.last_target is None:
            result = CommandResult(False, "no FormationTarget has been received")
        else:
            result = self.backend.goto_formation_target(self.last_target)
            self.last_command_target = self.last_target
            self.last_command_time = self.get_clock().now()
        return self._fill_trigger_response(response, result)

    def _hold_service(self, request, response):
        return self._fill_trigger_response(response, self.backend.hold())

    def _rtl_service(self, request, response):
        return self._fill_trigger_response(response, self.backend.rtl())

    def _fill_trigger_response(self, response, result: CommandResult):
        self._log_result("service", result)
        response.success = result.success
        response.message = result.message
        return response

    def _log_result(self, label: str, result: CommandResult) -> None:
        if result.success:
            self.get_logger().info(f"{label}: {result.message}")
        else:
            self.get_logger().warning(f"{label}: {result.message}")

    def _should_send_target(self, msg: FormationTarget) -> bool:
        if self.backend_type == "mock":
            return True
        if self.last_command_target is None:
            return True
        if msg.active != self.last_command_target.active:
            return True
        if self.last_command_time is None:
            return True

        elapsed_sec = (
            self.get_clock().now() - self.last_command_time
        ).nanoseconds / 1e9
        if elapsed_sec < self.goto_min_interval_sec:
            return False

        return (
            _target_distance(msg, self.last_command_target)
            >= self.goto_position_tolerance_m
        )

    def _ready_for_autonomous_target(self) -> bool:
        if self.backend_type == "mock":
            return True

        state = self.backend.read_state(0.0)
        if state.connected and state.healthy:
            if not state.armed:
                self._log_target_wait("backend is healthy but vehicle is not armed")
                return False
            return True

        reason = "backend is not connected"
        if state.connected and not state.healthy:
            reason = f"backend health is not ready: {state.status_text}"
        self._log_target_wait(reason)
        return False

    def _log_target_wait(self, reason: str) -> None:
        now = self.get_clock().now()
        if self.last_target_wait_log_time is not None:
            elapsed_sec = (now - self.last_target_wait_log_time).nanoseconds / 1e9
            if elapsed_sec < self.target_wait_log_interval_sec:
                return
        self.last_target_wait_log_time = now
        self.get_logger().info(
            f"Waiting before sending autonomous target for {self.drone_id}: {reason}"
        )


def _target_distance(a: FormationTarget, b: FormationTarget) -> float:
    dx = a.position.x - b.position.x
    dy = a.position.y - b.position.y
    dz = a.position.z - b.position.z
    return sqrt(dx * dx + dy * dy + dz * dz)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = Px4Bridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
