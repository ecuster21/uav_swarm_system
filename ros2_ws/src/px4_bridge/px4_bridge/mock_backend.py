from math import sqrt
from typing import Optional

from geometry_msgs.msg import Point, Vector3
from swarm_msgs.msg import FormationTarget

from px4_bridge.backend_base import BackendState, CommandResult, VehicleBackend


def _limit_vector(vector: Vector3, max_norm: float) -> Vector3:
    norm = sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z)
    if norm <= max_norm or norm <= 1e-9:
        return vector
    scale = max_norm / norm
    return Vector3(x=vector.x * scale, y=vector.y * scale, z=vector.z * scale)


class MockBackend(VehicleBackend):
    """轻量运动学后端，用于不启动 PX4/Gazebo 时验证 ROS2 控制链路。"""

    def __init__(
        self,
        initial_position: list[float],
        max_speed_m_s: float,
        takeoff_altitude_m: float = 2.5,
    ) -> None:
        self.initial_position = Point(
            x=float(initial_position[0]),
            y=float(initial_position[1]),
            z=float(initial_position[2]),
        )
        self.position = Point(
            x=self.initial_position.x,
            y=self.initial_position.y,
            z=self.initial_position.z,
        )
        self.velocity = Vector3()
        self.yaw = 0.0
        self.max_speed_m_s = float(max_speed_m_s)
        self.takeoff_altitude_m = float(takeoff_altitude_m)
        self.target: Optional[FormationTarget] = None
        self.connected = True
        self.armed = False
        self.flight_mode = "MOCK_IDLE"
        self.status_text = "mock backend ready"

    def connect(self) -> CommandResult:
        self.connected = True
        self.status_text = "mock backend connected"
        return CommandResult(True, self.status_text)

    def arm(self) -> CommandResult:
        if not self.connected:
            return CommandResult(False, "mock backend is not connected")
        self.armed = True
        self.flight_mode = "MOCK_ARMED"
        self.status_text = "mock vehicle armed"
        return CommandResult(True, self.status_text)

    def takeoff(self) -> CommandResult:
        if not self.connected:
            return CommandResult(False, "mock backend is not connected")
        if not self.armed:
            arm_result = self.arm()
            if not arm_result.success:
                return arm_result
        target = FormationTarget()
        target.active = True
        target.position = Point(
            x=self.position.x,
            y=self.position.y,
            z=max(self.position.z, self.takeoff_altitude_m),
        )
        target.yaw = self.yaw
        self.target = target
        self.flight_mode = "MOCK_TAKEOFF"
        self.status_text = "mock takeoff target accepted"
        return CommandResult(True, self.status_text)

    def land(self) -> CommandResult:
        if not self.connected:
            return CommandResult(False, "mock backend is not connected")
        target = FormationTarget()
        target.active = True
        target.position = Point(x=self.position.x, y=self.position.y, z=0.0)
        target.yaw = self.yaw
        self.target = target
        self.flight_mode = "MOCK_LAND"
        self.status_text = "mock land target accepted"
        return CommandResult(True, self.status_text)

    def goto_formation_target(self, target: FormationTarget) -> CommandResult:
        if not self.connected:
            return CommandResult(False, "mock backend is not connected")
        if not target.active:
            self.target = None
            self.flight_mode = "MOCK_HOLD"
            self.status_text = "mock target inactive; holding"
            return CommandResult(True, self.status_text)
        self.target = target if target.active else None
        self.flight_mode = "MOCK_GOTO"
        self.status_text = "mock goto target accepted"
        return CommandResult(True, self.status_text)

    def hold(self) -> CommandResult:
        if not self.connected:
            return CommandResult(False, "mock backend is not connected")
        self.target = None
        self.velocity = Vector3()
        self.flight_mode = "MOCK_HOLD"
        self.status_text = "mock hold"
        return CommandResult(True, self.status_text)

    def rtl(self) -> CommandResult:
        if not self.connected:
            return CommandResult(False, "mock backend is not connected")
        target = FormationTarget()
        target.active = True
        target.position = Point(
            x=self.initial_position.x,
            y=self.initial_position.y,
            z=max(self.initial_position.z, self.takeoff_altitude_m),
        )
        target.yaw = 0.0
        self.target = target
        self.flight_mode = "MOCK_RTL"
        self.status_text = "mock RTL target accepted"
        return CommandResult(True, self.status_text)

    def read_state(self, dt: float) -> BackendState:
        if self.target is None:
            self.velocity = Vector3()
            return self._state()

        dx = self.target.position.x - self.position.x
        dy = self.target.position.y - self.position.y
        dz = self.target.position.z - self.position.z
        distance = sqrt(dx * dx + dy * dy + dz * dz)

        if self.target.use_velocity:
            # 模拟真实 bridge 的 position + velocity 前馈模式，便于联调编队控制器。
            position_gain = 0.6
            command = Vector3(
                x=self.target.velocity.x + position_gain * dx,
                y=self.target.velocity.y + position_gain * dy,
                z=self.target.velocity.z + position_gain * dz,
            )
            self.velocity = _limit_vector(command, self.max_speed_m_s)
            self.position.x += self.velocity.x * dt
            self.position.y += self.velocity.y * dt
            self.position.z += self.velocity.z * dt
            self.yaw = self.target.yaw
            return self._state()

        if distance < 1e-6:
            self.velocity = Vector3()
            self.yaw = self.target.yaw
            if self.flight_mode == "MOCK_LAND" and self.position.z <= 0.05:
                self.armed = False
                self.flight_mode = "MOCK_IDLE"
            return self._state()

        step_distance = min(self.max_speed_m_s * dt, distance)
        scale = step_distance / distance
        vx = dx * scale / dt
        vy = dy * scale / dt
        vz = dz * scale / dt

        self.position.x += dx * scale
        self.position.y += dy * scale
        self.position.z += dz * scale
        self.velocity = Vector3(x=vx, y=vy, z=vz)
        self.yaw = self.target.yaw
        return self._state()

    def _state(self) -> BackendState:
        return BackendState(
            connected=self.connected,
            armed=self.armed,
            flight_mode=self.flight_mode,
            position=self.position,
            velocity=self.velocity,
            yaw=self.yaw,
            healthy=self.connected,
            status_text=self.status_text,
        )
