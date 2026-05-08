from dataclasses import dataclass

from geometry_msgs.msg import Point, Vector3
from swarm_msgs.msg import FormationTarget


@dataclass
class BackendState:
    connected: bool
    armed: bool
    flight_mode: str
    position: Point
    velocity: Vector3
    yaw: float
    healthy: bool
    status_text: str


@dataclass
class CommandResult:
    success: bool
    message: str


class VehicleBackend:
    def connect(self) -> CommandResult:
        raise NotImplementedError

    def arm(self) -> CommandResult:
        raise NotImplementedError

    def takeoff(self) -> CommandResult:
        raise NotImplementedError

    def land(self) -> CommandResult:
        raise NotImplementedError

    def goto_formation_target(self, target: FormationTarget) -> CommandResult:
        raise NotImplementedError

    def hold(self) -> CommandResult:
        raise NotImplementedError

    def rtl(self) -> CommandResult:
        raise NotImplementedError

    def read_state(self, dt: float) -> BackendState:
        raise NotImplementedError
