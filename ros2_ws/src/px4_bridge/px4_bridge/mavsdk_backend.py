import asyncio
from concurrent.futures import TimeoutError as FutureTimeoutError
from math import cos, degrees, pi, radians
import threading
from typing import Optional

from geometry_msgs.msg import Point, Vector3
from swarm_msgs.msg import FormationTarget

from px4_bridge.backend_base import BackendState, CommandResult, VehicleBackend

try:
    from mavsdk import System
except ImportError:
    System = None


EARTH_RADIUS_M = 6378137.0


class MavsdkBackend(VehicleBackend):
    """MAVSDK backend for PX4 SITL.

    This is a Phase 2 bridge backend for validation. The future uXRCE-DDS
    backend should own high-rate PX4 control.
    """

    def __init__(
        self,
        mavlink_udp_port: int,
        mavsdk_server_port: int,
        system_id: int,
        initial_position: list[float],
        connection_timeout_sec: float,
        command_timeout_sec: float,
        health_required: bool,
        takeoff_altitude_m: float,
        telemetry_rate_hz: float,
        connection_url_template: str,
    ) -> None:
        self.mavlink_udp_port = int(mavlink_udp_port)
        self.mavsdk_server_port = int(mavsdk_server_port)
        self.system_id = int(system_id)
        self.initial_position = Point(
            x=float(initial_position[0]),
            y=float(initial_position[1]),
            z=float(initial_position[2]),
        )
        self.connection_timeout_sec = float(connection_timeout_sec)
        self.command_timeout_sec = float(command_timeout_sec)
        self.health_required = bool(health_required)
        self.takeoff_altitude_m = float(takeoff_altitude_m)
        self.telemetry_rate_hz = float(telemetry_rate_hz)
        self.connection_url_template = str(connection_url_template)

        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._loop.run_forever, daemon=True)
        self._thread.start()

        self._drone = None
        self._connected = False
        self._armed = False
        self._flight_mode = "DISCONNECTED"
        self._position = Point(
            x=self.initial_position.x,
            y=self.initial_position.y,
            z=self.initial_position.z,
        )
        self._velocity = Vector3()
        self._yaw = 0.0
        self._health_global_position_ok = False
        self._health_home_position_ok = False
        self._status_text = "MAVSDK backend not connected"

        self._reference_lat_deg: Optional[float] = None
        self._reference_lon_deg: Optional[float] = None
        self._reference_abs_alt_m: Optional[float] = None
        self._telemetry_tasks = []
        self._lock = threading.Lock()

    def connect(self) -> CommandResult:
        if System is None:
            return CommandResult(
                False,
                "Python package 'mavsdk' is not installed for /usr/bin/python3",
            )
        if self._connected:
            return CommandResult(True, "MAVSDK already connected")

        return self._run(
            self._connect_async(),
            self.connection_timeout_sec,
            f"MAVSDK connected on {self._connection_url()} "
            f"(server_port={self.mavsdk_server_port})",
            cleanup_on_timeout=True,
        )

    def arm(self) -> CommandResult:
        precheck = self._command_precheck(require_health=True)
        if not precheck.success:
            return precheck
        return self._run(self._drone.action.arm(), self.command_timeout_sec, "armed")

    def takeoff(self) -> CommandResult:
        precheck = self._command_precheck(require_health=True)
        if not precheck.success:
            return precheck

        async def _takeoff():
            await self._drone.action.set_takeoff_altitude(self.takeoff_altitude_m)
            await self._drone.action.takeoff()

        return self._run(_takeoff(), self.command_timeout_sec, "takeoff accepted")

    def land(self) -> CommandResult:
        precheck = self._command_precheck(require_health=False)
        if not precheck.success:
            return precheck
        return self._run(self._drone.action.land(), self.command_timeout_sec, "land accepted")

    def goto_formation_target(self, target: FormationTarget) -> CommandResult:
        precheck = self._command_precheck(require_health=True)
        if not precheck.success:
            return precheck
        if not target.active:
            return self.hold()
        if self._reference_lat_deg is None or self._reference_lon_deg is None:
            return CommandResult(False, "MAVSDK telemetry origin is not available yet")

        lat_deg, lon_deg, absolute_alt_m = self._local_target_to_global(target.position)
        yaw_deg = degrees(target.yaw)
        return self._run(
            self._drone.action.goto_location(lat_deg, lon_deg, absolute_alt_m, yaw_deg),
            self.command_timeout_sec,
            "goto accepted",
        )

    def hold(self) -> CommandResult:
        precheck = self._command_precheck(require_health=False)
        if not precheck.success:
            return precheck
        return self._run(self._drone.action.hold(), self.command_timeout_sec, "hold accepted")

    def rtl(self) -> CommandResult:
        precheck = self._command_precheck(require_health=False)
        if not precheck.success:
            return precheck
        return self._run(
            self._drone.action.return_to_launch(),
            self.command_timeout_sec,
            "RTL accepted",
        )

    def read_state(self, dt: float) -> BackendState:
        with self._lock:
            healthy = self._connected and (
                not self.health_required
                or (self._health_global_position_ok and self._health_home_position_ok)
            )
            status_text = self._status_text
            if self._connected and self.health_required and not healthy:
                status_text = (
                    "waiting for PX4 health: "
                    f"global={self._health_global_position_ok}, "
                    f"home={self._health_home_position_ok}"
                )
            return BackendState(
                connected=self._connected,
                armed=self._armed,
                flight_mode=self._flight_mode,
                position=Point(
                    x=self._position.x,
                    y=self._position.y,
                    z=self._position.z,
                ),
                velocity=Vector3(
                    x=self._velocity.x,
                    y=self._velocity.y,
                    z=self._velocity.z,
                ),
                yaw=self._yaw,
                healthy=healthy,
                status_text=status_text,
            )

    async def _connect_async(self) -> None:
        self._drone = System(port=self.mavsdk_server_port)
        system_address = self._connection_url()
        await self._drone.connect(system_address=system_address)

        async def _wait_connected():
            async for state in self._drone.core.connection_state():
                if state.is_connected:
                    return

        await asyncio.wait_for(_wait_connected(), timeout=self.connection_timeout_sec)
        with self._lock:
            self._connected = True
            self._flight_mode = "CONNECTED"
            self._status_text = (
                f"MAVSDK connected to system_id={self.system_id} on {system_address}"
            )
        await self._configure_telemetry_rates()
        self._start_telemetry_tasks()

    async def _configure_telemetry_rates(self) -> None:
        rate_hz = max(self.telemetry_rate_hz, 0.5)
        setters = [
            ("position", self._drone.telemetry.set_rate_position(rate_hz)),
            ("velocity_ned", self._drone.telemetry.set_rate_velocity_ned(rate_hz)),
            ("attitude_euler", self._drone.telemetry.set_rate_attitude_euler(rate_hz)),
            ("health", self._drone.telemetry.set_rate_health(1.0)),
        ]
        for name, setter in setters:
            try:
                await setter
            except Exception as exc:
                self._set_status(f"could not set telemetry rate for {name}: {exc}")

    def _start_telemetry_tasks(self) -> None:
        if self._telemetry_tasks:
            return
        self._telemetry_tasks = [
            asyncio.run_coroutine_threadsafe(self._position_task(), self._loop),
            asyncio.run_coroutine_threadsafe(self._velocity_task(), self._loop),
            asyncio.run_coroutine_threadsafe(self._attitude_task(), self._loop),
            asyncio.run_coroutine_threadsafe(self._armed_task(), self._loop),
            asyncio.run_coroutine_threadsafe(self._flight_mode_task(), self._loop),
            asyncio.run_coroutine_threadsafe(self._health_task(), self._loop),
        ]

    async def _position_task(self) -> None:
        try:
            async for position in self._drone.telemetry.position():
                with self._lock:
                    if self._reference_lat_deg is None:
                        self._reference_lat_deg = position.latitude_deg
                        self._reference_lon_deg = position.longitude_deg
                        self._reference_abs_alt_m = position.absolute_altitude_m
                    self._position = self._global_to_local(
                        position.latitude_deg,
                        position.longitude_deg,
                        position.absolute_altitude_m,
                    )
        except Exception as exc:
            self._set_status(f"position telemetry stopped: {exc}")

    async def _velocity_task(self) -> None:
        try:
            async for velocity in self._drone.telemetry.velocity_ned():
                with self._lock:
                    self._velocity = Vector3(
                        x=float(velocity.east_m_s),
                        y=float(velocity.north_m_s),
                        z=float(-velocity.down_m_s),
                    )
        except Exception as exc:
            self._set_status(f"velocity telemetry stopped: {exc}")

    async def _attitude_task(self) -> None:
        try:
            async for attitude in self._drone.telemetry.attitude_euler():
                with self._lock:
                    self._yaw = radians(float(attitude.yaw_deg))
        except Exception as exc:
            self._set_status(f"attitude telemetry stopped: {exc}")

    async def _armed_task(self) -> None:
        try:
            async for armed in self._drone.telemetry.armed():
                with self._lock:
                    self._armed = bool(armed)
        except Exception as exc:
            self._set_status(f"armed telemetry stopped: {exc}")

    async def _flight_mode_task(self) -> None:
        try:
            async for flight_mode in self._drone.telemetry.flight_mode():
                with self._lock:
                    self._flight_mode = str(flight_mode)
        except Exception as exc:
            self._set_status(f"flight mode telemetry stopped: {exc}")

    async def _health_task(self) -> None:
        try:
            async for health in self._drone.telemetry.health():
                with self._lock:
                    self._health_global_position_ok = bool(health.is_global_position_ok)
                    self._health_home_position_ok = bool(health.is_home_position_ok)
        except Exception as exc:
            self._set_status(f"health telemetry stopped: {exc}")

    def _command_precheck(self, require_health: bool) -> CommandResult:
        if System is None:
            return CommandResult(False, "Python package 'mavsdk' is not installed")
        if not self._connected or self._drone is None:
            return CommandResult(False, "MAVSDK backend is not connected")
        if require_health and self.health_required:
            if not (self._health_global_position_ok and self._health_home_position_ok):
                return CommandResult(
                    False,
                    "PX4 health check failed: global/home position is not ready "
                    f"(global={self._health_global_position_ok}, "
                    f"home={self._health_home_position_ok})",
                )
        return CommandResult(True, "precheck passed")

    def _connection_url(self) -> str:
        return self.connection_url_template.format(
            port=self.mavlink_udp_port,
            mavlink_udp_port=self.mavlink_udp_port,
        )

    def _run(
        self,
        coroutine,
        timeout_sec: float,
        success_message: Optional[str] = None,
        cleanup_on_timeout: bool = False,
    ) -> CommandResult:
        future = asyncio.run_coroutine_threadsafe(coroutine, self._loop)
        try:
            future.result(timeout=timeout_sec)
        except FutureTimeoutError:
            future.cancel()
            if cleanup_on_timeout:
                self._stop_mavsdk_server()
            return CommandResult(False, f"MAVSDK command timed out after {timeout_sec:.1f}s")
        except Exception as exc:
            if cleanup_on_timeout:
                self._stop_mavsdk_server()
            self._set_status(f"MAVSDK command failed: {exc}")
            return CommandResult(False, str(exc))
        if success_message:
            self._set_status(success_message)
            return CommandResult(True, success_message)
        return CommandResult(True, "command accepted")

    def _local_target_to_global(self, target: Point) -> tuple[float, float, float]:
        delta_east_m = target.x - self.initial_position.x
        delta_north_m = target.y - self.initial_position.y
        delta_up_m = target.z - self.initial_position.z
        lat_rad = (self._reference_lat_deg or 0.0) * pi / 180.0
        lat_deg = (self._reference_lat_deg or 0.0) + (
            delta_north_m / EARTH_RADIUS_M
        ) * 180.0 / pi
        lon_deg = (self._reference_lon_deg or 0.0) + (
            delta_east_m / (EARTH_RADIUS_M * max(cos(lat_rad), 1e-6))
        ) * 180.0 / pi
        absolute_alt_m = (self._reference_abs_alt_m or 0.0) + delta_up_m
        return lat_deg, lon_deg, absolute_alt_m

    def _global_to_local(
        self,
        lat_deg: float,
        lon_deg: float,
        absolute_alt_m: float,
    ) -> Point:
        ref_lat = self._reference_lat_deg or lat_deg
        ref_lon = self._reference_lon_deg or lon_deg
        ref_alt = self._reference_abs_alt_m or absolute_alt_m
        lat_rad = ref_lat * pi / 180.0
        north_m = (lat_deg - ref_lat) * pi / 180.0 * EARTH_RADIUS_M
        east_m = (
            (lon_deg - ref_lon)
            * pi
            / 180.0
            * EARTH_RADIUS_M
            * max(cos(lat_rad), 1e-6)
        )
        up_m = absolute_alt_m - ref_alt
        return Point(
            x=self.initial_position.x + east_m,
            y=self.initial_position.y + north_m,
            z=self.initial_position.z + up_m,
        )

    def _stop_mavsdk_server(self) -> None:
        if self._drone is not None and hasattr(self._drone, "_stop_mavsdk_server"):
            try:
                self._drone._stop_mavsdk_server()
            except Exception:
                pass
        with self._lock:
            self._drone = None
            self._connected = False
            self._flight_mode = "DISCONNECTED"
            self._status_text = "MAVSDK backend not connected"

    def _set_status(self, text: str) -> None:
        with self._lock:
            self._status_text = text
