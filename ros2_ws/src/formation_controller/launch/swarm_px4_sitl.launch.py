from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_yaml(path: str) -> dict:
    with Path(path).open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}


def _normalize_namespace(namespace: str) -> str:
    return namespace.strip().strip("/")


def _launch_setup(context, *args, **kwargs):
    swarm_config_file = LaunchConfiguration("swarm_config_file").perform(context)
    formations_config_file = LaunchConfiguration("formations_config_file").perform(context)
    waypoints_config_file = LaunchConfiguration("waypoints_config_file").perform(context)
    formation_type = LaunchConfiguration("formation_type").perform(context)
    backend_type = LaunchConfiguration("backend_type").perform(context).strip().lower()

    swarm_data = _load_yaml(swarm_config_file)
    swarm = swarm_data.get("swarm", {})
    mavsdk_backend = swarm_data.get("mavsdk_backend", {})
    drones = swarm.get("drones", [])

    actions = []

    for drone in drones:
        drone_id = str(drone["id"])
        namespace = _normalize_namespace(str(drone.get("namespace", drone_id)))
        actions.append(
            Node(
                package="px4_bridge",
                executable="px4_bridge",
                namespace=namespace,
                name="px4_bridge",
                output="screen",
                parameters=[
                    {
                        "drone_id": drone_id,
                        "drone_namespace": namespace,
                        "role": str(drone.get("role", "unknown")),
                        "backend_type": backend_type,
                        "frame_id": str(swarm.get("frame_id", "local_enu")),
                        "system_id": int(drone.get("system_id", 1)),
                        "mavlink_udp_port": int(
                            drone.get("mavlink_udp_port", 14540)
                        ),
                        "mavsdk_server_port": int(
                            drone.get("mavsdk_server_port", 50051)
                        ),
                        "initial_position": drone.get(
                            "initial_position", [0.0, 0.0, 0.0]
                        ),
                        "state_rate_hz": float(
                            mavsdk_backend.get("state_rate_hz", 5.0)
                        ),
                        "auto_connect": bool(mavsdk_backend.get("auto_connect", True)),
                        "connection_timeout_sec": float(
                            mavsdk_backend.get("connection_timeout_sec", 10.0)
                        ),
                        "command_timeout_sec": float(
                            mavsdk_backend.get("command_timeout_sec", 8.0)
                        ),
                        "health_required": bool(
                            mavsdk_backend.get("health_required", True)
                        ),
                        "takeoff_altitude_m": float(
                            mavsdk_backend.get("takeoff_altitude_m", 2.5)
                        ),
                        "telemetry_rate_hz": float(
                            mavsdk_backend.get("telemetry_rate_hz", 5.0)
                        ),
                        "connection_url_template": str(
                            mavsdk_backend.get(
                                "connection_url_template",
                                "udpin://0.0.0.0:{mavlink_udp_port}",
                            )
                        ),
                        "goto_min_interval_sec": float(
                            mavsdk_backend.get("goto_min_interval_sec", 2.0)
                        ),
                        "goto_position_tolerance_m": float(
                            mavsdk_backend.get("goto_position_tolerance_m", 0.5)
                        ),
                        "target_wait_log_interval_sec": float(
                            mavsdk_backend.get("target_wait_log_interval_sec", 5.0)
                        ),
                    }
                ],
            )
        )

    actions.extend(
        [
            Node(
                package="swarm_manager",
                executable="swarm_manager",
                namespace="swarm",
                name="manager",
                output="screen",
                parameters=[
                    {
                        "swarm_config_file": swarm_config_file,
                        "publish_rate_hz": float(swarm.get("publish_rate_hz", 5.0)),
                        "state_timeout_sec": float(
                            swarm.get("state_timeout_sec", 2.0)
                        ),
                    }
                ],
            ),
            Node(
                package="formation_controller",
                executable="formation_controller",
                namespace="swarm",
                name="formation_controller",
                output="screen",
                parameters=[
                    {
                        "swarm_config_file": swarm_config_file,
                        "formations_config_file": formations_config_file,
                        "waypoints_config_file": waypoints_config_file,
                        "formation_type": formation_type,
                    }
                ],
            ),
        ]
    )

    return actions


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("formation_controller"))
    default_swarm_config = str(package_share / "config" / "swarm.yaml")
    default_formations_config = str(package_share / "config" / "formations.yaml")
    default_waypoints_config = str(package_share / "config" / "waypoints.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "swarm_config_file",
                default_value=default_swarm_config,
                description="Path to swarm.yaml",
            ),
            DeclareLaunchArgument(
                "formations_config_file",
                default_value=default_formations_config,
                description="Path to formations.yaml",
            ),
            DeclareLaunchArgument(
                "waypoints_config_file",
                default_value=default_waypoints_config,
                description="Path to waypoints.yaml",
            ),
            DeclareLaunchArgument(
                "formation_type",
                default_value="triangle",
                description="Formation type: triangle, line, or column",
            ),
            DeclareLaunchArgument(
                "backend_type",
                default_value="mavsdk",
                description="PX4 bridge backend type. Use mavsdk for PX4 SITL.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
