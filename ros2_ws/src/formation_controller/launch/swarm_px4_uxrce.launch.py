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

    swarm_data = _load_yaml(swarm_config_file)
    swarm = swarm_data.get("swarm", {})
    uxrce_backend = swarm_data.get("uxrce_backend", {})
    drones = swarm.get("drones", [])

    actions = []

    for drone in drones:
        drone_id = str(drone["id"])
        namespace = _normalize_namespace(str(drone.get("namespace", drone_id)))
        actions.append(
            Node(
                package="px4_bridge_uxrce",
                executable="px4_uxrce_bridge",
                namespace=namespace,
                name="px4_bridge",
                output="screen",
                parameters=[
                    {
                        "drone_id": drone_id,
                        "drone_namespace": namespace,
                        "role": str(drone.get("role", "unknown")),
                        "frame_id": str(swarm.get("frame_id", "local_enu")),
                        "system_id": int(drone.get("system_id", 1)),
                        "px4_topic_prefix": str(
                            drone.get("px4_topic_prefix", f"px4_{drone_id[-1]}")
                        ),
                        "initial_position": drone.get(
                            "initial_position", [0.0, 0.0, 0.0]
                        ),
                        "state_rate_hz": float(
                            uxrce_backend.get("state_rate_hz", 10.0)
                        ),
                        "offboard_rate_hz": float(
                            uxrce_backend.get("offboard_rate_hz", 20.0)
                        ),
                        "enable_offboard_from_target": bool(
                            uxrce_backend.get("enable_offboard_from_target", True)
                        ),
                        "offboard_warmup_cycles": int(
                            uxrce_backend.get("offboard_warmup_cycles", 10)
                        ),
                        "takeoff_altitude_m": float(
                            uxrce_backend.get("takeoff_altitude_m", 2.5)
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
            OpaqueFunction(function=_launch_setup),
        ]
    )
