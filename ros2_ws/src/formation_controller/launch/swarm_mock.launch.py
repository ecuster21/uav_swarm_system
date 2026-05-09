from pathlib import Path
import math
import tempfile

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


def _as_int(value: str, default: int = 0) -> int:
    if value is None or str(value).strip() == "":
        return default
    return int(value)


def _as_float(value: str, default: float) -> float:
    if value is None or str(value).strip() == "":
        return default
    return float(value)


def _default_grid_cols(vehicle_count: int) -> int:
    return max(1, math.ceil(math.sqrt(vehicle_count)))


def _spawn_xy(
    instance_number: int,
    vehicle_count: int,
    origin_x: float,
    origin_y: float,
    spacing_x: float,
    spacing_y: float,
    grid_cols: int,
) -> list[float]:
    cols = grid_cols if grid_cols > 0 else _default_grid_cols(vehicle_count)
    zero_based = instance_number - 1
    row_index = zero_based // cols
    col_index = zero_based % cols
    return [
        round(origin_x + spacing_x * col_index, 2),
        round(origin_y + spacing_y * row_index, 2),
    ]


def _generate_swarm_config(
    swarm_data: dict,
    vehicle_count: int,
    origin_x: float,
    origin_y: float,
    spacing_x: float,
    spacing_y: float,
    grid_cols: int,
) -> tuple[dict, str | None]:
    if vehicle_count <= 0:
        return swarm_data, None

    generated = dict(swarm_data)
    swarm = dict(generated.get("swarm", {}))
    template_drones = swarm.get("drones", [])
    default_z = 1.0
    if template_drones:
        default_position = template_drones[0].get("initial_position", [0.0, 0.0, default_z])
        if len(default_position) >= 3:
            default_z = float(default_position[2])

    drones = []
    for instance_number in range(1, vehicle_count + 1):
        x, y = _spawn_xy(
            instance_number, vehicle_count, origin_x, origin_y, spacing_x, spacing_y, grid_cols
        )
        drones.append(
            {
                "id": f"uav_{instance_number}",
                "namespace": f"uav_{instance_number}",
                "role": "leader" if instance_number == 1 else "follower",
                "system_id": instance_number + 1,
                "px4_topic_prefix": f"px4_{instance_number}",
                "initial_position": [x, y, default_z],
            }
        )

    swarm["leader_id"] = "uav_1"
    swarm["drones"] = drones
    generated["swarm"] = swarm

    runtime_file = tempfile.NamedTemporaryFile(
        mode="w", prefix="uav_swarm_runtime_", suffix=".yaml", delete=False, encoding="utf-8"
    )
    with runtime_file:
        yaml.safe_dump(generated, runtime_file, sort_keys=False)
    return generated, runtime_file.name


def _launch_setup(context, *args, **kwargs):
    swarm_config_file = LaunchConfiguration("swarm_config_file").perform(context)
    formations_config_file = LaunchConfiguration("formations_config_file").perform(context)
    waypoints_config_file = LaunchConfiguration("waypoints_config_file").perform(context)
    formation_type = LaunchConfiguration("formation_type").perform(context)
    vehicle_count = _as_int(LaunchConfiguration("vehicle_count").perform(context), 0)
    spawn_origin_x = _as_float(LaunchConfiguration("spawn_origin_x").perform(context), 0.0)
    spawn_origin_y = _as_float(LaunchConfiguration("spawn_origin_y").perform(context), 0.0)
    spawn_spacing_x = _as_float(LaunchConfiguration("spawn_spacing_x").perform(context), 3.0)
    spawn_spacing_y = _as_float(LaunchConfiguration("spawn_spacing_y").perform(context), 3.0)
    spawn_grid_cols = _as_int(LaunchConfiguration("spawn_grid_cols").perform(context), 0)
    swarm_data = _load_yaml(swarm_config_file)
    swarm_data, runtime_swarm_config_file = _generate_swarm_config(
        swarm_data,
        vehicle_count,
        spawn_origin_x,
        spawn_origin_y,
        spawn_spacing_x,
        spawn_spacing_y,
        spawn_grid_cols,
    )
    effective_swarm_config_file = runtime_swarm_config_file or swarm_config_file
    swarm = swarm_data.get("swarm", {})
    mock_backend = swarm_data.get("mock_backend", {})
    drones = swarm.get("drones", [])

    actions = []

    for drone in drones:
        drone_id = str(drone["id"])
        namespace = _normalize_namespace(str(drone.get("namespace", drone_id)))
        actions.append(
            Node(
                package="px4_bridge",
                executable="mock_px4_bridge",
                namespace=namespace,
                name="px4_bridge",
                output="screen",
                parameters=[
                    {
                        "drone_id": drone_id,
                        "drone_namespace": namespace,
                        "role": str(drone.get("role", "unknown")),
                        "backend_type": "mock",
                        "frame_id": str(swarm.get("frame_id", "local_enu")),
                        "initial_position": drone.get("initial_position", [0.0, 0.0, 0.0]),
                        "state_rate_hz": float(mock_backend.get("state_rate_hz", 20.0)),
                        "max_speed_m_s": float(mock_backend.get("max_speed_m_s", 1.5)),
                        "takeoff_altitude_m": float(
                            mock_backend.get("takeoff_altitude_m", 2.5)
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
                        "swarm_config_file": effective_swarm_config_file,
                        "publish_rate_hz": float(swarm.get("publish_rate_hz", 5.0)),
                        "state_timeout_sec": float(swarm.get("state_timeout_sec", 2.0)),
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
                        "swarm_config_file": effective_swarm_config_file,
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
                "vehicle_count",
                default_value="0",
                description="Generate uav_1..uav_N at launch time when > 0; 0 uses swarm.yaml",
            ),
            DeclareLaunchArgument(
                "spawn_origin_x",
                default_value="0.0",
                description="PX4 SITL grid origin x used for generated initial_position",
            ),
            DeclareLaunchArgument(
                "spawn_origin_y",
                default_value="0.0",
                description="PX4 SITL grid origin y used for generated initial_position",
            ),
            DeclareLaunchArgument(
                "spawn_spacing_x",
                default_value="3.0",
                description="PX4 SITL grid x spacing used for generated initial_position",
            ),
            DeclareLaunchArgument(
                "spawn_spacing_y",
                default_value="3.0",
                description="PX4 SITL grid y spacing used for generated initial_position",
            ),
            DeclareLaunchArgument(
                "spawn_grid_cols",
                default_value="0",
                description="PX4 SITL grid columns; 0 means ceil(sqrt(vehicle_count))",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
