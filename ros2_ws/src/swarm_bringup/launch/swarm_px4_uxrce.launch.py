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


def _as_bool(value: str, default: bool) -> bool:
    if value is None or str(value).strip() == "":
        return default
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _default_grid_cols(vehicle_count: int) -> int:
    return max(1, math.ceil(math.sqrt(vehicle_count)))


def _spawn_xy(
    local_index: int,
    vehicle_count: int,
    origin_x: float,
    origin_y: float,
    spacing_x: float,
    spacing_y: float,
    grid_cols: int,
) -> list[float]:
    cols = grid_cols if grid_cols > 0 else _default_grid_cols(vehicle_count)
    zero_based = local_index - 1
    row_index = zero_based // cols
    col_index = zero_based % cols
    return [
        round(origin_x + spacing_x * col_index, 2),
        round(origin_y + spacing_y * row_index, 2),
    ]


def _generate_swarm_config(
    swarm_data: dict,
    instance_start: int,
    vehicle_count: int,
    origin_x: float,
    origin_y: float,
    spacing_x: float,
    spacing_y: float,
    grid_cols: int,
    preserve_configured_leader: bool = False,
) -> tuple[dict, str | None]:
    if vehicle_count <= 0:
        return swarm_data, None

    # instance_start/vehicle_count must match PX4 SITL startup:
    # uav_N is the project namespace, px4_N is the PX4 DDS topic prefix,
    # and system_id uses N+1.
    generated = dict(swarm_data)
    swarm = dict(generated.get("swarm", {}))
    template_drones = swarm.get("drones", [])
    configured_leader_id = str(swarm.get("leader_id", "")).strip()
    default_z = 1.0
    if template_drones:
        default_position = template_drones[0].get("initial_position", [0.0, 0.0, default_z])
        if len(default_position) >= 3:
            default_z = float(default_position[2])

    generated_ids = {
        f"uav_{instance_start + local_index - 1}" for local_index in range(1, vehicle_count + 1)
    }
    if preserve_configured_leader and configured_leader_id:
        leader_id = configured_leader_id
    else:
        leader_id = configured_leader_id if configured_leader_id in generated_ids else f"uav_{instance_start}"

    drones = []
    for local_index in range(1, vehicle_count + 1):
        instance_number = instance_start + local_index - 1
        drone_id = f"uav_{instance_number}"
        x, y = _spawn_xy(
            local_index, vehicle_count, origin_x, origin_y, spacing_x, spacing_y, grid_cols
        )
        drones.append(
            {
                "id": drone_id,
                "namespace": drone_id,
                "role": "leader" if drone_id == leader_id else "follower",
                "system_id": instance_number + 1,
                "px4_topic_prefix": f"px4_{instance_number}",
                "initial_position": [x, y, default_z],
            }
        )

    swarm["leader_id"] = leader_id
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
    instance_start = _as_int(LaunchConfiguration("instance_start").perform(context), 1)
    vehicle_count = _as_int(LaunchConfiguration("vehicle_count").perform(context), 0)
    swarm_vehicle_count = _as_int(
        LaunchConfiguration("swarm_vehicle_count").perform(context), 0
    )
    enable_bridges = _as_bool(LaunchConfiguration("enable_bridges").perform(context), True)
    enable_swarm_nodes = _as_bool(
        LaunchConfiguration("enable_swarm_nodes").perform(context), True
    )
    distributed_followers = _as_bool(
        LaunchConfiguration("distributed_followers").perform(context), False
    )
    enable_local_follower_controllers = _as_bool(
        LaunchConfiguration("enable_local_follower_controllers").perform(context), True
    )
    spawn_origin_x = _as_float(LaunchConfiguration("spawn_origin_x").perform(context), 0.0)
    spawn_origin_y = _as_float(LaunchConfiguration("spawn_origin_y").perform(context), 0.0)
    spawn_spacing_x = _as_float(LaunchConfiguration("spawn_spacing_x").perform(context), 3.0)
    spawn_spacing_y = _as_float(LaunchConfiguration("spawn_spacing_y").perform(context), 3.0)
    spawn_grid_cols = _as_int(LaunchConfiguration("spawn_grid_cols").perform(context), 0)

    source_swarm_data = _load_yaml(swarm_config_file)
    swarm_data = source_swarm_data
    swarm_data, runtime_swarm_config_file = _generate_swarm_config(
        swarm_data,
        instance_start,
        vehicle_count,
        spawn_origin_x,
        spawn_origin_y,
        spawn_spacing_x,
        spawn_spacing_y,
        spawn_grid_cols,
        preserve_configured_leader=distributed_followers,
    )
    effective_swarm_config_file = runtime_swarm_config_file or swarm_config_file
    local_follower_swarm_config_file = effective_swarm_config_file
    if distributed_followers and swarm_vehicle_count > 0:
        _, runtime_full_swarm_config_file = _generate_swarm_config(
            source_swarm_data,
            1,
            swarm_vehicle_count,
            spawn_origin_x,
            spawn_origin_y,
            spawn_spacing_x,
            spawn_spacing_y,
            spawn_grid_cols,
            preserve_configured_leader=True,
        )
        local_follower_swarm_config_file = runtime_full_swarm_config_file or swarm_config_file
    swarm = swarm_data.get("swarm", {})
    uxrce_backend = swarm_data.get("uxrce_backend", {})
    drones = swarm.get("drones", [])

    actions = []

    if enable_bridges:
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

    if enable_swarm_nodes:
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
                            "swarm_config_file": effective_swarm_config_file,
                            "formations_config_file": formations_config_file,
                            "waypoints_config_file": waypoints_config_file,
                            "formation_type": formation_type,
                            "distributed_followers": distributed_followers,
                        }
                    ],
                ),
            ]
        )
    if distributed_followers and enable_local_follower_controllers:
        leader_id = str(swarm.get("leader_id", ""))
        for drone in drones:
            drone_id = str(drone["id"])
            if drone_id == leader_id or str(drone.get("role", "")).lower() == "leader":
                continue
            namespace = _normalize_namespace(str(drone.get("namespace", drone_id)))
            actions.append(
                Node(
                    package="formation_controller",
                    executable="local_follower_controller",
                    namespace=namespace,
                    name="local_follower_controller",
                    output="screen",
                    parameters=[
                        {
                                "drone_id": drone_id,
                                "leader_id": leader_id,
                                "swarm_config_file": local_follower_swarm_config_file,
                                "formations_config_file": formations_config_file,
                                "formation_type": formation_type,
                            }
                    ],
                )
            )

    return actions


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("swarm_bringup"))
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
                description="Generate uav_INSTANCE.. range at launch time when > 0; 0 uses swarm.yaml",
            ),
            DeclareLaunchArgument(
                "swarm_vehicle_count",
                default_value="0",
                description="Full swarm size used by local follower controllers when distributed_followers is true",
            ),
            DeclareLaunchArgument(
                "instance_start",
                default_value="1",
                description="First PX4 SITL instance number used when vehicle_count > 0",
            ),
            DeclareLaunchArgument(
                "enable_bridges",
                default_value="true",
                description="Start px4_bridge_uxrce nodes for generated/configured drones",
            ),
            DeclareLaunchArgument(
                "enable_swarm_nodes",
                default_value="true",
                description="Start swarm_manager and formation_controller",
            ),
            DeclareLaunchArgument(
                "distributed_followers",
                default_value="false",
                description="Start per-follower local_follower_controller nodes and stop central follower target publication",
            ),
            DeclareLaunchArgument(
                "enable_local_follower_controllers",
                default_value="true",
                description="When distributed_followers is true, start local follower target calculators for follower drones in this launch",
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
