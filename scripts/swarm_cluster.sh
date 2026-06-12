#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UAV_SWARM_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
CONFIG_FILE="$UAV_SWARM_ROOT/config/cluster_boards.yaml"
ACTION="${1:-}"
LIMIT=""
RUN_ID="$(date +%Y%m%d_%H%M%S)"
PX4_WAIT_SEC="${PX4_WAIT_SEC:-25}"
BRIDGE_WAIT_SEC="${BRIDGE_WAIT_SEC:-8}"
DISTRIBUTED_FOLLOWERS="${DISTRIBUTED_FOLLOWERS:-false}"
CONTROLLER_IP_REQUEST="${SWARM_CONTROLLER_IP:-auto}"

SSH_OPTS=(
    -o BatchMode=yes
    -o ConnectTimeout=5
    -o ServerAliveInterval=10
    -o ServerAliveCountMax=2
    -o StrictHostKeyChecking=accept-new
)
mapfile -t LOCAL_IPS < <(ip -o -4 addr show 2>/dev/null | awk '{ split($4, address, "/"); print address[1] }')

usage() {
    cat <<'EOF'
Usage:
  ./scripts/swarm_cluster.sh dry-run [--limit N] [--config PATH]
  ./scripts/swarm_cluster.sh check   [--limit N] [--config PATH]
  ./scripts/swarm_cluster.sh start   [--limit N] [--config PATH] [--run-id ID] [--controller-ip IP|auto|config]
  ./scripts/swarm_cluster.sh stop    [--limit N] [--config PATH]
  ./scripts/swarm_cluster.sh status  [--limit N] [--config PATH]

Environment:
  PX4_WAIT_SEC       Seconds to wait after starting PX4 before bridges. Default: 25
  BRIDGE_WAIT_SEC    Seconds to wait after starting bridges before swarm nodes. Default: 8
  DISTRIBUTED_FOLLOWERS
                     true starts per-board local follower controllers. Default: false
  SWARM_CONTROLLER_IP
                     auto uses the board running this script as controller when selected.
                     config uses cluster_boards.yaml. Or set an explicit IP. Default: auto

Examples:
  ./scripts/swarm_cluster.sh dry-run
  ./scripts/swarm_cluster.sh check
  ./scripts/swarm_cluster.sh start --limit 2
  ./scripts/swarm_cluster.sh start --limit 3 --controller-ip 192.168.1.42
  ./scripts/swarm_cluster.sh status
  ./scripts/swarm_cluster.sh stop
EOF
}

if [[ -z "$ACTION" || "$ACTION" == "-h" || "$ACTION" == "--help" ]]; then
    usage
    exit 0
fi

case "$ACTION" in
    dry-run|check|start|stop|status)
        shift
        ;;
    *)
        echo "Unknown action: $ACTION"
        usage
        exit 1
        ;;
esac

while [[ "$#" -gt 0 ]]; do
    case "$1" in
        --config)
            if [[ "$#" -lt 2 ]]; then
                echo "Missing value for --config"
                exit 1
            fi
            CONFIG_FILE="$2"
            shift 2
            ;;
        --config=*)
            CONFIG_FILE="${1#*=}"
            shift
            ;;
        --limit)
            if [[ "$#" -lt 2 ]]; then
                echo "Missing value for --limit"
                exit 1
            fi
            LIMIT="$2"
            shift 2
            ;;
        --limit=*)
            LIMIT="${1#*=}"
            shift
            ;;
        --run-id)
            if [[ "$#" -lt 2 ]]; then
                echo "Missing value for --run-id"
                exit 1
            fi
            RUN_ID="$2"
            shift 2
            ;;
        --run-id=*)
            RUN_ID="${1#*=}"
            shift
            ;;
        --controller-ip)
            if [[ "$#" -lt 2 ]]; then
                echo "Missing value for --controller-ip"
                exit 1
            fi
            CONTROLLER_IP_REQUEST="$2"
            shift 2
            ;;
        --controller-ip=*)
            CONTROLLER_IP_REQUEST="${1#*=}"
            shift
            ;;
        *)
            echo "Unknown option: $1"
            usage
            exit 1
            ;;
    esac
done

if [[ ! -f "$CONFIG_FILE" ]]; then
    echo "Config file not found: $CONFIG_FILE"
    exit 1
fi

if [[ -n "$LIMIT" ]] && { ! [[ "$LIMIT" =~ ^[0-9]+$ ]] || [[ "$LIMIT" -lt 1 ]]; }; then
    echo "--limit must be a positive integer, got: $LIMIT"
    exit 1
fi

BOARD_ROWS="$(python3 - "$CONFIG_FILE" "${LIMIT:-0}" "$CONTROLLER_IP_REQUEST" "${LOCAL_IPS[@]}" <<'PY'
from pathlib import Path
import ipaddress
import sys

import yaml

config_file = Path(sys.argv[1])
limit = int(sys.argv[2])
controller_ip_request = str(sys.argv[3]).strip()
local_ips = set(sys.argv[4:])

with config_file.open("r", encoding="utf-8") as stream:
    data = yaml.safe_load(stream) or {}

cluster = data.get("cluster") or {}
required = [
    "ssh_user",
    "controller_ip",
    "project_root",
    "ip_range",
    "vehicles_per_board",
    "model",
    "ros2",
    "px4_home",
    "spawn",
]
missing = [key for key in required if key not in cluster]
if missing:
    raise SystemExit(f"cluster_boards.yaml missing cluster keys: {', '.join(missing)}")

ip_start = ipaddress.ip_address(str(cluster["ip_range"]["start"]))
ip_end = ipaddress.ip_address(str(cluster["ip_range"]["end"]))
if ip_start.version != 4 or ip_end.version != 4:
    raise SystemExit("Only IPv4 board ranges are supported")
if int(ip_end) < int(ip_start):
    raise SystemExit("ip_range.end must be >= ip_range.start")

ips = [str(ipaddress.ip_address(value)) for value in range(int(ip_start), int(ip_end) + 1)]
if limit > 0:
    ips = ips[:limit]

ssh_user = str(cluster["ssh_user"])
configured_controller_ip = str(cluster["controller_ip"])
project_root = str(cluster["project_root"])
vehicles_per_board = int(cluster["vehicles_per_board"])
model = str(cluster["model"])
ros2 = cluster["ros2"]
ros_domain_id = str(ros2["domain_id"])
ros_localhost_only = str(ros2.get("localhost_only", 0))
rmw_implementation = str(ros2.get("rmw_implementation", "rmw_fastrtps_cpp"))
network = cluster.get("network") or {}
udp_buffer_bytes = int(network.get("udp_buffer_bytes", 16777216))
px4_home = cluster["px4_home"]
spawn = cluster["spawn"]
origin_x = float(spawn["origin_x"])
origin_y = float(spawn["origin_y"])
spacing_x = float(spawn["vehicle_spacing_x"])
spacing_y = float(spawn["vehicle_spacing_y"])
board_spacing_x = float(spawn.get("board_spacing_x", vehicles_per_board * spacing_x))
board_spacing_y = float(spawn.get("board_spacing_y", vehicles_per_board * spacing_y))
spawn_grid_cols = int(spawn.get("spawn_grid_cols", 0))

if vehicles_per_board <= 0:
    raise SystemExit("vehicles_per_board must be positive")

total_vehicles = len(ips) * vehicles_per_board
if spawn_grid_cols <= 0:
    if spacing_x == 0.0 and spacing_y != 0.0:
        spawn_grid_cols = 1
    else:
        spawn_grid_cols = vehicles_per_board

global_spawn_grid_cols = int(spawn.get("global_spawn_grid_cols", 0))
if global_spawn_grid_cols <= 0:
    if spacing_x == 0.0 and spacing_y != 0.0:
        global_spawn_grid_cols = 1
    else:
        global_spawn_grid_cols = total_vehicles

if controller_ip_request in {"", "auto"}:
    controller_ip = next((ip for ip in ips if ip in local_ips), configured_controller_ip)
elif controller_ip_request == "config":
    controller_ip = configured_controller_ip
else:
    controller_ip = controller_ip_request

try:
    ipaddress.ip_address(controller_ip)
except ValueError as exc:
    raise SystemExit(f"Invalid controller IP: {controller_ip}") from exc

for board_index, ip in enumerate(ips):
    instance_start = 1 + board_index * vehicles_per_board
    spawn_x = origin_x + board_index * board_spacing_x
    spawn_y = origin_y + board_index * board_spacing_y
    is_controller = "1" if ip == controller_ip else "0"
    fields = [
        str(board_index + 1),
        ip,
        ssh_user,
        is_controller,
        str(instance_start),
        str(vehicles_per_board),
        f"{spawn_x:g}",
        f"{spawn_y:g}",
        f"{spacing_x:g}",
        f"{spacing_y:g}",
        str(spawn_grid_cols),
        str(global_spawn_grid_cols),
        model,
        str(px4_home["lat"]),
        str(px4_home["lon"]),
        str(px4_home["alt"]),
        str(total_vehicles),
        controller_ip,
        project_root,
        ros_domain_id,
        ros_localhost_only,
        rmw_implementation,
        str(udp_buffer_bytes),
    ]
    print("\t".join(fields))
PY
)"

BOARD_INDEXES=()
BOARD_IPS=()
BOARD_USERS=()
BOARD_IS_CONTROLLERS=()
BOARD_INSTANCE_STARTS=()
BOARD_COUNTS=()
BOARD_SPAWN_XS=()
BOARD_SPAWN_YS=()
BOARD_SPACING_XS=()
BOARD_SPACING_YS=()
BOARD_GRID_COLS=()
BOARD_MODELS=()
BOARD_HOME_LATS=()
BOARD_HOME_LONS=()
BOARD_HOME_ALTS=()

TOTAL_VEHICLES=""
CONTROLLER_IP=""
PROJECT_ROOT=""
ROS_DOMAIN_ID_VALUE=""
ROS_LOCALHOST_ONLY_VALUE=""
RMW_IMPLEMENTATION_VALUE=""
UDP_BUFFER_BYTES=""
GLOBAL_GRID_COLS=""

while IFS=$'\t' read -r index ip user is_controller instance_start vehicle_count spawn_x spawn_y spacing_x spacing_y spawn_grid_cols global_spawn_grid_cols model home_lat home_lon home_alt total_vehicles controller_ip project_root ros_domain_id ros_localhost_only rmw_implementation udp_buffer_bytes; do
    [[ -z "$index" ]] && continue
    BOARD_INDEXES+=("$index")
    BOARD_IPS+=("$ip")
    BOARD_USERS+=("$user")
    BOARD_IS_CONTROLLERS+=("$is_controller")
    BOARD_INSTANCE_STARTS+=("$instance_start")
    BOARD_COUNTS+=("$vehicle_count")
    BOARD_SPAWN_XS+=("$spawn_x")
    BOARD_SPAWN_YS+=("$spawn_y")
    BOARD_SPACING_XS+=("$spacing_x")
    BOARD_SPACING_YS+=("$spacing_y")
    BOARD_GRID_COLS+=("$spawn_grid_cols")
    BOARD_MODELS+=("$model")
    BOARD_HOME_LATS+=("$home_lat")
    BOARD_HOME_LONS+=("$home_lon")
    BOARD_HOME_ALTS+=("$home_alt")
    TOTAL_VEHICLES="$total_vehicles"
    CONTROLLER_IP="$controller_ip"
    PROJECT_ROOT="$project_root"
    ROS_DOMAIN_ID_VALUE="$ros_domain_id"
    ROS_LOCALHOST_ONLY_VALUE="$ros_localhost_only"
    RMW_IMPLEMENTATION_VALUE="$rmw_implementation"
    UDP_BUFFER_BYTES="$udp_buffer_bytes"
    GLOBAL_GRID_COLS="$global_spawn_grid_cols"
done <<<"$BOARD_ROWS"

if [[ "${#BOARD_IPS[@]}" -eq 0 ]]; then
    echo "No boards selected from config: $CONFIG_FILE"
    exit 1
fi

quote() {
    printf "%q" "$1"
}

ssh_run() {
    local user="$1"
    local ip="$2"
    local command="$3"

    if is_local_ip "$ip"; then
        bash -s <<<"$command"
    else
        ssh "${SSH_OPTS[@]}" "$user@$ip" "bash -s" <<<"$command"
    fi
}

is_local_ip() {
    local ip="$1"
    local local_ip

    for local_ip in "${LOCAL_IPS[@]}"; do
        if [[ "$ip" == "$local_ip" ]]; then
            return 0
        fi
    done
    return 1
}

wait_for_jobs() {
    local failed=0
    local pid
    for pid in "$@"; do
        if ! wait "$pid"; then
            failed=1
        fi
    done
    return "$failed"
}

board_label() {
    local i="$1"
    printf "%s@%s" "${BOARD_USERS[$i]}" "${BOARD_IPS[$i]}"
}

remote_ros_env() {
    cat <<EOF
export ROS_DOMAIN_ID=$(quote "$ROS_DOMAIN_ID_VALUE")
export ROS_LOCALHOST_ONLY=$(quote "$ROS_LOCALHOST_ONLY_VALUE")
export RMW_IMPLEMENTATION=$(quote "$RMW_IMPLEMENTATION_VALUE")
EOF
}

remote_tune_command() {
    local udp_buffer_bytes="$1"

    cat <<EOF
set -eo pipefail
if sudo -n true >/dev/null 2>&1; then
    sudo sysctl -w net.core.rmem_max=$(quote "$udp_buffer_bytes") >/dev/null
    sudo sysctl -w net.core.wmem_max=$(quote "$udp_buffer_bytes") >/dev/null
    sudo sysctl -w net.core.rmem_default=$(quote "$udp_buffer_bytes") >/dev/null
    sudo sysctl -w net.core.wmem_default=$(quote "$udp_buffer_bytes") >/dev/null
    echo "udp buffers tuned to $(quote "$udp_buffer_bytes") bytes"
else
    echo "WARN sudo without password is unavailable; skip UDP buffer tuning"
fi
EOF
}

remote_stop_command() {
    local project_root="$1"

    cat <<EOF
set -eo pipefail
$(remote_ros_env)
cd $(quote "$project_root")
mkdir -p logs/cluster
pkill -f '[s]warm_px4_uxrce[.]launch[.]py' 2>/dev/null || true
pkill -f '[p]x4_bridge_uxrce/.*/px4_uxrce_bridge' 2>/dev/null || true
pkill -f '[s]warm_manager/.*/swarm_manager' 2>/dev/null || true
pkill -f '[f]ormation_controller/.*/formation_controller' 2>/dev/null || true
pkill -f '[f]ormation_controller/.*/local_follower_controller' 2>/dev/null || true
pkill -f '[M]icroXRCEAgent.*udp4.*-p 8888' 2>/dev/null || true
./scripts/stop_sitl_stack.sh >/dev/null 2>&1 || true
EOF
}

remote_check_command() {
    local project_root="$1"

    cat <<EOF
set -eo pipefail
$(remote_ros_env)
test -d $(quote "$project_root")
cd $(quote "$project_root")
test -x scripts/start_px4_multi_sitl.sh
test -x scripts/stop_sitl_stack.sh
test -f config/swarm.yaml
test -f config/formations.yaml
test -f config/waypoints.yaml
command -v MicroXRCEAgent >/dev/null
source scripts/setup_env.sh >/dev/null
ros2 pkg prefix swarm_bringup >/dev/null
test -x "\$PX4_DIR/build/px4_sitl_default/bin/px4"
echo "OK \$(hostname) \$(pwd)"
EOF
}

remote_start_agent_px4_command() {
    local project_root="$1"
    local log_dir="$2"
    local instance_start="$3"
    local vehicle_count="$4"
    local spawn_x="$5"
    local spawn_y="$6"
    local spacing_x="$7"
    local spacing_y="$8"
    local model="$9"
    local home_lat="${10}"
    local home_lon="${11}"
    local home_alt="${12}"

    cat <<EOF
set -eo pipefail
$(remote_ros_env)
cd $(quote "$project_root")
mkdir -p $(quote "$log_dir")
pkill -f '[s]warm_px4_uxrce[.]launch[.]py' 2>/dev/null || true
pkill -f '[M]icroXRCEAgent.*udp4.*-p 8888' 2>/dev/null || true
./scripts/stop_sitl_stack.sh >$(quote "$log_dir/stop_before_start.log") 2>&1 || true
setsid bash -lc 'cd $(quote "$project_root"); ./scripts/start_micro_xrce_agent.sh' >$(quote "$log_dir/micro_xrce_agent.log") 2>&1 </dev/null &
echo \$! >$(quote "$log_dir/micro_xrce_agent.pid")
setsid bash -lc 'cd $(quote "$project_root"); PX4_HOME_LAT=$(quote "$home_lat") PX4_HOME_LON=$(quote "$home_lon") PX4_HOME_ALT=$(quote "$home_alt") PX4_INSTANCE_START=$(quote "$instance_start") PX4_SPAWN_X=$(quote "$spawn_x") PX4_SPAWN_Y=$(quote "$spawn_y") PX4_SPAWN_X_STEP=$(quote "$spacing_x") PX4_SPAWN_Y_STEP=$(quote "$spacing_y") HEADLESS=1 ./scripts/start_px4_multi_sitl.sh $(quote "$vehicle_count") $(quote "$model")' >$(quote "$log_dir/px4_sitl.log") 2>&1 </dev/null &
echo \$! >$(quote "$log_dir/px4_sitl.pid")
echo "started agent and PX4: instance_start=$instance_start count=$vehicle_count spawn=($spawn_x,$spawn_y)"
EOF
}

remote_start_bridge_command() {
    local project_root="$1"
    local log_dir="$2"
    local instance_start="$3"
    local vehicle_count="$4"
    local total_vehicles="$5"
    local spawn_x="$6"
    local spawn_y="$7"
    local spacing_x="$8"
    local spacing_y="$9"
    local spawn_grid_cols="${10:-}"
    if [[ -z "$spawn_grid_cols" ]]; then
        echo "remote_start_bridge_command missing spawn_grid_cols" >&2
        exit 1
    fi

    cat <<EOF
set -eo pipefail
$(remote_ros_env)
cd $(quote "$project_root")
mkdir -p $(quote "$log_dir")
source scripts/setup_env.sh >/dev/null
setsid bash -lc 'source scripts/setup_env.sh >/dev/null; ros2 launch swarm_bringup swarm_px4_uxrce.launch.py \
  instance_start:=$(quote "$instance_start") vehicle_count:=$(quote "$vehicle_count") \
  swarm_vehicle_count:=$(quote "$total_vehicles") \
	  enable_swarm_nodes:=false \
	  distributed_followers:=$(quote "$DISTRIBUTED_FOLLOWERS") \
	  enable_local_follower_controllers:=true \
	  spawn_origin_x:=$(quote "$spawn_x") spawn_origin_y:=$(quote "$spawn_y") \
  spawn_spacing_x:=$(quote "$spacing_x") spawn_spacing_y:=$(quote "$spacing_y") \
  spawn_grid_cols:=$(quote "$spawn_grid_cols") \
  swarm_config_file:=$(quote "$project_root/config/swarm.yaml") \
  formations_config_file:=$(quote "$project_root/config/formations.yaml") \
  waypoints_config_file:=$(quote "$project_root/config/waypoints.yaml")' \
  >$(quote "$log_dir/bridge.launch.log") 2>&1 </dev/null &
echo \$! >$(quote "$log_dir/bridge.launch.pid")
echo "started bridge launch: instance_start=$instance_start count=$vehicle_count"
EOF
}

remote_start_swarm_command() {
    local project_root="$1"
    local log_dir="$2"
    local total_vehicles="$3"
    local spawn_x="$4"
    local spawn_y="$5"
    local spacing_x="$6"
    local spacing_y="$7"
    local spawn_grid_cols="$8"

    cat <<EOF
set -eo pipefail
$(remote_ros_env)
cd $(quote "$project_root")
mkdir -p $(quote "$log_dir")
source scripts/setup_env.sh >/dev/null
setsid bash -lc 'source scripts/setup_env.sh >/dev/null; ros2 launch swarm_bringup swarm_px4_uxrce.launch.py \
	  instance_start:=1 vehicle_count:=$(quote "$total_vehicles") \
	  enable_bridges:=false enable_swarm_nodes:=true \
	  distributed_followers:=$(quote "$DISTRIBUTED_FOLLOWERS") \
	  enable_local_follower_controllers:=false \
	  spawn_origin_x:=$(quote "$spawn_x") spawn_origin_y:=$(quote "$spawn_y") \
  spawn_spacing_x:=$(quote "$spacing_x") spawn_spacing_y:=$(quote "$spacing_y") \
  spawn_grid_cols:=$(quote "$spawn_grid_cols") \
  swarm_config_file:=$(quote "$project_root/config/swarm.yaml") \
  formations_config_file:=$(quote "$project_root/config/formations.yaml") \
  waypoints_config_file:=$(quote "$project_root/config/waypoints.yaml")' \
  >$(quote "$log_dir/swarm_nodes.launch.log") 2>&1 </dev/null &
echo \$! >$(quote "$log_dir/swarm_nodes.launch.pid")
echo "started swarm nodes for $total_vehicles vehicles"
EOF
}

remote_status_command() {
    local project_root="$1"
    local instance_start="$2"
    local vehicle_count="$3"

    cat <<EOF
set -eo pipefail
$(remote_ros_env)
cd $(quote "$project_root") 2>/dev/null || true
echo "host=\$(hostname) time=\$(date '+%F %T')"
echo "ros_domain_id=\${ROS_DOMAIN_ID:-unset} rmw=\${RMW_IMPLEMENTATION:-unset} localhost_only=\${ROS_LOCALHOST_ONLY:-unset}"
echo "udp_buffers=\$(sysctl -n net.core.rmem_max 2>/dev/null || echo unknown)/\$(sysctl -n net.core.wmem_max 2>/dev/null || echo unknown)"
echo "--- processes"
pgrep -af '[M]icroXRCEAgent|px4 -i|gzserver|swarm_px4_uxrce[.]launch[.]py|px4_uxrce_bridge|formation_controller|swarm_manager' || true
echo "--- counts"
printf 'px4=%s\\n' "\$(pgrep -af 'px4 -i' | wc -l)"
printf 'gzserver=%s\\n' "\$(pgrep -af 'gzserver' | wc -l)"
printf 'agent=%s\\n' "\$(pgrep -af '[M]icroXRCEAgent.*udp4.*-p 8888' | wc -l)"
printf 'bridge_launch=%s\\n' "\$(pgrep -af 'swarm_px4_uxrce[.]launch[.]py.*enable_swarm_nodes:=false' | wc -l)"
printf 'swarm_launch=%s\\n' "\$(pgrep -af 'swarm_px4_uxrce[.]launch[.]py.*enable_bridges:=false' | wc -l)"
printf 'bridge_nodes=%s\\n' "\$(pgrep -af 'px4_bridge_uxrce/.*/px4_uxrce_bridge' | wc -l)"
printf 'swarm_nodes=%s\\n' "\$(pgrep -af 'swarm_manager/.*/swarm_manager|formation_controller/.*/formation_controller|formation_controller/.*/local_follower_controller' | wc -l)"
echo "--- px4 dds publishers"
for n in \$(seq $(quote "$instance_start") \$(( $(quote "$instance_start") + $(quote "$vehicle_count") - 1 ))); do
    count=\$(ros2 topic info /px4_\${n}/fmu/out/vehicle_status 2>/dev/null | awk '/Publisher count:/ {print \$3}' || true)
    printf 'px4_%s_vehicle_status_publishers=%s\\n' "\$n" "\${count:-unknown}"
done
echo "--- udp errors"
netstat -su 2>/dev/null | awk '/packets to unknown port received|packet receive errors|receive buffer errors|send buffer errors/ {gsub(/^ +/, ""); print}' || true
EOF
}

print_board_table() {
    echo "Config: $CONFIG_FILE"
    echo "Run ID: $RUN_ID"
    echo "Selected boards: ${#BOARD_IPS[@]}"
    echo "Total vehicles: $TOTAL_VEHICLES"
    echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID_VALUE"
    echo "RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION_VALUE"
    echo "UDP buffer bytes: $UDP_BUFFER_BYTES"
    printf "%-3s %-15s %-7s %-12s %-15s %-15s %-10s %-10s %-9s %-10s\n" \
        "#" "IP" "count" "instance" "uav range" "MAV_SYS_ID" "spawn_x" "spawn_y" "grid" "role"

    local i start count end mav_start mav_end role
    for i in "${!BOARD_IPS[@]}"; do
        start="${BOARD_INSTANCE_STARTS[$i]}"
        count="${BOARD_COUNTS[$i]}"
        end=$((start + count - 1))
        mav_start=$((start + 1))
        mav_end=$((end + 1))
        if [[ "${BOARD_IS_CONTROLLERS[$i]}" == "1" ]]; then
            role="controller"
        else
            role="worker"
        fi
        printf "%-3s %-15s %-7s %-12s %-15s %-15s %-10s %-10s %-9s %-10s\n" \
            "${BOARD_INDEXES[$i]}" "${BOARD_IPS[$i]}" "$count" "$start" \
            "uav_${start}-${end}" "${mav_start}-${mav_end}" "${BOARD_SPAWN_XS[$i]}" \
            "${BOARD_SPAWN_YS[$i]}" "${BOARD_GRID_COLS[$i]}" "$role"
    done
}

print_remote_commands() {
    local i log_dir
    for i in "${!BOARD_IPS[@]}"; do
        log_dir="${PROJECT_ROOT}/logs/cluster/${RUN_ID}"
        echo
        echo "### $(board_label "$i")"
        echo "# tune UDP buffers:"
        remote_tune_command "$UDP_BUFFER_BYTES"
        echo "# stop/check:"
        remote_stop_command "$PROJECT_ROOT"
        echo "# start Agent + PX4:"
        remote_start_agent_px4_command \
            "$PROJECT_ROOT" "$log_dir" "${BOARD_INSTANCE_STARTS[$i]}" "${BOARD_COUNTS[$i]}" \
            "${BOARD_SPAWN_XS[$i]}" "${BOARD_SPAWN_YS[$i]}" "${BOARD_SPACING_XS[$i]}" \
            "${BOARD_SPACING_YS[$i]}" "${BOARD_MODELS[$i]}" "${BOARD_HOME_LATS[$i]}" \
            "${BOARD_HOME_LONS[$i]}" "${BOARD_HOME_ALTS[$i]}"
        echo "# start bridge:"
        remote_start_bridge_command \
            "$PROJECT_ROOT" "$log_dir" "${BOARD_INSTANCE_STARTS[$i]}" "${BOARD_COUNTS[$i]}" \
            "$TOTAL_VEHICLES" "${BOARD_SPAWN_XS[$i]}" "${BOARD_SPAWN_YS[$i]}" "${BOARD_SPACING_XS[$i]}" \
            "${BOARD_SPACING_YS[$i]}" "${BOARD_GRID_COLS[$i]}"
    done
    echo
    echo "### controller swarm nodes"
    remote_start_swarm_command \
        "$PROJECT_ROOT" "${PROJECT_ROOT}/logs/cluster/${RUN_ID}" "$TOTAL_VEHICLES" \
        "${BOARD_SPAWN_XS[0]}" "${BOARD_SPAWN_YS[0]}" "${BOARD_SPACING_XS[0]}" \
        "${BOARD_SPACING_YS[0]}" "$GLOBAL_GRID_COLS"
}

run_check() {
    echo "Checking selected boards..."
    local pids=()
    local i label
    for i in "${!BOARD_IPS[@]}"; do
        label="$(board_label "$i")"
        (
            echo "[$label] checking..."
            ssh_run "${BOARD_USERS[$i]}" "${BOARD_IPS[$i]}" "$(remote_check_command "$PROJECT_ROOT")"
        ) &
        pids+=("$!")
    done
    wait_for_jobs "${pids[@]}"
}

run_stop() {
    echo "Stopping selected boards..."
    local pids=()
    local i label
    for i in "${!BOARD_IPS[@]}"; do
        label="$(board_label "$i")"
        (
            echo "[$label] stopping..."
            ssh_run "${BOARD_USERS[$i]}" "${BOARD_IPS[$i]}" "$(remote_stop_command "$PROJECT_ROOT")"
            echo "[$label] stopped"
        ) &
        pids+=("$!")
    done
    wait_for_jobs "${pids[@]}"
}

run_start() {
    local log_dir="${PROJECT_ROOT}/logs/cluster/${RUN_ID}"
    echo "Starting cluster run_id=$RUN_ID..."
    run_check
    run_stop

    echo "Tuning UDP buffers on selected boards..."
    local tune_pids=()
    local i label
    for i in "${!BOARD_IPS[@]}"; do
        label="$(board_label "$i")"
        (
            echo "[$label] tuning UDP buffers..."
            ssh_run "${BOARD_USERS[$i]}" "${BOARD_IPS[$i]}" "$(remote_tune_command "$UDP_BUFFER_BYTES")"
        ) &
        tune_pids+=("$!")
    done
    wait_for_jobs "${tune_pids[@]}"

    echo "Starting MicroXRCEAgent and PX4/Gazebo on selected boards..."
    local pids=()
    for i in "${!BOARD_IPS[@]}"; do
        label="$(board_label "$i")"
        (
            echo "[$label] starting agent + PX4..."
            ssh_run "${BOARD_USERS[$i]}" "${BOARD_IPS[$i]}" "$(
                remote_start_agent_px4_command \
                    "$PROJECT_ROOT" "$log_dir" "${BOARD_INSTANCE_STARTS[$i]}" "${BOARD_COUNTS[$i]}" \
                    "${BOARD_SPAWN_XS[$i]}" "${BOARD_SPAWN_YS[$i]}" "${BOARD_SPACING_XS[$i]}" \
                    "${BOARD_SPACING_YS[$i]}" "${BOARD_MODELS[$i]}" "${BOARD_HOME_LATS[$i]}" \
                    "${BOARD_HOME_LONS[$i]}" "${BOARD_HOME_ALTS[$i]}"
            )"
        ) &
        pids+=("$!")
    done
    wait_for_jobs "${pids[@]}"

    echo "Waiting ${PX4_WAIT_SEC}s for PX4/Gazebo startup..."
    sleep "$PX4_WAIT_SEC"

    echo "Starting bridge launch on selected boards..."
    pids=()
    for i in "${!BOARD_IPS[@]}"; do
        label="$(board_label "$i")"
        (
            echo "[$label] starting bridges..."
            ssh_run "${BOARD_USERS[$i]}" "${BOARD_IPS[$i]}" "$(
                remote_start_bridge_command \
                    "$PROJECT_ROOT" "$log_dir" "${BOARD_INSTANCE_STARTS[$i]}" "${BOARD_COUNTS[$i]}" \
                    "$TOTAL_VEHICLES" "${BOARD_SPAWN_XS[$i]}" "${BOARD_SPAWN_YS[$i]}" "${BOARD_SPACING_XS[$i]}" \
                    "${BOARD_SPACING_YS[$i]}" "${BOARD_GRID_COLS[$i]}"
            )"
        ) &
        pids+=("$!")
    done
    wait_for_jobs "${pids[@]}"

    echo "Waiting ${BRIDGE_WAIT_SEC}s for bridge nodes..."
    sleep "$BRIDGE_WAIT_SEC"

    local controller_index=""
    for i in "${!BOARD_IPS[@]}"; do
        if [[ "${BOARD_IS_CONTROLLERS[$i]}" == "1" ]]; then
            controller_index="$i"
            break
        fi
    done
    if [[ -z "$controller_index" ]]; then
        echo "Controller IP $CONTROLLER_IP is not selected. Use a --limit that includes the controller board."
        exit 1
    fi

    label="$(board_label "$controller_index")"
    echo "[$label] starting swarm manager and formation controller..."
    ssh_run "${BOARD_USERS[$controller_index]}" "${BOARD_IPS[$controller_index]}" "$(
        remote_start_swarm_command \
            "$PROJECT_ROOT" "$log_dir" "$TOTAL_VEHICLES" "${BOARD_SPAWN_XS[0]}" \
            "${BOARD_SPAWN_YS[0]}" "${BOARD_SPACING_XS[0]}" "${BOARD_SPACING_YS[0]}" \
            "$GLOBAL_GRID_COLS"
    )"

    echo
    echo "Cluster start requested."
    echo "Logs are under: $log_dir on each board"
    echo "Next checks:"
    echo "  ./scripts/swarm_cluster.sh status${LIMIT:+ --limit $LIMIT}"
    echo "  source scripts/setup_env.sh && ros2 node list"
    echo "  ./scripts/swarm_arm_takeoff.sh --count $TOTAL_VEHICLES"
}

run_status() {
    echo "Status for selected boards..."
    local pids=()
    local i label
    for i in "${!BOARD_IPS[@]}"; do
        label="$(board_label "$i")"
        (
            echo
            echo "===== $label ====="
            ssh_run "${BOARD_USERS[$i]}" "${BOARD_IPS[$i]}" "$(
                remote_status_command "$PROJECT_ROOT" "${BOARD_INSTANCE_STARTS[$i]}" "${BOARD_COUNTS[$i]}"
            )"
        ) &
        pids+=("$!")
    done
    wait_for_jobs "${pids[@]}"
}

case "$ACTION" in
    dry-run)
        print_board_table
        print_remote_commands
        ;;
    check)
        print_board_table
        run_check
        ;;
    start)
        print_board_table
        run_start
        ;;
    stop)
        print_board_table
        run_stop
        ;;
    status)
        print_board_table
        run_status
        ;;
esac
