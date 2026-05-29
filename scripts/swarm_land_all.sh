#!/usr/bin/env bash

set -eo pipefail

source "$HOME/uav_swarm_system/scripts/setup_env.sh"

generate_drones() {
    local count="$1"
    local index

    if ! [[ "$count" =~ ^[0-9]+$ ]] || [ "$count" -le 0 ]; then
        return 1
    fi

    for index in $(seq 1 "$count"); do
        printf 'uav_%s\n' "$index"
    done
}

discover_vehicle_count_from_launch() {
    ps -eo cmd |
        sed -n 's/.*swarm_px4_uxrce\.launch\.py.*vehicle_count:=\([0-9][0-9]*\).*/\1/p' |
        sort -n |
        tail -n 1
}

discover_vehicle_count_from_bridge_processes() {
    ps -eo cmd |
        sed -n 's/.*__ns:=\/uav_\([0-9][0-9]*\).*/\1/p' |
        sort -n |
        tail -n 1
}

discover_vehicle_count_from_px4() {
    ps -eo cmd |
        sed -n 's/.*\/px4 .* -i \([0-9][0-9]*\) .*/\1/p' |
        sort -n |
        tail -n 1
}

COUNT=""
COUNT_SPECIFIED=0
EXPLICIT_DRONES=0
DRY_RUN=0
DRONES=()
while [ "$#" -gt 0 ]; do
    case "$1" in
        --dry-run)
            DRY_RUN=1
            shift
            ;;
        --count|-n)
            if [ "$#" -lt 2 ]; then
                echo "Missing value for $1"
                exit 1
            fi
            COUNT="${2:-}"
            COUNT_SPECIFIED=1
            shift 2
            ;;
        --count=*)
            COUNT="${1#*=}"
            COUNT_SPECIFIED=1
            shift
            ;;
        [0-9]*)
            COUNT="$1"
            COUNT_SPECIFIED=1
            shift
            ;;
        *)
            DRONES+=("$1")
            EXPLICIT_DRONES=1
            shift
            ;;
    esac
done

if [ -n "$COUNT" ]; then
    mapfile -t DRONES < <(generate_drones "$COUNT")
fi

# 未显式指定时按可靠性递减自动发现：/swarm/state、ROS 节点、launch 参数、PX4 进程、静态配置。
if [ "${#DRONES[@]}" -eq 0 ]; then
    mapfile -t DRONES < <(
        timeout 8s ros2 topic echo /swarm/state --once 2>/dev/null |
            sed -n "s/^[[:space:]]*drone_id: ['\"]\\?\\(uav_[0-9][0-9]*\\)['\"]\\?$/\\1/p" |
            sort -V -u
    )
fi

if [ "${#DRONES[@]}" -eq 0 ]; then
    mapfile -t DRONES < <(
        ros2 node list --no-daemon --spin-time 5 2>/dev/null |
            sed -n 's#^/\(uav_[0-9][0-9]*\)/px4_bridge$#\1#p' |
            sort -V -u
    )
fi

if [ "$COUNT_SPECIFIED" -eq 0 ] && [ "$EXPLICIT_DRONES" -eq 0 ]; then
    COUNT="$(discover_vehicle_count_from_launch)"
    if [ -n "$COUNT" ] && [ "$COUNT" -gt "${#DRONES[@]}" ]; then
        mapfile -t DRONES < <(generate_drones "$COUNT")
    fi
fi

if [ "$COUNT_SPECIFIED" -eq 0 ] && [ "$EXPLICIT_DRONES" -eq 0 ]; then
    COUNT="$(discover_vehicle_count_from_bridge_processes)"
    if [ -n "$COUNT" ] && [ "$COUNT" -gt "${#DRONES[@]}" ]; then
        mapfile -t DRONES < <(generate_drones "$COUNT")
    fi
fi

if [ "$COUNT_SPECIFIED" -eq 0 ] && [ "$EXPLICIT_DRONES" -eq 0 ]; then
    COUNT="$(discover_vehicle_count_from_px4)"
    if [ -n "$COUNT" ] && [ "$COUNT" -gt "${#DRONES[@]}" ]; then
        mapfile -t DRONES < <(generate_drones "$COUNT")
    fi
fi

if [ "${#DRONES[@]}" -eq 0 ]; then
    mapfile -t DRONES < <(
        python3 - <<'PY'
from pathlib import Path
import yaml

config = Path.home() / "uav_swarm_system" / "config" / "swarm.yaml"
with config.open("r", encoding="utf-8") as stream:
    swarm = (yaml.safe_load(stream) or {}).get("swarm", {})
for drone in swarm.get("drones", []):
    print(drone["id"])
PY
    )
fi

if [ "${#DRONES[@]}" -eq 0 ]; then
    echo "No drones found. Start the ROS2 swarm launch first or pass --count N."
    exit 1
fi

if [ "$DRY_RUN" -eq 1 ]; then
    echo "Discovered drones: ${DRONES[*]}"
    exit 0
fi

echo "Stopping autonomous formation controller before landing..."
pkill -f "install/formation_controller/lib/formation_controller/formation_controller" 2>/dev/null || true

# 先让 bridge 停止接收 active FormationTarget，再发送 land，避免编队控制继续推 Offboard 目标。
echo "Disabling active formation targets..."
for drone in "${DRONES[@]}"; do
    ros2 topic pub --once "/$drone/formation_target" swarm_msgs/msg/FormationTarget \
        "{drone_id: '$drone', source: 'swarm_land_all', position: {x: 0.0, y: 0.0, z: 0.0}, yaw: 0.0, active: false}" \
        >/dev/null
done

echo "Sending land to drones: ${DRONES[*]}"
for drone in "${DRONES[@]}"; do
    ros2 service call "/$drone/land" std_srvs/srv/Trigger "{}"
done

echo "Land requests completed."
