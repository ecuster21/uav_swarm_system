#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UAV_SWARM_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
CONFIG_FILE="$UAV_SWARM_ROOT/config/cluster_boards.yaml"
LIMIT=""
BUILD_AFTER_SYNC="false"

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
  ./scripts/swarm_sync_project.sh [--limit N] [--config PATH] [--build]

Syncs this board's project source/config/scripts/docs to selected boards.
Runtime folders are excluded: .git, references, ros2_ws/build, ros2_ws/install,
ros2_ws/log, logs, and Python caches.

Options:
  --build   Run colcon build for the ROS2 workspace on each synced remote board.
EOF
}

while [[ "$#" -gt 0 ]]; do
    case "$1" in
        --config)
            CONFIG_FILE="$2"
            shift 2
            ;;
        --config=*)
            CONFIG_FILE="${1#*=}"
            shift
            ;;
        --limit)
            LIMIT="$2"
            shift 2
            ;;
        --limit=*)
            LIMIT="${1#*=}"
            shift
            ;;
        --build)
            BUILD_AFTER_SYNC="true"
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            usage
            exit 1
            ;;
    esac
done

if [[ -n "$LIMIT" ]] && { ! [[ "$LIMIT" =~ ^[0-9]+$ ]] || [[ "$LIMIT" -lt 1 ]]; }; then
    echo "--limit must be a positive integer, got: $LIMIT"
    exit 1
fi

readarray -t BOARD_ROWS < <(python3 - "$CONFIG_FILE" "${LIMIT:-0}" <<'PY'
from pathlib import Path
import ipaddress
import sys

import yaml

config_file = Path(sys.argv[1])
limit = int(sys.argv[2])

with config_file.open("r", encoding="utf-8") as stream:
    data = yaml.safe_load(stream) or {}

cluster = data.get("cluster") or {}
ips = [
    str(ipaddress.ip_address(value))
    for value in range(
        int(ipaddress.ip_address(str(cluster["ip_range"]["start"]))),
        int(ipaddress.ip_address(str(cluster["ip_range"]["end"]))) + 1,
    )
]
if limit > 0:
    ips = ips[:limit]

for ip in ips:
    print("\t".join([str(cluster["ssh_user"]), ip, str(cluster["project_root"])]))
PY
)

is_local_ip() {
    local ip="$1"
    local local_ip
    for local_ip in "${LOCAL_IPS[@]}"; do
        [[ "$ip" == "$local_ip" ]] && return 0
    done
    return 1
}

sync_one_board() {
    local user="$1"
    local ip="$2"
    local project_root="$3"
    local label="$user@$ip"

    if is_local_ip "$ip"; then
        echo "[$label] local board, skip rsync"
        return
    fi

    ssh "${SSH_OPTS[@]}" "$label" "mkdir -p $(printf "%q" "$project_root")"
    rsync -az --delete \
        --exclude '.git/' \
        --exclude '.git' \
        --exclude '*/.git/' \
        --exclude '*/.git' \
        --exclude 'references/' \
        --exclude 'ros2_ws/build/' \
        --exclude 'ros2_ws/install/' \
        --exclude 'ros2_ws/log/' \
        --exclude 'logs/' \
        --exclude '__pycache__/' \
        --exclude '*.pyc' \
        -e "ssh ${SSH_OPTS[*]}" \
        "$UAV_SWARM_ROOT/" "$label:$project_root/"
    echo "[$label] synced"

    if [[ "$BUILD_AFTER_SYNC" == "true" ]]; then
        ssh "${SSH_OPTS[@]}" "$label" "bash -lc 'cd $(printf "%q" "$project_root")/ros2_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install'"
        echo "[$label] build ok"
    fi
}

pids=()
for row in "${BOARD_ROWS[@]}"; do
    IFS=$'\t' read -r user ip project_root <<<"$row"
    sync_one_board "$user" "$ip" "$project_root" &
    pids+=("$!")
done

failed=0
for pid in "${pids[@]}"; do
    if ! wait "$pid"; then
        failed=1
    fi
done

if [[ "$failed" -ne 0 ]]; then
    echo "Sync failed on at least one board"
    exit 1
fi

echo "Project sync complete"
