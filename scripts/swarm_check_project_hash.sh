#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UAV_SWARM_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
CONFIG_FILE="$UAV_SWARM_ROOT/config/cluster_boards.yaml"
LIMIT=""

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
  ./scripts/swarm_check_project_hash.sh [--limit N] [--config PATH]

Checks whether source/config/script/doc files are identical across selected boards.
Runtime folders such as build, install, log, logs, references, and .git are ignored.
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

project_hash_command() {
    local project_root="$1"
    printf "%s\n" "cd $(printf "%q" "$project_root") && find AGENTS.md README.md config docs scripts ros2_ws/src -type f ! -path '*/.git/*' ! -path '*/__pycache__/*' ! -name '.git' ! -name '*.pyc' -print0 | sort -z | xargs -0 sha256sum | sha256sum"
}

declare -A HASH_BY_LABEL=()
expected_hash=""
failed=0

for row in "${BOARD_ROWS[@]}"; do
    IFS=$'\t' read -r user ip project_root <<<"$row"
    label="$user@$ip"
    command="$(project_hash_command "$project_root")"
    if is_local_ip "$ip"; then
        hash="$(bash -lc "$command" | awk '{print $1}')"
    else
        hash="$(ssh "${SSH_OPTS[@]}" "$label" "$command" | awk '{print $1}')"
    fi
    HASH_BY_LABEL["$label"]="$hash"
    if [[ -z "$expected_hash" ]]; then
        expected_hash="$hash"
    elif [[ "$hash" != "$expected_hash" ]]; then
        failed=1
    fi
done

for label in "${!HASH_BY_LABEL[@]}"; do
    printf "%-18s %s\n" "$label" "${HASH_BY_LABEL[$label]}"
done | sort

if [[ "$failed" -eq 0 ]]; then
    echo "OK: selected boards have identical project source hashes"
else
    echo "MISMATCH: selected boards differ"
    exit 2
fi
