#!/usr/bin/env bash

set -e

source "$HOME/uav_swarm_system/scripts/setup_env.sh"

VEHICLE_COUNT="${1:-3}"
MODEL="${2:-iris}"

PX4_MULTI_SCRIPT="$PX4_DIR/Tools/simulation/gazebo-classic/sitl_multiple_run.sh"

if [ ! -f "$PX4_MULTI_SCRIPT" ]; then
    echo "Cannot find PX4 multi-vehicle script:"
    echo "$PX4_MULTI_SCRIPT"
    echo "Please check with:"
    echo "find ~/PX4-Autopilot/Tools -name sitl_multiple_run.sh"
    exit 1
fi

cd "$PX4_DIR"

echo "Starting PX4 multi-vehicle SITL..."
echo "Vehicle count: $VEHICLE_COUNT"
echo "Model: $MODEL"

bash "$PX4_MULTI_SCRIPT" -m "$MODEL" -n "$VEHICLE_COUNT"
