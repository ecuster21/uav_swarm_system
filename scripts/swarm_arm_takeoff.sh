#!/usr/bin/env bash

set -eo pipefail

source "$HOME/uav_swarm_system/scripts/setup_env.sh"

DRONES=("$@")
if [ "${#DRONES[@]}" -eq 0 ]; then
    mapfile -t DRONES < <(
        timeout 8s ros2 topic echo /swarm/state --once 2>/dev/null |
            sed -n "s/^[[:space:]]*drone_id: ['\"]\\?\\(uav_[0-9][0-9]*\\)['\"]\\?$/\\1/p" |
            sort -V -u
    )
fi

if [ "${#DRONES[@]}" -eq 0 ]; then
    mapfile -t DRONES < <(
        ros2 node list 2>/dev/null |
            sed -n 's#^/\(uav_[0-9][0-9]*\)/px4_bridge$#\1#p' |
            sort -V -u
    )
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

echo "Arming drones: ${DRONES[*]}"
for drone in "${DRONES[@]}"; do
    echo "Arming $drone..."
    timeout 8s ros2 service call "/$drone/arm" std_srvs/srv/Trigger "{}"
done

echo "Sending takeoff to drones: ${DRONES[*]}"
for drone in "${DRONES[@]}"; do
    echo "Taking off $drone..."
    timeout 8s ros2 service call "/$drone/takeoff" std_srvs/srv/Trigger "{}"
done

echo "Arm/takeoff requests completed."
