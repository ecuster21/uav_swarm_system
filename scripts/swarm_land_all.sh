#!/usr/bin/env bash

set -eo pipefail

source "$HOME/uav_swarm_system/scripts/setup_env.sh"

DRONES=("$@")
if [ "${#DRONES[@]}" -eq 0 ]; then
    DRONES=(uav_1 uav_2 uav_3)
fi

echo "Stopping autonomous formation controller before landing..."
pkill -f "install/formation_controller/lib/formation_controller/formation_controller" 2>/dev/null || true

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
