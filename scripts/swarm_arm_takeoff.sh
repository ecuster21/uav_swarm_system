#!/usr/bin/env bash

set -eo pipefail

source "$HOME/uav_swarm_system/scripts/setup_env.sh"

DRONES=("$@")
if [ "${#DRONES[@]}" -eq 0 ]; then
    DRONES=(uav_1 uav_2 uav_3)
fi

echo "Arming drones: ${DRONES[*]}"
for drone in "${DRONES[@]}"; do
    ros2 service call "/$drone/arm" std_srvs/srv/Trigger "{}"
done

echo "Sending takeoff to drones: ${DRONES[*]}"
for drone in "${DRONES[@]}"; do
    ros2 service call "/$drone/takeoff" std_srvs/srv/Trigger "{}"
done

echo "Arm/takeoff requests completed."
