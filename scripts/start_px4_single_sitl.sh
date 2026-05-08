#!/usr/bin/env bash

set -e

source "$HOME/uav_swarm_system/scripts/setup_env.sh"

cd "$PX4_DIR"

echo "Starting PX4 v1.14.4 single vehicle SITL with Gazebo Classic..."
make px4_sitl gazebo-classic
