#!/usr/bin/env bash

set -e

export UAV_SWARM_ROOT="$HOME/uav_swarm_system"
export PX4_DIR="$HOME/PX4-Autopilot"
export MICRO_XRCE_DDS_AGENT_DIR="$HOME/Micro-XRCE-DDS-Agent"

source /opt/ros/humble/setup.bash

if [ -f "$UAV_SWARM_ROOT/ros2_ws/install/setup.bash" ]; then
    source "$UAV_SWARM_ROOT/ros2_ws/install/setup.bash"
fi

echo "======================================"
echo "UAV Swarm Environment Loaded"
echo "======================================"
echo "UAV_SWARM_ROOT=$UAV_SWARM_ROOT"
echo "PX4_DIR=$PX4_DIR"
echo "MICRO_XRCE_DDS_AGENT_DIR=$MICRO_XRCE_DDS_AGENT_DIR"
echo "ROS_DISTRO=$ROS_DISTRO"
echo "Gazebo version:"
gazebo --version || true
echo "======================================"
