#!/usr/bin/env bash

set -e

export UAV_SWARM_ROOT="$HOME/uav_swarm_system"
export PX4_DIR="$HOME/PX4-Autopilot"
export MICRO_XRCE_DDS_AGENT_DIR="$HOME/Micro-XRCE-DDS-Agent"
export PX4_ROS_COM_WS="${PX4_ROS_COM_WS:-$HOME/px4_ros_com_ws}"

export PATH="/usr/bin:/bin:/opt/ros/humble/bin:$PATH"

# 只 source 固定版本环境和已构建工作区，不在这里安装或升级依赖。
source /opt/ros/humble/setup.bash

if [ -f "$PX4_ROS_COM_WS/install/setup.bash" ]; then
    source "$PX4_ROS_COM_WS/install/setup.bash"
fi

if [ -f "$UAV_SWARM_ROOT/ros2_ws/install/setup.bash" ]; then
    source "$UAV_SWARM_ROOT/ros2_ws/install/setup.bash"
else
    echo "ROS2 workspace install not found. Build ros2_ws before using custom messages."
fi

if [ -f "$UAV_SWARM_ROOT/config/cluster_boards.yaml" ]; then
    eval "$(
        python3 - "$UAV_SWARM_ROOT/config/cluster_boards.yaml" <<'PY'
import sys
import yaml

with open(sys.argv[1], "r", encoding="utf-8") as stream:
    config = yaml.safe_load(stream) or {}
ros2 = (config.get("cluster") or {}).get("ros2") or {}
if "domain_id" in ros2:
    print(f"export ROS_DOMAIN_ID={int(ros2['domain_id'])}")
if "localhost_only" in ros2:
    print(f"export ROS_LOCALHOST_ONLY={int(ros2['localhost_only'])}")
if "rmw_implementation" in ros2:
    print(f"export RMW_IMPLEMENTATION={ros2['rmw_implementation']}")
PY
    )"
fi

echo "======================================"
echo "UAV Swarm Environment Loaded"
echo "======================================"
echo "UAV_SWARM_ROOT=$UAV_SWARM_ROOT"
echo "PX4_DIR=$PX4_DIR"
echo "MICRO_XRCE_DDS_AGENT_DIR=$MICRO_XRCE_DDS_AGENT_DIR"
echo "PX4_ROS_COM_WS=$PX4_ROS_COM_WS"
echo "ROS_DISTRO=$ROS_DISTRO"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset}"
echo "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-unset}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-unset}"
echo "python3=$(command -v python3)"
echo "Gazebo version:"
if command -v gazebo >/dev/null 2>&1; then
    gazebo --version || true
else
    echo "gazebo not installed"
fi
echo "======================================"
