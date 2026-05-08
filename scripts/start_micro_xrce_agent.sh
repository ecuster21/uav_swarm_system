#!/usr/bin/env bash

set -e

source "$HOME/uav_swarm_system/scripts/setup_env.sh"

echo "Starting MicroXRCEAgent on UDP port 8888..."
MicroXRCEAgent udp4 -p 8888
