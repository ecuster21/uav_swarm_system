#!/usr/bin/env bash

set -eo pipefail

source "$HOME/uav_swarm_system/scripts/setup_env.sh"

ros2 topic echo /swarm/state --once
