#!/usr/bin/env bash

set -euo pipefail

echo "Stopping UAV swarm SITL helper processes..."

GAZEBO_MASTER_PORT="${GAZEBO_MASTER_PORT:-11345}"

list_gazebo_master_pids() {
    ss -ltnp "sport = :$GAZEBO_MASTER_PORT" 2>/dev/null \
        | sed -n 's/.*pid=\([0-9]\+\).*/\1/p' \
        | sort -u
}

kill_gazebo_master_owner() {
    local pid
    local command_name

    for pid in $(list_gazebo_master_pids); do
        command_name="$(ps -p "$pid" -o comm= 2>/dev/null || true)"
        case "$command_name" in
            gzserver|gzclient|gazebo)
                echo "Stopping Gazebo master owner pid=$pid command=$command_name"
                kill "$pid" 2>/dev/null || true
                ;;
            "")
                ;;
            *)
                echo "Gazebo master port $GAZEBO_MASTER_PORT is owned by non-Gazebo process pid=$pid command=$command_name"
                return 1
                ;;
        esac
    done
}

wait_until_clean() {
    local attempt
    for attempt in {1..20}; do
        if ! pgrep -f "px4-simulator_mavlink" >/dev/null \
            && ! pgrep -x px4 >/dev/null \
            && ! pgrep -x gzclient >/dev/null \
            && [ -z "$(list_gazebo_master_pids)" ]; then
            return 0
        fi
        sleep 0.25
    done
    return 1
}

pkill -f "px4-simulator_mavlink" 2>/dev/null || true
pkill -x px4 2>/dev/null || true
pkill -x gzclient 2>/dev/null || true
pkill -x gzserver 2>/dev/null || true
kill_gazebo_master_owner

if ! wait_until_clean; then
    echo "Some SITL processes did not stop after SIGTERM; sending SIGKILL to known PX4/Gazebo processes."
    pkill -9 -f "px4-simulator_mavlink" 2>/dev/null || true
    pkill -9 -x px4 2>/dev/null || true
    pkill -9 -x gzclient 2>/dev/null || true
    pkill -9 -x gzserver 2>/dev/null || true
    kill_gazebo_master_owner
fi

echo "Remaining PX4/Gazebo processes:"
ps -ef \
    | grep -E "px4|gzserver|gzclient" \
    | grep -v grep \
    | grep -v "<defunct>" \
    || true

if [ -n "$(list_gazebo_master_pids)" ]; then
    echo "Gazebo master port $GAZEBO_MASTER_PORT is still busy:"
    ss -ltnp "sport = :$GAZEBO_MASTER_PORT" || true
    exit 1
fi

echo "SITL stack stopped cleanly."
