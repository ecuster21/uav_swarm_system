#!/usr/bin/env bash

set -e

source "$HOME/uav_swarm_system/scripts/setup_env.sh"

VEHICLE_COUNT="${1:-3}"
MODEL="${2:-iris}"
export HEADLESS="${HEADLESS:-1}"
PX4_INSTANCE_START="${PX4_INSTANCE_START:-1}"
GAZEBO_WORLD="${GAZEBO_WORLD:-empty}"
PX4_TARGET="${PX4_TARGET:-px4_sitl_default}"
PX4_SPAWN_X="${PX4_SPAWN_X:-0.0}"
PX4_SPAWN_Y="${PX4_SPAWN_Y:-3.0}"
PX4_SPAWN_X_STEP="${PX4_SPAWN_X_STEP:-0.0}"
PX4_SPAWN_Y_STEP="${PX4_SPAWN_Y_STEP:-3.0}"
PX4_HOME_LAT="${PX4_HOME_LAT:-}"
PX4_HOME_LON="${PX4_HOME_LON:-}"
PX4_HOME_ALT="${PX4_HOME_ALT:-}"

PX4_MULTI_SCRIPT="$PX4_DIR/Tools/simulation/gazebo-classic/sitl_multiple_run.sh"

if [ ! -f "$PX4_MULTI_SCRIPT" ]; then
    echo "Cannot find PX4 multi-vehicle script:"
    echo "$PX4_MULTI_SCRIPT"
    echo "Please check with:"
    echo "find ~/PX4-Autopilot/Tools -name sitl_multiple_run.sh"
    exit 1
fi

if ! [[ "$VEHICLE_COUNT" =~ ^[0-9]+$ ]] || [ "$VEHICLE_COUNT" -lt 1 ]; then
    echo "VEHICLE_COUNT must be a positive integer, got: $VEHICLE_COUNT"
    exit 1
fi

if ! [[ "$PX4_INSTANCE_START" =~ ^[0-9]+$ ]] || [ "$PX4_INSTANCE_START" -lt 1 ]; then
    echo "PX4_INSTANCE_START must be a positive integer, got: $PX4_INSTANCE_START"
    exit 1
fi

validate_optional_float() {
    local name="$1"
    local value="$2"

    if [ -n "$value" ] && ! [[ "$value" =~ ^-?[0-9]+([.][0-9]+)?$ ]]; then
        echo "$name must be a decimal number, got: $value"
        exit 1
    fi
}

validate_optional_float "PX4_HOME_LAT" "$PX4_HOME_LAT"
validate_optional_float "PX4_HOME_LON" "$PX4_HOME_LON"
validate_optional_float "PX4_HOME_ALT" "$PX4_HOME_ALT"

if { [ -n "$PX4_HOME_LAT" ] && [ -z "$PX4_HOME_LON" ]; } || \
   { [ -z "$PX4_HOME_LAT" ] && [ -n "$PX4_HOME_LON" ]; }; then
    echo "PX4_HOME_LAT and PX4_HOME_LON must be set together."
    exit 1
fi

if [ -n "$PX4_HOME_LAT" ]; then
    export PX4_HOME_LAT
fi

if [ -n "$PX4_HOME_LON" ]; then
    export PX4_HOME_LON
fi

if [ -n "$PX4_HOME_ALT" ]; then
    export PX4_HOME_ALT
fi

cd "$PX4_DIR"

echo "Stopping stale PX4/Gazebo processes before launch..."
"$UAV_SWARM_ROOT/scripts/stop_sitl_stack.sh"

echo "Starting PX4 multi-vehicle SITL..."
echo "Vehicle count: $VEHICLE_COUNT"
echo "Model: $MODEL"
echo "Headless: $HEADLESS"
echo "PX4 instance start: $PX4_INSTANCE_START"
echo "First MAV_SYS_ID: $((PX4_INSTANCE_START + 1))"
echo "Gazebo world: $GAZEBO_WORLD"
echo "PX4 target: $PX4_TARGET"
if [ -n "$PX4_HOME_LAT$PX4_HOME_LON$PX4_HOME_ALT" ]; then
    echo "Home global origin override: lat=${PX4_HOME_LAT:-default} lon=${PX4_HOME_LON:-default} alt=${PX4_HOME_ALT:-default}"
else
    echo "Home global origin: PX4/Gazebo world default"
fi
echo "Spawn origin: x=$PX4_SPAWN_X y=$PX4_SPAWN_Y"
echo "Spawn step: x=$PX4_SPAWN_X_STEP y=$PX4_SPAWN_Y_STEP"

build_path="$PX4_DIR/build/$PX4_TARGET"

if [ ! -x "$build_path/bin/px4" ]; then
    echo "Cannot find PX4 binary:"
    echo "$build_path/bin/px4"
    echo "Build PX4 first with:"
    echo "cd $PX4_DIR && DONT_RUN=1 HEADLESS=1 make px4_sitl gazebo-classic"
    exit 1
fi

source "$PX4_DIR/Tools/simulation/gazebo-classic/setup_gazebo.bash" "$PX4_DIR" "$build_path"

if [[ "${ENABLE_GAZEBO_ROS_PLUGINS:-0}" == "1" && -n "${ROS_VERSION:-}" && "$ROS_VERSION" = "2" ]]; then
    ros_args="-s libgazebo_ros_init.so -s libgazebo_ros_factory.so"
else
    ros_args=""
fi

cleanup() {
    pkill -x px4 2>/dev/null || true
    pkill -x gzclient 2>/dev/null || true
    pkill -x gzserver 2>/dev/null || true
}

spawn_model() {
    local model="$1"
    local instance_id="$2"
    local x="$3"
    local y="$4"
    local working_dir="$build_path/rootfs/$instance_id"
    local sdf_file="/tmp/${model}_${instance_id}.sdf"

    mkdir -p "$working_dir"

    pushd "$working_dir" >/dev/null
    echo "Starting PX4 instance $instance_id in $(pwd)"
    "$build_path/bin/px4" -i "$instance_id" -d "$build_path/etc" >out.log 2>err.log &
    popd >/dev/null

    python3 "$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/scripts/jinja_gen.py" \
        "$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/$model/$model.sdf.jinja" \
        "$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic" \
        --mavlink_tcp_port "$((4560 + instance_id))" \
        --mavlink_udp_port "$((14560 + instance_id))" \
        --mavlink_id "$((1 + instance_id))" \
        --gst_udp_port "$((5600 + instance_id))" \
        --video_uri "$((5600 + instance_id))" \
        --mavlink_cam_udp_port "$((14530 + instance_id))" \
        --output-file "$sdf_file"

    echo "Spawning ${model}_${instance_id} at ${x} ${y}"
    gz model --spawn-file="$sdf_file" --model-name="${model}_${instance_id}" -x "$x" -y "$y" -z 0.83
}

trap cleanup SIGINT SIGTERM EXIT

echo "Starting Gazebo server..."
ROS_VERSION= gzserver "$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/$GAZEBO_WORLD.world" --verbose $ros_args &
sleep 5

for ((i = 0; i < VEHICLE_COUNT; i++)); do
    spawn_x="$(awk -v origin="$PX4_SPAWN_X" -v step="$PX4_SPAWN_X_STEP" -v i="$i" 'BEGIN { printf "%.3f", origin + step * i }')"
    spawn_y="$(awk -v origin="$PX4_SPAWN_Y" -v step="$PX4_SPAWN_Y_STEP" -v i="$i" 'BEGIN { printf "%.3f", origin + step * i }')"
    spawn_model "$MODEL" "$((PX4_INSTANCE_START + i))" "$spawn_x" "$spawn_y"
done

if [[ "$HEADLESS" == "1" || "$HEADLESS" == "true" ]]; then
    echo "HEADLESS=$HEADLESS, gazebo client disabled. Press Ctrl-C to stop."
    wait
else
    echo "Starting Gazebo client..."
    gzclient
fi
