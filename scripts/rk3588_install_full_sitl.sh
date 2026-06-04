#!/usr/bin/env bash

set -Eeuo pipefail

MODE="${1:-full-sitl}"
UAV_SWARM_ROOT="${UAV_SWARM_ROOT:-$HOME/uav_swarm_system}"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
MICRO_XRCE_DDS_AGENT_DIR="${MICRO_XRCE_DDS_AGENT_DIR:-$HOME/Micro-XRCE-DDS-Agent}"
ROS_DISTRO_TARGET="humble"
PX4_TAG="v1.14.4"
PX4_MSGS_BRANCH="release/1.14"
MICRO_XRCE_TAG="v2.4.1"

log() {
    printf "\n[RK3588 install] %s\n" "$*"
}

die() {
    printf "\n[RK3588 install][ERROR] %s\n" "$*" >&2
    exit 1
}

source_with_nounset_disabled() {
    set +u
    # shellcheck disable=SC1090
    source "$1"
    set -u
}

usage() {
    cat <<'EOF'
Usage:
  ./scripts/rk3588_install_full_sitl.sh [full-sitl|flight-only]

Modes:
  full-sitl   Install ROS2 Humble, MicroXRCEAgent 2.4.1, PX4 v1.14.4,
              Gazebo Classic 11, px4_msgs release/1.14, and build ros2_ws.
  flight-only Install only the RK3588 companion-computer flight environment:
              ROS2 Humble, MicroXRCEAgent 2.4.1, px4_msgs release/1.14,
              and build ros2_ws. No PX4 SITL or Gazebo.

Environment overrides:
  UAV_SWARM_ROOT=/home/jie/uav_swarm_system
  PX4_DIR=/home/jie/PX4-Autopilot
  MICRO_XRCE_DDS_AGENT_DIR=/home/jie/Micro-XRCE-DDS-Agent
  RK3588_INSTALL_FORCE=1   Allow running on non-arm64 or non-Ubuntu-22.04 hosts.
EOF
}

if [[ "${MODE}" == "-h" || "${MODE}" == "--help" ]]; then
    usage
    exit 0
fi

if [[ "${MODE}" != "full-sitl" && "${MODE}" != "flight-only" ]]; then
    usage
    die "Unknown mode: ${MODE}"
fi

if [[ "${EUID}" -eq 0 ]]; then
    die "Run this script as the normal user, not root. It will use sudo when needed."
fi

ARCH="$(dpkg --print-architecture)"
if [[ "${ARCH}" != "arm64" && "${RK3588_INSTALL_FORCE:-0}" != "1" ]]; then
    die "This installer is intended for RK3588 Ubuntu arm64. Current arch: ${ARCH}. Set RK3588_INSTALL_FORCE=1 to override."
fi

if [[ -r /etc/os-release ]]; then
    # shellcheck disable=SC1091
    source /etc/os-release
    if [[ "${VERSION_ID:-}" != "22.04" && "${RK3588_INSTALL_FORCE:-0}" != "1" ]]; then
        die "This project targets Ubuntu 22.04. Current VERSION_ID=${VERSION_ID:-unknown}. Set RK3588_INSTALL_FORCE=1 to override."
    fi
fi
UBUNTU_CODENAME_VALUE="$(lsb_release -cs)"

sudo -v
export DEBIAN_FRONTEND=noninteractive

install_base_packages() {
    log "Installing base system packages"
    sudo apt update
    sudo apt install -y \
        build-essential \
        ccache \
        cmake \
        curl \
        git \
        gnupg \
        htop \
        iproute2 \
        iputils-ping \
        lsb-release \
        net-tools \
        ninja-build \
        pkg-config \
        python3-dev \
        python3-pip \
        python3-venv \
        python3-yaml \
        screen \
        software-properties-common \
        tmux \
        udev \
        wget \
        chrony \
        locales

    sudo locale-gen en_US en_US.UTF-8 || true
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 || true
}

install_ros2_humble() {
    log "Installing ROS2 Humble"
    sudo add-apt-repository universe -y
    sudo install -m 0755 -d /usr/share/keyrings
    if [[ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]]; then
        curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
            sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
    fi

    echo "deb [arch=${ARCH} signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${UBUNTU_CODENAME_VALUE} main" | \
        sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null

    sudo apt update
    sudo apt install -y \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
        ros-${ROS_DISTRO_TARGET}-diagnostic-msgs \
        ros-${ROS_DISTRO_TARGET}-geometry-msgs \
        ros-${ROS_DISTRO_TARGET}-launch \
        ros-${ROS_DISTRO_TARGET}-launch-ros \
        ros-${ROS_DISTRO_TARGET}-nav-msgs \
        ros-${ROS_DISTRO_TARGET}-rmw-fastrtps-cpp \
        ros-${ROS_DISTRO_TARGET}-ros-base \
        ros-${ROS_DISTRO_TARGET}-rosidl-default-generators \
        ros-${ROS_DISTRO_TARGET}-rosidl-default-runtime \
        ros-${ROS_DISTRO_TARGET}-sensor-msgs \
        ros-${ROS_DISTRO_TARGET}-std-msgs \
        ros-${ROS_DISTRO_TARGET}-std-srvs \
        ros-${ROS_DISTRO_TARGET}-tf2-ros

    sudo rosdep init 2>/dev/null || true
    rosdep update
}

install_gazebo_classic() {
    log "Installing Gazebo Classic 11 packages"
    sudo apt update
    sudo apt install -y \
        gazebo \
        libgazebo11-dev \
        protobuf-compiler \
        libprotobuf-dev \
        libeigen3-dev \
        libopencv-dev
}

install_px4_build_deps() {
    log "Installing PX4 SITL build dependencies"
    sudo apt install -y \
        astyle \
        bc \
        clang \
        clang-tidy \
        genromfs \
        gstreamer1.0-plugins-bad \
        gstreamer1.0-plugins-base \
        gstreamer1.0-plugins-good \
        lcov \
        libgstreamer-plugins-base1.0-dev \
        libgstreamer1.0-dev \
        libxml2-utils \
        python3-empy \
        python3-jinja2 \
        python3-jsonschema \
        python3-kconfiglib \
        python3-numpy \
        python3-packaging \
        python3-serial \
        python3-setuptools \
        python3-toml \
        shellcheck \
        zip
}

clone_or_update_repo() {
    local url="$1"
    local dir="$2"
    local ref="$3"

    if [[ -d "${dir}/.git" ]]; then
        log "Updating ${dir}"
        git -C "${dir}" fetch --tags --prune
        git -C "${dir}" checkout "${ref}"
        git -C "${dir}" submodule sync --recursive
        git -C "${dir}" submodule update --init --recursive
    else
        log "Cloning ${url} into ${dir}"
        git clone --recursive --branch "${ref}" "${url}" "${dir}"
    fi
}

install_micro_xrce_agent() {
    log "Installing Micro-XRCE-DDS-Agent ${MICRO_XRCE_TAG}"
    clone_or_update_repo "https://github.com/eProsima/Micro-XRCE-DDS-Agent.git" "${MICRO_XRCE_DDS_AGENT_DIR}" "${MICRO_XRCE_TAG}"
    cmake -S "${MICRO_XRCE_DDS_AGENT_DIR}" -B "${MICRO_XRCE_DDS_AGENT_DIR}/build" -DCMAKE_BUILD_TYPE=Release
    cmake --build "${MICRO_XRCE_DDS_AGENT_DIR}/build" -j"$(nproc)"
    sudo cmake --install "${MICRO_XRCE_DDS_AGENT_DIR}/build"
    sudo ldconfig
}

install_px4_sitl() {
    log "Installing PX4-Autopilot ${PX4_TAG}"
    clone_or_update_repo "https://github.com/PX4/PX4-Autopilot.git" "${PX4_DIR}" "${PX4_TAG}"

    if [[ -x "${PX4_DIR}/Tools/setup/ubuntu.sh" ]]; then
        log "Running PX4 Ubuntu dependency installer with --no-nuttx"
        bash "${PX4_DIR}/Tools/setup/ubuntu.sh" --no-nuttx
    fi

    if [[ -f "${PX4_DIR}/Tools/setup/requirements.txt" ]]; then
        log "Installing PX4 Python requirements"
        python3 -m pip install --user -r "${PX4_DIR}/Tools/setup/requirements.txt"
    fi

    log "Building PX4 SITL Gazebo Classic target"
    (cd "${PX4_DIR}" && DONT_RUN=1 HEADLESS=1 make px4_sitl gazebo-classic)
}

prepare_project_workspace() {
    log "Preparing project ROS2 workspace"
    mkdir -p "${UAV_SWARM_ROOT}/ros2_ws/src"

    local px4_msgs_dir="${UAV_SWARM_ROOT}/ros2_ws/src/px4_msgs"
    clone_or_update_repo "https://github.com/PX4/px4_msgs.git" "${px4_msgs_dir}" "${PX4_MSGS_BRANCH}"

    log "Installing ROS dependencies for project workspace"
    source_with_nounset_disabled "/opt/ros/${ROS_DISTRO_TARGET}/setup.bash"
    rosdep install --from-paths "${UAV_SWARM_ROOT}/ros2_ws/src" --ignore-src -r -y

    log "Building project ros2_ws"
    (cd "${UAV_SWARM_ROOT}/ros2_ws" && colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3)
}

configure_shell_environment() {
    log "Configuring ~/.bashrc environment block"
    local bashrc="$HOME/.bashrc"
    local marker="uav_swarm_system rk3588 environment"

    if ! grep -Fq "${marker}" "${bashrc}" 2>/dev/null; then
        cat >>"${bashrc}" <<EOF

# >>> ${marker} >>>
export UAV_SWARM_ROOT="${UAV_SWARM_ROOT}"
export PX4_DIR="${PX4_DIR}"
export MICRO_XRCE_DDS_AGENT_DIR="${MICRO_XRCE_DDS_AGENT_DIR}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-42}
source /opt/ros/${ROS_DISTRO_TARGET}/setup.bash
if [ -f "\$UAV_SWARM_ROOT/ros2_ws/install/setup.bash" ]; then
    source "\$UAV_SWARM_ROOT/ros2_ws/install/setup.bash"
fi
# <<< ${marker} <<<
EOF
    fi
}

print_summary() {
    log "Install finished"
    cat <<EOF

Mode:
  ${MODE}

Installed paths:
  UAV_SWARM_ROOT=${UAV_SWARM_ROOT}
  PX4_DIR=${PX4_DIR}
  MICRO_XRCE_DDS_AGENT_DIR=${MICRO_XRCE_DDS_AGENT_DIR}

Next checks:
  cd ${UAV_SWARM_ROOT}
  source scripts/setup_env.sh
  ros2 --version
  MicroXRCEAgent --version
  gazebo --version

Single SITL smoke test:
  Terminal 1:
    cd ${UAV_SWARM_ROOT}
    source scripts/setup_env.sh
    ./scripts/start_micro_xrce_agent.sh

  Terminal 2:
    cd ${UAV_SWARM_ROOT}
    source scripts/setup_env.sh
    HEADLESS=1 ./scripts/start_px4_single_sitl.sh

Flight mode reminder:
  Real flight should not run Gazebo or PX4 SITL on RK3588.
EOF
}

install_base_packages
install_ros2_humble
install_micro_xrce_agent

if [[ "${MODE}" == "full-sitl" ]]; then
    install_gazebo_classic
    install_px4_build_deps
    install_px4_sitl
fi

prepare_project_workspace
configure_shell_environment
print_summary
