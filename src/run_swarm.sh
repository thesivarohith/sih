#!/usr/bin/env bash
# =========================================================================
# run_swarm.sh — SIH26123 Edge-AI Warehouse Swarm Launcher
# =========================================================================
# Configures rmw_zenoh_cpp for pure P2P communication (if available)
# and launches three AMRs into the Gazebo Harmonic warehouse simulation.
#
# Usage:
#   chmod +x run_swarm.sh
#   ./run_swarm.sh
# =========================================================================

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ---- Source ROS 2 Jazzy ----
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash

# ---- Configure Zenoh P2P middleware (if available) ----
if [ -f "/opt/ros/jazzy/lib/librmw_zenoh_cpp.so" ] || ldconfig -p 2>/dev/null | grep -q librmw_zenoh_cpp; then
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp
    export ZENOH_SESSION_CONFIG_URI="${SCRIPT_DIR}/zenoh_config.json5"
    export ZENOH_ROUTER_CHECK_ATTEMPTS=0
    RMW_MODE="Zenoh P2P (no router)"
else
    # Fallback: use default FastDDS with Simple Discovery
    # Namespace isolation and multi-spawn work identically on any RMW.
    RMW_MODE="FastDDS (default — install rmw_zenoh_cpp for P2P)"
fi

echo "=============================================="
echo "  SIH26123 — Edge-AI Warehouse Swarm"
echo "=============================================="
echo "  RMW:    ${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
echo "  Mode:   ${RMW_MODE}"
echo "  Fleet:  amr_1, amr_2, amr_3"
echo "=============================================="
echo ""

# ---- Launch the 3-AMR simulation ----
ros2 launch "${SCRIPT_DIR}/multi_spawn.launch.py"
