#!/usr/bin/env bash
# =========================================================================
# run_swarm.sh — SIH26123 Edge-AI Warehouse Swarm Launcher
# =========================================================================
# Configures rmw_zenoh_cpp for pure P2P communication and launches
# three AMRs into the Gazebo Harmonic warehouse simulation.
#
# Usage:
#   chmod +x run_swarm.sh
#   ./run_swarm.sh
# =========================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ---- Source ROS 2 Jazzy ----
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash

# ---- Configure Zenoh P2P middleware ----
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_SESSION_CONFIG_URI="${SCRIPT_DIR}/zenoh_config.json5"

# ---- Disable the default Zenoh router (pure peer mode) ----
# rmw_zenoh_cpp normally auto-starts a router process. In P2P mode
# we explicitly skip it since peers discover each other via multicast.
export ZENOH_ROUTER_CHECK_ATTEMPTS=0

echo "=============================================="
echo "  SIH26123 — Edge-AI Warehouse Swarm"
echo "=============================================="
echo "  RMW:    ${RMW_IMPLEMENTATION}"
echo "  Zenoh:  ${ZENOH_SESSION_CONFIG_URI}"
echo "  Mode:   Pure Peer-to-Peer (no router)"
echo "  Fleet:  amr_1, amr_2, amr_3"
echo "=============================================="
echo ""

# ---- Launch the 3-AMR simulation ----
ros2 launch "${SCRIPT_DIR}/multi_spawn.launch.py"
