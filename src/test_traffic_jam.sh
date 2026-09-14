#!/usr/bin/env bash
# =========================================================================
# test_traffic_jam.sh — 3-AMR Space-Time Conflict Resolution & Priority Aging Test
# =========================================================================
# Injects 3 conflicting tasks into the global P2P pool (`swarm/tasks/pool`)
# with a 0.1s delay to force simultaneous claims and intersection convergence.
#
# Conflicting Routes:
#   Task 1: Rack_Yellow_1_Bay_5  -> Rack_Blue_4_Bay_5   (Top to Bottom)
#   Task 2: Rack_Blue_4_Bay_1    -> Rack_Yellow_1_Bay_1 (Bottom to Top)
#   Task 3: Rack_Yellow_2_Bay_3  -> Rack_Blue_2_Bay_8   (Left to Right Cross)
# =========================================================================

set -eo pipefail

source /opt/ros/jazzy/setup.bash
if [ -f "/home/siva/mustang/install/setup.bash" ]; then
    source /home/siva/mustang/install/setup.bash
fi

echo "=============================================="
echo "  SIH26123 — Injecting 3-Way Traffic Jam Tasks"
echo "=============================================="

# Task 1: Top to Bottom (Yellow 1 -> Blue 4)
ros2 topic pub --once swarm/tasks/pool std_msgs/msg/String "data: '{\"msg_type\": \"AVAILABLE\", \"task_id\": \"task_jam_1\", \"pickup\": \"Rack_Yellow_1_Bay_5\", \"dropoff\": \"Rack_Blue_4_Bay_5\"}'" &
sleep 0.1

# Task 2: Bottom to Top (Blue 4 -> Yellow 1)
ros2 topic pub --once swarm/tasks/pool std_msgs/msg/String "data: '{\"msg_type\": \"AVAILABLE\", \"task_id\": \"task_jam_2\", \"pickup\": \"Rack_Blue_4_Bay_1\", \"dropoff\": \"Rack_Yellow_1_Bay_1\"}'" &
sleep 0.1

# Task 3: Left to Right Cross (Yellow 2 -> Blue 2)
ros2 topic pub --once swarm/tasks/pool std_msgs/msg/String "data: '{\"msg_type\": \"AVAILABLE\", \"task_id\": \"task_jam_3\", \"pickup\": \"Rack_Yellow_2_Bay_3\", \"dropoff\": \"Rack_Blue_2_Bay_8\"}'" &

wait
echo ""
echo ">>> All 3 conflicting tasks successfully injected into pool!"
