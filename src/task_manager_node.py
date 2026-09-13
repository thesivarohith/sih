#!/usr/bin/env python3
"""
task_manager_node.py — Decentralized Task Lifecycle Manager & KPI Logger
=============================================================================
ROS 2 Jazzy node for autonomous task claiming, state machine execution, and
time/KPI metrics logging in a decentralized 3-AMR warehouse swarm (SIH26123).

Architecture (rmw_zenoh P2P — No Central Master Server):
  1. Job Pool (`swarm/tasks/pool`): All AMRs listen to a global P2P channel for
     AVAILABLE tasks (task_id, pickup [x,y], dropoff [x,y]).
  2. Autonomous Claiming & Conflict Resolution:
     - When IDLE, an AMR calculates its Euclidean distance to the task pickup location
       and broadcasts a CLAIM message.
     - If multiple AMRs claim simultaneously, a deterministic SHA-256 tiebreaker +
       distance cost metric determines the unique winner on every peer—without master server negotiation.
  3. State Machine Flow:
     IDLE ──> CLAIMED ──> EN_ROUTE_PICKUP ──> DELIVERING ──> COMPLETED
       │                                         │
       └─────────── BLOCKED_AUCTION ─────────────┘ (Handoff to CNP / Return to Pool)
  4. KPI Metrics Logging:
     Tracks active travel time vs idle wait time (by monitoring /cmd_vel zero-velocity states)
     and logs order_to_delivery_latency_sec upon completion to `/swarm/<robot_id>/mission_metrics`.

Topics:
  PUB/SUB  swarm/tasks/pool                  (std_msgs/String, JSON task pool messages)
  SUB      /odom                             (nav_msgs/Odometry — current position)
  SUB      /cmd_vel                          (geometry_msgs/Twist — velocity monitoring for active vs wait time)
  SUB      /amr/obstruction_status           (std_msgs/String — SAFETY_STOP / BLOCKED_AUCTION_REQUIRED / CLEAR)
  PUB      /swarm/<robot_id>/mission_metrics (std_msgs/String — KPI metrics JSON)
  PUB      /task_manager_status              (std_msgs/String — state machine monitoring)
  PUB      /cmd_vel                          (geometry_msgs/Twist — motor control towards waypoints)
"""

import json
import math
import hashlib
import heapq
import time as _time
from typing import Dict, Any, Optional, List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class AStarPlanner:
    """
    Lightweight 2D Grid A* Pathfinding Planner for 15x15m SIH Warehouse Map.
    Grid resolution: 0.5 meters (30x30 cells).
    Includes obstacle bounding box definitions for warehouse racks & trap zone.
    """
    def __init__(self, resolution: float = 0.5, map_size: float = 15.0):
        self.res = resolution
        self.size = map_size
        self.grid_dim = int(map_size / resolution)

        # Hardcoded rack obstacle bounding boxes [xmin, xmax, ymin, ymax]
        # (Includes inflation margin for AMR footprint)
        self.obstacles = [
            # Row A Yellow Racks (y=10.0, x:[0.5..2.5, 3.5..5.5, 6.5..8.5, 9.5..11.5], y:[8.0..12.0])
            [0.1, 2.9, 7.6, 12.4],
            [3.1, 5.9, 7.6, 12.4],
            [6.1, 8.9, 7.6, 12.4],
            [9.1, 11.9, 7.6, 12.4],
            
            # Row B Blue Racks (y=5.0, x:[0.5..2.5, 3.5..5.5, 6.5..8.5, 9.5..11.5], y:[3.0..7.0])
            [0.1, 2.9, 2.6, 7.4],
            [3.1, 5.9, 2.6, 7.4],
            [6.1, 8.9, 2.6, 7.4],
            [9.1, 11.9, 2.6, 7.4],

            # Dead-End Trap Zone (pose 13.5 10.0, size 3x4 -> x:[12.0..15.0], y:[8.0..12.0])
            [11.6, 15.0, 7.6, 12.4]
        ]

    def _to_grid(self, x: float, y: float) -> Tuple[int, int]:
        gx = max(0, min(self.grid_dim - 1, int(x / self.res)))
        gy = max(0, min(self.grid_dim - 1, int(y / self.res)))
        return (gx, gy)

    def _to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        wx = (gx + 0.5) * self.res
        wy = (gy + 0.5) * self.res
        return (wx, wy)

    def is_obstacle(self, gx: int, gy: int) -> bool:
        wx, wy = self._to_world(gx, gy)
        if wx < 0.4 or wx > (self.size - 0.4) or wy < 0.4 or wy > (self.size - 0.4):
            return True
        for ob in self.obstacles:
            if ob[0] <= wx <= ob[1] and ob[2] <= wy <= ob[3]:
                return True
        return False

    def plan(self, start_pos: Tuple[float, float], goal_pos: Tuple[float, float]) -> List[Tuple[float, float]]:
        start_g = self._to_grid(start_pos[0], start_pos[1])
        goal_g = self._to_grid(goal_pos[0], goal_pos[1])

        if start_g == goal_g:
            return [goal_pos]

        open_set = []
        heapq.heappush(open_set, (0.0, 0.0, start_g))
        came_from = {}
        g_score = {start_g: 0.0}

        def heuristic(a, b):
            dx = abs(a[0] - b[0])
            dy = abs(a[1] - b[1])
            return math.sqrt(dx * dx + dy * dy)

        moves = [
            (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)
        ]

        closest_node = start_g
        closest_dist = heuristic(start_g, goal_g)

        while open_set:
            _, current_g, current = heapq.heappop(open_set)

            if current == goal_g:
                closest_node = goal_g
                break

            h_dist = heuristic(current, goal_g)
            if h_dist < closest_dist:
                closest_dist = h_dist
                closest_node = current

            for dx, dy, cost in moves:
                neighbor = (current[0] + dx, current[1] + dy)
                if not (0 <= neighbor[0] < self.grid_dim and 0 <= neighbor[1] < self.grid_dim):
                    continue
                if self.is_obstacle(neighbor[0], neighbor[1]) and neighbor != goal_g:
                    continue

                tentative_g = current_g + cost
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + heuristic(neighbor, goal_g)
                    came_from[neighbor] = current
                    heapq.heappush(open_set, (f_score, tentative_g, neighbor))

        path_grid = []
        curr = closest_node
        while curr in came_from:
            path_grid.append(curr)
            curr = came_from[curr]
        path_grid.append(start_g)
        path_grid.reverse()

        waypoints = [self._to_world(gx, gy) for gx, gy in path_grid]
        waypoints.append(goal_pos)
        return waypoints


class TaskManagerNode(Node):
    """
    Decentralized Task Lifecycle Manager and KPI Metrics Collector.
    Runs independently on each AMR in the swarm.
    """

    def __init__(self):
        super().__init__("task_manager_node")

        # ---------------------------------------------------------------
        # ROS 2 Parameters
        # ---------------------------------------------------------------
        self.declare_parameter("robot_id", "amr_1")
        self.declare_parameter("peer_ids", ["amr_2", "amr_3"])
        self.declare_parameter("claim_window_ms", 200)
        self.declare_parameter("waypoint_tolerance_m", 0.3)
        self.declare_parameter("linear_speed_m_s", 0.5)

        self._robot_id             = self.get_parameter("robot_id").value
        self._peer_ids             = self.get_parameter("peer_ids").value
        self._claim_window_sec     = float(self.get_parameter("claim_window_ms").value) / 1000.0
        self._waypoint_tolerance   = float(self.get_parameter("waypoint_tolerance_m").value)
        self._linear_speed         = float(self.get_parameter("linear_speed_m_s").value)

        # ---------------------------------------------------------------
        # State Machine & Navigation State
        # ---------------------------------------------------------------
        # States: IDLE, CLAIMED, EN_ROUTE_PICKUP, DELIVERING, COMPLETED, HANDOFF
        self._state: str = "IDLE"
        self._current_task: Optional[Dict[str, Any]] = None
        self._planner = AStarPlanner(resolution=0.5, map_size=15.0)
        self._waypoints: List[Tuple[float, float]] = []

        # Position tracking from Odom
        self._pos_x: float = 0.0
        self._pos_y: float = 0.0
        self._current_yaw: float = 0.0

        # Motion & KPI Metrics Tracking
        self._is_moving: bool = False
        self._claim_timestamp: float = 0.0
        self._active_travel_time_sec: float = 0.0
        self._idle_wait_time_sec: float = 0.0
        self._last_state_tick_time: float = _time.monotonic()

        # Claim resolution state
        self._pending_claim_task_id: Optional[str] = None
        self._claims_received: Dict[str, Dict[str, Any]] = {}
        self._claim_timer = None

        # ---------------------------------------------------------------
        # QoS Profiles
        # ---------------------------------------------------------------
        pool_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ---------------------------------------------------------------
        # Publishers & Subscriptions
        # ---------------------------------------------------------------
        # Global P2P Task Pool topic
        self._pub_pool = self.create_publisher(String, "swarm/tasks/pool", pool_qos)
        self._sub_pool = self.create_subscription(String, "swarm/tasks/pool", self._pool_message_cb, pool_qos)

        # Local metrics & status publishers
        self._pub_metrics = self.create_publisher(
            String, f"/swarm/{self._robot_id}/mission_metrics", pool_qos
        )
        self._pub_status = self.create_publisher(String, "/task_manager_status", pool_qos)

        # Motor velocity publisher
        self._pub_cmd_vel = self.create_publisher(Twist, f"/{self._robot_id}/cmd_vel", pool_qos)

        # Sensor & Status Subscriptions
        self._sub_odom = self.create_subscription(Odometry, f"/{self._robot_id}/odom", self._odom_cb, odom_qos)
        self._sub_cmd_vel = self.create_subscription(Twist, f"/{self._robot_id}/cmd_vel", self._cmd_vel_monitor_cb, odom_qos)
        self._sub_obstruction = self.create_subscription(String, f"/{self._robot_id}/obstruction_status", self._obstruction_cb, pool_qos)

        # ---------------------------------------------------------------
        # Main Execution Loop Timer (10 Hz for navigation & state machine)
        # ---------------------------------------------------------------
        self._loop_timer = self.create_timer(0.1, self._control_loop)
        self._status_broadcast_timer = self.create_timer(1.0, self._publish_status)

        self.get_logger().info(
            f"TaskManagerNode initialized for '{self._robot_id}' | "
            f"peers={self._peer_ids} | claim_window={self._claim_window_sec*1000:.0f}ms"
        )

    # ===================================================================
    # Callbacks & Data Monitoring
    # ===================================================================

    def _odom_cb(self, msg: Odometry):
        """Extract x, y position and yaw orientation from odometry."""
        self._pos_x = msg.pose.pose.position.x
        self._pos_y = msg.pose.pose.position.y

        # Extract yaw angle from orientation quaternion (z-axis rotation)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def _cmd_vel_monitor_cb(self, msg: Twist):
        """Monitor velocity commands to differentiate active travel vs idle wait time."""
        linear_vel = abs(msg.linear.x) + abs(msg.linear.y)
        angular_vel = abs(msg.angular.z)
        self._is_moving = (linear_vel > 0.01 or angular_vel > 0.01)

    def _obstruction_cb(self, msg: String):
        """
        React to path obstructions mid-mission.
        If BLOCKED_AUCTION_REQUIRED occurs while executing a task,
        trigger task handoff and return task to pool or hand over to CNP node.
        """
        status = msg.data.strip().upper()

        if status == "BLOCKED_AUCTION_REQUIRED" and self._state in ("EN_ROUTE_PICKUP", "DELIVERING"):
            self.get_logger().warn(
                f"[{self._robot_id}] Permanent obstruction detected during mission state '{self._state}'! Triggering task handoff..."
            )
            self._handle_mid_mission_obstruction()

    # ===================================================================
    # Task Pool P2P Communications
    # ===================================================================

    def _pool_message_cb(self, msg: String):
        """Parse and route messages on the global `swarm/tasks/pool` channel."""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Malformed JSON on swarm/tasks/pool: {e}")
            return

        msg_type = data.get("msg_type")

        if msg_type == "AVAILABLE":
            self._handle_available_task(data)
        elif msg_type == "CLAIM":
            self._handle_claim_message(data)
        elif msg_type == "RELEASED":
            self._handle_released_task(data)

    def _handle_available_task(self, data: Dict[str, Any]):
        """Evaluate new AVAILABLE task when IDLE."""
        if self._state != "IDLE" or self._pending_claim_task_id is not None:
            return

        task_id = data.get("task_id")
        pickup = data.get("pickup", [0.0, 0.0])

        dx = pickup[0] - self._pos_x
        dy = pickup[1] - self._pos_y
        dist_m = math.sqrt(dx * dx + dy * dy)

        # Broadcast CLAIM for this task
        self._pending_claim_task_id = task_id
        self._claims_received = {}

        claim_payload = {
            "msg_type": "CLAIM",
            "task_id": task_id,
            "robot_id": self._robot_id,
            "distance_m": round(dist_m, 3),
            "task_data": data,
            "timestamp": _time.time()
        }

        msg = String()
        msg.data = json.dumps(claim_payload)
        self._pub_pool.publish(msg)

        self.get_logger().info(
            f"[{self._robot_id}] Submitted CLAIM for task '{task_id}' (dist: {dist_m:.2f}m). "
            f"Awaiting {self._claim_window_sec*1000:.0f}ms claim window."
        )

        # Start 200ms claim resolution window timer
        if self._claim_timer is not None:
            self._claim_timer.cancel()
        self._claim_timer = self.create_timer(self._claim_window_sec, self._resolve_claims)

    def _handle_claim_message(self, data: Dict[str, Any]):
        """Collect claims from peer AMRs for simultaneous claim resolution."""
        task_id = data.get("task_id")
        robot_id = data.get("robot_id")

        if self._pending_claim_task_id == task_id:
            self._claims_received[robot_id] = data

    def _resolve_claims(self):
        """
        Executed at end of 200ms claim window.
        Uses SHA-256 asymmetric tiebreaker + distance metric to deterministically select claim winner.
        """
        if self._claim_timer is not None:
            self._claim_timer.cancel()
            self._claim_timer = None

        if not self._pending_claim_task_id:
            return

        task_id = self._pending_claim_task_id
        self.get_logger().info(
            f"[{self._robot_id}] Resolving claims for '{task_id}'. Total claims: {len(self._claims_received)}"
        )

        # Evaluate all claims
        candidates = []
        for rid, claim in self._claims_received.items():
            dist = claim.get("distance_m", 999.0)
            # Generate deterministic SHA-256 tiebreaker for exact distance ties
            tiebreaker = self._generate_tiebreaker(rid, task_id)
            score = dist + tiebreaker
            candidates.append((rid, score, claim))

        if candidates:
            # Lowest score wins (closest distance + deterministic tiebreaker)
            candidates.sort(key=lambda x: x[1])
            winner_id, winner_score, winning_claim = candidates[0]

            if winner_id == self._robot_id:
                self.get_logger().info(
                    f"[{self._robot_id}] *** WON CLAIM for task '{task_id}' *** (score: {winner_score:.4f})"
                )
                self._current_task = winning_claim.get("task_data")
                self._state = "CLAIMED"
                self._claim_timestamp = _time.time()
                self._active_travel_time_sec = 0.0
                self._idle_wait_time_sec = 0.0
            else:
                self.get_logger().info(
                    f"[{self._robot_id}] Task '{task_id}' awarded to peer '{winner_id}'."
                )

        self._pending_claim_task_id = None
        self._claims_received = {}

    def _generate_tiebreaker(self, robot_id: str, task_id: str) -> float:
        """Generates deterministic tiebreaker in range [0.001, 0.050] using SHA-256 hash."""
        seed = hashlib.sha256(f"{robot_id}:{task_id}".encode()).digest()
        raw = int.from_bytes(seed[:4], "big") / (2**32)
        return 0.001 + raw * 0.049

    def _handle_released_task(self, data: Dict[str, Any]):
        """Listen for released/re-queued tasks from peers."""
        self.get_logger().info(f"[{self._robot_id}] Task '{data.get('task_id')}' released back to pool.")

    # ===================================================================
    # Main Control Loop & State Machine Execution
    # ===================================================================

    def _control_loop(self):
        """10 Hz control loop executing state machine transitions and metric tracking."""
        now = _time.monotonic()
        dt = now - self._last_state_tick_time
        self._last_state_tick_time = now

        # Update active travel vs idle wait time counters
        if self._state in ("EN_ROUTE_PICKUP", "DELIVERING"):
            if self._is_moving:
                self._active_travel_time_sec += dt
            else:
                self._idle_wait_time_sec += dt

        # State Machine Transitions
        if self._state == "IDLE":
            pass

        elif self._state == "CLAIMED":
            pickup = self._current_task.get("pickup", [self._pos_x, self._pos_y])
            self.get_logger().info(
                f"[{self._robot_id}] Planning A* path to pickup {pickup} for '{self._current_task.get('task_id')}'"
            )
            self._waypoints = self._planner.plan((self._pos_x, self._pos_y), (pickup[0], pickup[1]))
            self._state = "EN_ROUTE_PICKUP"

        elif self._state == "EN_ROUTE_PICKUP":
            if not self._waypoints:
                self.get_logger().info(
                    f"[{self._robot_id}] Pickup reached for '{self._current_task.get('task_id')}'. Planning A* path to dropoff..."
                )
                dropoff = self._current_task.get("dropoff", [self._pos_x, self._pos_y])
                self._waypoints = self._planner.plan((self._pos_x, self._pos_y), (dropoff[0], dropoff[1]))
                self._state = "DELIVERING"
            else:
                target_wp = self._waypoints[0]
                arrived_wp = self._navigate_towards(target_wp[0], target_wp[1])
                if arrived_wp:
                    self._waypoints.pop(0)

        elif self._state == "DELIVERING":
            if not self._waypoints:
                self.get_logger().info(
                    f"[{self._robot_id}] Dropoff reached for '{self._current_task.get('task_id')}'. Task COMPLETED!"
                )
                self._complete_task()
            else:
                target_wp = self._waypoints[0]
                arrived_wp = self._navigate_towards(target_wp[0], target_wp[1])
                if arrived_wp:
                    self._waypoints.pop(0)

        elif self._state == "HANDOFF":
            # Stopping motors during handoff
            self._stop_motors()

    def _navigate_towards(self, target_x: float, target_y: float) -> bool:
        """Simple proportional velocity control towards target waypoint."""
        dx = target_x - self._pos_x
        dy = target_y - self._pos_y
        dist = math.sqrt(dx * dx + dy * dy)

        if dist <= self._waypoint_tolerance:
            self._stop_motors()
            return True

        # Calculate steering angle towards waypoint
        target_yaw = math.atan2(dy, dx)
        yaw_err = target_yaw - self._current_yaw
        # Normalize angle error to [-pi, pi]
        yaw_err = math.atan2(math.sin(yaw_err), math.cos(yaw_err))

        cmd = Twist()
        if abs(yaw_err) > 0.3:
            cmd.angular.z = 0.8 if yaw_err > 0 else -0.8
            cmd.linear.x = 0.0
        else:
            cmd.linear.x = min(self._linear_speed, dist * 0.5)
            cmd.angular.z = 1.0 * yaw_err

        self._pub_cmd_vel.publish(cmd)
        return False

    def _stop_motors(self):
        """Publish zero velocity command to halt motors."""
        cmd = Twist()
        self._pub_cmd_vel.publish(cmd)

    # ===================================================================
    # Task Completion & Mid-Mission Obstruction Handoff
    # ===================================================================

    def _complete_task(self):
        """Logs KPI metrics and broadcasts task completion."""
        if not self._current_task:
            self._state = "IDLE"
            return

        total_latency_sec = _time.time() - self._claim_timestamp

        # Publish KPI Metrics
        metrics_payload = {
            "task_id": self._current_task.get("task_id"),
            "robot_id": self._robot_id,
            "order_to_delivery_latency_sec": round(total_latency_sec, 2),
            "active_travel_time_sec": round(self._active_travel_time_sec, 2),
            "idle_wait_time_sec": round(self._idle_wait_time_sec, 2),
            "pickup_location": self._current_task.get("pickup"),
            "dropoff_location": self._current_task.get("dropoff"),
            "timestamp": _time.time()
        }

        metrics_msg = String()
        metrics_msg.data = json.dumps(metrics_payload)
        self._pub_metrics.publish(metrics_msg)

        # Broadcast Task Completion to Pool
        comp_payload = {
            "msg_type": "COMPLETED",
            "task_id": self._current_task.get("task_id"),
            "robot_id": self._robot_id,
            "latency_sec": round(total_latency_sec, 2)
        }
        pool_msg = String()
        pool_msg.data = json.dumps(comp_payload)
        self._pub_pool.publish(pool_msg)

        self.get_logger().info(
            f"[{self._robot_id}] Mission Completed! Latency: {total_latency_sec:.2f}s | "
            f"Travel Time: {self._active_travel_time_sec:.2f}s | Wait Time: {self._idle_wait_time_sec:.2f}s"
        )

        # Reset task state
        self._current_task = None
        self._state = "IDLE"

    def _handle_mid_mission_obstruction(self):
        """Handoff active task back to pool when obstructed mid-mission."""
        if not self._current_task:
            self._state = "IDLE"
            return

        self._stop_motors()
        task_id = self._current_task.get("task_id")

        # Broadcast RELEASED task message to pool so peer or CNP auction node can claim it
        release_payload = {
            "msg_type": "RELEASED",
            "task_id": task_id,
            "robot_id": self._robot_id,
            "reason": "OBSTRUCTION_BLOCKED",
            "task_data": self._current_task,
            "timestamp": _time.time()
        }

        msg = String()
        msg.data = json.dumps(release_payload)
        self._pub_pool.publish(msg)

        self.get_logger().warn(
            f"[{self._robot_id}] Task '{task_id}' released back to pool due to obstruction."
        )

        self._current_task = None
        self._state = "IDLE"

    # ===================================================================
    # Status Publisher
    # ===================================================================

    def _publish_status(self):
        """Broadcast state machine status for visual monitoring dashboards."""
        status_payload = {
            "robot_id": self._robot_id,
            "state": self._state,
            "current_task": self._current_task.get("task_id") if self._current_task else None,
            "pos": [round(self._pos_x, 2), round(self._pos_y, 2)],
            "timestamp": _time.time()
        }
        msg = String()
        msg.data = json.dumps(status_payload)
        self._pub_status.publish(msg)


# =======================================================================
# Entry Point
# =======================================================================
def main(args=None):
    rclpy.init(args=args)
    node = TaskManagerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down TaskManagerNode (Ctrl+C)")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
