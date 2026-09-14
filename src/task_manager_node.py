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
from typing import Dict, Any, Optional, List, Tuple, Union

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class WarehouseGraph:
    """
    Topological Graph (Virtual Rails) for 15x15m SIH Warehouse Map.
    Auto-generates:
      1. Center-aisle intersection nodes at main corridor intersections.
      2. 10 semantic parking bays around each physical rack (Row A & Row B).
      3. Connected edges strictly along center-lines to keep AMRs centered in aisles
         (at least 0.6m from rack bounds) to prevent corner clipping.
    """
    def __init__(self):
        self.nodes: Dict[str, Tuple[float, float]] = {}
        self.neighbors: Dict[str, List[Tuple[str, float]]] = {}
        self.aliases: Dict[str, str] = {}
        self._build_graph()

    def _build_graph(self):
        # Centerline coordinates for virtual rails
        # Vertical Aisles (X)
        x_aisles = [0.25, 3.0, 6.0, 9.0, 12.0, 13.5]
        # Horizontal Corridors (Y)
        y_corridors = [1.5, 7.5, 13.5]

        # 1. Intersection Nodes
        for x in x_aisles:
            for y in y_corridors:
                node_id = f"Int_X{x}_Y{y}"
                self.nodes[node_id] = (x, y)

        # 2. Racks & 10 Parking Bays per Rack
        # Physical Racks (size 2m x 4m):
        # Row A Yellow Racks (y=10.0, height y:[8.0..12.0])
        # Row B Blue Racks (y=5.0, height y:[3.0..7.0])
        racks = [
            # (Rack_Name, Rack_Alias, center_x, center_y, west_aisle_x, east_aisle_x)
            ("Rack_Yellow_1", "Rack_1A", 1.5, 10.0, 0.25, 3.0),
            ("Rack_Yellow_2", "Rack_2A", 4.5, 10.0, 3.0, 6.0),
            ("Rack_Yellow_3", "Rack_3A", 7.5, 10.0, 6.0, 9.0),
            ("Rack_Yellow_4", "Rack_4A", 10.5, 10.0, 9.0, 12.0),

            ("Rack_Blue_1", "Rack_1B", 1.5, 5.0, 0.25, 3.0),
            ("Rack_Blue_2", "Rack_2B", 4.5, 5.0, 3.0, 6.0),
            ("Rack_Blue_3", "Rack_3B", 7.5, 5.0, 6.0, 9.0),
            ("Rack_Blue_4", "Rack_4B", 10.5, 5.0, 9.0, 12.0),
        ]

        # Y offsets for 5 bays along West face and 5 bays along East face of each rack
        y_offsets = [-1.6, -0.8, 0.0, 0.8, 1.6]

        for rack_name, rack_alias, cx, cy, west_x, east_x in racks:
            # 5 West Bays (Bays 1 to 5) on vertical aisle west_x
            for i, offset in enumerate(y_offsets, start=1):
                bay_name = f"{rack_name}_Bay_{i}"
                alias_name = f"{rack_alias}_Bay_{i}"
                bx, by = west_x, round(cy + offset, 2)
                self.nodes[bay_name] = (bx, by)
                self.aliases[bay_name.lower()] = bay_name
                self.aliases[alias_name.lower()] = bay_name
                short_color = "yellow" if "Yellow" in rack_name else "blue"
                idx = rack_name.split("_")[-1]
                self.aliases[f"{short_color}_{idx}_bay_{i}"] = bay_name
                self.aliases[f"rack_{short_color}_{idx}_bay_{i}"] = bay_name

            # 5 East Bays (Bays 6 to 10) on vertical aisle east_x
            for i, offset in enumerate(y_offsets, start=6):
                bay_name = f"{rack_name}_Bay_{i}"
                alias_name = f"{rack_alias}_Bay_{i}"
                bx, by = east_x, round(cy + offset, 2)
                self.nodes[bay_name] = (bx, by)
                self.aliases[bay_name.lower()] = bay_name
                self.aliases[alias_name.lower()] = bay_name
                short_color = "yellow" if "Yellow" in rack_name else "blue"
                idx = rack_name.split("_")[-1]
                self.aliases[f"{short_color}_{idx}_bay_{i}"] = bay_name
                self.aliases[f"rack_{short_color}_{idx}_bay_{i}"] = bay_name

        # Ensure all node names map to themselves in lower case
        for name in list(self.nodes.keys()):
            self.aliases[name.lower()] = name

        # 3. Connect Graph Edges (Virtual Rails)
        for n in self.nodes:
            self.neighbors[n] = []

        # Connect vertical aisles (for each x_aisle)
        for x in x_aisles:
            nodes_on_x = [n for n, (nx, ny) in self.nodes.items() if abs(nx - x) < 0.05]
            nodes_on_x.sort(key=lambda n: self.nodes[n][1])
            for i in range(len(nodes_on_x) - 1):
                n1, n2 = nodes_on_x[i], nodes_on_x[i+1]
                dist = math.hypot(self.nodes[n1][0] - self.nodes[n2][0], self.nodes[n1][1] - self.nodes[n2][1])
                self.neighbors[n1].append((n2, dist))
                self.neighbors[n2].append((n1, dist))

        # Connect horizontal corridors (for each y_corridor)
        for y in y_corridors:
            intersections_on_y = [n for n, (nx, ny) in self.nodes.items() if n.startswith("Int_") and abs(ny - y) < 0.05]
            intersections_on_y.sort(key=lambda n: self.nodes[n][0])
            for i in range(len(intersections_on_y) - 1):
                n1, n2 = intersections_on_y[i], intersections_on_y[i+1]
                dist = math.hypot(self.nodes[n1][0] - self.nodes[n2][0], self.nodes[n1][1] - self.nodes[n2][1])
                self.neighbors[n1].append((n2, dist))
                self.neighbors[n2].append((n1, dist))

    def resolve_node(self, target: Any) -> str:
        """Resolve a semantic string name or (x, y) coordinate to a graph node_id."""
        if isinstance(target, str):
            clean = target.strip().lower()
            if clean in self.aliases:
                return self.aliases[clean]
            for alias, nid in self.aliases.items():
                if clean in alias or alias in clean:
                    return nid
            return self.find_nearest_node(13.0, 2.0)
        elif isinstance(target, (list, tuple)) and len(target) >= 2:
            return self.find_nearest_node(float(target[0]), float(target[1]))
        return self.find_nearest_node(13.0, 2.0)

    def get_coords(self, target: Any) -> Tuple[float, float]:
        """Get (x, y) coordinates for target node or coordinate."""
        if isinstance(target, (list, tuple)) and len(target) >= 2:
            return (float(target[0]), float(target[1]))
        nid = self.resolve_node(target)
        return self.nodes.get(nid, (7.5, 7.5))

    def find_nearest_node(self, x: float, y: float) -> str:
        """Find graph node with minimum Euclidean distance to (x, y)."""
        best_node = None
        best_dist = float('inf')
        for nid, (nx, ny) in self.nodes.items():
            dist = (nx - x)**2 + (ny - y)**2
            if dist < best_dist:
                best_dist = dist
                best_node = nid
        return best_node or "Int_X13.5_Y1.5"


class AStarPlanner:
    """
    Topological Graph A* Pathfinding Planner over WarehouseGraph (Virtual Rails).
    Searches exclusively across connected WarehouseGraph nodes, not a 30x30 grid.
    Guarantees AMRs stay centered in aisles to prevent corner clipping.
    """
    def __init__(self):
        self.graph = WarehouseGraph()

    def plan(self, start: Any, goal: Any) -> List[Tuple[float, float]]:
        start_node = self.graph.resolve_node(start)
        goal_node = self.graph.resolve_node(goal)

        goal_world_pos = None
        if isinstance(goal, (list, tuple)) and len(goal) >= 2:
            goal_world_pos = (float(goal[0]), float(goal[1]))

        if start_node == goal_node:
            waypoints = [self.graph.nodes[start_node]]
            if goal_world_pos:
                waypoints.append(goal_world_pos)
            return waypoints

        # A* Search over graph nodes
        open_set = []
        heapq.heappush(open_set, (0.0, 0.0, start_node))
        came_from: Dict[str, str] = {}
        g_score: Dict[str, float] = {start_node: 0.0}

        def heuristic(n1: str, n2: str) -> float:
            p1 = self.graph.nodes[n1]
            p2 = self.graph.nodes[n2]
            return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

        while open_set:
            _, current_g, current = heapq.heappop(open_set)

            if current == goal_node:
                break

            for neighbor, cost in self.graph.neighbors.get(current, []):
                tentative_g = current_g + cost
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + heuristic(neighbor, goal_node)
                    came_from[neighbor] = current
                    heapq.heappush(open_set, (f_score, tentative_g, neighbor))

        # Reconstruct path
        path_nodes = []
        curr = goal_node if goal_node in came_from or goal_node == start_node else None
        if not curr:
            curr = min(g_score.keys(), key=lambda n: heuristic(n, goal_node))

        while curr and curr in came_from:
            path_nodes.append(curr)
            curr = came_from[curr]
        if curr:
            path_nodes.append(curr)
        path_nodes.reverse()

        waypoints = [self.graph.nodes[nid] for nid in path_nodes]

        if goal_world_pos and (not waypoints or math.hypot(waypoints[-1][0] - goal_world_pos[0], waypoints[-1][1] - goal_world_pos[1]) > 0.1):
            waypoints.append(goal_world_pos)

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
        self._planner = AStarPlanner()
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

        # Step 4: Real-time LiDAR Safety & Step 5: Peer Space-Time Telemetry
        self._lidar_obstacle_detected: bool = False
        self._battery_level: float = 100.0
        self._peer_telemetry: Dict[str, Dict[str, Any]] = {}
        self._yield_until_time: float = 0.0

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
        self._sub_scan = self.create_subscription(LaserScan, f"/{self._robot_id}/scan", self._scan_cb, odom_qos)
        self._sub_cmd_vel = self.create_subscription(Twist, f"/{self._robot_id}/cmd_vel", self._cmd_vel_monitor_cb, odom_qos)
        self._sub_obstruction = self.create_subscription(String, f"/{self._robot_id}/obstruction_status", self._obstruction_cb, pool_qos)

        # Global P2P Telemetry Channel
        self._pub_telemetry = self.create_publisher(String, f"swarm/{self._robot_id}/telemetry", pool_qos)
        for pid in self._peer_ids:
            self.create_subscription(
                String,
                f"swarm/{pid}/telemetry",
                lambda msg, peer=pid: self._peer_telemetry_cb(msg, peer),
                pool_qos
            )

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

    def _scan_cb(self, msg: LaserScan):
        """
        Step 4: Real-Time LiDAR Front-Sector Safety Braking.
        Evaluates 360-degree laser range data in the front 60-degree cone [-30 deg, +30 deg].
        If any range reading is < 0.6m, sets _lidar_obstacle_detected = True.
        """
        if not msg.ranges:
            self._lidar_obstacle_detected = False
            return

        num_samples = len(msg.ranges)
        sector_span = max(1, int(num_samples * (60.0 / 360.0)))
        half_span = sector_span // 2

        front_indices = list(range(0, half_span)) + list(range(num_samples - half_span, num_samples))
        min_dist = 999.0
        for idx in front_indices:
            if idx < len(msg.ranges):
                r = msg.ranges[idx]
                if msg.range_min <= r <= msg.range_max:
                    if r < min_dist:
                        min_dist = r

        self._lidar_obstacle_detected = (min_dist < 0.6)

    def _peer_telemetry_cb(self, msg: String, peer_id: str):
        """
        Step 5: Process P2P space-time telemetry broadcast from peer AMRs.
        """
        try:
            data = json.loads(msg.data)
            self._peer_telemetry[peer_id] = data
        except Exception:
            pass

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
        pickup_raw = data.get("pickup", [0.0, 0.0])
        pickup_coords = self._planner.graph.get_coords(pickup_raw)

        dx = pickup_coords[0] - self._pos_x
        dy = pickup_coords[1] - self._pos_y
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
                f"[{self._robot_id}] Planning Topological Graph A* path to pickup '{pickup}' for '{self._current_task.get('task_id')}'"
            )
            self._waypoints = self._planner.plan((self._pos_x, self._pos_y), pickup)
            self._state = "EN_ROUTE_PICKUP"

        elif self._state == "EN_ROUTE_PICKUP":
            if not self._waypoints:
                self.get_logger().info(
                    f"[{self._robot_id}] Pickup reached for '{self._current_task.get('task_id')}'. Planning Topological Graph A* path to dropoff..."
                )
                dropoff = self._current_task.get("dropoff", [self._pos_x, self._pos_y])
                self._waypoints = self._planner.plan((self._pos_x, self._pos_y), dropoff)
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
        """Simple proportional velocity control towards target waypoint with LiDAR braking & Priority Aging."""
        dx = target_x - self._pos_x
        dy = target_y - self._pos_y
        dist = math.sqrt(dx * dx + dy * dy)

        if dist <= self._waypoint_tolerance:
            self._stop_motors()
            return True

        # Step 4: Real-time LiDAR Safety Brake
        if self._lidar_obstacle_detected:
            self._stop_motors()
            return False

        # Step 5: Inter-AMR Space-Time Conflict Resolution & Priority Aging Yield
        if _time.time() < self._yield_until_time:
            self._stop_motors()
            return False

        for peer_id, peer_data in self._peer_telemetry.items():
            ppos = peer_data.get("pos", [999.0, 999.0])
            pdist = math.sqrt((ppos[0] - self._pos_x)**2 + (ppos[1] - self._pos_y)**2)
            if pdist < 0.9 and peer_data.get("is_moving", False):
                # Calculate Priority Aging score
                self_score = (10.0 * self._idle_wait_time_sec) + self._battery_level
                peer_wait = peer_data.get("idle_wait_sec", 0.0)
                peer_batt = peer_data.get("battery_pct", 100.0)
                peer_score = (10.0 * peer_wait) + peer_batt

                if self_score < peer_score:
                    # Yield right of way with random jitter delay (10-200ms)
                    jitter = (hash(self._robot_id + str(_time.time())) % 190 + 10) / 1000.0
                    self._yield_until_time = _time.time() + 0.5 + jitter
                    self.get_logger().info(
                        f"[{self._robot_id}] Space-Time conflict with {peer_id}! Yielding right-of-way ({jitter*1000:.0f}ms jitter)."
                    )
                    self._stop_motors()
                    return False

        # Calculate steering angle towards waypoint
        target_yaw = math.atan2(dy, dx)
        yaw_err = target_yaw - self._current_yaw
        yaw_err = math.atan2(math.sin(yaw_err), math.cos(yaw_err))

        cmd = Twist()
        if abs(yaw_err) > 0.15:
            # Turn in place: zero linear speed, minimum 0.2 rad/s angular speed to prevent stalling
            ang_spd = max(0.2, min(1.0, abs(yaw_err) * 1.5))
            cmd.angular.z = ang_spd if yaw_err > 0 else -ang_spd
            cmd.linear.x = 0.0
        else:
            # Facing waypoint (within 0.15 rad): drive forward along virtual rail
            cmd.linear.x = min(self._linear_speed, max(0.15, dist * 0.5))
            ang_val = 1.0 * yaw_err
            if abs(ang_val) > 0.02:
                cmd.angular.z = math.copysign(max(0.1, abs(ang_val)), ang_val)
            else:
                cmd.angular.z = 0.0

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
        """Broadcast state machine status & P2P telemetry for monitoring & peer space-time checks."""
        status_payload = {
            "robot_id": self._robot_id,
            "state": self._state,
            "current_task": self._current_task.get("task_id") if self._current_task else None,
            "pos": [round(self._pos_x, 2), round(self._pos_y, 2)],
            "yaw": round(self._current_yaw, 3),
            "is_moving": self._is_moving,
            "idle_wait_sec": round(self._idle_wait_time_sec, 2),
            "battery_pct": round(self._battery_level, 1),
            "waypoints": [list(w) for w in self._waypoints[:5]],
            "timestamp": _time.time()
        }
        msg = String()
        msg.data = json.dumps(status_payload)
        self._pub_status.publish(msg)
        self._pub_telemetry.publish(msg)


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
