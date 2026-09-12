#!/usr/bin/env python3
"""
priority_negotiator_node.py — Decentralized Deadlock Resolution for SIH26123
=============================================================================
ROS 2 Jazzy node implementing Priority Aging with deterministic conflict
resolution for multi-AMR coordination in narrow warehouse aisles.

Architecture (rmw_zenoh P2P — No Central Server):
  Every AMR runs this node independently. Each node:
    1. Broadcasts its own telemetry (position, battery, wait state).
    2. Ingests peer telemetry from all other AMRs in the swarm.
    3. Detects spatial conflicts (two AMRs claiming the same zone).
    4. Computes Priority Aging scores and deterministically resolves
       who yields and who proceeds — identically on every peer.

Priority Aging Formula [cite: 1]:
  Score = (wait_duration_sec × 10.0)
        + (100.0 − battery_percentage)
        + stable_random_tiebreaker(0.01 … 0.20)

  - wait_duration: monotonically increases once halted (never resets
    while a conflict is active — prevents circular deadlocks).
  - battery penalty: a depleted AMR gets higher priority to finish
    its task before it runs out of charge.
  - random tiebreaker: computed deterministically per robot-pair so
    both peers agree on who wins, preventing hallway-dance livelocks.

Topics:
  PUB  /swarm/<robot_id>/telemetry       (std_msgs/String, JSON)
  SUB  /swarm/<peer_id>/telemetry        (std_msgs/String, JSON)  × N peers
  PUB  /negotiation_status               (std_msgs/String)
  SUB  /amr/obstruction_status           (std_msgs/String)  — from vision node
"""

import json
import math
import hashlib
import time as _time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


# ---------------------------------------------------------------------------
# Telemetry message schema (JSON-encoded over std_msgs/String to avoid
# requiring a custom .msg package — keeps the workspace dependency-free)
# ---------------------------------------------------------------------------
# {
#   "robot_id":            "amr_01",
#   "x":                   7.5,
#   "y":                   13.0,
#   "battery_pct":         82.5,
#   "wait_duration_sec":   3.7,
#   "is_halted":           true,
#   "timestamp":           1789219500.123
# }
# ---------------------------------------------------------------------------


class PriorityNegotiatorNode(Node):
    """
    Decentralized priority-aging negotiator.
    One instance per AMR — no central coordinator required.
    """

    def __init__(self):
        super().__init__("priority_negotiator_node")

        # ---------------------------------------------------------------
        # ROS 2 Parameters (override at launch with --ros-args -p)
        # ---------------------------------------------------------------
        self.declare_parameter("robot_id", "amr_01")
        self.declare_parameter("peer_ids", ["amr_02", "amr_03"])
        self.declare_parameter("battery_pct", 100.0)
        self.declare_parameter("conflict_radius_m", 1.2)
        self.declare_parameter("telemetry_rate_hz", 10.0)

        self._robot_id        = self.get_parameter("robot_id").value
        self._peer_ids        = self.get_parameter("peer_ids").value
        self._battery_pct     = self.get_parameter("battery_pct").value
        self._conflict_radius = self.get_parameter("conflict_radius_m").value
        telemetry_hz          = self.get_parameter("telemetry_rate_hz").value

        # ---------------------------------------------------------------
        # Internal state
        # ---------------------------------------------------------------
        self._pos_x = 0.0                     # latest odom position
        self._pos_y = 0.0
        self._is_halted = False                # set True when stopped
        self._halt_start_time = None           # monotonic time when halt began
        self._wait_duration_sec = 0.0          # strictly increasing while halted

        # Peer state table: { "amr_02": { ...telemetry dict... }, ... }
        self._peer_states: dict[str, dict] = {}

        # Tracks which peer we are actively in conflict with, and the
        # stable random offset generated for that specific conflict pair.
        self._active_conflict_peer: str | None = None
        self._stable_tiebreaker: float = 0.0

        # Track the current negotiation outcome
        self._current_status = "IDLE"

        # ---------------------------------------------------------------
        # QoS: reliable + volatile for negotiation (no stale data)
        # ---------------------------------------------------------------
        negotiation_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ---------------------------------------------------------------
        # Publishers
        # ---------------------------------------------------------------
        self._pub_telemetry = self.create_publisher(
            String,
            f"/swarm/{self._robot_id}/telemetry",
            negotiation_qos,
        )
        self._pub_status = self.create_publisher(
            String, "/negotiation_status", negotiation_qos
        )

        # ---------------------------------------------------------------
        # Subscriptions — peer telemetry (one per known peer)
        #
        # ROS 2 does not support MQTT-style wildcard topics (swarm/+/telemetry).
        # With rmw_zenoh each peer's topic is a distinct Zenoh key expression.
        # We create one subscription per declared peer_id.
        # ---------------------------------------------------------------
        self._peer_subs = []
        for peer_id in self._peer_ids:
            sub = self.create_subscription(
                String,
                f"/swarm/{peer_id}/telemetry",
                lambda msg, pid=peer_id: self._peer_telemetry_cb(pid, msg),
                negotiation_qos,
            )
            self._peer_subs.append(sub)
            self.get_logger().info(f"Subscribed to /swarm/{peer_id}/telemetry")

        # ---------------------------------------------------------------
        # Subscription — own odometry (from Gazebo diff-drive plugin)
        # ---------------------------------------------------------------
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._sub_odom = self.create_subscription(
            Odometry, "/odom", self._odom_cb, odom_qos
        )

        # ---------------------------------------------------------------
        # Subscription — obstruction status (from yolo_vision_node)
        # Used to detect when this AMR is halted by the vision pipeline.
        # ---------------------------------------------------------------
        self._sub_obstruction = self.create_subscription(
            String, "/amr/obstruction_status", self._obstruction_cb,
            negotiation_qos,
        )

        # ---------------------------------------------------------------
        # Timers
        # ---------------------------------------------------------------
        # Telemetry broadcast timer
        self._telemetry_timer = self.create_timer(
            1.0 / telemetry_hz, self._publish_telemetry
        )

        # Conflict evaluation timer (2 Hz — lightweight check)
        self._conflict_timer = self.create_timer(0.5, self._evaluate_conflicts)

        # Battery drain simulation timer (slow drain for demo realism)
        self._battery_timer = self.create_timer(5.0, self._simulate_battery_drain)

        self.get_logger().info(
            f"PriorityNegotiatorNode initialized | "
            f"robot_id={self._robot_id} peers={self._peer_ids} "
            f"conflict_radius={self._conflict_radius}m"
        )

    # ===================================================================
    # Callbacks — Incoming Data
    # ===================================================================

    def _odom_cb(self, msg: Odometry):
        """Update own position from odometry."""
        self._pos_x = msg.pose.pose.position.x
        self._pos_y = msg.pose.pose.position.y

    def _obstruction_cb(self, msg: String):
        """
        React to vision node obstruction classifications.
        SAFETY_STOP or BLOCKED_AUCTION_REQUIRED → mark self as halted.
        CLEAR → mark self as free (only if no active conflict holds us).
        """
        status = msg.data.strip().upper()

        if status in ("SAFETY_STOP", "BLOCKED_AUCTION_REQUIRED"):
            if not self._is_halted:
                self._is_halted = True
                self._halt_start_time = _time.monotonic()
                self.get_logger().info(
                    f"Halt triggered by vision: {status}"
                )
        elif status == "CLEAR":
            # Only clear halt if we are not actively yielding to a peer
            if self._current_status != "YIELDING":
                self._is_halted = False
                self._halt_start_time = None
                self._wait_duration_sec = 0.0

    def _peer_telemetry_cb(self, peer_id: str, msg: String):
        """
        Decode and store a peer's latest telemetry snapshot.
        Stale entries are naturally overwritten each tick.
        """
        try:
            data = json.loads(msg.data)
            self._peer_states[peer_id] = data
        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().warn(
                f"Malformed telemetry from {peer_id}: {e}"
            )

    # ===================================================================
    # Telemetry Broadcast
    # ===================================================================

    def _publish_telemetry(self):
        """
        Broadcast this AMR's telemetry to the swarm at the configured rate.
        All fields needed by any peer to compute our Priority Aging score.
        """
        # ---- Update wait duration (monotonically increasing while halted) ----
        if self._is_halted and self._halt_start_time is not None:
            self._wait_duration_sec = _time.monotonic() - self._halt_start_time
            # NOTE: wait_duration_sec NEVER decreases while is_halted is True.
            # This is the key invariant that prevents circular deadlocks:
            # a waiting AMR's score can only grow, guaranteeing eventual
            # convergence to a unique winner.

        payload = {
            "robot_id":          self._robot_id,
            "x":                 round(self._pos_x, 4),
            "y":                 round(self._pos_y, 4),
            "battery_pct":       round(self._battery_pct, 2),
            "wait_duration_sec": round(self._wait_duration_sec, 3),
            "is_halted":         self._is_halted,
            "timestamp":         _time.time(),
        }

        msg = String()
        msg.data = json.dumps(payload)
        self._pub_telemetry.publish(msg)

    # ===================================================================
    # Priority Aging Score
    # ===================================================================

    def _compute_score(self, wait_sec: float, battery_pct: float,
                       tiebreaker: float) -> float:
        """
        Priority Aging formula [cite: 1]:

          Score = (wait_duration_sec × 10.0)
                + (100.0 − battery_pct)
                + tiebreaker

        Properties:
          • Dominated by wait_duration: a robot that has been waiting
            longer will almost always win, resolving starvation.
          • Battery penalty: tie-breaks in favor of the robot closer
            to depletion, so it can finish before running out.
          • Tiebreaker ∈ [0.01, 0.20]: deterministic per robot-pair,
            prevents livelock when both wait times are identical.

        Returns:
            float — the composite priority score (higher wins).
        """
        return (wait_sec * 10.0) + (100.0 - battery_pct) + tiebreaker

    def _generate_stable_tiebreaker(self, id_a: str, id_b: str) -> float:
        """
        Generate a deterministic random value in [0.01, 0.20] that is:
          • Stable across ticks for the same (id_a, id_b) pair.
          • Asymmetric: tiebreaker(A, B) ≠ tiebreaker(B, A), so both
            peers compute different scores and agree on a winner.

        Implementation:
          SHA-256 hash of the concatenated IDs → first 4 bytes → float.
          The caller always passes (self_id, peer_id) in that order,
          so each side of the conflict hashes a different string.
        """
        seed = hashlib.sha256(f"{id_a}:{id_b}".encode()).digest()
        # Convert first 4 bytes to an unsigned int, normalize to [0, 1)
        raw = int.from_bytes(seed[:4], "big") / (2**32)
        # Map to [0.01, 0.20]
        return 0.01 + raw * 0.19

    # ===================================================================
    # Conflict Detection & Resolution
    # ===================================================================

    def _evaluate_conflicts(self):
        """
        Called at 2 Hz. Scans all peer states for spatial conflicts
        and resolves them via Priority Aging score comparison.
        """
        now = _time.time()
        conflict_found = False
        status_msg = String()

        for peer_id, peer in self._peer_states.items():
            # ---- Skip stale peers (no update in >2 seconds) ----
            if now - peer.get("timestamp", 0) > 2.0:
                continue

            # ---- Euclidean proximity check ----
            dx = self._pos_x - peer.get("x", 0.0)
            dy = self._pos_y - peer.get("y", 0.0)
            dist = math.sqrt(dx * dx + dy * dy)

            if dist > self._conflict_radius:
                continue

            # ============================================================
            # SPATIAL CONFLICT DETECTED — two AMRs in the same zone
            # ============================================================
            conflict_found = True

            # Generate stable tiebreakers (asymmetric per peer pair)
            my_tiebreaker   = self._generate_stable_tiebreaker(
                self._robot_id, peer_id
            )
            peer_tiebreaker = self._generate_stable_tiebreaker(
                peer_id, self._robot_id
            )

            # Track the active conflict for halt-state management
            if self._active_conflict_peer != peer_id:
                self._active_conflict_peer = peer_id
                self._stable_tiebreaker = my_tiebreaker
                self.get_logger().info(
                    f"Conflict entered with {peer_id} at dist={dist:.2f}m"
                )

            # ---- Compute both scores ----
            my_score = self._compute_score(
                self._wait_duration_sec,
                self._battery_pct,
                my_tiebreaker,
            )

            peer_score = self._compute_score(
                peer.get("wait_duration_sec", 0.0),
                peer.get("battery_pct", 100.0),
                peer_tiebreaker,
            )

            # ---- Deterministic resolution ----
            if my_score >= peer_score:
                # We win right-of-way — proceed through the zone
                self._current_status = "PROCEEDING"
                status_msg.data = json.dumps({
                    "robot_id":   self._robot_id,
                    "status":     "PROCEEDING",
                    "vs_peer":    peer_id,
                    "my_score":   round(my_score, 4),
                    "peer_score": round(peer_score, 4),
                    "distance_m": round(dist, 3),
                })
                self._pub_status.publish(status_msg)

                # Release halt — we have right-of-way
                if self._is_halted:
                    self._is_halted = False
                    self._halt_start_time = None
                    self._wait_duration_sec = 0.0

                self.get_logger().info(
                    f"PROCEEDING vs {peer_id} | "
                    f"my={my_score:.3f} > peer={peer_score:.3f}"
                )
            else:
                # We lose — yield and continue accumulating wait time
                self._current_status = "YIELDING"
                if not self._is_halted:
                    self._is_halted = True
                    self._halt_start_time = _time.monotonic()
                    self.get_logger().info(
                        f"Halt started: yielding to {peer_id}"
                    )

                status_msg.data = json.dumps({
                    "robot_id":   self._robot_id,
                    "status":     "YIELDING",
                    "vs_peer":    peer_id,
                    "my_score":   round(my_score, 4),
                    "peer_score": round(peer_score, 4),
                    "distance_m": round(dist, 3),
                })
                self._pub_status.publish(status_msg)

                self.get_logger().info(
                    f"YIELDING to {peer_id} | "
                    f"my={my_score:.3f} < peer={peer_score:.3f}"
                )

            # Only resolve against the closest conflicting peer
            # (first match in the inner loop). Multi-way conflicts
            # degenerate to pairwise since the loser halts and its
            # wait_duration grows, eventually making it the winner.
            break

        # ---- No conflict — clear state ----
        if not conflict_found:
            if self._active_conflict_peer is not None:
                self.get_logger().info(
                    f"Conflict resolved with {self._active_conflict_peer}"
                )
                self._active_conflict_peer = None
                self._stable_tiebreaker = 0.0
                self._current_status = "IDLE"

                # Reset wait accumulator only after conflict fully clears
                if self._is_halted:
                    self._is_halted = False
                    self._halt_start_time = None
                    self._wait_duration_sec = 0.0

                status_msg.data = json.dumps({
                    "robot_id": self._robot_id,
                    "status":   "IDLE",
                })
                self._pub_status.publish(status_msg)

    # ===================================================================
    # Battery Simulation (for demo / SIH judges)
    # ===================================================================

    def _simulate_battery_drain(self):
        """
        Slowly drain battery to demonstrate the battery component
        of the Priority Aging formula during the SIH live demo.
        Drain rate: ~0.1% every 5 seconds (≈1.2%/min).
        """
        if self._battery_pct > 0.0:
            self._battery_pct = max(0.0, self._battery_pct - 0.1)


# =======================================================================
# Entry Point
# =======================================================================
def main(args=None):
    rclpy.init(args=args)
    node = PriorityNegotiatorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down PriorityNegotiatorNode (Ctrl+C)")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
