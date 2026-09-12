#!/usr/bin/env python3
"""
cnp_auction_node.py — Decentralized Task Reallocation via Contract Net Protocol
=============================================================================
ROS 2 Jazzy node implementing Contract Net Protocol (CNP) task auctioning 
for multi-AMR coordination when an AMR's path is permanently obstructed.

Architecture (rmw_zenoh P2P — Decentralized Swarm Task Reallocation):
  When an AMR encounters a permanent path obstruction (e.g. static pallet/box
  detected by yolo_vision_node.py), it triggers a CNP auction sequence:
    1. Initiator broadcasts Call for Proposals (CFP) to global topic `swarm/tasks/auction`.
    2. Peer AMRs check eligibility (Battery > 25%, busy state) and reply with a BID
       calculating distance-to-task + battery penalty cost score.
    3. Initiator waits strictly 500ms for bids, selects the best (lowest cost) bidder,
       and publishes an AWARD message.
    4. Auto-retries (up to 3 attempts) are handled if no valid bids arrive within 500ms.

CNP Sequence & State Machine:
  [Obstruction Detected] -> CFP Broadcast -> [500ms Timer Active]
                                                      |
                    +---------------------------------+---------------------------------+
                    |                                                                   |
          [Valid Bids Received]                                              [Zero Valid Bids Received]
                    |                                                                   |
            Select Best Bid                                                 Retry < 3?
                    |                                                        /       \
          Publish AWARD Message                                            YES        NO
                    |                                                      /           \
          Reassign Task to Winner                                     Re-broadcast     Publish AUCTION_FAILED
                                                                      CFP (Retry++)    Log Critical Failure

Topics:
  PUB/SUB  swarm/tasks/auction          (std_msgs/String, JSON CNP messages)
  SUB      /amr/obstruction_status      (std_msgs/String — SAFETY_STOP / BLOCKED_AUCTION_REQUIRED / CLEAR)
  SUB      /odom                        (nav_msgs/Odometry — current position for bid distance calculation)
  PUB      /cnp_auction_status          (std_msgs/String — telemetry log of active auctions)
"""

import json
import math
import time as _time
from typing import Dict, Any, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
from nav_msgs.msg import Odometry


class CNPAuctionNode(Node):
    """
    Decentralized Contract Net Protocol (CNP) Task Auctioneer & Bidder Node.
    Runs on every AMR in the swarm to enable autonomous task handoff.
    """

    def __init__(self):
        super().__init__("cnp_auction_node")

        # ---------------------------------------------------------------
        # ROS 2 Parameters
        # ---------------------------------------------------------------
        self.declare_parameter("robot_id", "amr_1")
        self.declare_parameter("peer_ids", ["amr_2", "amr_3"])
        self.declare_parameter("battery_pct", 100.0)
        self.declare_parameter("auction_timeout_ms", 500)
        self.declare_parameter("min_battery_threshold", 25.0)
        self.declare_parameter("max_auction_retries", 3)

        self._robot_id              = self.get_parameter("robot_id").value
        self._peer_ids              = self.get_parameter("peer_ids").value
        self._battery_pct           = float(self.get_parameter("battery_pct").value)
        self._auction_timeout_sec   = float(self.get_parameter("auction_timeout_ms").value) / 1000.0
        self._min_battery_threshold = float(self.get_parameter("min_battery_threshold").value)
        self._max_retries           = int(self.get_parameter("max_auction_retries").value)

        # ---------------------------------------------------------------
        # Internal State
        # ---------------------------------------------------------------
        self._pos_x: float = 0.0
        self._pos_y: float = 0.0
        self._is_busy: bool = False
        self._current_task: Optional[Dict[str, Any]] = None

        # Active auction state as Initiator
        self._active_initiator_auction: Optional[Dict[str, Any]] = None
        self._bids_received: Dict[str, Dict[str, Any]] = {}
        self._auction_timer = None
        self._auction_retry_count: int = 0

        # QoS for P2P Auction Network: Reliable, Volatile, Depth 10
        auction_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Odom QoS: Best Effort
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ---------------------------------------------------------------
        # Publishers & Subscriptions
        # ---------------------------------------------------------------
        # Global CNP Auction Network topic (Zenoh P2P shared across swarm)
        self._pub_auction = self.create_publisher(
            String, "swarm/tasks/auction", auction_qos
        )
        self._sub_auction = self.create_subscription(
            String, "swarm/tasks/auction", self._cnp_message_cb, auction_qos
        )

        # Local obstruction status from yolo_vision_node
        self._sub_obstruction = self.create_subscription(
            String, "/amr/obstruction_status", self._obstruction_cb, auction_qos
        )

        # Local odometry for position tracking
        self._sub_odom = self.create_subscription(
            Odometry, "/odom", self._odom_cb, odom_qos
        )

        # Local telemetry/status publisher for monitoring
        self._pub_status = self.create_publisher(
            String, "/cnp_auction_status", auction_qos
        )

        # Timer for periodic status logging / battery simulation
        self._status_timer = self.create_timer(2.0, self._publish_status)

        self.get_logger().info(
            f"CNPAuctionNode initialized for '{self._robot_id}' | "
            f"peers={self._peer_ids} | timeout={self._auction_timeout_sec*1000:.0f}ms | "
            f"min_battery={self._min_battery_threshold}% | max_retries={self._max_retries}"
        )

    # ===================================================================
    # Callbacks
    # ===================================================================

    def _odom_cb(self, msg: Odometry):
        """Update current position coordinates."""
        self._pos_x = msg.pose.pose.position.x
        self._pos_y = msg.pose.pose.position.y

    def _obstruction_cb(self, msg: String):
        """
        Trigger task auctioning when path is permanently blocked.
        Trigger message: "BLOCKED_AUCTION_REQUIRED" from yolo_vision_node.
        """
        status = msg.data.strip().upper()

        if status == "BLOCKED_AUCTION_REQUIRED":
            self.get_logger().warn(
                f"[{self._robot_id}] Permanent obstruction detected! Triggering CNP Task Auction..."
            )
            self._trigger_auction()

    # ===================================================================
    # CNP Initiator Logic: Triggering & Managing Auctions
    # ===================================================================

    def _trigger_auction(self):
        """Initiates a new Call for Proposals (CFP) for task handoff."""
        if self._active_initiator_auction is not None:
            self.get_logger().warn(
                f"[{self._robot_id}] Auction already in progress ({self._active_initiator_auction['auction_id']}). Ignoring new trigger."
            )
            return

        auction_id = f"auc_{self._robot_id}_{int(_time.time() * 1000)}"
        
        # Sample task specification (pickup waypoint or destination)
        task_data = {
            "task_id": f"task_{self._robot_id}_reallocate",
            "target_pose": [10.0, 12.0],  # Example target coordinate
            "priority": "HIGH",
            "payload_weight_kg": 15.0
        }

        self._active_initiator_auction = {
            "auction_id": auction_id,
            "task": task_data,
            "start_time": _time.monotonic()
        }
        self._bids_received = {}
        self._auction_retry_count = 0

        self._broadcast_cfp()

    def _broadcast_cfp(self):
        """Broadcasts Call for Proposals (CFP) over the swarm P2P network."""
        if not self._active_initiator_auction:
            return

        cfp_msg = {
            "msg_type": "CFP",
            "auction_id": self._active_initiator_auction["auction_id"],
            "initiator_id": self._robot_id,
            "task": self._active_initiator_auction["task"],
            "retry_count": self._auction_retry_count,
            "timestamp": _time.time()
        }

        msg = String()
        msg.data = json.dumps(cfp_msg)
        self._pub_auction.publish(msg)

        self.get_logger().info(
            f"[{self._robot_id}] Broadcasted CFP '{cfp_msg['auction_id']}' (attempt {self._auction_retry_count + 1}/{self._max_retries + 1}). "
            f"Starting {self._auction_timeout_sec * 1000:.0f}ms bid window."
        )

        # Cancel any active timer before starting a new 500ms single-shot timer
        if self._auction_timer is not None:
            self._auction_timer.cancel()

        self._auction_timer = self.create_timer(
            self._auction_timeout_sec, self._on_auction_timeout
        )

    def _on_auction_timeout(self):
        """
        Executed strictly when the 500ms auction timer expires.
        Evaluates incoming bids and determines winner or triggers retry.
        """
        # Cancel the timer so it doesn't repeat
        if self._auction_timer is not None:
            self._auction_timer.cancel()
            self._auction_timer = None

        if not self._active_initiator_auction:
            return

        auction_id = self._active_initiator_auction["auction_id"]
        task = self._active_initiator_auction["task"]

        self.get_logger().info(
            f"[{self._robot_id}] 500ms Auction window closed for '{auction_id}'. "
            f"Total bids received: {len(self._bids_received)}"
        )

        # Filter valid bids (must not be busy, battery > min threshold)
        valid_bids = []
        for bidder_id, bid in self._bids_received.items():
            if bid.get("is_busy", False):
                self.get_logger().info(f"[{self._robot_id}] Bid from {bidder_id} ignored: Robot is busy.")
                continue
            if bid.get("battery_pct", 0.0) <= self._min_battery_threshold:
                self.get_logger().info(f"[{self._robot_id}] Bid from {bidder_id} ignored: Low battery ({bid.get('battery_pct')}%).")
                continue
            valid_bids.append((bidder_id, bid))

        if valid_bids:
            # Select winner with lowest bid_cost (closest distance + battery factor)
            valid_bids.sort(key=lambda item: item[1]["bid_cost"])
            winner_id, winning_bid = valid_bids[0]

            self.get_logger().info(
                f"[{self._robot_id}] AUCTION WON by '{winner_id}' with bid_cost={winning_bid['bid_cost']:.2f} "
                f"(dist={winning_bid['distance_m']:.2f}m, battery={winning_bid['battery_pct']}%)"
            )

            # Broadcast AWARD message
            award_msg = {
                "msg_type": "AWARD",
                "auction_id": auction_id,
                "initiator_id": self._robot_id,
                "winner_id": winner_id,
                "task": task,
                "winning_cost": winning_bid["bid_cost"],
                "timestamp": _time.time()
            }
            msg = String()
            msg.data = json.dumps(award_msg)
            self._pub_auction.publish(msg)

            # Reset local auction state
            self._active_initiator_auction = None
            self._bids_received = {}

        else:
            # Handle Zero Valid Bids / Retry Logic
            self.get_logger().warn(
                f"[{self._robot_id}] No valid bids received for '{auction_id}' within 500ms."
            )

            if self._auction_retry_count < self._max_retries:
                self._auction_retry_count += 1
                self.get_logger().info(
                    f"[{self._robot_id}] Retrying auction (Attempt {self._auction_retry_count + 1}/{self._max_retries + 1})..."
                )
                # Short 200ms delay before retrying CFP
                self.create_timer(0.2, self._retry_cfp_callback)
            else:
                self.get_logger().error(
                    f"[{self._robot_id}] AUCTION FAILED for '{auction_id}' after {self._max_retries + 1} attempts. Swarm occupied or out of battery range."
                )

                fail_msg = {
                    "msg_type": "AUCTION_FAILED",
                    "auction_id": auction_id,
                    "initiator_id": self._robot_id,
                    "reason": "NO_ELIGIBLE_BIDS_AFTER_RETRIES",
                    "timestamp": _time.time()
                }
                msg = String()
                msg.data = json.dumps(fail_msg)
                self._pub_auction.publish(msg)

                self._active_initiator_auction = None
                self._bids_received = {}

    def _retry_cfp_callback(self):
        """Single-shot timer callback for auction retry."""
        self._broadcast_cfp()

    # ===================================================================
    # CNP Message Router & Bidder Logic
    # ===================================================================

    def _cnp_message_cb(self, msg: String):
        """Ingests and routes incoming CNP messages across the swarm P2P network."""
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse CNP JSON message: {e}")
            return

        msg_type = payload.get("msg_type")

        if msg_type == "CFP":
            self._handle_cfp(payload)
        elif msg_type == "BID":
            self._handle_bid(payload)
        elif msg_type == "AWARD":
            self._handle_award(payload)
        elif msg_type == "AUCTION_FAILED":
            self._handle_auction_failed(payload)

    def _handle_cfp(self, cfp: Dict[str, Any]):
        """
        Processes incoming Call for Proposals (CFP).
        Evaluates eligibility (battery, busy state) and submits a BID.
        """
        initiator_id = cfp.get("initiator_id")
        auction_id = cfp.get("auction_id")
        task = cfp.get("task", {})

        # Ignore self-generated CFPs
        if initiator_id == self._robot_id:
            return

        self.get_logger().info(
            f"[{self._robot_id}] Received CFP '{auction_id}' from '{initiator_id}'."
        )

        # Check Battery Eligibility Threshold (> 25%)
        if self._battery_pct <= self._min_battery_threshold:
            self.get_logger().info(
                f"[{self._robot_id}] Skipping bid for '{auction_id}': Low battery ({self._battery_pct:.1f}% <= {self._min_battery_threshold}%)."
            )
            return

        # Check Busy State
        if self._is_busy:
            self.get_logger().info(
                f"[{self._robot_id}] Submitting BUSY bid for '{auction_id}'."
            )
            self._send_bid(auction_id, initiator_id, cost=9999.0, dist=9999.0, is_busy=True)
            return

        # Calculate Distance to Task Target Pose
        target_pose = task.get("target_pose", [self._pos_x, self._pos_y])
        target_x, target_y = target_pose[0], target_pose[1]
        
        dx = target_x - self._pos_x
        dy = target_y - self._pos_y
        dist_m = math.sqrt(dx * dx + dy * dy)

        # Calculate Deterministic Bid Cost Score
        # Formula: bid_cost = distance_m + (100.0 - battery_pct) * 0.05
        # (Lower score is better; rewards closer robots with higher battery)
        battery_penalty = (100.0 - self._battery_pct) * 0.05
        bid_cost = round(dist_m + battery_penalty, 3)

        self.get_logger().info(
            f"[{self._robot_id}] Submitting BID for '{auction_id}' | dist={dist_m:.2f}m, battery={self._battery_pct:.1f}%, bid_cost={bid_cost}"
        )

        self._send_bid(auction_id, initiator_id, cost=bid_cost, dist=dist_m, is_busy=False)

    def _send_bid(self, auction_id: str, initiator_id: str, cost: float, dist: float, is_busy: bool):
        """Constructs and publishes BID message to the swarm auction network."""
        bid_msg = {
            "msg_type": "BID",
            "auction_id": auction_id,
            "bidder_id": self._robot_id,
            "initiator_id": initiator_id,
            "bid_cost": cost,
            "distance_m": round(dist, 2),
            "battery_pct": round(self._battery_pct, 1),
            "is_busy": is_busy,
            "timestamp": _time.time()
        }

        msg = String()
        msg.data = json.dumps(bid_msg)
        self._pub_auction.publish(msg)

    def _handle_bid(self, bid: Dict[str, Any]):
        """Processes incoming BID messages when acting as Initiator."""
        if not self._active_initiator_auction:
            return

        auction_id = bid.get("auction_id")
        bidder_id = bid.get("bidder_id")

        if auction_id == self._active_initiator_auction["auction_id"]:
            self._bids_received[bidder_id] = bid
            self.get_logger().info(
                f"[{self._robot_id}] Ingested BID from '{bidder_id}' for '{auction_id}' (cost: {bid.get('bid_cost')})."
            )

    def _handle_award(self, award: Dict[str, Any]):
        """Processes AWARD decision published by an Initiator."""
        winner_id = award.get("winner_id")
        auction_id = award.get("auction_id")
        task = award.get("task", {})

        if winner_id == self._robot_id:
            self.get_logger().warn(
                f"[{self._robot_id}] *** TASK AWARDED TO THIS AMR *** | Auction: '{auction_id}' | Task: {task.get('task_id')}"
            )
            self._is_busy = True
            self._current_task = task
        else:
            self.get_logger().info(
                f"[{self._robot_id}] Auction '{auction_id}' awarded to peer '{winner_id}'."
            )

    def _handle_auction_failed(self, msg: Dict[str, Any]):
        """Handles auction failure notifications."""
        auction_id = msg.get("auction_id")
        initiator_id = msg.get("initiator_id")
        self.get_logger().warn(
            f"[{self._robot_id}] Auction '{auction_id}' by '{initiator_id}' failed: {msg.get('reason')}"
        )

    # ===================================================================
    # Status Monitoring Publisher
    # ===================================================================

    def _publish_status(self):
        """Periodically publishes CNP state summary to /cnp_auction_status."""
        status_payload = {
            "robot_id": self._robot_id,
            "battery_pct": round(self._battery_pct, 1),
            "is_busy": self._is_busy,
            "pos": [round(self._pos_x, 2), round(self._pos_y, 2)],
            "active_auction": self._active_initiator_auction["auction_id"] if self._active_initiator_auction else None,
            "timestamp": _time.time()
        }
        msg = String()
        msg.data = json.dumps(status_payload)
        self._pub_status.publish(msg)


# =======================================================================
# Main Entry Point
# =======================================================================
def main(args=None):
    rclpy.init(args=args)
    node = CNPAuctionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down CNPAuctionNode (Ctrl+C)")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
