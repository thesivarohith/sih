#!/usr/bin/env python3
"""
app.py — Passive Web Dashboard Backend (SIH26123)
=============================================================================
Flask-SocketIO server subscribing to ROS 2 swarm telemetry, CNP auctions,
and mission metrics via a background rclpy thread.

Read-only Architecture:
  - Subscribes strictly to ROS 2 topics over P2P network.
  - Emits JSON events directly to the HTML5 brutalist frontend via WebSockets.
  - Does NOT publish any commands back to robots to preserve decentralization.

Topics Subscribed:
  - /swarm/<robot_id>/telemetry     (Position, battery, wait time, halt state)
  - /swarm/<robot_id>/mission_metrics (Task completion KPIs, latency, travel vs wait time)
  - swarm/tasks/auction            (CNP auction CFP, BID, AWARD messages)
  - /task_manager_status           (Task manager state machine status)
"""

import json
import os
import sys
import threading
import time
from typing import Dict, Any

from flask import Flask, render_template
from flask_socketio import SocketIO, emit

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String


# Initialize Flask App & SocketIO
app = Flask(__name__, template_folder="templates")
app.config["SECRET_KEY"] = "sih26123_dashboard_secret"
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")


class DashboardSubscriberNode(Node):
    """
    ROS 2 Passive Subscriber Node.
    Forwards received telemetry and mission metrics to SocketIO clients.
    """

    def __init__(self, socket_instance: SocketIO):
        super().__init__("dashboard_subscriber_node")
        self._socketio = socket_instance

        # Known AMR Fleet IDs
        self._robot_ids = ["amr_1", "amr_2", "amr_3"]

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        # 1. Telemetry Subscriptions per robot
        self._telemetry_subs = []
        for rid in self._robot_ids:
            topic = f"/swarm/{rid}/telemetry"
            sub = self.create_subscription(
                String,
                topic,
                lambda msg, robot=rid: self._handle_telemetry(robot, msg),
                qos,
            )
            self._telemetry_subs.append(sub)

        # 2. Mission Metrics Subscriptions per robot
        self._metrics_subs = []
        for rid in self._robot_ids:
            topic = f"/swarm/{rid}/mission_metrics"
            sub = self.create_subscription(
                String,
                topic,
                lambda msg, robot=rid: self._handle_metrics(robot, msg),
                qos,
            )
            self._metrics_subs.append(sub)

        # 3. CNP Auction Network Topic Subscription
        self._sub_auction = self.create_subscription(
            String, "swarm/tasks/auction", self._handle_auction, qos
        )

        # 4. Task Manager Status Subscription
        self._sub_task_status = self.create_subscription(
            String, "/task_manager_status", self._handle_task_status, qos
        )

        self.get_logger().info("DashboardSubscriberNode listening on swarm telemetry & CNP topics")

    def _handle_telemetry(self, robot_id: str, msg: String):
        """Relay telemetry payload to frontend."""
        try:
            data = json.loads(msg.data)
            self._socketio.emit("telemetry_update", data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Telemetry JSON decode error: {e}")

    def _handle_metrics(self, robot_id: str, msg: String):
        """Relay mission KPI metrics payload to frontend."""
        try:
            data = json.loads(msg.data)
            self._socketio.emit("metrics_update", data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Metrics JSON decode error: {e}")

    def _handle_auction(self, msg: String):
        """Relay CNP auction event to frontend."""
        try:
            data = json.loads(msg.data)
            self._socketio.emit("auction_update", data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Auction JSON decode error: {e}")

    def _handle_task_status(self, msg: String):
        """Relay task manager state update to frontend."""
        try:
            data = json.loads(msg.data)
            self._socketio.emit("task_status_update", data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Task status JSON decode error: {e}")


def _run_ros_thread(socket_instance: SocketIO):
    """Background thread spinning ROS 2 executor."""
    rclpy.init()
    node = DashboardSubscriberNode(socket_instance)
    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"[Dashboard ROS Thread] Exception: {e}", file=sys.stderr)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


@app.route("/")
def index():
    """Render brutalist engineering dashboard interface."""
    return render_template("index.html")


if __name__ == "__main__":
    # Start background ROS 2 subscriber thread
    ros_thread = threading.Thread(
        target=_run_ros_thread, args=(socketio,), daemon=True
    )
    ros_thread.start()

    print("[Dashboard] Starting Flask-SocketIO server on http://0.0.0.0:5050")
    socketio.run(app, host="0.0.0.0", port=5050, debug=False, allow_unsafe_werkzeug=True)
