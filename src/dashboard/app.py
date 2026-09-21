#!/usr/bin/env python3
"""
app.py — Passive Web Dashboard Backend (SIH26123)
=============================================================================
Flask server subscribing to ROS 2 swarm telemetry, CNP auctions,
and mission metrics via a background rclpy thread.

Offline Architecture:
  - Uses native Server-Sent Events (SSE) `/stream` + WebSocket/JSON endpoints
    to eliminate all external CDN network dependencies.
  - Subscribes strictly to ROS 2 topics over P2P network.
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
from typing import Dict, Any, List

from flask import Flask, render_template, Response, jsonify
from flask_socketio import SocketIO

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from std_msgs.msg import String
    HAS_ROS = True
except ImportError:
    HAS_ROS = False
    Node = object
    String = Any



# Global In-Memory State Caches (Thread-safe reads for SSE stream)
TELEMETRY_DATA: Dict[str, Dict[str, Any]] = {
    "amr_1": {"robot_id": "amr_1", "x": 9.375, "y": 13.75, "battery_pct": 100.0, "wait_duration_sec": 0.0, "is_halted": False},
    "amr_2": {"robot_id": "amr_2", "x": 11.25, "y": 13.75, "battery_pct": 100.0, "wait_duration_sec": 0.0, "is_halted": False},
    "amr_3": {"robot_id": "amr_3", "x": 13.125, "y": 13.75, "battery_pct": 100.0, "wait_duration_sec": 0.0, "is_halted": False},
}

TASK_STATUS_DATA: Dict[str, str] = {
    "amr_1": "IDLE",
    "amr_2": "IDLE",
    "amr_3": "IDLE",
}

METRICS_DATA: List[Dict[str, Any]] = []
AUCTION_LOGS: List[Dict[str, Any]] = []

state_lock = threading.Lock()


# Initialize Flask App & SocketIO
app = Flask(__name__, template_folder="templates")
app.config["SECRET_KEY"] = "sih26123_dashboard_secret"
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")


class DashboardSubscriberNode(Node):
    """
    ROS 2 Passive Subscriber Node.
    Forwards received telemetry and mission metrics to global state cache.
    """

    def __init__(self, socket_instance: SocketIO):
        super().__init__("dashboard_subscriber_node")
        self._socketio = socket_instance
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
        """Update telemetry cache and emit event."""
        try:
            data = json.loads(msg.data)
            with state_lock:
                TELEMETRY_DATA[robot_id] = data
            self._socketio.emit("telemetry_update", data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Telemetry JSON decode error: {e}")

    def _handle_metrics(self, robot_id: str, msg: String):
        """Update metrics cache and emit event."""
        try:
            data = json.loads(msg.data)
            with state_lock:
                METRICS_DATA.append(data)
                if len(METRICS_DATA) > 50:
                    METRICS_DATA.pop(0)
            self._socketio.emit("metrics_update", data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Metrics JSON decode error: {e}")

    def _handle_auction(self, msg: String):
        """Update auction logs cache and emit event."""
        try:
            data = json.loads(msg.data)
            with state_lock:
                AUCTION_LOGS.append(data)
                if len(AUCTION_LOGS) > 50:
                    AUCTION_LOGS.pop(0)
            self._socketio.emit("auction_update", data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Auction JSON decode error: {e}")

    def _handle_task_status(self, msg: String):
        """Update task status cache and emit event."""
        try:
            data = json.loads(msg.data)
            rid = data.get("robot_id")
            if rid:
                with state_lock:
                    TASK_STATUS_DATA[rid] = data.get("state", "IDLE")
            self._socketio.emit("task_status_update", data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Task status JSON decode error: {e}")


def _run_ros_thread(socket_instance: SocketIO):
    """Background thread spinning ROS 2 executor with non-blocking GIL releases."""
    if not HAS_ROS:
        print("[Dashboard] ROS 2 (rclpy) not available. Dashboard running in standalone mode.")
        while True:
            time.sleep(1)
        return

    rclpy.init()
    node = DashboardSubscriberNode(socket_instance)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.01)
    except Exception as e:
        print(f"[Dashboard ROS Thread] Exception: {e}", file=sys.stderr)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


@app.route("/")
def index():
    """Render brutalist engineering dashboard interface."""
    return render_template("index.html")


@app.route("/injector")
def injector():
    """Render MissionInjector 3-column task queueing interface."""
    return render_template("injector.html")


@app.route("/api/inject_task", methods=["POST"])
def inject_task():
    """Receive task injection payload from Mission Injector UI."""
    from flask import request
    data = request.get_json(silent=True) or {}
    pickup = data.get("pickup")
    drop = data.get("drop")
    task_id = data.get("task_id", f"task_{int(time.time())}")

    auction_event = {
        "msg_type": "AVAILABLE",
        "auction_id": task_id,
        "initiator_id": "MISSION_INJECTOR",
        "pickup": pickup,
        "drop": drop,
        "timestamp": time.time()
    }

    with state_lock:
        AUCTION_LOGS.append(auction_event)
        if len(AUCTION_LOGS) > 50:
            AUCTION_LOGS.pop(0)

    socketio.emit("auction_update", auction_event)
    return jsonify({"status": "SUCCESS", "task_id": task_id, "event": auction_event})


@app.route("/api/clear_tasks", methods=["POST", "GET"])
def clear_tasks():
    """Clear all auction task logs and reset metrics."""
    with state_lock:
        AUCTION_LOGS.clear()
        METRICS_DATA.clear()
    return jsonify({"status": "CLEARED"})


@app.route("/api/state")
def get_state():
    """REST endpoint returning full snapshot of current swarm state."""
    with state_lock:
        snapshot = {
            "telemetry": TELEMETRY_DATA,
            "task_status": TASK_STATUS_DATA,
            "metrics": METRICS_DATA,
            "auctions": AUCTION_LOGS,
        }
    return jsonify(snapshot)


@app.route("/stream")
def stream():
    """Server-Sent Events (SSE) stream for zero-dependency real-time updates."""
    def event_stream():
        while True:
            with state_lock:
                payload = json.dumps({
                    "telemetry": TELEMETRY_DATA,
                    "task_status": TASK_STATUS_DATA,
                    "metrics": METRICS_DATA,
                    "auctions": AUCTION_LOGS,
                })
            yield f"data: {payload}\n\n"
            time.sleep(0.2)  # 5 Hz stream update rate

    return Response(event_stream(), mimetype="text/event-stream")


if __name__ == "__main__":
    ros_thread = threading.Thread(
        target=_run_ros_thread, args=(socketio,), daemon=True
    )
    ros_thread.start()

    print("[Dashboard] Starting Flask-SocketIO server on http://0.0.0.0:5050")
    socketio.run(app, host="0.0.0.0", port=5050, debug=False, allow_unsafe_werkzeug=True)
