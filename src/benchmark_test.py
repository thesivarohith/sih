#!/usr/bin/env python3
"""
Automated Benchmark Suite for SIH26123 AMR Swarm.
Dispatches a batch of 5 tasks to the P2P task pool.
"""

import json
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String

class BenchmarkSuite(Node):
    def __init__(self):
        super().__init__("benchmark_suite_node")
        pool_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._pub_pool = self.create_publisher(String, "swarm/tasks/pool", pool_qos)
        self.get_logger().info("Benchmark Suite Node Initialized. Preparing batch dispatch...")

    def run_benchmark(self):
        tasks = [
            {
                "msg_type": "AVAILABLE",
                "task_id": "mission_001",
                "pickup": [2.5, 6.0],
                "dropoff": [2.5, 13.0]
            },
            {
                "msg_type": "AVAILABLE",
                "task_id": "mission_002",
                "pickup": [7.5, 6.0],
                "dropoff": [7.5, 13.0]
            },
            {
                "msg_type": "AVAILABLE",
                "task_id": "mission_003",
                "pickup": [12.5, 6.0],
                "dropoff": [12.5, 13.0]
            },
            {
                "msg_type": "AVAILABLE",
                "task_id": "mission_004",
                "pickup": [2.5, 8.0],
                "dropoff": [5.0, 13.0]
            },
            {
                "msg_type": "AVAILABLE",
                "task_id": "mission_005",
                "pickup": [12.5, 8.0],
                "dropoff": [10.0, 13.0]
            }
        ]

        # Allow ROS graph publishers/subscribers connection to settle
        time.sleep(1.0)

        for task in tasks:
            msg = String()
            msg.data = json.dumps(task)
            self._pub_pool.publish(msg)
            self.get_logger().info(f"Dispatched task '{task['task_id']}' -> Pickup: {task['pickup']}, Dropoff: {task['dropoff']}")
            time.sleep(1.5)

        self.get_logger().info("Batch dispatch of 5 missions complete.")

def main(args=None):
    rclpy.init(args=args)
    node = BenchmarkSuite()
    node.run_benchmark()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
