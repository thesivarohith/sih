"""
multi_spawn.launch.py — 3-AMR Warehouse Swarm Launch (SIH26123)
================================================================
Spawns three independent edge_amr instances into the sih_warehouse
world with fully namespaced sensor bridges for P2P coordination.

Topology:
  amr_1  →  (7.5, 13.0)  — top center corridor
  amr_2  →  (13.5, 10.0) — near dead-end trap zone
  amr_3  →  (1.5,  5.0)  — bottom-left aisle entrance

Each AMR gets isolated:
  /<ns>/cmd_vel           (ROS → GZ)
  /<ns>/scan              (GZ → ROS)
  /<ns>/camera/image_raw  (GZ → ROS)
"""

import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    GroupAction,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory


# ---------------------------------------------------------------------------
# AMR Fleet Configuration
# ---------------------------------------------------------------------------
FLEET = [
    {"ns": "amr_1", "x": "7.5",  "y": "13.0", "z": "0.5", "yaw": "0.0"},
    {"ns": "amr_2", "x": "13.5", "y": "10.0", "z": "0.5", "yaw": "1.5708"},
    {"ns": "amr_3", "x": "1.5",  "y": "5.0",  "z": "0.5", "yaw": "-1.5708"},
]


def _namespace_urdf(urdf_str: str, ns: str) -> str:
    """
    Rewrite Gazebo topic names inside the URDF so each spawned model
    publishes on unique GZ transport topics, avoiding cross-talk.

    Replacements:
      <topic>cmd_vel</topic>           → <topic>{ns}/cmd_vel</topic>
      <topic>scan</topic>              → <topic>{ns}/scan</topic>
      <topic>camera/image_raw</topic>  → <topic>{ns}/camera/image_raw</topic>
    """
    replacements = {
        "<topic>cmd_vel</topic>":          f"<topic>{ns}/cmd_vel</topic>",
        "<topic>scan</topic>":             f"<topic>{ns}/scan</topic>",
        "<topic>camera/image_raw</topic>": f"<topic>{ns}/camera/image_raw</topic>",
    }
    for old, new in replacements.items():
        urdf_str = urdf_str.replace(old, new)
    return urdf_str


def generate_launch_description():
    base_dir = os.path.dirname(os.path.abspath(__file__))
    warehouse_path = os.path.join(base_dir, "sih_warehouse.sdf")
    urdf_path = os.path.join(base_dir, "amr.urdf")

    with open(urdf_path, "r") as f:
        urdf_template = f.read()

    # ------------------------------------------------------------------
    # 1. Start Gazebo Harmonic with the warehouse world
    # ------------------------------------------------------------------
    gz_sim_pkg = get_package_share_directory("ros_gz_sim")
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_sim_pkg, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"-r {warehouse_path}"}.items(),
    )

    # ------------------------------------------------------------------
    # 2. Per-AMR spawn + bridge groups
    # ------------------------------------------------------------------
    fleet_actions = []

    for amr in FLEET:
        ns = amr["ns"]
        urdf_str = _namespace_urdf(urdf_template, ns)

        # ---- Robot State Publisher (namespaced TF tree) ----
        robot_state_pub = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=ns,
            parameters=[{"robot_description": urdf_str}],
            output="screen",
        )

        # ---- Spawn into Gazebo ----
        spawner = Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-string", urdf_str,
                "-name",   ns,
                "-x",      amr["x"],
                "-y",      amr["y"],
                "-z",      amr["z"],
                "-Y",      amr["yaw"],
            ],
            output="screen",
        )

        # ---- ros_gz_bridge: namespaced topic mapping ----
        #
        # Bridge syntax:  /gz_topic@ros_type[direction]gz_type
        #   ]  = ROS → GZ  (cmd_vel: ROS publishes, GZ subscribes)
        #   [  = GZ → ROS  (scan, camera: GZ publishes, ROS subscribes)
        #
        # GZ topics are rewritten by _namespace_urdf() to {ns}/...
        # ROS topics are pushed into /{ns}/... by the namespace.
        bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            namespace=ns,
            arguments=[
                f"/{ns}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
                f"/{ns}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                f"/{ns}/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            ],
            remappings=[
                (f"/{ns}/cmd_vel",          "cmd_vel"),
                (f"/{ns}/scan",             "scan"),
                (f"/{ns}/camera/image_raw", "camera/image_raw"),
            ],
            output="screen",
        )

        # ---- ros_gz_image: dedicated image bridge for compressed transports ----
        image_bridge = Node(
            package="ros_gz_image",
            executable="image_bridge",
            namespace=ns,
            arguments=[f"/{ns}/camera/image_raw"],
            remappings=[
                (f"/{ns}/camera/image_raw", "camera/image_raw"),
            ],
            output="screen",
        )

        # ---- Group all actions under this AMR's namespace ----
        group = GroupAction(
            actions=[
                LogInfo(msg=f"Spawning {ns} at ({amr['x']}, {amr['y']})"),
                robot_state_pub,
                spawner,
                bridge,
                image_bridge,
            ]
        )
        fleet_actions.append(group)

    # ------------------------------------------------------------------
    # 3. Assemble full launch description
    # ------------------------------------------------------------------
    return LaunchDescription([gazebo_sim] + fleet_actions)
