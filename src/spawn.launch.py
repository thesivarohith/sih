import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    base_dir = os.path.dirname(os.path.abspath(__file__))
    warehouse_path = os.path.join(base_dir, 'sih_warehouse.sdf')
    urdf_path = os.path.join(base_dir, 'amr.urdf')

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    gz_sim_pkg = get_package_share_directory('ros_gz_sim')
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_sim_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {warehouse_path}'}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-string', robot_desc,
                   '-name', 'edge_amr',
                   '-x', '7.5',
                   '-y', '13.0',
                   '-z', '0.5'],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image'
        ],
        output='screen'
    )

    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image_raw'],
        output='screen'
    )

    return LaunchDescription([
        gazebo_sim,
        robot_state_publisher,
        gz_spawn_entity,
        bridge,
        image_bridge,
    ])
