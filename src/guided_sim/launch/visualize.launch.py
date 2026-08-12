"""
visualize.launch.py — Launch visualization tools for the quadcopter.

Starts:
  - robot_state_publisher  (publishes URDF model joints to /tf_static)
  - pose_to_tf             (bridges the ground-station preview pose to isolated TF)
  - rviz2                  (loads the packaged quadcopter.rviz config)

Usage:
  ros2 launch guided_sim visualize.launch.py

Startup ordering:
  This launch may start before ControlStatus is available. Its pose subscription
  waits until the operator enables preview, allowing RViz to warm up in parallel.

The packaged RViz config preselects the map fixed frame, RobotModel and TF.
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Launch the URDF publisher, MAVROS pose bridge and configured RViz2."""
    pkg_share = get_package_share_directory('guided_sim')

    # Paths
    urdf_path = os.path.join(pkg_share, 'urdf', 'quadcopter.urdf')
    rviz_config = os.path.join(pkg_share, 'rviz', 'quadcopter.rviz')

    # Read URDF content for robot_state_publisher
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    # 地面预览使用独立节点名、description topic 和 TF frame prefix，避免覆盖远端 TF。
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ground_station_preview_robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 30.0,
            'frame_prefix': 'ground_station_preview/',
        }],
        remappings=[
            ('robot_description', '/ground_station_preview/robot_description'),
        ],
    )

    # 只桥接地面站聚合后的预览位姿，不增加对远端 MAVROS 标准话题的订阅。
    pose_to_tf = Node(
        package='guided_sim',
        executable='pose_to_tf.py',
        name='ground_station_preview_pose_to_tf',
        output='screen',
        parameters=[{
            'pose_topic': '/ground_station/vehicle_pose',
            'parent_frame': 'map',
            'child_frame': 'ground_station_preview/base_link',
        }],
    )

    # RViz 仅加载只读显示工具，不安装可绕过 Qt 门控的 2D/3D 命令工具。
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='ground_station_waypoint_preview_rviz',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        robot_state_publisher,
        pose_to_tf,
        rviz2,
    ])
