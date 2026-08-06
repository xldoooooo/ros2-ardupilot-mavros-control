"""
visualize.launch.py — Launch visualization tools for the quadcopter.

Starts:
  - robot_state_publisher  (publishes URDF model joints to /tf_static)
  - pose_to_tf             (bridges /mavros/local_position/pose to TF map->base_link)
  - rviz2                  (loads the packaged quadcopter.rviz config)

Usage:
  ros2 launch guided_sim visualize.launch.py

Prerequisites:
  MAVROS must be running and publishing /mavros/local_position/pose.

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

    # robot_state_publisher — publishes /robot_description + /tf_static
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 30.0,
        }],
    )

    # pose_to_tf bridge — subscribes /mavros/local_position/pose → TF map->base_link
    pose_to_tf = Node(
        package='guided_sim',
        executable='pose_to_tf.py',
        name='pose_to_tf',
        output='screen',
    )

    # rviz2 — load package config (RobotModel + TF + Fixed Frame=map)
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        robot_state_publisher,
        pose_to_tf,
        rviz2,
    ])
