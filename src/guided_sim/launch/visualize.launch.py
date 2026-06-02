"""
visualize.launch.py — Launch visualization tools for the quadcopter.

Starts:
  - robot_state_publisher  (publishes URDF model joints to /tf_static)
  - pose_to_tf             (bridges /mavros/local_position/pose to TF map->base_link)
  - rviz2                  (3D visualization, default config)

Usage:
  ros2 launch guided_sim visualize.launch.py

Prerequisites:
  MAVROS must be running and publishing /mavros/local_position/pose.

After RViz2 opens, add displays manually:
  1. "Add" → "RobotModel" → set Description Topic to /robot_description
  2. "Add" → "TF" (optional, to see frames)
  3. "Add" → "Path" → set Topic to /mavros/local_position/pose
  Set "Fixed Frame" to "map" in Global Options.
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('guided_sim')

    # Paths
    urdf_path = os.path.join(pkg_share, 'urdf', 'quadcopter.urdf')

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

    # rviz2 — no config file, starts with clean defaults
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher,
        pose_to_tf,
        rviz2,
    ])
