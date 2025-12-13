#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch MAVROS node configured for ArduPilot SITL connection.

    ArduPilot SITL default ports:
    - 14550: First MAVLink output (GCS)
    - 14551: Second MAVLink output
    """

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'fcu_url',
            default_value='udp://:14550@',
            description='FCU connection URL for ArduPilot SITL'
        ),

        DeclareLaunchArgument(
            'gcs_url',
            default_value='',
            description='GCS connection URL (leave empty if not needed)'
        ),

        DeclareLaunchArgument(
            'tgt_system',
            default_value='1',
            description='Target system ID'
        ),

        DeclareLaunchArgument(
            'tgt_component',
            default_value='1',
            description='Target component ID'
        ),

        DeclareLaunchArgument(
            'log_output',
            default_value='screen',
            description='Output destination for logs'
        ),

        DeclareLaunchArgument(
            'fcu_protocol',
            default_value='v2.0',
            description='MAVLink protocol version'
        ),

        # MAVROS node
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            output=LaunchConfiguration('log_output'),
            parameters=[{
                'fcu_url': LaunchConfiguration('fcu_url'),
                'gcs_url': LaunchConfiguration('gcs_url'),
                'target_system_id': LaunchConfiguration('tgt_system'),
                'target_component_id': LaunchConfiguration('tgt_component'),
                'fcu_protocol': LaunchConfiguration('fcu_protocol'),
            }]
        ),
    ])
