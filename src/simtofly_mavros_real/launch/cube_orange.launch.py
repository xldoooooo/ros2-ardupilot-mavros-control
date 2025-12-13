#!/usr/bin/env python3
"""
MAVROS Launch File for Real Hardware (Cube Orange)
---------------------------------------------------
Author: Sidharth Mohan Nair
GitHub: https://github.com/sidharthmohannair

Connection: USB Serial to Cube Orange
Device: /dev/ttyACM0 (or /dev/cube_orange if udev rule set)
Baud: 921600
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch MAVROS node configured for Cube Orange via USB serial.

    Prerequisites:
    1. Cube Orange SERIAL0_PROTOCOL = 2 (MAVLink2)
    2. Cube Orange SERIAL0_BAUD = 921600
    3. USB cable connected: Cube Orange -> Raspberry Pi
    4. User in dialout group: sudo usermod -a -G dialout $USER
    """

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'fcu_url',
            default_value='/dev/ttyACM0:921600',
            description='FCU serial device and baud rate'
        ),

        DeclareLaunchArgument(
            'gcs_url',
            default_value='',
            description='GCS connection URL (optional)'
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
