"""Launch the headless onboard control service with the shared production parameters."""

from __future__ import annotations

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Start the same C++ control node used by both SITL and hardware deployments."""
    package_share = get_package_share_directory("onboard_control")
    parameter_file = f"{package_share}/config/control.yaml"
    return LaunchDescription(
        [
            Node(
                package="onboard_control",
                executable="onboard_control_node",
                name="onboard_control_node",
                output="screen",
                parameters=[parameter_file],
            )
        ]
    )
