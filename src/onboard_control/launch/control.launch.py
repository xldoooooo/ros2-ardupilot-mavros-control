"""Launch the headless onboard control service with the shared production parameters."""

from __future__ import annotations

from pathlib import Path
import runpy
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Start the same C++ control node used by both SITL and hardware deployments."""
    root = next(parent for parent in Path(__file__).resolve().parents
                if (parent / "project_configuration.py").is_file())
    configuration = runpy.run_path(str(root / "project_configuration.py"))
    parameter_file = str(configuration["config_file"]("control.yaml", root))
    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=parameter_file,
                                  description="Selected project source parameter file."),
            DeclareLaunchArgument(
                "navigation_switch_script", default_value="",
                description="Absolute ground-only navigation restart script; empty disables switching.",
            ),
            Node(
                package="onboard_control",
                executable="onboard_control_node",
                name="onboard_control_node",
                output="screen",
                parameters=[LaunchConfiguration("config_file"), {
                    "navigation_switch_script": ParameterValue(
                        LaunchConfiguration("navigation_switch_script"), value_type=str,
                    ),
                }],
            )
        ]
    )
