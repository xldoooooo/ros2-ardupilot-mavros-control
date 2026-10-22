"""Launch standalone Odin with one selected YAML and optional visualization."""

import os
from pathlib import Path
import runpy

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node



def get_odin_runtime_dir():
    """Match the writable calibration directory used by the SDK."""
    if os.environ.get("ODIN_CALIB_DIR"):
        return os.environ["ODIN_CALIB_DIR"]
    root = next(parent for parent in Path(__file__).resolve().parents
                if (parent / "project_configuration.py").is_file())
    configuration = runpy.run_path(str(root / "project_configuration.py"))
    return str(configuration["config_file"]("odin/calibration", root))


def launch_nodes(context):
    """Use the same selected configuration for the SDK and every optional helper."""
    config_file = str(Path(LaunchConfiguration("config_file").perform(context)).resolve())
    with open(config_file, encoding="utf-8") as stream:
        settings = yaml.safe_load(stream)
    if not isinstance(settings, dict) or not isinstance(settings.get("register_keys"), dict):
        raise RuntimeError(f"Odin configuration lacks register_keys: {config_file}")
    parameters = {"config_file": config_file,
                  "calib_file_path": str(Path(get_odin_runtime_dir()) / "calib.yaml")}
    nodes = [Node(package="odin_ros_driver", executable="host_sdk_sample",
                  output="screen", parameters=[parameters])]
    for switch, executable in (("senddepth", "pcd2depth_ros2_node"),
                               ("sendreprojection", "cloud_reprojection_ros2_node"),
                               ("sendoverlay", "image_overlay_node")):
        if settings["register_keys"].get(switch, 0):
            nodes.append(Node(package="odin_ros_driver", executable=executable,
                              output="screen", parameters=[settings, parameters]))
    if LaunchConfiguration("rviz").perform(context).lower() in ("true", "1"):
        nodes.append(Node(package="rviz2", executable="rviz2", output="screen",
                          arguments=["-d", LaunchConfiguration("rviz_config")]))
    return nodes


def generate_launch_description():
    """Default to odometry; keep native ros2 launch usable without project services."""
    package = Path(get_package_share_directory("odin_ros_driver"))
    root = next(parent for parent in Path(__file__).resolve().parents
                if (parent / "project_configuration.py").is_file())
    configuration = runpy.run_path(str(root / "project_configuration.py"))
    return LaunchDescription([
        SetEnvironmentVariable("ODIN_CALIB_DIR", get_odin_runtime_dir()),
        DeclareLaunchArgument("config_file", default_value=str(configuration["config_file"]("odin/odom.yaml", root)),
                              description="Full path to the selected Odin YAML"),
        DeclareLaunchArgument("rviz", default_value="false", description="Open Odin RViz"),
        DeclareLaunchArgument("rviz_config", default_value=str(package / "config/odin_ros2.rviz")),
        OpaqueFunction(function=launch_nodes),
    ])
