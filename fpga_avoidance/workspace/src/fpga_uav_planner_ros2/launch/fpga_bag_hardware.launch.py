import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def _local_bag_default() -> str:
    configured = os.environ.get("FPGA_UAV_BAG", "")
    if configured:
        return configured
    source_file = Path(__file__).resolve()
    candidates = [
        source_file.parents[4] / "data" / "full_ros2_mybag",
        Path("/home/nvidia/UAV_Project_ROS2/data/full_ros2_mybag"),
    ]
    for candidate in candidates:
        if (candidate / "metadata.yaml").is_file():
            return str(candidate)
    return "BAG_PATH_REQUIRED"


def generate_launch_description():
    share = Path(get_package_share_directory("fpga_uav_planner_ros2"))
    planner_config = str(share / "config" / "planner_defaults.yaml")
    rviz_config = str(share / "rviz" / "fpga_uav_ros2.rviz")

    goal_yaml = (
        "{header: {frame_id: camera_init}, pose: {position: {x: 5.0, y: 0.0, z: 1.0}, "
        "orientation: {w: 1.0}}}"
    )

    return LaunchDescription([
        DeclareLaunchArgument("bag_path", default_value=_local_bag_default()),
        DeclareLaunchArgument("rate", default_value="0.3"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("fpga_ip", default_value=EnvironmentVariable("FPGA_IP", default_value="192.168.2.10")),
        DeclareLaunchArgument("bind_ip", default_value=EnvironmentVariable("FPGA_BIND_IP", default_value="192.168.2.100")),
        DeclareLaunchArgument("udp_port", default_value=EnvironmentVariable("FPGA_PORT", default_value="4000")),
        DeclareLaunchArgument("cloud_replan_min_period_s", default_value="0.10"),
        DeclareLaunchArgument("cloud_defer_during_solve", default_value="true"),
        DeclareLaunchArgument("replan_distance_m", default_value="0.25"),
        DeclareLaunchArgument("replan_period_s", default_value="0.0"),
        DeclareLaunchArgument("solver_persistent_worker", default_value="true"),
        DeclareLaunchArgument("solver_gc_guard", default_value="true"),
        DeclareLaunchArgument("solver_preflight_ping", default_value="true"),
        DeclareLaunchArgument("clearance_cache_policy", default_value="versioned"),
        DeclareLaunchArgument("solver_cpu_affinity", default_value=EnvironmentVariable("FPGA_SOLVER_CPU", default_value="-1")),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="camera_init_to_odom_tf",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--yaw", "0", "--pitch", "0", "--roll", "0",
                "--frame-id", "camera_init", "--child-frame-id", "odom",
            ],
            output="screen",
        ),
        Node(
            package="fpga_uav_planner_ros2",
            executable="fpga_kino_planner",
            output="screen",
            parameters=[planner_config, {
                "mock": False,
                "fpga_ip": LaunchConfiguration("fpga_ip"),
                "bind_ip": LaunchConfiguration("bind_ip"),
                "udp_port": LaunchConfiguration("udp_port"),
                "cloud_replan_min_period_s": LaunchConfiguration("cloud_replan_min_period_s"),
                "cloud_defer_during_solve": LaunchConfiguration("cloud_defer_during_solve"),
                "replan_distance_m": LaunchConfiguration("replan_distance_m"),
                "replan_period_s": LaunchConfiguration("replan_period_s"),
                "solver_persistent_worker": LaunchConfiguration("solver_persistent_worker"),
                "solver_gc_guard": LaunchConfiguration("solver_gc_guard"),
                "solver_preflight_ping": LaunchConfiguration("solver_preflight_ping"),
                "clearance_cache_policy": LaunchConfiguration("clearance_cache_policy"),
                "solver_cpu_affinity": LaunchConfiguration("solver_cpu_affinity"),
                "native_cpp_accel": True,
                "udp_broadcast_fallback": False,
                "candidate_cpp_accel": True,
                "incremental_qubo": True,
            }],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
            condition=IfCondition(LaunchConfiguration("rviz")),
            output="screen",
        ),
        ExecuteProcess(
            cmd=[
                "ros2", "bag", "play", LaunchConfiguration("bag_path"),
                "--rate", LaunchConfiguration("rate"),
                "--delay", "1.0", "--disable-keyboard-controls",
                "--remap",
                "/odin1/odometry_highfreq:=/mavros/odometry/out",
                "/odin1/cloud_slam:=/cloud_registered",
            ],
            output="screen",
        ),
        TimerAction(
            period=2.0,
            actions=[ExecuteProcess(
                cmd=[
                    "ros2", "topic", "pub", "--once", "/goal",
                    "geometry_msgs/msg/PoseStamped", goal_yaml,
                ],
                output="screen",
            )],
        ),
    ])
