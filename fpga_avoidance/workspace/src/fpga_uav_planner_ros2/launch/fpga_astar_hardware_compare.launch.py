import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
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
    compare_config = str(share / "config" / "compare_defaults.yaml")
    astar_config = str(share / "config" / "astar_defaults.yaml")
    rviz_config = str(share / "rviz" / "fpga_uav_ros2.rviz")

    return LaunchDescription([
        DeclareLaunchArgument("bag_path", default_value=_local_bag_default()),
        DeclareLaunchArgument("rate", default_value="0.3"),
        DeclareLaunchArgument("loop", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("fpga_ip", default_value=EnvironmentVariable("FPGA_IP", default_value="192.168.2.10")),
        DeclareLaunchArgument("bind_ip", default_value=EnvironmentVariable("FPGA_BIND_IP", default_value="192.168.2.100")),
        DeclareLaunchArgument("udp_port", default_value=EnvironmentVariable("FPGA_PORT", default_value="4000")),
        DeclareLaunchArgument("output_csv", default_value="/tmp/fpga_uav_ros2/fpga_vs_astar.csv"),
        DeclareLaunchArgument("summary_csv", default_value="/tmp/fpga_uav_ros2/fpga_vs_astar_summary.csv"),
        DeclareLaunchArgument("native_cpp_accel", default_value="true"),
        DeclareLaunchArgument("candidate_cpp_accel", default_value="true"),
        DeclareLaunchArgument("incremental_qubo", default_value="true"),
        DeclareLaunchArgument("incremental_qubo_cache_size", default_value="128"),
        DeclareLaunchArgument("udp_broadcast_fallback", default_value="false"),
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

        # Original kinodynamic A*.
        Node(
            package="path_planning_cpp_ros2",
            executable="path_planning_node",
            output="screen",
            parameters=[astar_config],
        ),

        # Real FPGA planner: mock is explicitly disabled.
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
                "native_cpp_accel": LaunchConfiguration("native_cpp_accel"),
                "candidate_cpp_accel": LaunchConfiguration("candidate_cpp_accel"),
                "incremental_qubo": LaunchConfiguration("incremental_qubo"),
                "incremental_qubo_cache_size": LaunchConfiguration("incremental_qubo_cache_size"),
                "udp_broadcast_fallback": LaunchConfiguration("udp_broadcast_fallback"),
            }],
        ),

        # Common metrics recorder and visualization markers.
        Node(
            package="fpga_uav_planner_ros2",
            executable="fpga_astar_compare",
            output="screen",
            parameters=[compare_config, {
                "bag_rate": LaunchConfiguration("rate"),
                "output_csv": LaunchConfiguration("output_csv"),
                "summary_csv": LaunchConfiguration("summary_csv"),
            }],
        ),

        # Publishes the same goal after odometry is available.
        Node(
            package="fpga_uav_planner_ros2",
            executable="publish_demo_goal",
            output="screen",
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
            condition=UnlessCondition(LaunchConfiguration("loop")),
            output="screen",
        ),
        ExecuteProcess(
            cmd=[
                "ros2", "bag", "play", LaunchConfiguration("bag_path"),
                "--rate", LaunchConfiguration("rate"),
                "--delay", "1.0", "--disable-keyboard-controls", "--loop",
                "--remap",
                "/odin1/odometry_highfreq:=/mavros/odometry/out",
                "/odin1/cloud_slam:=/cloud_registered",
            ],
            condition=IfCondition(LaunchConfiguration("loop")),
            output="screen",
        ),
    ])
