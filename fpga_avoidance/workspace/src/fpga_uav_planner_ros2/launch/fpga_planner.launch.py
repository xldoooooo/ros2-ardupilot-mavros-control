from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    share = Path(get_package_share_directory("fpga_uav_planner_ros2"))
    config = str(share / "config" / "planner_defaults.yaml")
    rviz_config = str(share / "rviz" / "fpga_uav_ros2.rviz")
    return LaunchDescription([
        DeclareLaunchArgument("mock", default_value="false"),
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
        DeclareLaunchArgument("fpga_ip", default_value=EnvironmentVariable("FPGA_IP", default_value="192.168.2.10")),
        DeclareLaunchArgument("bind_ip", default_value=EnvironmentVariable("FPGA_BIND_IP", default_value="192.168.2.100")),
        DeclareLaunchArgument("udp_port", default_value=EnvironmentVariable("FPGA_PORT", default_value="4000")),
        DeclareLaunchArgument("native_cpp_accel", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="false"),
        Node(
            package="fpga_uav_planner_ros2",
            executable="fpga_kino_planner",
            output="screen",
            parameters=[config, {
                "mock": LaunchConfiguration("mock"),
                "candidate_cpp_accel": LaunchConfiguration("candidate_cpp_accel"),
                "incremental_qubo": LaunchConfiguration("incremental_qubo"),
                "incremental_qubo_cache_size": LaunchConfiguration("incremental_qubo_cache_size"),
                "udp_broadcast_fallback": LaunchConfiguration("udp_broadcast_fallback"),
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
            }],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
            condition=IfCondition(LaunchConfiguration("rviz")),
            output="screen",
        ),
    ])
