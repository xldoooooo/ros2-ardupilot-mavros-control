"""Launch only the independent onboard map adapter and service-only planner."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    mode = LaunchConfiguration('coordinate_mode')
    frame = PythonExpression(["'map' if '", mode, "' == 'identity' else 'odom'"])
    return LaunchDescription([
        DeclareLaunchArgument('coordinate_mode', default_value='extnav'),
        DeclareLaunchArgument('cloud', default_value='/odin1/cloud_slam'),
        DeclareLaunchArgument('odom', default_value='/odin1/odometry_highfreq'),
        DeclareLaunchArgument('preview', default_value='false'),
        Node(package='path_planning', executable='path_planning_node', namespace='avoidance', name='path_planning',
             output='screen', parameters=[{
                 'planning_frame': ParameterValue(frame, value_type=str), 'stamp_clock': 'receive',
                 'service_only': True, 'cloud.map_mode': 'latest_observation', 'cloud.blind_radius': 0.0,
                 # Independent physical limits match current trapezoidal waypoint references.
                 'search.max_vel': 1.0, 'search.max_acc': 0.35,
                 'search.max_horizontal_vel': 1.0, 'search.max_vertical_vel': 0.2,
                 'search.max_horizontal_acc': 0.35, 'search.max_vertical_acc': 0.15,
                 # Low acceleration needs sufficient travel to escape the 0.1m search hash cell.
                 'search.max_tau': 1.0,
                 'search.safe_distance': 0.45, 'search.voxel_size': 0.1, 'search.collision_step': 0.05,
                 'search.search_budget': 0.3, 'input_timeout': 1.0,
                 'search.lower': [-50.0, -50.0, -2.0], 'search.upper': [50.0, 50.0, 10.0],
             }], remappings=[('cloud', '/avoidance/map')]),
        Node(package='avoidance_bridge', executable='avoidance_bridge_node', output='screen', parameters=[{
            'coordinate_mode': ParameterValue(mode, value_type=str),
            'cloud_topic': LaunchConfiguration('cloud'), 'odometry_topic': LaunchConfiguration('odom'),
            'preview_enabled': ParameterValue(LaunchConfiguration('preview'), value_type=bool),
            'input_timeout': 1.0,
        }]),
    ])
