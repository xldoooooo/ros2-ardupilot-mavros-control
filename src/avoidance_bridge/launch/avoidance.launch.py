"""Task38 adapter with the deployed legacy planner's YAML; preserve search tuning."""
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    """Load the unqualified aircraft YAML also when launching in /avoidance."""
    config = LaunchConfiguration('config').perform(context)
    with open(config, encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    key = next(key for key in ('path_planning', '/path_planning', '/**') if key in data)
    parameters = data[key]['ros__parameters']
    mode = LaunchConfiguration('coordinate_mode').perform(context)
    # Transport/map settings match the 169 bench; search tuning comes from aircraft YAML.
    parameters.update({
        'planning_frame': 'map' if mode == 'identity' else 'odom',
        'stamp_clock': 'receive', 'service_only': True,
        'cloud.map_mode': 'latest_observation', 'cloud.blind_radius': 0.0,
        'search.lower': [-50.0, -50.0, -2.0],
    })
    return [
        Node(package='path_planning', executable='path_planning_node', namespace='avoidance',
             name='path_planning', output='screen', parameters=[parameters],
             remappings=[('cloud', '/avoidance/map')]),
        Node(package='avoidance_bridge', executable='avoidance_bridge_node', output='screen',
             parameters=[{
                 'coordinate_mode': mode,
                 'cloud_topic': LaunchConfiguration('cloud').perform(context),
                 'odometry_topic': LaunchConfiguration('odom').perform(context),
                 'preview_enabled': LaunchConfiguration('preview').perform(context).lower() == 'true',
                 'input_timeout': float(parameters.get('input_timeout', 0.5)),
             }]),
    ]


def generate_launch_description():
    config = str(Path(get_package_share_directory('path_planning')) / 'config/planner.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=config),
        DeclareLaunchArgument('coordinate_mode', default_value='extnav'),
        DeclareLaunchArgument('cloud', default_value='/odin1/cloud_slam_down'),
        DeclareLaunchArgument('odom', default_value='/odin1/odometry_highfreq'),
        DeclareLaunchArgument('preview', default_value='false'),
        OpaqueFunction(function=_nodes),
    ])
