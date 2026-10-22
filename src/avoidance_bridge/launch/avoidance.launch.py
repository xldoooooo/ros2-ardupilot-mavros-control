"""Launch the native dual-tree planner and its coordinate/goal adapter."""
from pathlib import Path
import runpy
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    """Use the native planner tuning; the adapter only selects input and coordinate interfaces."""
    helper = runpy.run_path(str(Path(get_package_share_directory('path_planning')) /
                                'launch/parameter_config.py'))
    data = helper['load_config'](LaunchConfiguration('config').perform(context))
    parameters = helper['node_parameters'](
        data, ('/**', 'path_planning', '/path_planning'), {})
    topics = helper['node_parameters'](data, ('odin_launch',), {
        'cloud': '/odin1/cloud_slam_down',
    })
    bridge = helper['node_parameters'](data, ('avoidance_bridge', '/avoidance_bridge'), {
        'coordinate_mode': 'extnav',
        'odometry_topic': '/mavros/local_position/odom', 'preview_enabled': False,
    })
    bridge['coordinate_mode'] = helper['launch_value'](context, 'coordinate_mode', bridge['coordinate_mode'])
    cloud = helper['launch_value'](context, 'cloud', topics['cloud'])
    bridge['odometry_topic'] = helper['launch_value'](context, 'odom', bridge['odometry_topic'])
    if context.launch_configurations.get('preview'):
        raw = context.launch_configurations['preview'].lower()
        if raw not in ('true', 'false'):
            raise ValueError('preview must be true or false')
        bridge['preview_enabled'] = raw == 'true'
    # Identity is an explicit simulation coordinate selection; callers can still override the frame.
    if context.launch_configurations.get('coordinate_mode') == 'identity':
        parameters['planning_frame'] = 'map'
    parameters = helper['planner_overrides'](context, parameters)
    bridge.setdefault('input_timeout', parameters.get('input_timeout', 0.5))
    bridge['planning_frame'] = parameters.get('planning_frame', 'odom')
    # Bridge-only tuning uses bridge.<parameter> to avoid collisions with planner parameters.
    for key, raw in context.launch_configurations.items():
        if key.startswith('bridge.') and raw:
            name = key.removeprefix('bridge.')
            value = yaml.safe_load(raw)
            if isinstance(bridge.get(name), float) and isinstance(value, (int, float)):
                value = float(value)
            bridge[name] = value
    return [
        Node(package='path_planning', executable='path_planning_node', namespace='avoidance',
             name='path_planning', output='screen', parameters=[parameters],
             remappings=[('cloud', cloud), ('odom', '/avoidance/odom'),
                         ('goal', '/avoidance/goal')]),
        Node(package='avoidance_bridge', executable='avoidance_bridge_node', output='screen',
             parameters=[bridge]),
    ]


def generate_launch_description():
    """Declare optional overrides; the selected YAML remains the configuration authority."""
    root = next(parent for parent in Path(__file__).resolve().parents
                if (parent / 'project_configuration.py').is_file())
    configuration = runpy.run_path(str(root / 'project_configuration.py'))
    config = str(configuration['config_file']('avoidance/planner.yaml', root))
    arguments = [DeclareLaunchArgument('config', default_value=config)]
    arguments.extend(DeclareLaunchArgument(name, default_value='') for name in (
        'coordinate_mode', 'cloud', 'odom', 'preview', 'blind_radius', 'map_mode',
        'planning_frame', 'stamp_clock', 'search_lower',
    ))
    return LaunchDescription([*arguments, OpaqueFunction(function=_nodes)])
