"""Local SITL scan, map adapter and planner, started together by the GUI workflow."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """Compose only in the isolated simulation domain, before creating any demo process."""
    if os.environ.get('ROS_DOMAIN_ID') != '231' or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
        raise RuntimeError('Avoidance demo is only available in SITL domain 231/LOCALHOST')
    bridge_launch = os.path.join(get_package_share_directory('avoidance_bridge'), 'launch', 'avoidance.launch.py')
    return LaunchDescription([
        Node(package='guided_sim', executable='simulation_obstacles.py', output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(bridge_launch), launch_arguments={
            'coordinate_mode': 'identity', 'cloud': '/simulation/scan',
            'odom': '/simulation/scan_odom', 'preview': 'true',
        }.items()),
    ])
