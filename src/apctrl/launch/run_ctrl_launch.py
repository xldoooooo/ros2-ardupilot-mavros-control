#-*- coding: utf-8 -*-

from launch import LaunchDescription
from launch_ros.actions import Node

from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # 获取包路径
    pkg_share = FindPackageShare('apctrl').find('apctrl')
    # 获取配置文件路径
    config_path = os.path.join(pkg_share, 'config', 'ctrl_param_fpv.yaml')
    # 创建节点
    apctrl_node = Node(
        package='apctrl',
        executable='apctrl_node',
        name='apctrl_node',
        output='screen',
        parameters=[config_path],
        remappings=[
            ('odom', '/mavros/local_position/odom'),
            ('cmd', '/position_cmd')
        ]
    )
    # 返回启动描述
    return LaunchDescription([
        apctrl_node
    ])