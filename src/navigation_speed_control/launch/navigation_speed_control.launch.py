"""
@file navigation_speed_control.launch.py
@author Abubakarsiddiq Navid shaikh
@date 2024-10-05
@brief Auto-generated author information
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('navigation_speed_control')
    config_file = os.path.join(pkg_share, 'config', 'navigation_speed_control.yaml')

    return LaunchDescription([
        Node(
            package='navigation_speed_control',
            executable='navigation_speed_control_node',
            name='navigation_speed_control',
            output='screen',
            parameters=[config_file]
        )
    ])
