"""
@file lower_level_controller.launch.py
@author Abubakarsiddiq Navid shaikh
@date 2024-10-05
@brief Auto-generated author information
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('lower_level_controller')
    config_file = os.path.join(pkg_share, 'config', 'lower_level_controller.yaml')

    return LaunchDescription([
        Node(
            package='lower_level_controller',
            executable='lower_level_controller_node',
            name='lower_level_controller',
            output='screen',
            parameters=[config_file]
        )
    ])
