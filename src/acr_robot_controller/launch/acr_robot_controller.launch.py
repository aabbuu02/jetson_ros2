"""
@file acr_robot_controller.launch.py
@author Abubakarsiddiq Navid shaikh
@date 2024-10-05
@brief Auto-generated author information
"""

#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('acr_robot_controller')
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_share, 'config', 'acr_robot_controller.yaml'),
        description='Path to the config file'
    )
    
    # ACR Robot Controller Node
    acr_robot_controller_node = Node(
        package='acr_robot_controller',
        executable='acr_robot_controller',
        name='acr_robot_controller',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        respawn=False
    )
    
    return LaunchDescription([
        config_file_arg,
        acr_robot_controller_node
    ])
