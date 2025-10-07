"""
@file tag_monitor.launch.py
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
    pkg_share = get_package_share_directory('tag_monitor')
    
    # Declare launch arguments
    output_arg = DeclareLaunchArgument(
        'output_to',
        default_value='screen',
        description='Output to screen or log'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_share, 'config', 'tag_monitor.yaml'),
        description='Path to the config file'
    )
    
    # Tag Monitor Node
    tag_monitor_node = Node(
        package='tag_monitor',
        executable='tag_monitor_node',
        name='tag_monitor',
        output=LaunchConfiguration('output_to'),
        parameters=[LaunchConfiguration('config_file')]
    )
    
    return LaunchDescription([
        output_arg,
        config_file_arg,
        tag_monitor_node
    ])
