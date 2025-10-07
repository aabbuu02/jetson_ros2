"""
@file emergency_handler.launch.py
@author Abubakarsiddiq Navid shaikh
@date 2024-10-05
@brief Auto-generated author information
"""

#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Emergency Handler Node
    emergency_handler_node = Node(
        package='emergency_handler',
        executable='emergency_handler_node',
        name='emergency_handler',
        output='screen'
    )
    
    return LaunchDescription([
        emergency_handler_node
    ])
