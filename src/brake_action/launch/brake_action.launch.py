"""
@file brake_action.launch.py
@author Abubakarsiddiq Navid shaikh
@date 2024-10-05
@brief Auto-generated author information
"""

#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('brake_action')
    
    # Brake Action Node
    brake_action_node = Node(
        package='brake_action',
        executable='brake_action_node',
        name='brake_activator',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'HigherLevelInterface.yaml'),
            os.path.join(pkg_share, 'config', 'ModbusCommunicator.yaml')
        ]
    )
    
    return LaunchDescription([
        brake_action_node
    ])
