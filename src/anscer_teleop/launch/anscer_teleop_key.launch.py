"""
@file anscer_teleop_key.launch.py
@author Abubakarsiddiq Navid shaikh
@date 2024-10-05
@brief Auto-generated author information
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='anscer_teleop',
            executable='anscer_teleop_key',
            name='anscer_teleop_key',
            output='screen',
            prefix='xterm -e' # This will open a new terminal window for keyboard input
        )
    ])
