"""
@file qr_mission_scheduler.launch.py
@author Abubakarsiddiq Navid shaikh
@date 2024-10-05
@brief Auto-generated author information
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='qr_mission_scheduler',
            executable='mission_scheduler_server',
            name='mission_scheduler_server',
            output='screen'
        )
    ])
