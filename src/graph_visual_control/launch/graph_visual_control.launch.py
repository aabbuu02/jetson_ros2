"""
@file graph_visual_control.launch.py
@author Abubakarsiddiq Navid shaikh
@date 2024-10-05
@brief Auto-generated author information
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='graph_visual_control',
            executable='graph_viz_ctrl_node',
            name='graph_viz_ctrl',
            output='screen'
        )
    ])
