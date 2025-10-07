"""
@file graph_server.launch.py
@author Abubakarsiddiq Navid shaikh
@date 2024-10-05
@brief Auto-generated author information
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'graph_file',
            default_value='',
            description='Path to the graphml file (e.g., package://graph_server/graphs/arapl_v6.graphml)'
        ),
        Node(
            package='graph_server',
            executable='graph_server_node',
            name='graph_server',
            output='screen',
            parameters=[
                {'graph_file': LaunchConfiguration('graph_file')}
            ]
        )
    ])
