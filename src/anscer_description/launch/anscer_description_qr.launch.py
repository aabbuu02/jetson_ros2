"""
@file anscer_description_qr.launch.py
@author Abubakarsiddiq Navid shaikh
@date 2024-10-05
@brief Auto-generated author information
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Get paths
    pkg_share = get_package_share_directory('anscer_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'anscer_qr.xacro') # The only change is this line
    
    # Process xacro file
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Create launch description
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description_raw,
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        )
    ])
