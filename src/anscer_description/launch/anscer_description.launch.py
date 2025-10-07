"""
@file anscer_description.launch.py
@author Abubakarsiddiq Navid shaikh
@date 2024-10-05
@brief Auto-generated author information
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # Correct path to the URDF file
    urdf_file = os.path.join(
        get_package_share_directory('anscer_description'),
        'urdf',
        'anscer_amr.urdf') # <-- FILENAME CORRECTED

    # Read the URDF file content
    with open(urdf_file, 'r') as infp:
        robot_description_raw = infp.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_raw}]
        )
    ])
