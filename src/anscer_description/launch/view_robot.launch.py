"""
@file view_robot.launch.py
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
from launch.conditions import IfCondition

def generate_launch_description():

    use_gui = LaunchConfiguration('use_gui', default='true')
    
    # Correct path to the URDF file
    urdf_file = os.path.join(
        get_package_share_directory('anscer_description'),
        'urdf',
        'anscer_amr.urdf') # <-- FILENAME CORRECTED

    # Read the URDF file content
    with open(urdf_file, 'r') as infp:
        robot_description_raw = infp.read()

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}]
    )

    # Joint state publisher GUI
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(use_gui)
    )
    
    # RViz2 node
    rviz_config_file = os.path.join(
        get_package_share_directory('anscer_description'), 'rviz', 'urdf.rviz')
        
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_gui', default_value='true'),
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
