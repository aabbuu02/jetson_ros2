"""
@file start_anscer_agv.launch.py
@author Abubakarsiddiq Navid shaikh
@date 2024-10-05
@brief Auto-generated author information
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # Include the launch files from the packages we have already migrated
    anscer_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('anscer_description'), 'launch', 'anscer_description.launch.py')])
    )

    lower_level_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('lower_level_controller'), 'launch', 'lower_level_controller.launch.py')])
    )

    twist_mux_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('twist_mux'), 'launch', 'twist_mux.launch.py')])
    )

    anscer_teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('anscer_teleop'), 'launch', 'anscer_teleop_key.launch.py')])
    )

    return LaunchDescription([
        # Launch all the core components
        anscer_description_launch,
        lower_level_controller_launch,
        twist_mux_launch,
        anscer_teleop_launch,

        # Start the robot pose publisher node
        Node(
            package='robot_pose_publisher',
            executable='robot_pose_publisher_node',
            name='robot_pose_publisher',
            output='screen'
        ),

        # Publish the static transform from base_footprint to base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
        )
    ])
