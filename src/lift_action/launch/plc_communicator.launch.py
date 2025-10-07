"""
@file plc_communicator.launch.py
@author Abubakarsiddiq Navid shaikh
@date 2024-10-05
@brief Auto-generated author information
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to the package's share directory
    pkg_share = get_package_share_directory('lift_action')

    # Get the path to the YAML config file
    config_file = os.path.join(pkg_share, 'config', 'HigherLevelInterface.yaml')

    return LaunchDescription([
        Node(
            package='lift_action',
            executable='higher_level_interface_node',
            name='higher_level_interface',
            parameters=[config_file],
            output='screen'
        ),
        # You can add the other nodes here as well if they need to be launched together
        # Node(
        #     package='lift_action',
        #     executable='modbus_communicator_node',
        #     name='modbus_communicator'
        # ),
        # Node(
        #     package='lift_action',
        #     executable='ui_interface_node',
        #     name='ui_interface'
        # )
    ])
