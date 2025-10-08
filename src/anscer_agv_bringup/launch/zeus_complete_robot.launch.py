#!/usr/bin/env python3
"""
Complete Zeus Robot System Launch File
This launches all necessary components for a fully functional AMR
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directories
    pkg_zeus_bringup = FindPackageShare(package='anscer_agv_bringup').find('anscer_agv_bringup')
    pkg_emergency_handler = FindPackageShare(package='emergency_handler').find('emergency_handler')
    pkg_tag_monitor = FindPackageShare(package='tag_monitor').find('tag_monitor')
    pkg_global_planner = FindPackageShare(package='global_planner').find('global_planner')
    pkg_lower_level_controller = FindPackageShare(package='lower_level_controller').find('lower_level_controller')
    pkg_twist_mux = FindPackageShare(package='twist_mux').find('twist_mux')
    pkg_navigation = FindPackageShare(package='nav2_bringup').find('nav2_bringup')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    map_file = LaunchConfiguration('map_file')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack')
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_zeus_bringup, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use')
    
    declare_map_file_cmd = DeclareLaunchArgument(
        'map_file',
        default_value=os.path.join(pkg_zeus_bringup, 'maps', 'warehouse_map.yaml'),
        description='Full path to map file to load')

    # Robot State Publisher
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')])

    # Emergency Handler
    emergency_handler_cmd = Node(
        package='emergency_handler',
        executable='emergency_handler_node',
        name='emergency_handler',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}])

    # Tag Monitor
    tag_monitor_cmd = Node(
        package='tag_monitor',
        executable='tag_monitor_node',
        name='tag_monitor',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}])

    # Global Planner
    global_planner_cmd = Node(
        package='global_planner',
        executable='path_graph_planner_node',
        name='global_planner',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}])

    # Lower Level Controller (Motor Control)
    lower_level_controller_cmd = Node(
        package='lower_level_controller',
        executable='lower_level_controller_node',
        name='lower_level_controller',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'controller_type': 1,  # Roboteq
            'roboteq_port': '/dev/roboteq',
            'plc_ip': '192.168.1.125',
            'plc_port': 502
        }])

    # Twist Mux
    twist_mux_cmd = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[os.path.join(pkg_twist_mux, 'config', 'twist_mux.yaml')])

    # Navigation Stack
    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_navigation, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
            'map': map_file
        }.items())

    # Robot Pose Publisher
    robot_pose_publisher_cmd = Node(
        package='robot_pose_publisher',
        executable='robot_pose_publisher',
        name='robot_pose_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}])

    # Power Control
    power_control_cmd = Node(
        package='power_control',
        executable='power_control_node',
        name='power_control',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}])

    # QR Navigation
    qr_navigation_cmd = Node(
        package='qr_navigation',
        executable='qr_navigation_node',
        name='qr_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}])

    # Mission Scheduler
    mission_scheduler_cmd = Node(
        package='qr_mission_scheduler',
        executable='mission_scheduler_node',
        name='mission_scheduler',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_map_file_cmd)

    # Add the commands
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(emergency_handler_cmd)
    ld.add_action(tag_monitor_cmd)
    ld.add_action(global_planner_cmd)
    ld.add_action(lower_level_controller_cmd)
    ld.add_action(twist_mux_cmd)
    ld.add_action(robot_pose_publisher_cmd)
    ld.add_action(power_control_cmd)
    ld.add_action(qr_navigation_cmd)
    ld.add_action(mission_scheduler_cmd)
    ld.add_action(nav2_bringup_cmd)

    return ld