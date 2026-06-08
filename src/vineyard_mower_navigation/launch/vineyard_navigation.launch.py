#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directories
    pkg_vineyard_mower_navigation = get_package_share_directory('vineyard_mower_navigation')

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    slam = LaunchConfiguration('slam', default='False')
    navigation = LaunchConfiguration('navigation', default='True')
    use_rviz = LaunchConfiguration('use_rviz', default='True')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    map_yaml_file = LaunchConfiguration('map_yaml_file', default='')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart', default='true')

    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether to run SLAM or localization')

    declare_navigation_cmd = DeclareLaunchArgument(
        'navigation',
        default_value='True',
        description='Whether to run navigation')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(pkg_vineyard_mower_navigation, 'config', 'nav2_default_view.rviz'),
        description='Full path to the RVIZ config file to use')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map_yaml_file',
        default_value='',
        description='Full path to map yaml file to load (only used in localization mode)')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_vineyard_mower_navigation, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    # If slam is enabled, start SLAM
    start_slam_cmd = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_vineyard_mower_navigation, 'launch', 'slam.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
            condition=IfCondition(slam)
        )
    ])

    # If slam is not enabled, start localization
    start_localization_cmd = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_vineyard_mower_navigation, 'launch', 'localization.launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'autostart': autostart}.items(),
            condition=UnlessCondition(slam)
        ),

        # Map server (only needed for localization)
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'yaml_filename': map_yaml_file}],
            condition=UnlessCondition(slam)
        ),

        # Lifecycle manager for map server
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': ['map_server']}],
            condition=UnlessCondition(slam)
        )
    ])

    # Start navigation if enabled
    start_navigation_cmd = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_vineyard_mower_navigation, 'launch', 'navigation.launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'autostart': autostart}.items(),
            condition=IfCondition(navigation)
        )
    ])

    # Start RViz
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_rviz))

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_navigation_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)

    # Add the actions to launch SLAM/localization and navigation
    ld.add_action(start_slam_cmd)
    ld.add_action(start_localization_cmd)
    ld.add_action(start_navigation_cmd)
    ld.add_action(start_rviz_cmd)

    return ld
