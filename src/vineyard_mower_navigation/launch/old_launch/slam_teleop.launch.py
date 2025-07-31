#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directories
    pkg_vineyard_mower_gazebo = get_package_share_directory('vineyard_mower_gazebo')
    pkg_vineyard_mower_navigation = get_package_share_directory('vineyard_mower_navigation')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    configuration_directory = LaunchConfiguration(
        'configuration_directory',
        default=os.path.join(pkg_vineyard_mower_navigation, 'config')
    )
    configuration_basename = LaunchConfiguration(
        'configuration_basename',
        default='vineyard_mower_2d.lua'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'configuration_directory',
            default_value=os.path.join(pkg_vineyard_mower_navigation, 'config'),
            description='Full path to config file to load'),

        DeclareLaunchArgument(
            'configuration_basename',
            default_value='vineyard_mower_2d.lua',
            description='Name of config file to load'),

        # Include robot simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_vineyard_mower_gazebo, 'launch', 'robot_simulation.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        # Cartographer node
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-configuration_directory', configuration_directory,
                '-configuration_basename', configuration_basename
            ],
            remappings=[
                ('scan', '/scan'),
                ('odom', '/odom'),
            ]
        ),

        # Cartographer occupancy grid node
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
        ),

        # Keyboard teleoperation node
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            output='screen',
            prefix='xterm -e',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[
                ('/cmd_vel', '/cmd_vel'),
            ]
        ),
    ])
