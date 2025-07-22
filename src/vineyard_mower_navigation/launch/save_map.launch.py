#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directories
    pkg_vineyard_mower_navigation = get_package_share_directory('vineyard_mower_navigation')
    
    # Launch configuration variables
    map_name = LaunchConfiguration('map_name', default='vineyard_map')
    map_directory = LaunchConfiguration(
        'map_directory',
        default=os.path.join(pkg_vineyard_mower_navigation, 'maps')
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_name',
            default_value='vineyard_map',
            description='Name of the map file (without extension)'),

        DeclareLaunchArgument(
            'map_directory',
            default_value=os.path.join(pkg_vineyard_mower_navigation, 'maps'),
            description='Directory to save the map'),

        # Map saver node
        Node(
            package='nav2_map_server',
            executable='map_saver_cli',
            name='map_saver',
            output='screen',
            arguments=[
                '-f', [map_directory, '/', map_name],
                '--ros-args', '--log-level', 'info'
            ]
        ),
    ])
