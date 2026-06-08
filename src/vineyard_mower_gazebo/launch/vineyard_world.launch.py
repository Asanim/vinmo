#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get the package directories
    pkg_vineyard_mower_gazebo = get_package_share_directory('vineyard_mower_gazebo')
    
    # Realistic vineyard world file
    world = os.path.join(pkg_vineyard_mower_gazebo, 'worlds', 'realistic_vineyard.world')

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui', default='true')
    headless = LaunchConfiguration('headless', default='false')

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
            
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Start Gazebo with GUI if true'),
            
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Start Gazebo in headless mode if true'),

        # Start Gz Sim with realistic vineyard world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('ros_gz_sim'), 'launch'),
                '/gz_sim.launch.py']),
            launch_arguments={
                'gz_args': ['-r -v4 ', world],
                'use_sim_time': use_sim_time
            }.items()
        ),
    ])
