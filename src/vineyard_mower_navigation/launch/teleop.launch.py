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
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'teleop_method',
            default_value='keyboard',
            choices=['keyboard', 'joystick'],
            description='Method for teleoperation: keyboard or joystick'),

        # Include robot simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_vineyard_mower_gazebo, 'launch', 'robot_simulation.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        # Keyboard teleoperation node
        Node(
            condition=LaunchConfiguration('teleop_method').equals('keyboard'),
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

        # Joystick driver node
        Node(
            condition=LaunchConfiguration('teleop_method').equals('joystick'),
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # Joystick teleoperation node
        Node(
            condition=LaunchConfiguration('teleop_method').equals('joystick'),
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[
                ('/cmd_vel', '/cmd_vel'),
            ]
        ),
    ])
