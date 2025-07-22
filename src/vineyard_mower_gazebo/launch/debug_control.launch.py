#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directories
    pkg_vineyard_mower_description = get_package_share_directory('vineyard_mower_description')
    pkg_vineyard_mower_gazebo = get_package_share_directory('vineyard_mower_gazebo')

    # Get the urdf file
    urdf_file_name = 'vineyard_mower.urdf.xacro'
    urdf = os.path.join(pkg_vineyard_mower_description, 'urdf', urdf_file_name)

    # World file path
    world = os.path.join(pkg_vineyard_mower_gazebo, 'worlds', 'vineyard.world')

    return LaunchDescription([
        # Start Gz Sim
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('ros_gz_sim'), 'launch'),
                '/gz_sim.launch.py']),
            launch_arguments={'gz_args': ['-r -v4 ', world]}.items()
        ),

        # Clock Bridge (CRITICAL: Must start immediately)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge_clock',
            arguments=['/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'],
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True, 'robot_description': Command(['xacro ', urdf])}],
        ),

        # Spawn robot (delayed to ensure robot_state_publisher is ready)
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    name='urdf_spawner',
                    output='screen',
                    arguments=['-topic', '/robot_description',
                              '-name', 'vineyard_mower',
                              '-x', '0.0', '-y', '0.0', '-z', '0.20']),
            ]
        ),

        # Controllers (delayed to ensure robot is spawned and ros2_control plugin is loaded)
        TimerAction(
            period=8.0,
            actions=[
                # Joint State Broadcaster
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                    parameters=[{'use_sim_time': True}],
                    output='screen',
                ),
            ]
        ),

        TimerAction(
            period=10.0,
            actions=[
                # Differential Drive Controller
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
                    parameters=[{'use_sim_time': True}],
                    output='screen',
                ),
            ]
        ),

        # Basic sensor bridge
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    name='ros_gz_bridge_basic',
                    arguments=[
                        '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                    ],
                    output='screen',
                    parameters=[{'use_sim_time': True}]
                ),
            ]
        ),
    ])
