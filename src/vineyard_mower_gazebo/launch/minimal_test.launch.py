#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
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

    # Control config file
    control_config = os.path.join(pkg_vineyard_mower_description, 'config', 'control_config.yaml')

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'x_pose', default_value='0.0',
            description='x position of robot'),

        DeclareLaunchArgument(
            'y_pose', default_value='0.0',
            description='y position of robot'),

        # Start Gz Sim
        ExecuteProcess(
            cmd=['gz', 'sim', world, '-v', '4'],
            output='screen'
        ),

        # Clock bridge (critical - must start early)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge_clock',
            arguments=['/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'],
            output='screen'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': Command(['xacro ', urdf])}],
            arguments=[urdf]),

        # Spawn robot
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    name='urdf_spawner',
                    output='screen',
                    arguments=['-topic', '/robot_description',
                              '-name', 'vineyard_mower',
                              '-x', x_pose, '-y', y_pose, '-z', '0.20']),
            ]
        ),

        # Controller Manager (delayed to ensure clock sync)
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='ros2_control_node',
                    parameters=[{'use_sim_time': use_sim_time, 'robot_description': Command(['xacro ', urdf])}, control_config],
                    output='screen',
                ),
            ]
        ),

        # Controllers (delayed further)
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_broadcaster'],
                    parameters=[{'use_sim_time': use_sim_time}],
                    output='screen',
                ),
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['diff_drive_controller'],
                    parameters=[{'use_sim_time': use_sim_time}],
                    output='screen',
                ),
            ]
        ),

        # Sensor bridges
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge_sensors',
            arguments=[
                '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU'
            ],
            output='screen'
        ),

        # Teleoperation for testing
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='teleop_twist_keyboard',
                    executable='teleop_twist_keyboard',
                    name='teleop_keyboard',
                    output='screen',
                    prefix='xterm -e',
                    remappings=[('/cmd_vel', '/diff_drive_controller/cmd_vel_unstamped')],
                ),
            ]
        ),
    ])
