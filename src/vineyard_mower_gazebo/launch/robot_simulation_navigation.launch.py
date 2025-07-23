#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    # Get the package directories
    pkg_vineyard_mower_description = get_package_share_directory('vineyard_mower_description')
    pkg_vineyard_mower_gazebo = get_package_share_directory('vineyard_mower_gazebo')
    pkg_vineyard_mower_navigation = get_package_share_directory('vineyard_mower_navigation')

    # Get the urdf file
    urdf_file_name = 'vineyard_mower.urdf.xacro'
    urdf = os.path.join(pkg_vineyard_mower_description, 'urdf', urdf_file_name)

    # World file path
    world = os.path.join(pkg_vineyard_mower_gazebo, 'worlds', 'vineyard.world')

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    enable_teleop = LaunchConfiguration('enable_teleop', default='true')
    enable_slam = LaunchConfiguration('enable_slam', default='false')
    enable_rviz = LaunchConfiguration('enable_rviz', default='true')
    enable_navigation = LaunchConfiguration('enable_navigation', default='false')

    return LaunchDescription([
        # Launch Arguments
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

        DeclareLaunchArgument(
            'enable_teleop', default_value='true',
            description='Enable teleoperation'),

        DeclareLaunchArgument(
            'enable_slam', default_value='false',
            description='Enable SLAM mapping'),

        DeclareLaunchArgument(
            'enable_rviz', default_value='true',
            description='Enable RViz visualization'),

        DeclareLaunchArgument(
            'enable_navigation', default_value='false',
            description='Enable autonomous navigation (Nav2)'),

        # Gazebo Simulation Group
        GroupAction([
            # Clock Bridge (MUST BE FIRST!)
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='ros_gz_bridge_clock',
                arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
                output='screen',
                parameters=[{'use_sim_time': True}],
                remappings=[
                    ('/clock', '/clock')
                ]
            ),

            # Start Gz Sim
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(get_package_share_directory('ros_gz_sim'), 'launch'),
                    '/gz_sim.launch.py']),
                launch_arguments={'gz_args': ['-r -v4 ', world]}.items()
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
            Node(
                package='ros_gz_sim',
                executable='create',
                name='urdf_spawner',
                output='screen',
                arguments=['-topic', '/robot_description',
                          '-name', 'vineyard_mower',
                          '-x', x_pose, '-y', y_pose, '-z', '0.20']),
        ]),

        # Controllers (delayed to ensure Gazebo and robot are ready)
        TimerAction(
            period=5.0,
            actions=[
                GroupAction([
                    # Joint State Broadcaster
                    Node(
                        package='controller_manager',
                        executable='spawner',
                        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                        # parameters=[{'use_sim_time': use_sim_time}],
                parameters=[{'use_sim_time': True}],
                        output='screen',
                    ),

                    # Differential Drive Controller
                    Node(
                        package='controller_manager',
                        executable='spawner',
                        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
                        # parameters=[{'use_sim_time': use_sim_time}],
                parameters=[{'use_sim_time': True}],
                        output='screen',
                    ),
                ])
            ]
        ),

        # Sensor Bridge Group (delayed to ensure clock is established)
        TimerAction(
            period=2.0,
            actions=[
                GroupAction([
                    # ROS-Gz Bridge for all sensor data
                    Node(
                        package='ros_gz_bridge',
                        executable='parameter_bridge',
                        name='ros_gz_bridge_sensors',
                        arguments=[
                            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                            '/front_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
                            '/front_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
                            '/front_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                            '/rear_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
                            '/rear_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
                            '/rear_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU'
                        ],
                        output='screen',
                        parameters=[{'use_sim_time': use_sim_time}]
                    ),

                    # Transform for sensors
                    Node(
                        package='tf2_ros',
                        executable='static_transform_publisher',
                        name='base_link_to_lidar',
                        arguments=['0', '0', '0.15', '0', '0', '0', 'base_link', 'lidar_link'],
                        parameters=[{'use_sim_time': use_sim_time}]
                    ),

                    Node(
                        package='tf2_ros',
                        executable='static_transform_publisher',
                        name='base_link_to_front_camera',
                        arguments=['0.2', '0', '0.1', '0', '0', '0', 'base_link', 'front_camera_link'],
                        parameters=[{'use_sim_time': use_sim_time}]
                    ),

                    Node(
                        package='tf2_ros',
                        executable='static_transform_publisher',
                        name='base_link_to_rear_camera',
                        arguments=['-0.2', '0', '0.1', '0', '0', '3.14159', 'base_link', 'rear_camera_link'],
                        parameters=[{'use_sim_time': use_sim_time}]
                    ),
                ])
            ]
        ),

        # Teleoperation Group
        GroupAction([
            # Keyboard Teleoperation
            Node(
                package='teleop_twist_keyboard',
                executable='teleop_twist_keyboard',
                name='teleop_keyboard',
                output='screen',
                prefix='xterm -e',
                remappings=[('/cmd_vel', '/diff_drive_controller/cmd_vel_unstamped')],
                condition=IfCondition(enable_teleop)
            ),

            # Joy Teleoperation (if joystick connected)
            Node(
                package='joy',
                executable='joy_node',
                name='joy_node',
                condition=IfCondition(enable_teleop)
            ),

            Node(
                package='teleop_twist_joy',
                executable='teleop_node',
                name='teleop_joy',
                parameters=[{
                    'axis_linear.x': 1,
                    'axis_angular.yaw': 0,
                    'scale_linear.x': 0.7,
                    'scale_angular.yaw': 1.0,
                    'enable_button': 0,
                    'enable_turbo_button': -1,
                }],
                remappings=[('/cmd_vel', '/diff_drive_controller/cmd_vel_unstamped')],
                condition=IfCondition(enable_teleop)
            ),
        ]),

        # SLAM Group
        GroupAction([
            # Cartographer SLAM
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(pkg_vineyard_mower_navigation, 'launch'),
                    '/slam.launch.py']),
                launch_arguments={'use_sim_time': use_sim_time}.items(),
                condition=IfCondition(enable_slam)
            ),
        ]),

        # Navigation Group (Nav2)
        GroupAction([
            # Navigation2 Stack (when implemented)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(pkg_vineyard_mower_navigation, 'launch'),
                    '/navigation.launch.py']),
                launch_arguments={'use_sim_time': use_sim_time}.items(),
                condition=IfCondition(enable_navigation)
            ),
        ]),

        # Visualization Group
        GroupAction([
            # RViz with custom config
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', os.path.join(pkg_vineyard_mower_navigation, 'config', 'slam_config.rviz')],
                parameters=[{'use_sim_time': use_sim_time}],
                condition=IfCondition(enable_rviz)
            ),
        ]),

        # Diagnostic and Monitoring Group
        GroupAction([
            # Robot diagnostic monitor
            Node(
                package='diagnostic_aggregator',
                executable='aggregator_node',
                name='diagnostic_aggregator',
                parameters=[{
                    'analyzers.sensors.type': 'diagnostic_aggregator/GenericAnalyzer',
                    'analyzers.sensors.path': 'Sensors',
                    'analyzers.sensors.find_and_remove_prefix': 'vineyard_mower',
                    'analyzers.controls.type': 'diagnostic_aggregator/GenericAnalyzer',
                    'analyzers.controls.path': 'Controls',
                    'analyzers.controls.find_and_remove_prefix': 'diff_drive_controller',
                }],
                output='screen'
            ),

            # Topic monitor for debugging
            Node(
                package='rqt_topic',
                executable='rqt_topic',
                name='topic_monitor',
                condition=IfCondition(LaunchConfiguration('debug', default='false'))
            ),
        ]),
    ])