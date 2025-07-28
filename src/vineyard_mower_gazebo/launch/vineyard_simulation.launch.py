#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction, SetEnvironmentVariable
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

    # Realistic vineyard world file
    world = os.path.join(pkg_vineyard_mower_gazebo, 'worlds', 'realistic_vineyard.world')

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    enable_teleop = LaunchConfiguration('enable_teleop', default='true')
    enable_slam = LaunchConfiguration('enable_slam', default='true')
    enable_rviz = LaunchConfiguration('enable_rviz', default='true')
    enable_navigation = LaunchConfiguration('enable_navigation', default='false')
    vineyard_season = LaunchConfiguration('vineyard_season', default='summer')
    weather_enabled = LaunchConfiguration('weather_enabled', default='false')

    return LaunchDescription([
        # Set Gazebo model path
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=os.path.join(pkg_vineyard_mower_gazebo, 'models')
        ),

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
            'enable_slam', default_value='true',
            description='Enable SLAM mapping'),

        DeclareLaunchArgument(
            'enable_rviz', default_value='true',
            description='Enable RViz visualization'),

        DeclareLaunchArgument(
            'enable_navigation', default_value='false',
            description='Enable autonomous navigation (Nav2)'),

        DeclareLaunchArgument(
            'vineyard_season', default_value='summer',
            description='Vineyard season: spring, summer, autumn, winter'),

        DeclareLaunchArgument(
            'weather_enabled', default_value='false',
            description='Enable weather effects'),

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

            # Start Gz Sim with realistic vineyard world
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

            # Spawn robot in vineyard
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
                        parameters=[{'use_sim_time': True}],
                        output='screen',
                    ),

                    # Differential Drive Controller
                    Node(
                        package='controller_manager',
                        executable='spawner',
                        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
                        parameters=[{'use_sim_time': True}],
                        output='screen',
                    ),
                ])
            ]
        ),

        # Enhanced Sensor Bridge Group for vineyard environment
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
                            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
                            '/gps@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat'
                        ],
                        output='screen',
                        parameters=[{'use_sim_time': use_sim_time}],
                        remappings=[
                            ('/scan', '/scan_raw'),
                            ('/gps', '/fix')
                        ]
                    ),

                    # Laser scan frame remapper to fix frame_id issues
                    Node(
                        package='vineyard_mower_navigation',
                        executable='laser_scan_frame_remapper.py',
                        name='laser_scan_frame_remapper',
                        output='screen',
                        parameters=[{'use_sim_time': use_sim_time}]
                    ),

                    # Transform publishers for vineyard-specific frames
                    Node(
                        package='tf2_ros',
                        executable='static_transform_publisher',
                        name='base_link_to_gps',
                        arguments=['0', '0', '0.5', '0', '0', '0', 'base_link', 'gps_link'],
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

                    # Vineyard-specific coordinate frame
                    Node(
                        package='tf2_ros',
                        executable='static_transform_publisher',
                        name='map_to_vineyard_frame',
                        arguments=['0', '0', '0', '0', '0', '0', 'map', 'vineyard_frame'],
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
                parameters=[{
                    'key_forward': 'w',
                    'key_backward': 's',
                    'key_left': 'a',
                    'key_right': 'd',
                    'key_run': 'shift',
                    'key_teleop': 'q',
                    'scale_linear': 0.5,
                    'scale_angular': 1.0
                }],
                remappings=[('/cmd_vel', '/cmd_vel_unstamped')],
                condition=IfCondition(enable_teleop)
            ),

            # Convert unstamped Twist to TwistStamped
            Node(
                package='twist_stamper',
                executable='twist_stamper',
                name='twist_stamper',
                parameters=[{'use_sim_time': use_sim_time}],
                remappings=[
                    ('/cmd_vel_in', '/cmd_vel_unstamped'),
                    ('/cmd_vel_out', '/diff_drive_controller/cmd_vel')
                ],
                condition=IfCondition(enable_teleop)
            ),

            # Joy Teleoperation
            Node(
                package='joy',
                executable='joy_node',
                name='joy_node',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'dev': '/dev/input/js0',
                    'deadzone': 0.3,
                    'autorepeat_rate': 20.0,
                }],
                condition=IfCondition(enable_teleop)
            ),

            Node(
                package='teleop_twist_joy',
                executable='teleop_node',
                name='teleop_joy',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'axis_linear.x': 1,
                    'axis_angular.yaw': 0,
                    'scale_linear.x': 0.7,
                    'scale_angular.yaw': 1.0,
                    'enable_button': 0,
                    'enable_turbo_button': -1,
                    'require_enable_button': True,
                }],
                remappings=[('/cmd_vel', '/cmd_vel_unstamped')],
                condition=IfCondition(enable_teleop)
            ),
        ]),

        # SLAM Group optimized for vineyard navigation
        GroupAction([
            # Cartographer SLAM with vineyard-specific configuration
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(pkg_vineyard_mower_navigation, 'launch'),
                    '/slam.launch.py']),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'configuration_directory': os.path.join(pkg_vineyard_mower_navigation, 'config', 'vineyard'),
                    'configuration_basename': 'vineyard_slam.lua'
                }.items(),
                condition=IfCondition(enable_slam)
            ),
        ]),

        # Vineyard Navigation Group (when ready)
        # GroupAction([
        #     # Navigation2 Stack configured for vineyard rows
        #     IncludeLaunchDescription(
        #         PythonLaunchDescriptionSource([
        #             os.path.join(pkg_vineyard_mower_navigation, 'launch'),
        #             '/vineyard_navigation.launch.py']),
        #         launch_arguments={
        #             'use_sim_time': use_sim_time,
        #             'map_file': os.path.join(pkg_vineyard_mower_navigation, 'maps', 'vineyard_map.yaml')
        #         }.items(),
        #         condition=IfCondition(enable_navigation)
        #     ),
        # ]),

        # Enhanced Visualization Group
        GroupAction([
            # RViz with vineyard-specific configuration
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2_vineyard',
                output='screen',
                arguments=['-d', os.path.join(pkg_vineyard_mower_navigation, 'config', 'vineyard_simulation.rviz')],
                parameters=[{'use_sim_time': use_sim_time}],
                condition=IfCondition(enable_rviz)
            ),
        ]),

        # Vineyard Environment Monitoring
        GroupAction([
            # Environment parameter server
            Node(
                package='vineyard_mower_gazebo',
                executable='vineyard_environment_monitor.py',
                name='vineyard_environment_monitor',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'vineyard_config_file': os.path.join(pkg_vineyard_mower_gazebo, 'config', 'vineyard_config.yaml'),
                    'current_season': vineyard_season,
                    'weather_enabled': weather_enabled
                }],
            ),

            # GPS reference point publisher
            Node(
                package='vineyard_mower_navigation',
                executable='gps_reference_publisher.py',
                name='gps_reference_publisher',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'latitude': 45.5017,   # Example vineyard location
                    'longitude': -122.6750,
                    'altitude': 50.0
                }],
            ),

            # Vineyard row detection node
            Node(
                package='vineyard_mower_navigation',
                executable='vineyard_row_detector.py',
                name='vineyard_row_detector',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'row_spacing': 2.5,
                    'detection_range': 10.0
                }],
                remappings=[
                    ('/scan', '/scan'),
                    ('/camera/image', '/front_camera/image')
                ]
            ),

            # Diagnostic aggregator for vineyard operations
            Node(
                package='diagnostic_aggregator',
                executable='aggregator_node',
                name='vineyard_diagnostic_aggregator',
                parameters=[{
                    'analyzers.sensors.type': 'diagnostic_aggregator/GenericAnalyzer',
                    'analyzers.sensors.path': 'Vineyard Sensors',
                    'analyzers.sensors.find_and_remove_prefix': 'vineyard_mower',
                    'analyzers.navigation.type': 'diagnostic_aggregator/GenericAnalyzer',
                    'analyzers.navigation.path': 'Vineyard Navigation',
                    'analyzers.navigation.find_and_remove_prefix': 'nav2',
                    'analyzers.environment.type': 'diagnostic_aggregator/GenericAnalyzer',
                    'analyzers.environment.path': 'Vineyard Environment',
                    'analyzers.environment.find_and_remove_prefix': 'vineyard'
                }],
                output='screen'
            ),
        ]),
    ])
