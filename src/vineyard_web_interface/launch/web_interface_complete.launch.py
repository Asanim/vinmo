#!/usr/bin/env python3
"""
Complete Vineyard Web Interface with Simulation Launch Script

This launch script starts the complete vineyard management system including:
- Gazebo vineyard simulation
- Robot navigation stack
- Costmap generation and path planning services
- Web interface with frontend and backend
- ROSbridge for web communication
- Monitoring and diagnostics

Based on robot_simulation_navigation.launch.py but extended for web interface integration.

Usage:
    ros2 launch vineyard_web_interface web_interface_complete.launch.py
    ros2 launch vineyard_web_interface web_interface_complete.launch.py enable_simulation:=false enable_web:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription, 
    GroupAction, 
    TimerAction,
    ExecuteProcess
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    # Get the package directories
    pkg_vineyard_mower_description = get_package_share_directory('vineyard_mower_description')
    pkg_vineyard_mower_gazebo = get_package_share_directory('vineyard_mower_gazebo')
    pkg_vineyard_mower_navigation = get_package_share_directory('vineyard_mower_navigation')
    pkg_vineyard_web_interface = get_package_share_directory('vineyard_web_interface')

    # Get the urdf file
    urdf_file_name = 'vineyard_mower.urdf.xacro'
    urdf = os.path.join(pkg_vineyard_mower_description, 'urdf', urdf_file_name)

    # World file path
    world = os.path.join(pkg_vineyard_mower_gazebo, 'worlds', 'realistic_vineyard.world')

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    
    # System enablement flags
    enable_simulation = LaunchConfiguration('enable_simulation', default='true')
    enable_teleop = LaunchConfiguration('enable_teleop', default='true')
    enable_slam = LaunchConfiguration('enable_slam', default='true')
    enable_rviz = LaunchConfiguration('enable_rviz', default='true')
    enable_navigation = LaunchConfiguration('enable_navigation', default='true')
    enable_web = LaunchConfiguration('enable_web', default='true')
    enable_costmap_services = LaunchConfiguration('enable_costmap_services', default='true')
    enable_path_planning = LaunchConfiguration('enable_path_planning', default='true')
    
    # Web interface configuration
    enable_frontend = LaunchConfiguration('enable_frontend', default='true')
    enable_backend = LaunchConfiguration('enable_backend', default='true')
    rosbridge_port = LaunchConfiguration('rosbridge_port', default='9090')
    backend_port = LaunchConfiguration('backend_port', default='8000')
    frontend_port = LaunchConfiguration('frontend_port', default='3000')
    web_app_path = LaunchConfiguration('web_app_path', 
                                     default='/home/sam/vinmo/vineyard_costmap_web')

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        DeclareLaunchArgument(
            'x_pose', default_value='0.0',
            description='x position of robot'
        ),

        DeclareLaunchArgument(
            'y_pose', default_value='0.0',
            description='y position of robot'
        ),

        # System enablement arguments
        DeclareLaunchArgument(
            'enable_simulation', default_value='true',
            description='Enable Gazebo simulation'
        ),

        DeclareLaunchArgument(
            'enable_teleop', default_value='true',
            description='Enable teleoperation'
        ),

        DeclareLaunchArgument(
            'enable_slam', default_value='true',
            description='Enable SLAM mapping'
        ),

        DeclareLaunchArgument(
            'enable_rviz', default_value='true',
            description='Enable RViz visualization'
        ),

        DeclareLaunchArgument(
            'enable_navigation', default_value='true',
            description='Enable autonomous navigation (Nav2)'
        ),
        
        DeclareLaunchArgument(
            'enable_web', default_value='true',
            description='Enable web interface'
        ),
        
        DeclareLaunchArgument(
            'enable_costmap_services', default_value='true',
            description='Enable costmap generation services'
        ),
        
        DeclareLaunchArgument(
            'enable_path_planning', default_value='true',
            description='Enable path planning services'
        ),

        # Web interface arguments
        DeclareLaunchArgument(
            'enable_frontend', default_value='true',
            description='Enable React frontend development server'
        ),
        
        DeclareLaunchArgument(
            'enable_backend', default_value='true',
            description='Enable backend API server'
        ),
        
        DeclareLaunchArgument(
            'rosbridge_port', default_value='9090',
            description='Port for ROSbridge WebSocket server'
        ),
        
        DeclareLaunchArgument(
            'backend_port', default_value='8000',
            description='Port for backend API server'
        ),
        
        DeclareLaunchArgument(
            'frontend_port', default_value='3000',
            description='Port for frontend development server'
        ),
        
        DeclareLaunchArgument(
            'web_app_path',
            default_value='/home/sam/vinmo/vineyard_costmap_web',
            description='Path to the web application directory'
        ),

        # Gazebo Simulation Group (identical to original)
        GroupAction([
            # Clock Bridge (MUST BE FIRST!)
            Node(
                condition=IfCondition(enable_simulation),
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='ros_gz_bridge_clock',
                arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
                output='screen',
                parameters=[{'use_sim_time': True}],
                remappings=[('/clock', '/clock')]
            ),

            # Start Gz Sim
            IncludeLaunchDescription(
                condition=IfCondition(enable_simulation),
                launch_description_source=PythonLaunchDescriptionSource([
                    os.path.join(get_package_share_directory('ros_gz_sim'), 'launch'),
                    '/gz_sim.launch.py']),
                launch_arguments={'gz_args': ['-r -v4 ', world]}.items()
            ),

            # Robot State Publisher
            Node(
                condition=IfCondition(enable_simulation),
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time, 'robot_description': Command(['xacro ', urdf])}],
                arguments=[urdf]
            ),

            # Spawn robot
            Node(
                condition=IfCondition(enable_simulation),
                package='ros_gz_sim',
                executable='create',
                name='urdf_spawner',
                output='screen',
                arguments=['-topic', '/robot_description',
                          '-name', 'vineyard_mower',
                          '-x', x_pose, '-y', y_pose, '-z', '0.20']
            ),
        ]),

        # Controllers (delayed to ensure Gazebo and robot are ready)
        TimerAction(
            condition=IfCondition(enable_simulation),
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

        # Sensor Bridge Group (delayed to ensure clock is established)
        TimerAction(
            condition=IfCondition(enable_simulation),
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
                        parameters=[{'use_sim_time': use_sim_time}],
                        remappings=[('/scan', '/scan_raw')]
                    ),

                    # Laser scan frame remapper to fix frame_id issues
                    Node(
                        package='vineyard_mower_navigation',
                        executable='laser_scan_frame_remapper.py',
                        name='laser_scan_frame_remapper',
                        output='screen',
                        parameters=[{'use_sim_time': use_sim_time}]
                    ),

                    # Frame remapper for laser scan to use correct frame names
                    Node(
                        package='tf2_ros',
                        executable='static_transform_publisher',
                        name='gazebo_to_ros_frames',
                        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'vineyard_mower/base_link'],
                        parameters=[{'use_sim_time': use_sim_time}]
                    ),

                    Node(
                        package='tf2_ros',
                        executable='static_transform_publisher',
                        name='lidar_frame_bridge',
                        arguments=['0', '0', '0', '0', '0', '0', 'lidar_link', 'vineyard_mower/base_link/lidar_link'],
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
                condition=IfCondition(enable_teleop),
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
                remappings=[('/cmd_vel', '/cmd_vel_unstamped')]
            ),

            # Convert unstamped Twist to TwistStamped
            Node(
                condition=IfCondition(enable_teleop),
                package='twist_stamper',
                executable='twist_stamper',
                name='twist_stamper',
                parameters=[{'use_sim_time': use_sim_time}],
                remappings=[
                    ('/cmd_vel_in', '/cmd_vel_unstamped'),
                    ('/cmd_vel_out', '/diff_drive_controller/cmd_vel')
                ]
            ),
        ]),

        # SLAM Group
        GroupAction([
            # Cartographer SLAM
            IncludeLaunchDescription(
                condition=IfCondition(enable_slam),
                launch_description_source=PythonLaunchDescriptionSource([
                    os.path.join(pkg_vineyard_mower_navigation, 'launch'),
                    '/slam.launch.py']),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            ),
        ]),

        # Navigation Group (Nav2)
        GroupAction([
            # Navigation2 Stack
            IncludeLaunchDescription(
                condition=IfCondition(enable_navigation),
                launch_description_source=PythonLaunchDescriptionSource([
                    os.path.join(pkg_vineyard_mower_navigation, 'launch'),
                    '/vineyard_navigation.launch.py']),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'slam': 'False',  # Use localization mode by default
                    'navigation': 'True',
                    'use_rviz': 'False'  # We'll use our own RViz
                }.items()
            ),
        ]),

        # Costmap Services Group (NEW)
        GroupAction([
            # Costmap Service Node
            Node(
                condition=IfCondition(enable_costmap_services),
                package='vineyard_mower_navigation',
                executable='costmap_service',
                name='costmap_service_node',
                output='screen',
                parameters=[
                    PathJoinSubstitution([
                        pkg_vineyard_mower_navigation,
                        'config',
                        'costmap_config.yaml'
                    ]),
                    {'use_sim_time': use_sim_time}
                ]
            ),
        ]),

        # Path Planning Services Group (NEW)
        GroupAction([
            # Path Planning Service Node
            Node(
                condition=IfCondition(enable_path_planning),
                package='vineyard_mower_navigation',
                executable='path_planning_server',
                name='path_planning_server_node',
                output='screen',
                parameters=[
                    PathJoinSubstitution([
                        pkg_vineyard_mower_navigation,
                        'config',
                        'path_planning_config.yaml'
                    ]),
                    {'use_sim_time': use_sim_time}
                ]
            ),
        ]),

        # Web Interface Group (NEW - Main addition)
        GroupAction([
            # ROSbridge WebSocket Server
            Node(
                condition=IfCondition(enable_web),
                package='rosbridge_server',
                executable='rosbridge_websocket',
                name='rosbridge_websocket',
                parameters=[{
                    'port': rosbridge_port,
                    'address': '0.0.0.0',
                    'retry_startup_delay': 5,
                    'fragment_timeout': 600,
                    'delay_between_messages': 0,
                    'max_message_size': None,
                    'unregister_timeout': 10.0,
                    'use_sim_time': use_sim_time
                }],
                output='screen'
            ),

            # ROSbridge TCP Server (alternative connection method)
            Node(
                condition=IfCondition(enable_web),
                package='rosbridge_server',
                executable='rosbridge_tcp',
                name='rosbridge_tcp',
                parameters=[{
                    'port': 9091,
                    'address': '0.0.0.0',
                    'use_sim_time': use_sim_time
                }],
                output='screen'
            ),

            # TF2 Web Republisher (for web visualization)
            Node(
                condition=IfCondition(enable_web),
                package='tf2_web_republisher',
                executable='tf2_web_republisher',
                name='tf2_web_republisher',
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            ),
        ]),

        # Backend API Server (NEW)
        ExecuteProcess(
            condition=IfCondition(enable_backend),
            cmd=[
                'python3',
                PathJoinSubstitution([web_app_path, 'backend', 'src', 'main.py'])
            ],
            cwd=PathJoinSubstitution([web_app_path, 'backend']),
            output='screen',
            name='backend_server',
            env={'PORT': backend_port, 'USE_SIM_TIME': use_sim_time}
        ),

        # Frontend Development Server (NEW)
        ExecuteProcess(
            condition=IfCondition(enable_frontend),
            cmd=['npm', 'run', 'dev', '--', '--port', frontend_port, '--host', '0.0.0.0'],
            cwd=PathJoinSubstitution([web_app_path, 'frontend']),
            output='screen',
            name='frontend_server'
        ),

        # Visualization Group
        GroupAction([
            # RViz with custom config
            Node(
                condition=IfCondition(enable_rviz),
                package='rviz2',
                executable='rviz2',
                name='rviz2_slam',
                output='screen',
                arguments=['-d', os.path.join(pkg_vineyard_mower_navigation, 'config', 'slam.rviz')],
                parameters=[{'use_sim_time': use_sim_time}]
            ),
            
            # Additional RViz instance for robot visualization
            Node(
                condition=IfCondition(enable_rviz),
                package='rviz2',
                executable='rviz2',
                name='rviz2_robot',
                output='screen',
                arguments=['-d', os.path.join(pkg_vineyard_mower_description, 'config', 'display.rviz')],
                parameters=[{'use_sim_time': use_sim_time}]
            ),
        ]),

        # Enhanced Diagnostic and Monitoring Group
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
                    'analyzers.web_interface.type': 'diagnostic_aggregator/GenericAnalyzer',
                    'analyzers.web_interface.path': 'Web Interface',
                    'analyzers.web_interface.find_and_remove_prefix': 'web_interface',
                    'use_sim_time': use_sim_time
                }],
                output='screen'
            ),

            # Web Interface Monitor
            ExecuteProcess(
                condition=IfCondition(enable_web),
                cmd=[
                    'python3',
                    PathJoinSubstitution([
                        pkg_vineyard_web_interface,
                        'scripts',
                        'web_interface_monitor.py'
                    ])
                ],
                output='screen',
                name='web_interface_monitor',
                env={'USE_SIM_TIME': use_sim_time}
            ),

            # System Status Publisher (publishes system health to ROS topics)
            Node(
                package='vineyard_web_interface',
                executable='system_status_publisher.py',
                name='system_status_publisher',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            ),
        ]),

        # Integration Testing Group (for CI/CD)
        GroupAction([
            # Integration test node (can be disabled for normal operation)
            Node(
                condition=IfCondition(LaunchConfiguration('run_integration_tests', default='false')),
                package='vineyard_web_interface',
                executable='integration_test_runner.py',
                name='integration_test_runner',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            ),
        ]),
    ])
