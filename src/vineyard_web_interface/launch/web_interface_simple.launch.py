#!/usr/bin/env python3
"""
Simple Web Interface Launch Script

This is a basic launch script to test the vineyard web interface functionality.
It starts the essential components needed for the web application:
- ROSbridge WebSocket server for ROS2-Web communication
- Costmap service nodes
- Backend API server
- Frontend development server (optional)

Usage:
    ros2 launch vineyard_web_interface web_interface_simple.launch.py
    ros2 launch vineyard_web_interface web_interface_simple.launch.py enable_frontend:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    # Get package directories
    pkg_vineyard_web_interface = get_package_share_directory('vineyard_web_interface')
    pkg_vineyard_mower_navigation = get_package_share_directory('vineyard_mower_navigation')

    # Launch configuration variables
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
            'enable_frontend',
            default_value='true',
            description='Enable React frontend development server'
        ),
        
        DeclareLaunchArgument(
            'enable_backend',
            default_value='true',
            description='Enable backend API server'
        ),
        
        DeclareLaunchArgument(
            'rosbridge_port',
            default_value='9090',
            description='Port for ROSbridge WebSocket server'
        ),
        
        DeclareLaunchArgument(
            'backend_port',
            default_value='8000',
            description='Port for backend API server'
        ),
        
        DeclareLaunchArgument(
            'frontend_port',
            default_value='3000',
            description='Port for frontend development server'
        ),
        
        DeclareLaunchArgument(
            'web_app_path',
            default_value='/home/sam/vinmo/vineyard_costmap_web',
            description='Path to the web application directory'
        ),

        # ROSbridge WebSocket Server
        GroupAction([
            Node(
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
                    'unregister_timeout': 10.0
                }],
                output='screen'
            ),
            
            # ROSbridge TCP Server (alternative connection method)
            Node(
                package='rosbridge_server',
                executable='rosbridge_tcp',
                name='rosbridge_tcp',
                parameters=[{
                    'port': 9091,
                    'address': '0.0.0.0'
                }],
                output='screen'
            ),
        ]),

        # Costmap Services Group
        GroupAction([
            # Costmap Service Node
            Node(
                package='vineyard_mower_navigation',
                executable='costmap_service',
                name='costmap_service_node',
                output='screen',
                parameters=[
                    PathJoinSubstitution([
                        pkg_vineyard_mower_navigation,
                        'config',
                        'costmap_config.yaml'
                    ])
                ]
            ),
            
            # Path Planning Service Node
            Node(
                package='vineyard_mower_navigation',
                executable='path_planning_server',
                name='path_planning_server_node',
                output='screen',
                parameters=[
                    PathJoinSubstitution([
                        pkg_vineyard_mower_navigation,
                        'config',
                        'path_planning_config.yaml'
                    ])
                ]
            ),
        ]),

        # Backend API Server
        ExecuteProcess(
            condition=IfCondition(enable_backend),
            cmd=[
                'python3',
                PathJoinSubstitution([web_app_path, 'backend', 'src', 'main.py'])
            ],
            cwd=PathJoinSubstitution([web_app_path, 'backend']),
            output='screen',
            name='backend_server',
            env={'PORT': backend_port}
        ),

        # Frontend Development Server
        ExecuteProcess(
            condition=IfCondition(enable_frontend),
            cmd=['npm', 'run', 'dev', '--', '--port', frontend_port, '--host', '0.0.0.0'],
            cwd=PathJoinSubstitution([web_app_path, 'frontend']),
            output='screen',
            name='frontend_server'
        ),

        # Web Interface Monitor (simple status checker)
        ExecuteProcess(
            cmd=[
                'python3',
                PathJoinSubstitution([
                    pkg_vineyard_web_interface,
                    'scripts',
                    'web_interface_monitor.py'
                ])
            ],
            output='screen',
            name='web_interface_monitor'
        ),
    ])
