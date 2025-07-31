#!/usr/bin/env python
"""
launch file for the Vineyard Costmap Web System
Includes: ROS nodes, rosbridge, web backend, and frontend
"""

import os
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Package directory
    pkg_share = FindPackageShare('vineyard_mower_navigation')
    
    # Launch arguments
    google_api_key_arg = DeclareLaunchArgument(
        'google_api_key',
        default_value='',
        description='Google Maps API key for satellite imagery'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'costmap_config.yaml']),
        description='Path to costmap configuration file'
    )
    
    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value='0.1',
        description='Costmap resolution in meters per pixel'
    )
    
    width_arg = DeclareLaunchArgument(
        'width',
        default_value='1000',
        description='Costmap width in cells'
    )
    
    height_arg = DeclareLaunchArgument(
        'height',
        default_value='1000',
        description='Costmap height in cells'
    )
    
    auto_update_rate_arg = DeclareLaunchArgument(
        'auto_update_rate',
        default_value='30.0',
        description='Auto update rate in seconds'
    )
    
    save_costmaps_arg = DeclareLaunchArgument(
        'save_costmaps',
        default_value='true',
        description='Whether to save generated costmaps'
    )
    
    save_path_arg = DeclareLaunchArgument(
        'save_path',
        default_value='/tmp/costmaps/',
        description='Directory to save costmaps'
    )

    # Web system arguments
    rosbridge_port_arg = DeclareLaunchArgument(
        'rosbridge_port',
        default_value='9091',
        description='Port for rosbridge websocket server'
    )
    
    backend_port_arg = DeclareLaunchArgument(
        'backend_port',
        default_value='8000',
        description='Port for FastAPI backend server'
    )
    
    frontend_port_arg = DeclareLaunchArgument(
        'frontend_port',
        default_value='3000',
        description='Port for React frontend development server'
    )

    web_backend_path_arg = DeclareLaunchArgument(
        'web_backend_path',
        default_value='/home/sam/vinmo/vineyard_costmap_web/backend/src',
        description='Path to web backend source directory'
    )

    web_frontend_path_arg = DeclareLaunchArgument(
        'web_frontend_path',
        default_value='/home/sam/vinmo/vineyard_costmap_web/frontend',
        description='Path to web frontend source directory'
    )

    # ROS Nodes
    costmap_web_service_node = Node(
        package='vineyard_mower_navigation',
        executable='costmap_web_service.py',
        name='costmap_web_service',
        output='screen',
        parameters=[{
            'default_resolution': LaunchConfiguration('resolution'),
            'default_width': LaunchConfiguration('width'), 
            'default_height': LaunchConfiguration('height'),
            'frame_id': 'map',
            'costmap_topic': '/costmap/updates',
            'job_updates_topic': '/costmap/job_updates'
        }]
    )

    costmap_publisher_node = Node(
        package='vineyard_mower_navigation',
        executable='costmap_publisher.py',
        name='costmap_publisher',
        output='screen',
        parameters=[{
            'costmap_config_file': LaunchConfiguration('config_file'),
            'resolution': LaunchConfiguration('resolution'),
            'width': LaunchConfiguration('width'),
            'height': LaunchConfiguration('height')
        }]
    )

    # ROSbridge WebSocket Server (using different port to avoid conflicts)
    rosbridge_server = ExecuteProcess(
        cmd=[
            'ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml',
            'port:=9091'  # Use port 9091 instead of 9090
        ],
        output='screen',
        name='rosbridge_websocket'
    )

    # Web Backend (FastAPI)
    web_backend = ExecuteProcess(
        cmd=[
            '/bin/bash', '-c', 
            'source /opt/ros/jazzy/setup.bash && '
            'source /home/sam/vinmo/install/setup.bash && '
            'cd /home/sam/vinmo/vineyard_costmap_web/backend/src && '
            'python main.py'
        ],
        output='screen',
        name='web_backend',
        additional_env={
            'DATABASE_URL': 'postgresql://vineyard_user:vineyard_pass@localhost:5432/vineyard_costmap',
            'ROS_DOMAIN_ID': '0'
        }
    )

    # Web Frontend (React/Vite dev server)
    web_frontend = ExecuteProcess(
        cmd=[
            '/bin/bash', '-c',
            'export PATH="/usr/bin:/usr/local/bin:$PATH" && '
            'cd /home/sam/vinmo/vineyard_costmap_web/frontend && '
            'npm run dev'
        ],
        output='screen',
        name='web_frontend',
        additional_env={
            'VITE_API_URL': 'http://localhost:8000',
            'VITE_WS_URL': 'ws://localhost:9091'
        }
    )

    return LaunchDescription([
        # Arguments
        google_api_key_arg,
        config_file_arg,
        resolution_arg,
        width_arg,
        height_arg,
        auto_update_rate_arg,
        save_costmaps_arg,
        save_path_arg,
        rosbridge_port_arg,
        backend_port_arg,
        frontend_port_arg,
        web_backend_path_arg,
        web_frontend_path_arg,
        
        # Log messages
        LogInfo(msg='Starting Vineyard Costmap Web System'),
        LogInfo(msg='This will start: ROS nodes, ROSbridge, Web backend, and Frontend'),
        LogInfo(msg='Frontend will be available at: http://localhost:3000'),
        LogInfo(msg='Backend API will be available at: http://localhost:8000'),
        LogInfo(msg='ROSbridge WebSocket will be available at: ws://localhost:9091'),
        
        # ROS Nodes
        GroupAction([
            LogInfo(msg='Starting ROS Costmap Nodes...'),
            costmap_web_service_node,
            costmap_publisher_node
        ]),

        # ROSbridge Server
        GroupAction([
            LogInfo(msg='Starting ROSbridge WebSocket Server on port 9091...'),
            rosbridge_server
        ]),

        # Web Backend
        GroupAction([
            LogInfo(msg='Starting Web Backend (FastAPI) on port 8000...'),
            web_backend
        ]),

        # Web Frontend  
        GroupAction([
            LogInfo(msg='Starting Web Frontend (React/Vite) on port 3000...'),
            web_frontend
        ])
    ])
