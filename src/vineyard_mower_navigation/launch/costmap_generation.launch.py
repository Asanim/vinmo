#!/usr/bin/env python3
"""
Launch file for costmap generation system
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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
    
    # Costmap service node
    costmap_service_node = Node(
        package='vineyard_mower_navigation',
        executable='costmap_service',
        name='costmap_service',
        output='screen',
        parameters=[{
            'google_maps_api_key': LaunchConfiguration('google_api_key'),
            'costmap_config_file': LaunchConfiguration('config_file'),
            'default_resolution': LaunchConfiguration('resolution'),
            'default_width': LaunchConfiguration('width'), 
            'default_height': LaunchConfiguration('height'),
            'auto_update_rate': LaunchConfiguration('auto_update_rate'),
            'save_costmaps': LaunchConfiguration('save_costmaps'),
            'costmap_save_path': LaunchConfiguration('save_path')
        }]
    )
    
    # Costmap publisher node (separate for global/local costmaps)
    costmap_publisher_node = Node(
        package='vineyard_mower_navigation',
        executable='costmap_publisher',
        name='costmap_publisher',
        output='screen',
        parameters=[{
            'costmap_config_file': LaunchConfiguration('config_file'),
            'resolution': LaunchConfiguration('resolution'),
            'width': LaunchConfiguration('width'),
            'height': LaunchConfiguration('height')
        }]
    )
    
    return LaunchDescription([
        google_api_key_arg,
        config_file_arg,
        resolution_arg,
        width_arg,
        height_arg,
        auto_update_rate_arg,
        save_costmaps_arg,
        save_path_arg,
        
        LogInfo(msg='Starting Vineyard Costmap Generation System'),
        
        costmap_service_node,
        costmap_publisher_node
    ])
