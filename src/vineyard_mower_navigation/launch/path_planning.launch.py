#!/usr/bin/env python3
"""
Launch file for vineyard path planning system
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package directory
    pkg_share = FindPackageShare('vineyard_mower_navigation')
    
    # Launch arguments
    config_file_arg = DeclareLaunchArgument(
        'path_config_file',
        default_value=PathJoinSubstitution([
            pkg_share, 'config', 'path_planning_config.yaml'
        ]),
        description='Path to path planning configuration file'
    )
    
    costmap_config_arg = DeclareLaunchArgument(
        'costmap_config_file',
        default_value=PathJoinSubstitution([
            pkg_share, 'config', 'costmap_config.yaml'
        ]),
        description='Path to costmap configuration file'
    )
    
    vehicle_width_arg = DeclareLaunchArgument(
        'vehicle_width',
        default_value='1.5',
        description='Vehicle width in meters'
    )
    
    vehicle_length_arg = DeclareLaunchArgument(
        'vehicle_length', 
        default_value='2.0',
        description='Vehicle length in meters'
    )
    
    min_turning_radius_arg = DeclareLaunchArgument(
        'min_turning_radius',
        default_value='2.0',
        description='Minimum turning radius in meters'
    )
    
    row_spacing_arg = DeclareLaunchArgument(
        'row_spacing',
        default_value='2.5',
        description='Vineyard row spacing in meters'
    )
    
    max_velocity_arg = DeclareLaunchArgument(
        'max_velocity',
        default_value='2.0',
        description='Maximum vehicle velocity in m/s'
    )
    
    waypoint_density_arg = DeclareLaunchArgument(
        'waypoint_density',
        default_value='1.0',
        description='Waypoint density per meter'
    )
    
    save_paths_arg = DeclareLaunchArgument(
        'save_paths',
        default_value='true',
        description='Whether to save generated paths'
    )
    
    path_save_directory_arg = DeclareLaunchArgument(
        'path_save_directory',
        default_value='/tmp/vineyard_paths/',
        description='Directory to save generated paths'
    )
    
    enable_visualization_arg = DeclareLaunchArgument(
        'enable_visualization',
        default_value='true',
        description='Enable path visualization'
    )
    
    visualization_topic_arg = DeclareLaunchArgument(
        'visualization_topic',
        default_value='path_visualization',
        description='Topic for path visualization markers'
    )
    
    google_api_key_arg = DeclareLaunchArgument(
        'google_api_key',
        default_value='',
        description='Google Maps API key for satellite imagery'
    )
    
    # Costmap generation system
    costmap_group = GroupAction([
        Node(
            package='vineyard_mower_navigation',
            executable='costmap_service',
            name='costmap_service',
            output='screen',
            parameters=[{
                'google_maps_api_key': LaunchConfiguration('google_api_key'),
                'costmap_config_file': LaunchConfiguration('costmap_config_file'),
                'save_costmaps': True,
                'costmap_save_path': '/tmp/costmaps/'
            }]
        ),
        
        Node(
            package='vineyard_mower_navigation',
            executable='costmap_publisher',
            name='costmap_publisher', 
            output='screen',
            parameters=[{
                'costmap_config_file': LaunchConfiguration('costmap_config_file')
            }]
        )
    ])
    
    # Path planning action server
    path_planning_server = Node(
        package='vineyard_mower_navigation',
        executable='path_planning_server',
        name='path_planning_action_server',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('path_config_file'),
            'vehicle_width': LaunchConfiguration('vehicle_width'),
            'vehicle_length': LaunchConfiguration('vehicle_length'),
            'min_turning_radius': LaunchConfiguration('min_turning_radius'),
            'row_spacing': LaunchConfiguration('row_spacing'),
            'max_velocity': LaunchConfiguration('max_velocity'),
            'waypoint_density': LaunchConfiguration('waypoint_density'),
            'save_paths': LaunchConfiguration('save_paths'),
            'path_save_directory': LaunchConfiguration('path_save_directory'),
            'visualization_topic': LaunchConfiguration('visualization_topic')
        }]
    )
    
    # Path visualization node (optional)
    path_visualizer = Node(
        package='vineyard_mower_navigation',
        executable='path_visualizer',
        name='path_visualizer',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_visualization')),
        parameters=[{
            'visualization_topic': LaunchConfiguration('visualization_topic'),
            'save_visualizations': True,
            'visualization_save_path': '/tmp/path_visualizations/'
        }]
    )
    
    # RViz for visualization (optional)
    rviz_config_file = PathJoinSubstitution([
        pkg_share, 'rviz', 'path_planning.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('enable_visualization')),
        output='screen'
    )
    
    return LaunchDescription([
        # Launch arguments
        config_file_arg,
        costmap_config_arg,
        vehicle_width_arg,
        vehicle_length_arg,
        min_turning_radius_arg,
        row_spacing_arg,
        max_velocity_arg,
        waypoint_density_arg,
        save_paths_arg,
        path_save_directory_arg,
        enable_visualization_arg,
        visualization_topic_arg,
        google_api_key_arg,
        
        LogInfo(msg='Starting Vineyard Path Planning System'),
        
        # Core nodes
        costmap_group,
        path_planning_server,
        
        # Optional visualization
        path_visualizer,
        rviz_node,
        
        LogInfo(msg='All path planning nodes started')
    ])
