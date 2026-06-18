#!/usr/bin/env python3
"""
Launch file for depth-based obstacle avoidance with keyboard override
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    return LaunchDescription([
        DeclareLaunchArgument(
            'safe_distance',
            default_value='2.0',
            description='Safe distance threshold in meters'
        ),
        
        DeclareLaunchArgument(
            'min_obstacle_area',
            default_value='500',
            description='Minimum obstacle area in pixels to consider'
        ),
        
        DeclareLaunchArgument(
            'linear_speed',
            default_value='0.3',
            description='Linear speed for autonomous navigation (m/s)'
        ),
        
        DeclareLaunchArgument(
            'angular_speed',
            default_value='0.8',
            description='Maximum angular speed (rad/s)'
        ),
        
        DeclareLaunchArgument(
            'pid_kp',
            default_value='1.5',
            description='PID proportional gain'
        ),
        
        DeclareLaunchArgument(
            'pid_ki',
            default_value='0.0',
            description='PID integral gain'
        ),
        
        DeclareLaunchArgument(
            'pid_kd',
            default_value='0.2',
            description='PID derivative gain'
        ),
        
        DeclareLaunchArgument(
            'manual_timeout',
            default_value='2.0',
            description='Timeout for manual mode in seconds'
        ),
        
        # Depth obstacle avoidance node
        Node(
            package='vineyard_mower_teleop',
            executable='depth_obstacle_avoidance',
            name='depth_obstacle_avoidance',
            output='screen',
            parameters=[{
                'safe_distance': LaunchConfiguration('safe_distance'),
                'min_obstacle_area': LaunchConfiguration('min_obstacle_area'),
                'linear_speed': LaunchConfiguration('linear_speed'),
                'angular_speed': LaunchConfiguration('angular_speed'),
                'pid_kp': LaunchConfiguration('pid_kp'),
                'pid_ki': LaunchConfiguration('pid_ki'),
                'pid_kd': LaunchConfiguration('pid_kd'),
                'manual_timeout': LaunchConfiguration('manual_timeout'),
            }]
        ),
    ])
