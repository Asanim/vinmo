#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for keyboard teleoperation"""
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    linear_speed_arg = DeclareLaunchArgument(
        'linear_speed',
        default_value='0.5',
        description='Default linear speed (m/s)'
    )
    
    angular_speed_arg = DeclareLaunchArgument(
        'angular_speed',
        default_value='1.0',
        description='Default angular speed (rad/s)'
    )
    
    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    linear_speed = LaunchConfiguration('linear_speed')
    angular_speed = LaunchConfiguration('angular_speed')
    
    # Keyboard teleop node
    keyboard_teleop_node = Node(
        package='vineyard_mower_teleop',
        executable='keyboard_teleop',
        name='keyboard_teleop',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'linear_speed': linear_speed,
            'angular_speed': angular_speed,
            'linear_increment': 0.05,
            'angular_increment': 0.1,
            'publish_rate': 10.0,
        }],
        prefix='xterm -e',  # Run in separate terminal
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        linear_speed_arg,
        angular_speed_arg,
        keyboard_teleop_node,
    ])
