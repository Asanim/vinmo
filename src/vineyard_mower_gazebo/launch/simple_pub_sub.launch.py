#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    Launch a simple publisher-subscriber demo.
    
    This launch file demonstrates basic ROS2 pub-sub communication by:
    - Starting a publisher node that sends "Hello World" messages
    - Starting a subscriber node that receives and logs the messages
    """
    
    # Declare launch arguments
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level for nodes (debug, info, warn, error, fatal)'
    )
    
    # Publisher node
    publisher_node = Node(
        package='vineyard_mower_gazebo',
        executable='simple_publisher.py',
        name='simple_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )
    
    # Subscriber node
    subscriber_node = Node(
        package='vineyard_mower_gazebo',
        executable='simple_subscriber.py',
        name='simple_subscriber',
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )
    
    # Log info message
    log_message = LogInfo(
        msg="Starting simple publisher-subscriber demo..."
    )
    
    return LaunchDescription([
        log_level_arg,
        log_message,
        publisher_node,
        subscriber_node
    ])
