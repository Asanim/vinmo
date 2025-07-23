#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file for ROS-Gz clock bridge.
    This ensures proper time synchronization between ROS2 and Gazebo.
    Based on Articulated Robotics' solution for clock synchronization issues.
    """
    
    return LaunchDescription([
        # Critical clock bridge for time synchronization
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
    ])
