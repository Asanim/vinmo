#!/usr/bin/env python3
"""
Launch file for the complete vineyard mower perception system.
Launches all perception nodes for multi-sensor obstacle detection.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_vineyard_mower_perception = get_package_share_directory('vineyard_mower_perception')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    enable_front_camera_arg = DeclareLaunchArgument(
        'enable_front_camera',
        default_value='true',
        description='Enable front camera processing'
    )
    
    enable_rear_camera_arg = DeclareLaunchArgument(
        'enable_rear_camera',
        default_value='true',
        description='Enable rear camera processing'
    )
    
    enable_vine_detection_arg = DeclareLaunchArgument(
        'enable_vine_detection',
        default_value='true',
        description='Enable vine detection system'
    )
    
    enable_sensor_health_arg = DeclareLaunchArgument(
        'enable_sensor_health',
        default_value='true',
        description='Enable sensor health monitoring'
    )
    
    debug_visualization_arg = DeclareLaunchArgument(
        'debug_visualization',
        default_value='false',
        description='Enable debug visualization outputs'
    )
    
    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_front_camera = LaunchConfiguration('enable_front_camera')
    enable_rear_camera = LaunchConfiguration('enable_rear_camera')
    enable_vine_detection = LaunchConfiguration('enable_vine_detection')
    enable_sensor_health = LaunchConfiguration('enable_sensor_health')
    debug_visualization = LaunchConfiguration('debug_visualization')
    
    # Config file paths
    perception_config = os.path.join(pkg_vineyard_mower_perception, 'config', 'perception_params.yaml')
    
    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        enable_front_camera_arg,
        enable_rear_camera_arg,
        enable_vine_detection_arg,
        enable_sensor_health_arg,
        debug_visualization_arg,
        
        # Core sensor fusion node
        Node(
            package='vineyard_mower_perception',
            executable='advanced_perception_fusion.py',
            name='advanced_perception_fusion',
            output='screen',
            parameters=[
                perception_config,
                {'use_sim_time': use_sim_time}
            ],
            remappings=[
                # Add any necessary topic remappings here
            ]
        ),
        
        # Front camera depth processing
        Node(
            package='vineyard_mower_perception',
            executable='depth_obstacle_mapper.py',
            name='front_depth_mapper',
            output='screen',
            parameters=[
                perception_config,
                {
                    'use_sim_time': use_sim_time,
                    'camera_namespace': 'front_camera',
                    'target_frame': 'base_link'
                }
            ],
            condition=IfCondition(enable_front_camera)
        ),
        
        # Rear camera depth processing
        Node(
            package='vineyard_mower_perception',
            executable='depth_obstacle_mapper.py',
            name='rear_depth_mapper',
            output='screen',
            parameters=[
                perception_config,
                {
                    'use_sim_time': use_sim_time,
                    'camera_namespace': 'rear_camera',
                    'target_frame': 'base_link'
                }
            ],
            condition=IfCondition(enable_rear_camera)
        ),
        
        # Front camera vine detection
        Node(
            package='vineyard_mower_perception',
            executable='vine_detection_node.py',
            name='front_vine_detection',
            output='screen',
            parameters=[
                perception_config,
                {
                    'use_sim_time': use_sim_time,
                    'camera_namespace': 'front_camera',
                    'debug_visualization': debug_visualization
                }
            ],
            condition=IfCondition(
                PythonExpression([enable_vine_detection, ' and ', enable_front_camera])
            )
        ),
        
        # Rear camera vine detection
        Node(
            package='vineyard_mower_perception',
            executable='vine_detection_node.py',
            name='rear_vine_detection',
            output='screen',
            parameters=[
                perception_config,
                {
                    'use_sim_time': use_sim_time,
                    'camera_namespace': 'rear_camera',
                    'debug_visualization': debug_visualization
                }
            ],
            condition=IfCondition(
                PythonExpression([enable_vine_detection, ' and ', enable_rear_camera])
            )
        ),
        
        # Sensor health monitoring
        Node(
            package='vineyard_mower_perception',
            executable='sensor_health_monitor.py',
            name='sensor_health_monitor',
            output='screen',
            parameters=[
                perception_config,
                {'use_sim_time': use_sim_time}
            ],
            condition=IfCondition(enable_sensor_health)
        ),
    ])
