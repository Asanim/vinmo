# Vineyard Mower Navigation System

This package provides Navigation2 (Nav2) integration for autonomous vineyard navigation, specifically designed for agricultural vehicles operating in structured vineyard environments.

## Features

### 🚜 **Agricultural Vehicle Optimization**
- **Robot Footprint**: Configured for 1.5m wide agricultural vehicle
- **Safety Margins**: Enhanced inflation parameters for vineyard safety
- **Narrow Corridor Navigation**: Optimized for vineyard row navigation
- **Differential Drive**: Supports tracked/wheeled agricultural platforms

### 🗺️ **Localization & Mapping**
- **AMCL Localization**: GPS-assisted particle filter localization
- **Cartographer SLAM**: Simultaneous localization and mapping
- **GPS Fallback**: GPS reference publisher for outdoor localization
- **Multi-sensor Fusion**: Laser, IMU, and odometry integration

### 🧭 **Navigation Stack**
- **Path Planning**: NavFn and SMAC planners for optimal route planning
- **Local Control**: DWB local planner optimized for agricultural dynamics
- **Behavior Trees**: Vineyard-specific recovery behaviors
- **Collision Avoidance**: Enhanced safety systems for agricultural environments

### 📊 **Monitoring & Visualization**
- **RViz Integration**: Custom configuration for vineyard navigation
- **Real-time Diagnostics**: Navigation status monitoring
- **Performance Metrics**: Path planning and execution statistics

## Quick Start

### 1. Setup and Installation
```bash
# From your workspace root
./setup_navigation.sh

# Build the workspace
colcon build --packages-select vineyard_mower_navigation

# Source the workspace
source install/setup.bash
```

### 2. Launch Complete System (Simulation + Navigation)
```bash
# Launch Gazebo simulation with navigation enabled
ros2 launch vineyard_mower_gazebo robot_simulation_navigation.launch.py \
    enable_navigation:=true \
    enable_slam:=false \
    enable_rviz:=true
```

### 3. Launch Navigation Only (with existing robot)
```bash
# For SLAM mode (mapping)
ros2 launch vineyard_mower_navigation vineyard_navigation.launch.py \
    slam:=True navigation:=True

# For localization mode (with existing map)
ros2 launch vineyard_mower_navigation vineyard_navigation.launch.py \
    slam:=False navigation:=True map_yaml_file:=/path/to/your/map.yaml
```

### 4. Test the System
```bash
# Run integration tests
ros2 run vineyard_mower_navigation test_navigation_integration.py

# Monitor navigation status
ros2 topic echo /navigation_status
```

## Configuration Files

### Core Navigation Parameters
- **`config/nav2_params.yaml`**: Main Navigation2 configuration
- **`config/amcl_params.yaml`**: AMCL localization parameters
- **`config/robot_footprint.yaml`**: Robot dimensions and safety margins

### Behavior Configuration
- **`config/vineyard_behavior_tree.xml`**: Custom behavior tree for vineyard navigation
- **`config/nav2_default_view.rviz`**: RViz visualization configuration

## Launch Files

### Primary Launch Files
- **`vineyard_navigation.launch.py`**: Complete navigation system
- **`navigation.launch.py`**: Nav2 stack only
- **`localization.launch.py`**: AMCL localization only

### Component Launch Files
- **`slam.launch.py`**: Cartographer SLAM
- **`teleop.launch.py`**: Manual teleoperation

## Navigation Modes

### 1. SLAM Mode (Mapping)
Used for creating new maps of vineyard environments:
```bash
ros2 launch vineyard_mower_navigation vineyard_navigation.launch.py slam:=True
```

### 2. Localization Mode (Navigation)
Used for autonomous navigation with existing maps:
```bash
ros2 launch vineyard_mower_navigation vineyard_navigation.launch.py \
    slam:=False map_yaml_file:=/path/to/vineyard_map.yaml
```

### 3. GPS-Assisted Mode
For outdoor environments with GPS fallback:
```bash
# GPS reference publisher will automatically start with localization
ros2 run vineyard_mower_navigation gps_reference_publisher.py
```

## Vineyard-Specific Features

### Enhanced Recovery Behaviors
The system includes specialized recovery behaviors for vineyard environments:
- **Clearing Rotation**: 90° rotation with costmap clearing
- **Backup and Clear**: Controlled backward movement with obstacle clearing
- **Assisted Teleop**: Manual intervention fallback

### Agricultural Vehicle Dynamics
- **Speed Limits**: Optimized for agricultural vehicle capabilities (0.8 m/s max)
- **Acceleration Limits**: Conservative acceleration for heavy agricultural equipment
- **Turn Radius**: Configured for wide agricultural vehicles
- **Safety Margins**: Enhanced inflation for crop protection

### Structured Environment Optimization
- **Row Detection**: Vineyard row detection algorithms
- **Corridor Navigation**: Optimized for narrow vineyard rows
- **Obstacle Avoidance**: Enhanced for grape vines and vineyard infrastructure

## Coordinate Frames

The navigation system uses the following coordinate frame structure:
```
map → odom → base_link → sensors (lidar_link, camera_links, etc.)
```

- **`map`**: Global reference frame
- **`odom`**: Odometry frame (published by differential drive controller)
- **`base_link`**: Robot's main coordinate frame
- **`lidar_link`**: Laser scanner frame

## Topics and Services

### Key Topics
- **`/goal_pose`**: Navigation goals
- **`/initialpose`**: Initial pose for localization
- **`/plan`**: Global path from planner
- **`/local_plan`**: Local path from controller
- **`/cmd_vel`**: Velocity commands to robot

### Key Services
- **`/navigate_to_pose`**: Navigation action service
- **`/global_localization`**: Global localization trigger
- **`/clear_entirely_global_costmap`**: Clear global costmap
- **`/clear_entirely_local_costmap`**: Clear local costmap

## Troubleshooting

### Common Issues

1. **Robot not localizing**
   ```bash
   # Set initial pose manually
   ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "..."
   
   # Or trigger global localization
   ros2 service call /global_localization std_srvs/srv/Empty
   ```

2. **Path planning failures**
   ```bash
   # Check costmaps
   ros2 topic echo /global_costmap/costmap --once
   ros2 topic echo /local_costmap/costmap --once
   
   # Clear costmaps
   ros2 service call /clear_entirely_global_costmap std_srvs/srv/Empty
   ```

3. **Navigation getting stuck**
   - The behavior tree will automatically trigger recovery behaviors
   - Use assisted teleop for manual intervention
   - Check for obstacles in sensor data: `ros2 topic echo /scan`

### Parameter Tuning

For specific vineyard conditions, you may need to adjust:
- **Inflation radius** in costmap parameters
- **Speed limits** in controller parameters
- **Recovery behavior** parameters in behavior tree
- **AMCL parameters** for localization accuracy

## Development and Testing

### Integration Testing
```bash
# Run comprehensive navigation tests
ros2 run vineyard_mower_navigation test_navigation_integration.py

# Test specific components
ros2 launch vineyard_mower_navigation localization.launch.py
ros2 launch vineyard_mower_navigation navigation.launch.py
```

### Performance Monitoring
```bash
# Monitor navigation performance
ros2 topic hz /plan  # Path planning frequency
ros2 topic hz /cmd_vel  # Control frequency
ros2 topic echo /diagnostics  # System diagnostics
```

## Contributing

When contributing to the navigation system:
1. Test thoroughly in simulation before real-world deployment
2. Update parameter documentation for any new configurations
3. Ensure agricultural safety standards are maintained
4. Test with various vineyard layouts and conditions

## Safety Notes

⚠️ **Important Safety Considerations:**
- Always test in simulation before real-world deployment
- Ensure adequate safety margins around crops and infrastructure
- Monitor the robot during autonomous operation
- Have manual override capabilities available
- Respect agricultural safety standards and regulations

## Support

For issues, feature requests, or questions:
1. Check the troubleshooting section above
2. Review ROS2 Navigation2 documentation
3. Test with the provided integration scripts
4. Check system logs with `ros2 topic echo /rosout`
