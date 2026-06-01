# Vineyard Mower Robot - ROS2 Project

A ROS2-based differential drive robot designed for autonomous vineyard mowing operations.

**Completed Systems:**
- Complete robot description with sensors (LiDAR, depth cameras)
- Realistic vineyard simulation environment
- Full Nav2 navigation stack with agricultural optimizations
- SLAM and localization capabilities
- Basic obstacle avoidance using costmaps
- Teleoperation and diagnostics
- **Multi-sensor perception system with sensor fusion**
- **Advanced depth camera processing and 3D obstacle detection**
- **Vine detection system with proximity warnings**
- **Comprehensive sensor health monitoring**

**Current Gaps (Emergency & Advanced Safety):**
- No emergency stop mechanisms
- No sensor failure fallback behaviors
- No boundary enforcement systems
- No predictive collision avoidance
- No environmental hazard detection

## Notes

```bash
PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages:/home/sam/vinmo/install/vineyard_mower_navigation/lib/python3/dist-packages/
```

## Project Structure

```text
vinmo2/
├── src/
│   ├── vineyard_mower_description/    # Robot URDF and visualization
│   │   ├── urdf/                      # Robot description files
│   │   ├── launch/                    # Launch files for visualization
│   │   ├── config/                    # RViz configuration
│   │   └── meshes/                    # 3D mesh files (empty for now)
│   ├── vineyard_mower_gazebo/         # Gazebo simulation
│   │   ├── launch/                    # Simulation launch files
│   │   └── worlds/                    # Gazebo world files
│   ├── vineyard_mower_navigation/     # Navigation stack
│   │   ├── launch/                    # Navigation launch files
│   │   ├── config/                    # Nav2 configuration
│   │   └── scripts/                   # Navigation utilities
│   ├── vineyard_mower_perception/     # Multi-sensor perception (NEW)
│   │   ├── scripts/                   # Perception nodes
│   │   ├── launch/                    # Perception launch files
│   │   └── config/                    # Perception parameters
│   └── vineyard_mower_interfaces/     # Custom ROS2 messages
│       ├── msg/                       # Message definitions
│       └── srv/                       # Service definitions
├── setup_workspace.sh                 # Workspace setup script
└── prompts_breakdown.md              # Development phase breakdown
```

## Robot Specifications

- **Dimensions**: 1.5m wide × 0.5m long × 0.5m high
- **Drive System**: Differential drive with caterpillar tracks
- **Sensors** (ready for integration):
  - Odometry sensors
  - 2D LiDAR (mounted on top center)
  - Front-facing Intel RealSense depth camera
  - Rear-facing Intel RealSense depth camera

## Quick Start

### Prerequisites

- ROS2 Humble installation
- Gazebo 11+
- RViz2

### Setup and Build

1. **Clone and setup the workspace:**

   ```bash
   cd $HOME/vinmo
   ./setup_workspace.sh
   ```

2. **Source the workspace:**

   ```bash
   source install/setup.bash
   ```

### Running the Simulation

#### Full Simulation (Gazebo + RViz)

```bash
ros2 launch vineyard_mower_gazebo robot_simulation.launch.py
```

#### RViz Visualization Only

```bash
ros2 launch vineyard_mower_description display.launch.py
```

#### Gazebo Simulation Only

```bash
ros2 launch vineyard_mower_gazebo gazebo.launch.py
```

### Robot Control

Once the simulation is running, you can control the robot using:

```bash
# Basic movement commands
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Stop the robot
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### Available Topics

- `/cmd_vel` - Velocity commands (geometry_msgs/Twist)
- `/odom` - Odometry data (nav_msgs/Odometry)
- `/robot_description` - Robot URDF description
- `/joint_states` - Joint state information

## License

MIT License

## Contributing

This is part of a structured development process. Each phase should be completed before moving to the next prompt in the development sequence.
