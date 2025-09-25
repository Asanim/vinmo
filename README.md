# Vineyard Mower Robot

ROS2-based autonomous navigation system for vineyard environments with satellite imagery processing and web-based mission planning.

## What is this?

This project provides autonomous navigation capabilities for vineyard environments through:

- **Satellite imagery processing** that detects vine rows, obstacles, and navigable areas
- **Costmap generation** for path planning and obstacle avoidance  
- **Web interface** for mission planning, parameter tuning, and real-time monitoring
- **ROS2 integration** with standard navigation stack compatibility

The system processes satellite images to generate navigation costmaps, enabling robots to autonomously navigate vineyard terrain while avoiding obstacles and following designated paths.

## Architecture 

```text
vinmo/
├── src/
│   ├── vineyard_mower_description/    # Robot URDF and 3D models
│   ├── vineyard_mower_gazebo/         # Simulation environments
│   ├── vineyard_mower_interfaces/     # Custom ROS2 service definitions
│   ├── vineyard_mower_navigation/     # Core navigation and costmap generation
│   └── vineyard_web_interface/        # ROS2 web integration services
├── vineyard_costmap_web/              # React frontend + FastAPI backend
├── doc/                               # Documentation
└── launch_vineyard_web.sh            # Single-command system startup
```

### Core Components

**Navigation Stack** (`vineyard_mower_navigation/`)
- Satellite image processing with OpenCV
- Vine row detection using Hough transforms
- Costmap generation for ROS2 nav stack
- GPS coordinate handling and transformations

**Web Application** (`vineyard_costmap_web/`)
- React frontend with real-time visualization
- FastAPI backend with PostgreSQL storage
- ROSbridge WebSocket communication
- Parameter tuning and mission management

**Simulation** (`vineyard_mower_gazebo/`)
- Realistic vineyard world generation
- Robot physics and sensor simulation
- Testing environments for navigation algorithms

## Quick Start

```bash
# Clone and build
./setup_workspace.sh

# Start complete system (ROS nodes + web interface)
./launch_vineyard_web.sh
```

**Access Points:**
- Web Interface: http://localhost:3000
- API Documentation: http://localhost:8000/docs
- ROSbridge WebSocket: ws://localhost:9091

For detailed setup instructions, see [doc/QUICKSTART.md](doc/QUICKSTART.md).

## How it Works

1. **Image Processing**: Upload satellite imagery through web interface
2. **Detection**: Computer vision algorithms identify vine rows, obstacles, and free space
3. **Costmap Generation**: Detection results convert to ROS2 OccupancyGrid messages
4. **Mission Planning**: Web interface allows parameter adjustment and progress monitoring
5. **Navigation**: Generated costmaps integrate with standard ROS2 navigation stack

## Use Cases

**Farm Operations**
- Generate navigation maps from aerial imagery
- Plan autonomous mowing patterns
- Monitor field conditions and obstacles

**Research & Development**  
- Test navigation algorithms in simulation
- Analyze detection parameter performance
- Export data for further analysis

## ROS2 Integration

The system provides standard ROS2 interfaces:

**Services:**
- `/costmap/generate_costmap` - Process satellite imagery
- `/costmap/get_costmap_info` - Retrieve map metadata  
- `/costmap/update_costmap_layer` - Modify specific map layers

**Topics:**
- `/costmap/updates` (nav_msgs/OccupancyGrid) - Real-time map updates
- `/costmap/job_updates` (std_msgs/String) - Processing status

Compatible with nav2, move_base, and other ROS2 navigation frameworks.

## System Requirements

- ROS2 Jazzy or Humble
- Python 3.10+
- Node.js 18+ (for web interface)
- OpenCV, NumPy, scikit-image
- PostgreSQL (for web backend)

## Development

```bash
# Build specific components
colcon build --packages-select vineyard_mower_navigation

# Run tests
colcon test --packages-select vineyard_mower_navigation

# Launch simulation only
ros2 launch vineyard_mower_gazebo robot_simulation.launch.py
```

### Available ROS2 Topics

- `/cmd_vel` - Robot velocity commands (geometry_msgs/Twist)
- `/odom` - Odometry data (nav_msgs/Odometry)
- `/costmap/updates` - Navigation costmap updates (nav_msgs/OccupancyGrid)
- `/robot_description` - Robot URDF description

## Contributing

We welcome contributions to improve navigation algorithms, add new detection methods, or enhance the web interface. Please see individual component READMEs for specific development guidelines.

## License

MIT License
