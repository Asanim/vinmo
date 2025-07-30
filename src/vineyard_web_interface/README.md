# Vineyard Web Interface Package

ROS2 package for launching and managing the vineyard costmap web interface system.

## Overview

This package provides launch scripts and utilities for running the complete vineyard management web application alongside ROS2 simulation and navigation systems.

## Launch Scripts

### 1. Simple Web Interface (`web_interface_simple.launch.py`)

Basic launch script for testing web interface functionality.

**Components Started:**
- ROSbridge WebSocket server (port 9090)
- ROSbridge TCP server (port 9091)
- Costmap service nodes
- Path planning service nodes
- Backend API server (port 8000)
- Frontend development server (port 3000, optional)
- Web interface monitor

**Usage:**
```bash
# Start all components
ros2 launch vineyard_web_interface web_interface_simple.launch.py

# Start without frontend (backend and ROS only)
ros2 launch vineyard_web_interface web_interface_simple.launch.py enable_frontend:=false

# Custom ports
ros2 launch vineyard_web_interface web_interface_simple.launch.py rosbridge_port:=9095 backend_port:=8080
```

### 2. Complete Web Interface (`web_interface_complete.launch.py`)

launch script that includes the full simulation environment.

**Components Started:**
- Complete Gazebo vineyard simulation
- Robot with sensors and controllers
- SLAM and navigation stack
- Costmap generation services
- Path planning services
- Web interface (frontend + backend)
- ROSbridge servers
- Monitoring and diagnostics
- RViz visualization

**Usage:**
```bash
# Start complete system
ros2 launch vineyard_web_interface web_interface_complete.launch.py

# Web interface only (no simulation)
ros2 launch vineyard_web_interface web_interface_complete.launch.py enable_simulation:=false

# Simulation without web interface
ros2 launch vineyard_web_interface web_interface_complete.launch.py enable_web:=false

# Custom configuration
ros2 launch vineyard_web_interface web_interface_complete.launch.py \
  enable_slam:=true \
  enable_navigation:=true \
  enable_web:=true \
  rosbridge_port:=9090 \
  backend_port:=8000 \
  frontend_port:=3000
```

## Configuration

Main configuration file: `config/web_interface_config.yaml`

Key configuration sections:
- Server ports and addresses
- Database connection settings
- Authentication configuration
- File upload limits
- Monitoring intervals
- ROS service mappings

## Scripts

### Web Interface Monitor (`scripts/web_interface_monitor.py`)

Monitors health of web interface components:
- ROSbridge connectivity
- Backend API health
- Frontend server status
- Publishes status to `/web_interface/status`

### System Status Publisher (`scripts/system_status_publisher.py`)

Publishes system information:
- System resources (CPU, memory, disk)
- ROS service availability
- Process status
- Publishes to `/system/status` and `/diagnostics`

### Integration Test Runner (`scripts/integration_test_runner.py`)

Automated testing of system integration:
- ROSbridge connectivity tests
- API endpoint tests
- ROS service availability tests
- Web-to-ROS communication tests

## Topics

### Published Topics

- `/web_interface/status` (std_msgs/String): Web interface component status
- `/system/status` (std_msgs/String): System status information
- `/diagnostics` (diagnostic_msgs/DiagnosticArray): System diagnostics
- `/integration_tests/results` (std_msgs/String): Integration test results

### Subscribed Topics

- `/costmap/updates` (nav_msgs/OccupancyGrid): Costmap updates for web display
- `/path_planning/status` (std_msgs/String): Path planning status updates

## Services

The web interface integrates with these ROS services:

- `/costmap/generate_costmap` (vineyard_mower_interfaces/GenerateCostmap)
- `/costmap/update_costmap_layer` (vineyard_mower_interfaces/UpdateCostmapLayer)
- `/costmap/get_costmap_info` (vineyard_mower_interfaces/GetCostmapInfo)
- `/path_planning/plan_path` (vineyard_mower_interfaces/PlanPath)

## Dependencies

### ROS2 Packages
- `rosbridge_server`: WebSocket bridge for web communication
- `tf2_web_republisher`: TF2 data for web visualization
- `vineyard_mower_interfaces`: Custom service definitions
- `vineyard_mower_navigation`: Navigation and costmap services
- `vineyard_mower_gazebo`: Simulation environment
- `vineyard_mower_description`: Robot model

### Python Packages
- `websocket-client`: WebSocket connectivity testing
- `requests`: HTTP client for API testing
- `psutil`: System resource monitoring

### Web Application
The web application should be located at the path specified by the `web_app_path` parameter (default: `/home/sam/vinmo/vineyard_costmap_web`).

## Development

### Adding New Components

1. Add launch configuration in the appropriate launch file
2. Update monitoring scripts to check new component health
3. Add integration tests for new functionality
4. Update configuration file with new parameters

### Testing

Run integration tests:
```bash
ros2 launch vineyard_web_interface web_interface_complete.launch.py run_integration_tests:=true
```

Monitor system status:
```bash
ros2 topic echo /system/status
ros2 topic echo /web_interface/status
```

## Troubleshooting

### Common Issues

1. **ROSbridge not connecting**: Check if port 9090 is available
2. **Backend API not responding**: Verify Python dependencies are installed
3. **Frontend not loading**: Ensure Node.js and npm dependencies are installed
4. **Service calls failing**: Check if costmap and path planning nodes are running

### Debug Mode

Enable debug logging:
```bash
ros2 launch vineyard_web_interface web_interface_complete.launch.py --debug
```

### Port Conflicts

If default ports are in use, specify alternative ports:
```bash
ros2 launch vineyard_web_interface web_interface_simple.launch.py \
  rosbridge_port:=9095 \
  backend_port:=8080 \
  frontend_port:=3001
```

## Security Notes

- Change default database credentials in production
- Update authentication secret keys
- Configure firewall rules for web server ports
- Use HTTPS in production environments
