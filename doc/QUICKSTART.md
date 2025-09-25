# Vineyard Mower - Quick Start Guide

This guide gets you up and running with the complete vineyard navigation system in under 10 minutes.

## Prerequisites

Before starting, ensure you have:

- **Operating System**: Ubuntu 22.04 or 24.04
- **ROS2**: Jazzy or Humble installed
- **Python**: 3.10 or higher
- **Node.js**: 18 or higher
- **Hardware**: 8GB RAM minimum, dedicated GPU recommended

## Installation

### 1. Clone Repository

```bash
cd $HOME
git clone <repository-url> vinmo
cd vinmo
```

### 2. System Dependencies

```bash
# Install ROS2 dependencies
sudo apt update
sudo apt install -y ros-$ROS_DISTRO-navigation2 ros-$ROS_DISTRO-nav2-bringup
sudo apt install -y ros-$ROS_DISTRO-gazebo-ros-pkgs ros-$ROS_DISTRO-rosbridge-suite

# Install Python dependencies
pip install opencv-python numpy scikit-image fastapi uvicorn sqlalchemy psycopg2-binary

# Install Node.js dependencies (for web interface)
curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
sudo apt-get install -y nodejs
```

### 3. Build Workspace

```bash
./setup_workspace.sh
source install/setup.bash
```

## Running the System

### Complete System (Recommended)

Start everything with a single command:

```bash
./launch_vineyard_web.sh
```

This launches:
- ROS2 costmap services
- ROSbridge WebSocket server (port 9091)
- Web backend API (port 8000)  
- React frontend (port 3000)

**Access the system:**
- **Web Interface**: http://localhost:3000
- **API Documentation**: http://localhost:8000/docs
- **ROSbridge**: ws://localhost:9091

### Individual Components

If you prefer to run components separately:

```bash
# Terminal 1: ROS2 services
ros2 launch vineyard_mower_navigation vineyard_web_system.launch.py

# Terminal 2: Web backend (optional, included in launch file)
cd vineyard_costmap_web/backend/src
python main.py

# Terminal 3: Web frontend (optional, included in launch file)  
cd vineyard_costmap_web/frontend
npm run dev
```

### Simulation Only

For testing without the web interface:

```bash
# Basic robot simulation
ros2 launch vineyard_mower_gazebo robot_simulation.launch.py

# With realistic vineyard environment
ros2 launch vineyard_mower_gazebo realistic_vineyard_world.launch.py
```

## First Usage

### 1. Upload Satellite Image

1. Open http://localhost:3000
2. Navigate to "Satellite Images" 
3. Click "Upload Image"
4. Select a satellite image of vineyard terrain
5. Click "Process Image"

### 2. Generate Costmap

1. Go to "Costmap Generation"
2. Select your uploaded image
3. Adjust detection parameters:
   - **Hough Threshold**: Line detection sensitivity
   - **Min Line Length**: Minimum vine row length
   - **Inflation Radius**: Obstacle buffer zone
4. Click "Generate Costmap"

### 3. Monitor Progress

- View real-time processing status in "Jobs" tab
- See generated costmap overlay on satellite imagery
- Export costmap data for external navigation systems

### 4. ROS2 Integration

Test the ROS2 services:

```bash
# Check available services
ros2 service list | grep costmap

# Get costmap info
ros2 service call /costmap/get_costmap_info vineyard_mower_interfaces/srv/GetCostmapInfo

# Subscribe to costmap updates
ros2 topic echo /costmap/updates
```

## Common Issues

### Port Already in Use

If ports 3000, 8000, or 9091 are in use:

```bash
# Kill processes using ports
sudo lsof -ti:3000,8000,9091 | xargs kill -9

# Or use different ports in launch file
ros2 launch vineyard_mower_navigation vineyard_web_system.launch.py frontend_port:=3001
```

### ROS2 Environment

Ensure ROS2 is properly sourced:

```bash
echo $ROS_DISTRO  # Should show 'jazzy' or 'humble'
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
```

### Python Path Issues

If modules aren't found:

```bash
export PYTHONPATH=/opt/ros/$ROS_DISTRO/lib/python3.12/site-packages:$HOME/vinmo/install/vineyard_mower_navigation/lib/python3/dist-packages:$PYTHONPATH
```

### Database Connection

For development, the system uses PostgreSQL. If connection fails:

```bash
# Start PostgreSQL service
sudo systemctl start postgresql

# Or use Docker
cd vineyard_costmap_web
docker-compose up postgres -d
```

## Next Steps

- **Navigation Testing**: Try the simulation environments
- **Parameter Tuning**: Experiment with detection algorithms  
- **Custom Integration**: Use the ROS2 services in your navigation stack
- **Advanced Features**: Explore mission planning and batch processing

## Getting Help

- Check `doc/README_NAVIGATION.md` for navigation details
- See `doc/README_COSTMAP.md` for costmap generation
- Review component READMEs in individual directories
- Check ROS2 logs: `ros2 run rqt_console rqt_console`

The system is designed to work out of the box. If you encounter issues, most problems are related to ROS2 environment setup or missing dependencies.
