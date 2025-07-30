#!/bin/bash

# Navigation2 Setup Script for Vineyard Mower
echo "Setting up Navigation2 stack for vineyard autonomous navigation..."

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if we're in a ROS2 workspace
if [ ! -f "src/vineyard_mower_navigation/package.xml" ]; then
    echo -e "${RED}Error: Please run this script from the root of your ROS2 workspace${NC}"
    exit 1
fi

# Install Nav2 dependencies
echo -e "${YELLOW}Installing Navigation2 dependencies...${NC}"
sudo apt update
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-common \
    ros-humble-nav2-msgs \
    ros-humble-nav2-rviz-plugins \
    ros-humble-behaviortree-cpp-v3 \
    ros-humble-twist-stamper \
    ros-humble-robot-localization \
    ros-humble-tf2-sensor-msgs

# Build the workspace
echo -e "${YELLOW}Building workspace...${NC}"
colcon build --packages-select vineyard_mower_navigation --symlink-install

# Source the workspace
echo -e "${YELLOW}Sourcing workspace...${NC}"
source install/setup.bash

# Make scripts executable
echo -e "${YELLOW}Making scripts executable...${NC}"
chmod +x src/vineyard_mower_navigation/scripts/*.py

# Verify installation
echo -e "${YELLOW}Verifying Nav2 installation...${NC}"
if ros2 pkg list | grep -q nav2_bringup; then
    echo -e "${GREEN}✓ Nav2 packages installed successfully${NC}"
else
    echo -e "${RED}✗ Nav2 installation failed${NC}"
    exit 1
fi

# Check for required Nav2 nodes
echo -e "${YELLOW}Checking Nav2 node availability...${NC}"
nav2_nodes=(
    "nav2_controller"
    "nav2_planner"
    "nav2_bt_navigator"
    "nav2_lifecycle_manager"
    "nav2_map_server"
    "nav2_amcl"
)

for node in "${nav2_nodes[@]}"; do
    if ros2 pkg executables nav2_bringup | grep -q "$node" || \
       ros2 pkg executables "${node}" | grep -q "${node}" 2>/dev/null; then
        echo -e "${GREEN}✓ $node available${NC}"
    else
        echo -e "${YELLOW}⚠ $node not found (may be in different package)${NC}"
    fi
done

echo -e "${GREEN}Navigation2 setup completed!${NC}"
echo ""
echo -e "${YELLOW}Next steps:${NC}"
echo "1. Build your workspace: colcon build"
echo "2. Source the workspace: source install/setup.bash"
echo "3. Launch simulation with navigation:"
echo "   ros2 launch vineyard_mower_gazebo robot_simulation_navigation.launch.py enable_navigation:=true"
echo "4. Or launch navigation separately:"
echo "   ros2 launch vineyard_mower_navigation vineyard_navigation.launch.py"
echo ""
echo -e "${YELLOW}Testing:${NC}"
echo "Run integration tests with:"
echo "   ros2 run vineyard_mower_navigation test_navigation_integration.py"
