#!/bin/bash

# Vineyard Costmap Web System Launcher
# This script provides an easy way to start the entire system

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}=================================================================================${NC}"
echo -e "${BLUE}                    Vineyard Costmap Web System Launcher${NC}"
echo -e "${BLUE}=================================================================================${NC}"

# Check if we're in the right directory
if [ ! -d "vineyard_costmap_web" ]; then
    echo -e "${RED}Error: vineyard_costmap_web directory not found!${NC}"
    echo -e "${YELLOW}Please run this script from /home/sam/vinmo/${NC}"
    exit 1
fi

# Function to check if a port is in use
check_port() {
    local port=$1
    if lsof -Pi :$port -sTCP:LISTEN -t >/dev/null ; then
        return 0  # Port is in use
    else
        return 1  # Port is free
    fi
}

# Function to kill process on port
kill_port() {
    local port=$1
    echo -e "${YELLOW}Killing process on port $port...${NC}"
    fuser -k $port/tcp 2>/dev/null || true
    sleep 2
}

# Check and handle port conflicts
echo -e "${YELLOW}Checking for port conflicts...${NC}"

if check_port 9090; then
    echo -e "${YELLOW}Port 9090 is in use. Killing process...${NC}"
    kill_port 9090
fi

if check_port 9091; then
    echo -e "${YELLOW}Port 9091 is in use. Killing process...${NC}"
    kill_port 9091
fi

if check_port 8000; then
    echo -e "${YELLOW}Port 8000 is in use. Killing process...${NC}"
    kill_port 8000
fi

if check_port 3000; then
    echo -e "${YELLOW}Port 3000 is in use. Killing process...${NC}"
    kill_port 3000
fi

# Set up ROS environment
echo -e "${YELLOW}Setting up ROS environment...${NC}"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Build the workspace if needed
if [ "$1" = "--build" ]; then
    echo -e "${YELLOW}Building workspace...${NC}"
    colcon build --packages-select vineyard_mower_navigation
    source install/setup.bash
fi

echo -e "${GREEN}Starting Vineyard Costmap Web System...${NC}"
echo -e "${BLUE}This will start:${NC}"
echo -e "  - ROS Costmap Service & Publisher"
echo -e "  - ROSbridge WebSocket Server (port 9091)"
echo -e "  - Web Backend API (port 8000)"
echo -e "  - Web Frontend (port 3000)"
echo ""
echo -e "${GREEN}Access the web interface at: http://localhost:3000${NC}"
echo -e "${GREEN}API documentation at: http://localhost:8000/docs${NC}"
echo ""
echo -e "${YELLOW}Press Ctrl+C to stop all services${NC}"
echo ""

# Launch the system
ros2 launch vineyard_mower_navigation vineyard_web_system.launch.py
