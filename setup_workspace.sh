#!/bin/bash

# ROS2 Vineyard Mower Robot Workspace Setup Script

echo "Setting up ROS2 Vineyard Mower Robot workspace..."

# Source ROS2 installation
source /opt/ros/jazzy/setup.bash

# Build the workspace
echo "Building workspace..."
cd /home/sam/vinmo
colcon build --symlink-install

# Source the workspace
source install/setup.bash

echo "Workspace setup complete!"
echo ""
echo "To run the robot simulation:"
echo "  ros2 launch vineyard_mower_gazebo robot_simulation.launch.py"
echo ""
echo "To view the robot in RViz only:"
echo "  ros2 launch vineyard_mower_description display.launch.py"
echo ""
echo "To run Gazebo only:"
echo "  ros2 launch vineyard_mower_gazebo gazebo.launch.py"
