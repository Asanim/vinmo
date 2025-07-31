#!/bin/bash

# ROS2 Vineyard Mower Robot Workspace Setup Script

echo "Setting up ROS2 Vineyard Mower Robot workspace..."

first_install() {
    # sudo apt install -y ros-jazzy-gazebo-msgs ros-jazzy-turtlebot3-gazebo
    # sudo apt update && sudo apt install -y ros-jazzy-gazebo-ros-pkgs ros-jazzy-gazebo-ros
    export ROS_DISTRO=jazzy
    sudo apt install -y ros-jazzy-gz-ros2-control

    pip install catkin_pkg

    sudo apt update && sudo apt install -y ros-jazzy-ros-gz ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge ros-jazzy-gz-sim-vendor

    sudo apt install -y  ros-jazzy-ros-gz ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-image ros-jazzy-ros-gz-interfaces\
    sudo apt install -y  ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-sim-demos
    ros-jazzy-gazebo-msgs  ros-jazzy-gz-common-vendor ros-jazzy-gz-launch-vendor ros-jazzy-gz-gui-vendor
    sudo apt-get install -y ros-${ROS_DISTRO}-ros-gz

    sudo apt-get install -y ros-${ROS_DISTRO}-gz-tools-vendor ros-${ROS_DISTRO}-gz-sim-vendor
    sudo apt update && sudo apt install -y ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-gz-ros2-control ros-jazzy-controller-manager ros-jazzy-diff-drive-controller ros-jazzy-joint-state-broadcaster
    sudo apt install -y xterm
    sudo apt-get install -y ros-jazzy-diagnostic-aggregator

    sudo apt install -y ros-jazzy-cartographer ros-jazzy-cartographer-ros ros-jazzy-cartographer-ros-msgs ros-jazzy-cartographer-rviz

    sudo apt install -y ros-jazzy-twist-stamper
    sudo apt  install docker-compose

    sudo systemctl enable docker.service
    sudo systemctl enable containerd.service

    sudo apt install ros-jazzy-rosbridge-*
    pip install psutil bson argcomplete
    pip install pymongo tornado
<<<<<<< Updated upstream
=======
    sudo systemctl enable docker.service
    sudo systemctl enable containerd.service

    sudo apt install ros-jazzy-rosbridge-*
    pip install psutil bson argcomplete
    pip install pymongo tornado

    pip uninstall em
    pip install empty
>>>>>>> Stashed changes
}

first_install

# Source ROS2 installation
source /opt/ros/jazzy/setup.bash

# Build the workspace
echo "Building workspace..."
cd $HOME/vinmo
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
