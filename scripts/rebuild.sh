# source /opt/ros/jazzy/setup.bash
# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
rm -rf build/ install/ log/
colcon build --symlink-install
source install/setup.bash