cmake_minimum_required(VERSION 3.8)
project(vineyard_mower_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

# Try to find nlohmann_json, if not available, we'll provide instructions
find_package(nlohmann_json QUIET)
if(NOT nlohmann_json_FOUND)
  message(WARNING "nlohmann_json not found. Install with: sudo apt install nlohmann-json3-dev")
endif()

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Export dependencies and include directories
ament_export_include_directories(include)
ament_export_dependencies(builtin_interfaces std_msgs geometry_msgs nav_msgs std_srvs rclcpp rclcpp_action)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
