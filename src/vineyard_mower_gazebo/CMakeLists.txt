cmake_minimum_required(VERSION 3.8)
project(vineyard_mower_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install directories
install(
  DIRECTORY launch worlds models config materials
  DESTINATION share/${PROJECT_NAME}
)

# Install scripts
install(
  PROGRAMS
    scripts/vineyard_environment_monitor.py
    scripts/generate_vineyard_world.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
