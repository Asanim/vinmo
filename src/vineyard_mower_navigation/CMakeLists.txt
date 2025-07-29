cmake_minimum_required(VERSION 3.8)
project(vineyard_mower_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install directories
install(
  DIRECTORY launch config maps scripts vineyard_mower_navigation test
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(
  PROGRAMS 
    scripts/laser_scan_frame_remapper.py
    scripts/gps_reference_publisher.py
    scripts/vineyard_row_detector.py
    scripts/test_navigation_integration.py
    scripts/test_costmap_client.py
    scripts/generate_test_images.py
    scripts/costmap_publisher.py
    scripts/costmap_service.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(
  PROGRAMS 
    scripts/costmap_publisher.py
    scripts/costmap_service.py
  DESTINATION bin/${PROJECT_NAME}
)


# Install Python modules
install(
  DIRECTORY vineyard_mower_navigation/
  DESTINATION lib/python3/dist-packages/vineyard_mower_navigation
  FILES_MATCHING PATTERN "*.py"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
