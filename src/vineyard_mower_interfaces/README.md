# Vineyard Mower Interfaces - rosidl Alternatives

This package provides alternatives to using `rosidl` for custom ROS 2 interfaces. If you're having compilation issues with `rosidl`, you can use one of the following approaches:

## Alternative 1: Header-Only Structs

Use the structures defined in `include/vineyard_mower_interfaces/service_structs.hpp`:

```cpp
#include "vineyard_mower_interfaces/service_structs.hpp"

// Example usage
vineyard_mower_interfaces::GenerateCostmapRequest request;
request.center_latitude = 45.0;
request.center_longitude = -123.0;
request.zoom_level = 15;
request.use_local_image = false;
request.image_path = "";

vineyard_mower_interfaces::GenerateCostmapResponse response;
// Process request and fill response...
```

## Alternative 2: JSON-based Communication

Use standard ROS 2 String messages with JSON payloads:

```cpp
#include "vineyard_mower_interfaces/json_services.hpp"

// Create a JSON request
auto request_msg = JsonServiceWrapper::generateCostmapRequestToJson(
    45.0, -123.0, 15, false, "");

// Publish the request
publisher->publish(request_msg);

// Parse JSON response
auto response_json = JsonServiceWrapper::parseJsonResponse(response_msg);
bool success = response_json["success"];
std::string message = response_json["message"];
```

## Alternative 3: Topic-based Communication

Use the action wrapper classes for topic-based service communication:

```cpp
#include "vineyard_mower_interfaces/action_wrappers.hpp"

auto node = rclcpp::Node::make_shared("vineyard_client");
auto costmap_service = std::make_shared<CostmapServiceTopics>(node);

// Request costmap generation
costmap_service->requestGenerateCostmap(45.0, -123.0, 15, false, "");
```

## Alternative 4: Standard ROS 2 Services

You can also use standard ROS 2 services like `std_srvs/srv/Trigger` and pass data as JSON strings in the request/response data fields.

## Installation

1. Install nlohmann/json (required for JSON alternatives):
   ```bash
   sudo apt install nlohmann-json3-dev
   ```

2. Build the package:
   ```bash
   cd /home/sam/vinmo
   colcon build --packages-select vineyard_mower_interfaces
   ```

## Migration from rosidl

If you were previously using the custom services, here's how to migrate:

### Old (rosidl) way:
```cpp
#include "vineyard_mower_interfaces/srv/generate_costmap.hpp"
using GenerateCostmap = vineyard_mower_interfaces::srv::GenerateCostmap;

auto client = node->create_client<GenerateCostmap>("generate_costmap");
auto request = std::make_shared<GenerateCostmap::Request>();
// ...
```

### New (header-only) way:
```cpp
#include "vineyard_mower_interfaces/service_structs.hpp"
#include <std_srvs/srv/trigger.hpp>

auto client = node->create_client<std_srvs::srv::Trigger>("generate_costmap");
// Use JSON or the struct approach to pass data
```

## Troubleshooting

If you still want to try fixing rosidl:

1. Install missing dependencies:
   ```bash
   sudo apt update
   sudo apt install ros-${ROS_DISTRO}-rosidl-default-generators
   sudo apt install ros-${ROS_DISTRO}-rosidl-default-runtime
   ```

2. Check your ROS 2 installation:
   ```bash
   ros2 pkg list | grep rosidl
   ```

3. Try building with verbose output:
   ```bash
   colcon build --packages-select vineyard_mower_interfaces --cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON
   ```
