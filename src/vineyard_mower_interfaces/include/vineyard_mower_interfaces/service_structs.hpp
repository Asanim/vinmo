#ifndef VINEYARD_MOWER_INTERFACES_SERVICE_STRUCTS_HPP_
#define VINEYARD_MOWER_INTERFACES_SERVICE_STRUCTS_HPP_

#include <string>
#include <vector>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace vineyard_mower_interfaces {

// GenerateCostmap Service Structures
struct GenerateCostmapRequest {
    double center_latitude;
    double center_longitude;
    int32_t zoom_level;
    bool use_local_image;
    std::string image_path;
};

struct GenerateCostmapResponse {
    bool success;
    std::string message;
    nav_msgs::msg::OccupancyGrid costmap;
    int32_t rows_detected;
    int32_t obstacles_detected;
    double row_spacing;
};

// UpdateCostmapLayer Service Structures
struct UpdateCostmapLayerRequest {
    std::string layer_name;
    uint32_t width;
    uint32_t height;
    std::vector<uint8_t> layer_data;
};

struct UpdateCostmapLayerResponse {
    bool success;
    std::string message;
};

// GetCostmapInfo Service Structures
struct GetCostmapInfoRequest {
    // Empty request
};

struct GetCostmapInfoResponse {
    bool has_costmap;
    uint32_t width;
    uint32_t height;
    double resolution;
    double origin_x;
    double origin_y;
    std::string frame_id;
    int32_t rows_detected;
    int32_t obstacles_detected;
    double row_spacing;
    bool auto_update_enabled;
};

} // namespace vineyard_mower_interfaces

#endif // VINEYARD_MOWER_INTERFACES_SERVICE_STRUCTS_HPP_
