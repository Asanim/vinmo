#ifndef VINEYARD_MOWER_INTERFACES_JSON_SERVICES_HPP_
#define VINEYARD_MOWER_INTERFACES_JSON_SERVICES_HPP_

#include <nlohmann/json.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <rclcpp/rclcpp.hpp>

namespace vineyard_mower_interfaces {

class JsonServiceWrapper {
public:
    // Convert GenerateCostmap request to JSON
    static std_msgs::msg::String generateCostmapRequestToJson(
        double center_latitude, double center_longitude, int32_t zoom_level,
        bool use_local_image, const std::string& image_path) {
        
        nlohmann::json j;
        j["service_type"] = "GenerateCostmap";
        j["center_latitude"] = center_latitude;
        j["center_longitude"] = center_longitude;
        j["zoom_level"] = zoom_level;
        j["use_local_image"] = use_local_image;
        j["image_path"] = image_path;
        
        std_msgs::msg::String msg;
        msg.data = j.dump();
        return msg;
    }
    
    // Convert UpdateCostmapLayer request to JSON
    static std_msgs::msg::String updateCostmapLayerRequestToJson(
        const std::string& layer_name, uint32_t width, uint32_t height,
        const std::vector<uint8_t>& layer_data) {
        
        nlohmann::json j;
        j["service_type"] = "UpdateCostmapLayer";
        j["layer_name"] = layer_name;
        j["width"] = width;
        j["height"] = height;
        j["layer_data"] = layer_data;
        
        std_msgs::msg::String msg;
        msg.data = j.dump();
        return msg;
    }
    
    // Convert GetCostmapInfo request to JSON
    static std_msgs::msg::String getCostmapInfoRequestToJson() {
        nlohmann::json j;
        j["service_type"] = "GetCostmapInfo";
        
        std_msgs::msg::String msg;
        msg.data = j.dump();
        return msg;
    }
    
    // Parse JSON response
    static nlohmann::json parseJsonResponse(const std_msgs::msg::String& response) {
        return nlohmann::json::parse(response.data);
    }
};

} // namespace vineyard_mower_interfaces

#endif // VINEYARD_MOWER_INTERFACES_JSON_SERVICES_HPP_
