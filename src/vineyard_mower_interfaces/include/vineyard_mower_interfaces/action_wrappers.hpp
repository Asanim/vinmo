#ifndef VINEYARD_MOWER_INTERFACES_ACTION_WRAPPERS_HPP_
#define VINEYARD_MOWER_INTERFACES_ACTION_WRAPPERS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace vineyard_mower_interfaces {

// Base class for action-based service communication
class ActionServiceBase {
public:
    explicit ActionServiceBase(rclcpp::Node::SharedPtr node) : node_(node) {}
    
protected:
    rclcpp::Node::SharedPtr node_;
    
    // Generic action goal/result structure using String messages
    struct GenericGoal {
        std_msgs::msg::String request_data;
    };
    
    struct GenericResult {
        std_msgs::msg::String response_data;
        bool success;
    };
    
    struct GenericFeedback {
        std_msgs::msg::String status_message;
        double progress_percentage;
    };
};

// Costmap generation using topic-based communication
class CostmapServiceTopics {
public:
    explicit CostmapServiceTopics(rclcpp::Node::SharedPtr node) : node_(node) {
        // Publishers for requests
        generate_costmap_pub_ = node_->create_publisher<std_msgs::msg::String>(
            "vineyard/generate_costmap/request", 10);
        update_layer_pub_ = node_->create_publisher<std_msgs::msg::String>(
            "vineyard/update_costmap_layer/request", 10);
        get_info_pub_ = node_->create_publisher<std_msgs::msg::String>(
            "vineyard/get_costmap_info/request", 10);
            
        // Subscribers for responses
        generate_costmap_sub_ = node_->create_subscription<std_msgs::msg::String>(
            "vineyard/generate_costmap/response", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                this->handleGenerateCostmapResponse(msg);
            });
        update_layer_sub_ = node_->create_subscription<std_msgs::msg::String>(
            "vineyard/update_costmap_layer/response", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                this->handleUpdateLayerResponse(msg);
            });
        get_info_sub_ = node_->create_subscription<std_msgs::msg::String>(
            "vineyard/get_costmap_info/response", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                this->handleGetInfoResponse(msg);
            });
    }
    
    void requestGenerateCostmap(double lat, double lon, int32_t zoom, 
                               bool use_local, const std::string& path) {
        std_msgs::msg::String msg;
        nlohmann::json j;
        j["center_latitude"] = lat;
        j["center_longitude"] = lon;
        j["zoom_level"] = zoom;
        j["use_local_image"] = use_local;
        j["image_path"] = path;
        j["request_id"] = generateRequestId();
        msg.data = j.dump();
        generate_costmap_pub_->publish(msg);
    }
    
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr generate_costmap_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr update_layer_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr get_info_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr generate_costmap_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr update_layer_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr get_info_sub_;
    
    std::string generateRequestId() {
        return std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
    }
    
    void handleGenerateCostmapResponse(const std_msgs::msg::String::SharedPtr msg) {
        // Handle response
        RCLCPP_INFO(node_->get_logger(), "Received generate costmap response: %s", msg->data.c_str());
    }
    
    void handleUpdateLayerResponse(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(node_->get_logger(), "Received update layer response: %s", msg->data.c_str());
    }
    
    void handleGetInfoResponse(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(node_->get_logger(), "Received get info response: %s", msg->data.c_str());
    }
};

} // namespace vineyard_mower_interfaces

#endif // VINEYARD_MOWER_INTERFACES_ACTION_WRAPPERS_HPP_
