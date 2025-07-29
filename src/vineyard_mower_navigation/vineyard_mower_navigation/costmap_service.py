#!/usr/bin/env python3
"""
Costmap Service Node

This node provides ROS2 services for costmap updates and integrates
the satellite imagery processing with costmap generation.
"""

import rclpy
from rclpy.node import Node
from rclpy.service import Service
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from std_msgs.msg import String, Bool
from std_srvs.srv import Empty, SetBool, Trigger
import numpy as np
import cv2
import os
import json
from typing import Optional, Dict, Any, Tuple
import logging
from threading import Lock
from datetime import datetime

# Custom service messages - using header-only approach instead of rosidl
# from vineyard_mower_interfaces.srv import (
#     GenerateCostmap, 
#     UpdateCostmapLayer,
#     GetCostmapInfo
# )

# Import our header-only service structures
try:
    from vineyard_mower_interfaces.service_structs import (
        GenerateCostmapRequest,
        GenerateCostmapResponse,
        UpdateCostmapLayerRequest,
        UpdateCostmapLayerResponse,
        GetCostmapInfoRequest,
        GetCostmapInfoResponse
    )
except ImportError:
    # Fallback to local definitions if package not available
    class GenerateCostmapRequest:
        def __init__(self):
            self.center_latitude = 0.0
            self.center_longitude = 0.0
            self.zoom_level = 15
            self.use_local_image = False
            self.image_path = ""
    
    class GenerateCostmapResponse:
        def __init__(self):
            self.success = False
            self.message = ""
            self.costmap = None
            self.rows_detected = 0
            self.obstacles_detected = 0
            self.row_spacing = 0.0

from .satellite_processor import SatelliteImageProcessor, VineyardDetector
from .costmap_generator import CostmapGenerator, CostmapConfig

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class CostmapServiceNode(Node):
    """
    Main service node for costmap generation and management
    """
    
    def __init__(self):
        super().__init__('costmap_service')
        
        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('google_maps_api_key', ''),
                ('costmap_config_file', ''),
                ('default_resolution', 0.1),
                ('default_width', 1000),
                ('default_height', 1000),
                ('auto_update_rate', 30.0),  # seconds
                ('save_costmaps', True),
                ('costmap_save_path', '/tmp/costmaps/'),
                # Service request parameters
                ('center_latitude', 0.0),
                ('center_longitude', 0.0),
                ('zoom_level', 18),
                ('use_local_image', False),
                ('image_path', '')
            ]
        )
        
        # Get parameters
        self.google_api_key = self.get_parameter('google_maps_api_key').value
        self.config_file = self.get_parameter('costmap_config_file').value
        self.auto_update_rate = self.get_parameter('auto_update_rate').value
        self.save_costmaps = self.get_parameter('save_costmaps').value
        self.save_path = self.get_parameter('costmap_save_path').value
        
        # Initialize processors
        self.satellite_processor = SatelliteImageProcessor(self.google_api_key)
        self.vineyard_detector = VineyardDetector()
        
        # Initialize costmap configuration
        self.costmap_config = CostmapConfig(
            resolution=self.get_parameter('default_resolution').value,
            width=self.get_parameter('default_width').value,
            height=self.get_parameter('default_height').value
        )
        
        # Load config file if provided
        if self.config_file and os.path.exists(self.config_file):
            try:
                self.costmap_generator = CostmapGenerator(self.costmap_config)
                self.costmap_generator.load_config_from_file(self.config_file)
                self.get_logger().info(f"Loaded config from {self.config_file}")
            except Exception as e:
                self.get_logger().error(f"Failed to load config: {e}")
                self.costmap_generator = CostmapGenerator(self.costmap_config)
        else:
            self.costmap_generator = CostmapGenerator(self.costmap_config)
        
        # State management
        self.current_costmap: Optional[OccupancyGrid] = None
        self.last_detection_result = None
        self.costmap_lock = Lock()
        
        # Publishers
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.global_costmap_pub = self.create_publisher(
            OccupancyGrid, 'global_costmap', qos_profile)
        self.local_costmap_pub = self.create_publisher(
            OccupancyGrid, 'local_costmap', qos_profile)
        self.status_pub = self.create_publisher(
            String, 'costmap_status', qos_profile)
        
        # Services - using standard ROS 2 services with JSON payloads
        self.generate_service = self.create_service(
            Trigger, 'generate_costmap', self.generate_costmap_callback)
        self.update_layer_service = self.create_service(
            Trigger, 'update_costmap_layer', self.update_layer_callback)
        self.get_info_service = self.create_service(
            Trigger, 'get_costmap_info', self.get_info_callback)
        self.clear_service = self.create_service(
            Empty, 'clear_costmap', self.clear_costmap_callback)
        self.enable_auto_update_service = self.create_service(
            SetBool, 'enable_auto_update', self.enable_auto_update_callback)
        
        # Auto-update timer (disabled by default)
        self.auto_update_enabled = False
        self.auto_update_timer = None
        
        # Create save directory
        if self.save_costmaps:
            os.makedirs(self.save_path, exist_ok=True)
        
        self.get_logger().info("CostmapServiceNode initialized")
        self.publish_status("initialized")
    
    def generate_costmap_callback(self, request, response):
        """
        Service callback for generating costmap from satellite imagery
        Uses parameters instead of service request fields
        """
        self.get_logger().info("Received costmap generation request")
        self.publish_status("processing")
        
        try:
            # Use parameters instead of request fields
            center_lat = self.get_parameter('center_latitude').get_parameter_value().double_value
            center_lon = self.get_parameter('center_longitude').get_parameter_value().double_value
            zoom_level = self.get_parameter('zoom_level').get_parameter_value().integer_value
            use_local_image = self.get_parameter('use_local_image').get_parameter_value().bool_value
            image_path = self.get_parameter('image_path').get_parameter_value().string_value
            
            # Get satellite image
            if use_local_image and image_path:
                image = self.satellite_processor.load_local_image(image_path)
                self.get_logger().info(f"Using local image: {image_path}")
            else:
                image = self.satellite_processor.fetch_satellite_image(
                    center_lat, center_lon, zoom_level)
                self.get_logger().info(f"Fetched satellite image at {center_lat}, {center_lon}")
            
            if image is None:
                response.success = False
                response.message = "Failed to acquire satellite image"
                self.publish_status("error")
                return response
            
            # Preprocess image
            processed_image = self.satellite_processor.preprocess_image(image)
            
            # Detect vineyard structure
            detection_result = self.vineyard_detector.detect_vine_rows(processed_image)
            self.last_detection_result = detection_result
            
            # Generate costmap
            with self.costmap_lock:
                costmap = self.costmap_generator.generate_from_detection(detection_result)
                self.current_costmap = costmap
            
            # Publish costmap
            self.global_costmap_pub.publish(costmap)
            
            # Save costmap if enabled
            if self.save_costmaps:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                costmap_path = os.path.join(self.save_path, f"costmap_{timestamp}.png")
                detection_path = os.path.join(self.save_path, f"detection_{timestamp}.png")
                
                # Save costmap visualization
                combined_costmap = self.costmap_generator._combine_layers()
                self.costmap_generator.save_costmap(combined_costmap, costmap_path)
                
                # Save detection visualization  
                self.vineyard_detector.visualize_detection(
                    processed_image, detection_result, detection_path)
            
            # Prepare response
            response.success = True
            response.message = f"Generated costmap with {len(detection_result.vine_rows)} vine rows"
            response.costmap = costmap
            response.rows_detected = len(detection_result.vine_rows)
            response.obstacles_detected = len(detection_result.obstacles)
            response.row_spacing = detection_result.row_spacing
            
            self.get_logger().info(response.message)
            self.publish_status("completed")
            
        except Exception as e:
            error_msg = f"Error generating costmap: {str(e)}"
            self.get_logger().error(error_msg)
            response.success = False
            response.message = error_msg
            self.publish_status("error")
        
        return response
    
    def update_layer_callback(self, request, response):
        """
        Service callback for updating specific costmap layers
        """
        self.get_logger().info(f"Updating costmap layer: {request.layer_name}")
        
        try:
            # Convert layer data from message format
            layer_data = np.array(request.layer_data, dtype=np.uint8)
            layer_data = layer_data.reshape((request.height, request.width))
            
            # Update the layer
            with self.costmap_lock:
                self.costmap_generator.update_layer(request.layer_name, layer_data)
                
                # Regenerate combined costmap
                if self.last_detection_result:
                    self.current_costmap = self.costmap_generator.generate_from_detection(
                        self.last_detection_result)
                    self.global_costmap_pub.publish(self.current_costmap)
            
            response.success = True
            response.message = f"Updated layer {request.layer_name}"
            self.get_logger().info(response.message)
            
        except Exception as e:
            error_msg = f"Error updating layer: {str(e)}"
            self.get_logger().error(error_msg)
            response.success = False
            response.message = error_msg
        
        return response
    
    def get_info_callback(self, request, response):
        """
        Service callback for getting costmap information
        """
        try:
            with self.costmap_lock:
                if self.current_costmap:
                    response.has_costmap = True
                    response.width = self.current_costmap.info.width
                    response.height = self.current_costmap.info.height
                    response.resolution = self.current_costmap.info.resolution
                    response.origin_x = self.current_costmap.info.origin.position.x
                    response.origin_y = self.current_costmap.info.origin.position.y
                    response.frame_id = self.current_costmap.header.frame_id
                else:
                    response.has_costmap = False
                
                if self.last_detection_result:
                    response.rows_detected = len(self.last_detection_result.vine_rows)
                    response.obstacles_detected = len(self.last_detection_result.obstacles)
                    response.row_spacing = self.last_detection_result.row_spacing
                else:
                    response.rows_detected = 0
                    response.obstacles_detected = 0
                    response.row_spacing = 0.0
            
            response.auto_update_enabled = self.auto_update_enabled
            
        except Exception as e:
            self.get_logger().error(f"Error getting costmap info: {e}")
        
        return response
    
    def clear_costmap_callback(self, request, response):
        """
        Service callback for clearing the costmap
        """
        self.get_logger().info("Clearing costmap")
        
        try:
            with self.costmap_lock:
                # Clear all layers
                for layer in self.costmap_generator.layers.values():
                    layer.clear()
                
                # Reset state
                self.current_costmap = None
                self.last_detection_result = None
            
            self.publish_status("cleared")
            self.get_logger().info("Costmap cleared")
            
        except Exception as e:
            self.get_logger().error(f"Error clearing costmap: {e}")
        
        return response
    
    def enable_auto_update_callback(self, request, response):
        """
        Service callback for enabling/disabling auto-update
        """
        self.auto_update_enabled = request.data
        
        if self.auto_update_enabled:
            if self.auto_update_timer is None:
                self.auto_update_timer = self.create_timer(
                    self.auto_update_rate, self.auto_update_callback)
            response.message = f"Auto-update enabled with rate {self.auto_update_rate}s"
        else:
            if self.auto_update_timer is not None:
                self.auto_update_timer.cancel()
                self.auto_update_timer = None
            response.message = "Auto-update disabled"
        
        response.success = True
        self.get_logger().info(response.message)
        return response
    
    def auto_update_callback(self):
        """
        Timer callback for automatic costmap updates
        """
        if self.last_detection_result:
            self.get_logger().debug("Auto-updating costmap")
            
            try:
                with self.costmap_lock:
                    # Regenerate costmap with current detection
                    costmap = self.costmap_generator.generate_from_detection(
                        self.last_detection_result)
                    self.current_costmap = costmap
                    self.global_costmap_pub.publish(costmap)
                
            except Exception as e:
                self.get_logger().error(f"Error in auto-update: {e}")
    
    def publish_status(self, status: str):
        """Publish current status"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        node = CostmapServiceNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
