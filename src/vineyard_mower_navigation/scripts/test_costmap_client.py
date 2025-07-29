#!/usr/bin/env python3
"""
Test client for costmap generation services
"""

import rclpy
from rclpy.node import Node
import argparse
import sys
import os
from vineyard_mower_interfaces.srv import GenerateCostmap, GetCostmapInfo
from std_srvs.srv import Empty, SetBool


class CostmapTestClient(Node):
    """Test client for costmap services"""
    
    def __init__(self):
        super().__init__('costmap_test_client')
        
        # Service clients
        self.generate_client = self.create_client(GenerateCostmap, 'generate_costmap')
        self.info_client = self.create_client(GetCostmapInfo, 'get_costmap_info')
        self.clear_client = self.create_client(Empty, 'clear_costmap')
        self.auto_update_client = self.create_client(SetBool, 'enable_auto_update')
        
        # Wait for services
        self.get_logger().info("Waiting for costmap services...")
        self.generate_client.wait_for_service(timeout_sec=10.0)
        self.info_client.wait_for_service(timeout_sec=10.0)
        self.get_logger().info("Services available!")
    
    def test_generate_from_coordinates(self, lat: float, lon: float, zoom: int = 18):
        """Test generating costmap from GPS coordinates"""
        request = GenerateCostmap.Request()
        request.center_latitude = lat
        request.center_longitude = lon
        request.zoom_level = zoom
        request.use_local_image = False
        request.image_path = ""
        
        self.get_logger().info(f"Requesting costmap generation for {lat}, {lon}")
        
        future = self.generate_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Success: {response.message}")
                self.get_logger().info(f"Rows detected: {response.rows_detected}")
                self.get_logger().info(f"Obstacles detected: {response.obstacles_detected}")
                self.get_logger().info(f"Row spacing: {response.row_spacing:.2f}m")
                return True
            else:
                self.get_logger().error(f"Failed: {response.message}")
                return False
        else:
            self.get_logger().error("Service call failed")
            return False
    
    def test_generate_from_local_image(self, image_path: str):
        """Test generating costmap from local image"""
        if not os.path.exists(image_path):
            self.get_logger().error(f"Image file not found: {image_path}")
            return False
        
        request = GenerateCostmap.Request()
        request.center_latitude = 0.0
        request.center_longitude = 0.0
        request.zoom_level = 18
        request.use_local_image = True
        request.image_path = image_path
        
        self.get_logger().info(f"Requesting costmap generation from {image_path}")
        
        future = self.generate_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Success: {response.message}")
                self.get_logger().info(f"Rows detected: {response.rows_detected}")
                self.get_logger().info(f"Obstacles detected: {response.obstacles_detected}")
                self.get_logger().info(f"Row spacing: {response.row_spacing:.2f}m")
                return True
            else:
                self.get_logger().error(f"Failed: {response.message}")
                return False
        else:
            self.get_logger().error("Service call failed")
            return False
    
    def get_costmap_info(self):
        """Get current costmap information"""
        request = GetCostmapInfo.Request()
        
        future = self.info_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info("Costmap Information:")
            self.get_logger().info(f"  Has costmap: {response.has_costmap}")
            if response.has_costmap:
                self.get_logger().info(f"  Dimensions: {response.width}x{response.height}")
                self.get_logger().info(f"  Resolution: {response.resolution}m/pixel")
                self.get_logger().info(f"  Origin: ({response.origin_x}, {response.origin_y})")
                self.get_logger().info(f"  Frame: {response.frame_id}")
                self.get_logger().info(f"  Rows detected: {response.rows_detected}")
                self.get_logger().info(f"  Obstacles detected: {response.obstacles_detected}")
                self.get_logger().info(f"  Row spacing: {response.row_spacing:.2f}m")
            self.get_logger().info(f"  Auto-update: {response.auto_update_enabled}")
            return True
        else:
            self.get_logger().error("Service call failed")
            return False
    
    def clear_costmap(self):
        """Clear the current costmap"""
        request = Empty.Request()
        
        future = self.clear_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info("Costmap cleared successfully")
            return True
        else:
            self.get_logger().error("Failed to clear costmap")
            return False
    
    def enable_auto_update(self, enable: bool):
        """Enable or disable auto-update"""
        request = SetBool.Request()
        request.data = enable
        
        future = self.auto_update_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(response.message)
                return True
            else:
                self.get_logger().error(f"Failed: {response.message}")
                return False
        else:
            self.get_logger().error("Service call failed")
            return False


def main():
    """Main function with command line interface"""
    parser = argparse.ArgumentParser(description='Test costmap generation services')
    parser.add_argument('--mode', choices=['coordinates', 'local', 'info', 'clear', 'auto-update'],
                       required=True, help='Test mode')
    parser.add_argument('--lat', type=float, help='Latitude for coordinates mode')
    parser.add_argument('--lon', type=float, help='Longitude for coordinates mode')
    parser.add_argument('--zoom', type=int, default=18, help='Zoom level for satellite imagery')
    parser.add_argument('--image', type=str, help='Path to local image file')
    parser.add_argument('--enable', action='store_true', help='Enable auto-update (disable if not set)')
    
    args = parser.parse_args()
    
    # Initialize ROS2
    rclpy.init()
    
    try:
        client = CostmapTestClient()
        success = False
        
        if args.mode == 'coordinates':
            if args.lat is None or args.lon is None:
                print("Error: --lat and --lon required for coordinates mode")
                return 1
            success = client.test_generate_from_coordinates(args.lat, args.lon, args.zoom)
            
        elif args.mode == 'local':
            if args.image is None:
                print("Error: --image required for local mode")
                return 1
            success = client.test_generate_from_local_image(args.image)
            
        elif args.mode == 'info':
            success = client.get_costmap_info()
            
        elif args.mode == 'clear':
            success = client.clear_costmap()
            
        elif args.mode == 'auto-update':
            success = client.enable_auto_update(args.enable)
        
        return 0 if success else 1
        
    except KeyboardInterrupt:
        print("\nInterrupted by user")
        return 0
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    exit(main())
