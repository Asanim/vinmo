#!/usr/bin/env python3
"""
Advanced Perception Fusion Node for Vineyard Mower Robot

This node fuses LiDAR and depth camera data to create a comprehensive
obstacle detection system with confidence weighting and sensor health monitoring.

Author: Vineyard Robotics Team
License: MIT
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import numpy as np
import cv2
from cv_bridge import CvBridge
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import TransformException
import math
from threading import Lock

# ROS2 message types
from sensor_msgs.msg import LaserScan, Image, PointCloud2, CameraInfo
from geometry_msgs.msg import PointStamped, TransformStamped
from std_msgs.msg import Header, Float32, String
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import OccupancyGrid

# Point cloud processing
import pcl_ros
from laser_geometry import LaserProjection

# Message filtering for synchronization
from message_filters import Subscriber, TimeSynchronizer, ApproximateTimeSynchronizer


class AdvancedPerceptionFusion(Node):
    """
    Advanced sensor fusion node that combines LiDAR and depth camera data
    for robust obstacle detection in vineyard environments.
    """
    
    def __init__(self):
        super().__init__('advanced_perception_fusion')
        
        # Initialize parameters
        self.declare_parameters()
        self.load_parameters()
        
        # Initialize utilities
        self.cv_bridge = CvBridge()
        self.laser_projection = LaserProjection()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.data_lock = Lock()
        
        # Sensor health tracking
        self.sensor_health = {
            'lidar': {'last_update': 0.0, 'confidence': 1.0, 'active': False},
            'front_camera': {'last_update': 0.0, 'confidence': 1.0, 'active': False},
            'rear_camera': {'last_update': 0.0, 'confidence': 1.0, 'active': False}
        }
        
        # Data buffers
        self.latest_scan = None
        self.latest_front_depth = None
        self.latest_rear_depth = None
        self.latest_front_rgb = None
        self.latest_rear_rgb = None
        
        # Initialize QoS profiles
        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5
        )
        
        self.reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Setup subscribers
        self.setup_subscribers()
        
        # Setup publishers
        self.setup_publishers()
        
        # Setup timers
        self.fusion_timer = self.create_timer(1.0 / self.fusion_rate, self.fusion_callback)
        self.health_timer = self.create_timer(1.0, self.health_monitoring_callback)
        
        self.get_logger().info("Advanced Perception Fusion Node initialized")
    
    def declare_parameters(self):
        """Declare node parameters with default values"""
        self.declare_parameter('fusion_rate', 30.0)
        self.declare_parameter('max_range', 10.0)
        self.declare_parameter('min_range', 0.1)
        self.declare_parameter('grid_resolution', 0.05)
        self.declare_parameter('grid_width', 400)  # 20m at 0.05m resolution
        self.declare_parameter('grid_height', 400)
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('sensor_timeout', 2.0)
        self.declare_parameter('lidar_weight', 0.6)
        self.declare_parameter('depth_weight', 0.4)
        self.declare_parameter('obstacle_threshold', 0.65)
        self.declare_parameter('free_threshold', 0.25)
    
    def load_parameters(self):
        """Load parameters from ROS parameter server"""
        self.fusion_rate = self.get_parameter('fusion_rate').get_parameter_value().double_value
        self.max_range = self.get_parameter('max_range').get_parameter_value().double_value
        self.min_range = self.get_parameter('min_range').get_parameter_value().double_value
        self.grid_resolution = self.get_parameter('grid_resolution').get_parameter_value().double_value
        self.grid_width = self.get_parameter('grid_width').get_parameter_value().integer_value
        self.grid_height = self.get_parameter('grid_height').get_parameter_value().integer_value
        self.confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.sensor_timeout = self.get_parameter('sensor_timeout').get_parameter_value().double_value
        self.lidar_weight = self.get_parameter('lidar_weight').get_parameter_value().double_value
        self.depth_weight = self.get_parameter('depth_weight').get_parameter_value().double_value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').get_parameter_value().double_value
        self.free_threshold = self.get_parameter('free_threshold').get_parameter_value().double_value
        
        # Initialize occupancy grid
        self.fused_grid = np.full((self.grid_height, self.grid_width), -1, dtype=np.int8)
        self.confidence_grid = np.zeros((self.grid_height, self.grid_width), dtype=np.float32)
    
    def setup_subscribers(self):
        """Setup sensor data subscribers"""
        # LiDAR subscriber
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            self.sensor_qos
        )
        
        # Depth camera subscribers
        self.front_depth_sub = self.create_subscription(
            Image,
            '/front_camera/depth_image',
            self.front_depth_callback,
            self.sensor_qos
        )
        
        self.rear_depth_sub = self.create_subscription(
            Image,
            '/rear_camera/depth_image',
            self.rear_depth_callback,
            self.sensor_qos
        )
        
        # RGB camera subscribers for visual processing
        self.front_rgb_sub = self.create_subscription(
            Image,
            '/front_camera/image',
            self.front_rgb_callback,
            self.sensor_qos
        )
        
        self.rear_rgb_sub = self.create_subscription(
            Image,
            '/rear_camera/image',
            self.rear_rgb_callback,
            self.sensor_qos
        )
    
    def setup_publishers(self):
        """Setup output publishers"""
        # Fused obstacle data
        self.fused_obstacles_pub = self.create_publisher(
            OccupancyGrid,
            '/sensors/fused_obstacles',
            self.reliable_qos
        )
        
        # Confidence metrics
        self.confidence_pub = self.create_publisher(
            Float32,
            '/sensors/perception_confidence',
            self.reliable_qos
        )
        
        # Sensor health status
        self.health_pub = self.create_publisher(
            String,
            '/sensors/sensor_health',
            self.reliable_qos
        )
        
        # Visualization markers
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/sensors/perception_markers',
            self.reliable_qos
        )
    
    def scan_callback(self, msg):
        """Process LiDAR scan data"""
        with self.data_lock:
            self.latest_scan = msg
            self.sensor_health['lidar']['last_update'] = self.get_clock().now().nanoseconds / 1e9
            self.sensor_health['lidar']['active'] = True
            
            # Update LiDAR confidence based on data quality
            valid_ranges = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]
            if len(valid_ranges) > 0:
                self.sensor_health['lidar']['confidence'] = min(1.0, len(valid_ranges) / len(msg.ranges))
            else:
                self.sensor_health['lidar']['confidence'] = 0.0
    
    def front_depth_callback(self, msg):
        """Process front depth camera data"""
        with self.data_lock:
            self.latest_front_depth = msg
            self.sensor_health['front_camera']['last_update'] = self.get_clock().now().nanoseconds / 1e9
            self.sensor_health['front_camera']['active'] = True
            
            # Convert depth image and assess quality
            try:
                depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
                valid_pixels = np.count_nonzero(~np.isnan(depth_image))
                total_pixels = depth_image.shape[0] * depth_image.shape[1]
                self.sensor_health['front_camera']['confidence'] = valid_pixels / total_pixels
            except Exception as e:
                self.get_logger().warn(f"Front depth processing error: {e}")
                self.sensor_health['front_camera']['confidence'] = 0.0
    
    def rear_depth_callback(self, msg):
        """Process rear depth camera data"""
        with self.data_lock:
            self.latest_rear_depth = msg
            self.sensor_health['rear_camera']['last_update'] = self.get_clock().now().nanoseconds / 1e9
            self.sensor_health['rear_camera']['active'] = True
            
            # Convert depth image and assess quality
            try:
                depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
                valid_pixels = np.count_nonzero(~np.isnan(depth_image))
                total_pixels = depth_image.shape[0] * depth_image.shape[1]
                self.sensor_health['rear_camera']['confidence'] = valid_pixels / total_pixels
            except Exception as e:
                self.get_logger().warn(f"Rear depth processing error: {e}")
                self.sensor_health['rear_camera']['confidence'] = 0.0
    
    def front_rgb_callback(self, msg):
        """Process front RGB camera data"""
        with self.data_lock:
            self.latest_front_rgb = msg
    
    def rear_rgb_callback(self, msg):
        """Process rear RGB camera data"""
        with self.data_lock:
            self.latest_rear_rgb = msg
    
    def fusion_callback(self):
        """Main sensor fusion processing loop"""
        with self.data_lock:
            # Reset grids
            self.fused_grid.fill(-1)
            self.confidence_grid.fill(0.0)
            
            current_time = self.get_clock().now().nanoseconds / 1e9
            
            # Process LiDAR data if available and recent
            if (self.latest_scan is not None and 
                current_time - self.sensor_health['lidar']['last_update'] < self.sensor_timeout):
                self.process_lidar_data()
            
            # Process depth camera data
            if (self.latest_front_depth is not None and 
                current_time - self.sensor_health['front_camera']['last_update'] < self.sensor_timeout):
                self.process_depth_data(self.latest_front_depth, 'front_camera_link')
            
            if (self.latest_rear_depth is not None and 
                current_time - self.sensor_health['rear_camera']['last_update'] < self.sensor_timeout):
                self.process_depth_data(self.latest_rear_depth, 'rear_camera_link')
            
            # Publish fused results
            self.publish_fused_obstacles()
            self.publish_confidence_metrics()
            self.publish_visualization_markers()
    
    def process_lidar_data(self):
        """Process LiDAR scan data into occupancy grid"""
        scan = self.latest_scan
        
        # Convert scan to points in base_link frame
        for i, range_val in enumerate(scan.ranges):
            if scan.range_min <= range_val <= scan.range_max:
                angle = scan.angle_min + i * scan.angle_increment
                
                # Convert to Cartesian coordinates
                x = range_val * math.cos(angle)
                y = range_val * math.sin(angle)
                
                # Convert to grid coordinates
                grid_x = int((x + self.grid_width * self.grid_resolution / 2) / self.grid_resolution)
                grid_y = int((y + self.grid_height * self.grid_resolution / 2) / self.grid_resolution)
                
                # Check bounds
                if 0 <= grid_x < self.grid_width and 0 <= grid_y < self.grid_height:
                    # Mark obstacle
                    weight = self.lidar_weight * self.sensor_health['lidar']['confidence']
                    self.update_grid_cell(grid_x, grid_y, 100, weight)
                    
                    # Mark free space along ray
                    self.mark_free_space(0, 0, grid_x, grid_y, weight)
    
    def process_depth_data(self, depth_msg, camera_frame):
        """Process depth camera data into occupancy grid"""
        try:
            # Convert depth image
            depth_image = self.cv_bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
            
            # Get camera info for projection (simplified - in real implementation, subscribe to camera_info)
            height, width = depth_image.shape
            fx = fy = width / 2  # Simplified focal length
            cx, cy = width / 2, height / 2
            
            # Sample points from depth image (not every pixel for performance)
            step = 10  # Process every 10th pixel
            
            for v in range(0, height, step):
                for u in range(0, width, step):
                    depth = depth_image[v, u]
                    
                    if not np.isnan(depth) and self.min_range < depth < self.max_range:
                        # Convert pixel to 3D point in camera frame
                        x_cam = (u - cx) * depth / fx
                        y_cam = (v - cy) * depth / fy
                        z_cam = depth
                        
                        # Transform to base_link frame (simplified transform)
                        # In real implementation, use tf2 for proper transformation
                        if camera_frame == 'front_camera_link':
                            x_base = z_cam  # Camera facing forward
                            y_base = -x_cam
                        else:  # rear camera
                            x_base = -z_cam  # Camera facing backward
                            y_base = x_cam
                        
                        # Convert to grid coordinates
                        grid_x = int((x_base + self.grid_width * self.grid_resolution / 2) / self.grid_resolution)
                        grid_y = int((y_base + self.grid_height * self.grid_resolution / 2) / self.grid_resolution)
                        
                        # Check bounds and update grid
                        if 0 <= grid_x < self.grid_width and 0 <= grid_y < self.grid_height:
                            weight = self.depth_weight * self.sensor_health[camera_frame.split('_')[0] + '_camera']['confidence']
                            self.update_grid_cell(grid_x, grid_y, 100, weight)
                            
        except Exception as e:
            self.get_logger().warn(f"Depth processing error for {camera_frame}: {e}")
    
    def update_grid_cell(self, x, y, occupancy_value, weight):
        """Update grid cell with weighted occupancy value"""
        current_confidence = self.confidence_grid[y, x]
        
        if current_confidence == 0.0:
            # First observation
            self.fused_grid[y, x] = occupancy_value
            self.confidence_grid[y, x] = weight
        else:
            # Weighted average with existing observation
            total_weight = current_confidence + weight
            weighted_value = (self.fused_grid[y, x] * current_confidence + occupancy_value * weight) / total_weight
            self.fused_grid[y, x] = int(weighted_value)
            self.confidence_grid[y, x] = min(1.0, total_weight)
    
    def mark_free_space(self, x0, y0, x1, y1, weight):
        """Mark free space along a line using Bresenham's algorithm"""
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        x, y = x0, y0
        
        while True:
            if x == x1 and y == y1:
                break
                
            if 0 <= x < self.grid_width and 0 <= y < self.grid_height:
                # Only mark as free if not already marked as obstacle with high confidence
                if self.confidence_grid[y, x] < 0.8 or self.fused_grid[y, x] < self.obstacle_threshold * 100:
                    self.update_grid_cell(x, y, 0, weight * 0.5)  # Lower weight for free space
            
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
    
    def publish_fused_obstacles(self):
        """Publish fused obstacle occupancy grid"""
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = 'base_link'
        
        grid_msg.info.resolution = self.grid_resolution
        grid_msg.info.width = self.grid_width
        grid_msg.info.height = self.grid_height
        grid_msg.info.origin.position.x = -self.grid_width * self.grid_resolution / 2
        grid_msg.info.origin.position.y = -self.grid_height * self.grid_resolution / 2
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation.w = 1.0
        
        # Apply confidence thresholding
        final_grid = self.fused_grid.copy()
        low_confidence_mask = self.confidence_grid < self.confidence_threshold
        final_grid[low_confidence_mask] = -1  # Unknown for low confidence areas
        
        grid_msg.data = final_grid.flatten().tolist()
        self.fused_obstacles_pub.publish(grid_msg)
    
    def publish_confidence_metrics(self):
        """Publish overall perception confidence"""
        # Calculate weighted average confidence
        active_sensors = [sensor for sensor, health in self.sensor_health.items() if health['active']]
        
        if active_sensors:
            total_confidence = sum(self.sensor_health[sensor]['confidence'] for sensor in active_sensors)
            average_confidence = total_confidence / len(active_sensors)
        else:
            average_confidence = 0.0
        
        confidence_msg = Float32()
        confidence_msg.data = average_confidence
        self.confidence_pub.publish(confidence_msg)
    
    def publish_visualization_markers(self):
        """Publish visualization markers for RViz"""
        marker_array = MarkerArray()
        
        # Create confidence visualization marker
        confidence_marker = Marker()
        confidence_marker.header.frame_id = 'base_link'
        confidence_marker.header.stamp = self.get_clock().now().to_msg()
        confidence_marker.ns = 'perception_confidence'
        confidence_marker.id = 0
        confidence_marker.type = Marker.TEXT_VIEW_FACING
        confidence_marker.action = Marker.ADD
        confidence_marker.pose.position.x = 0.0
        confidence_marker.pose.position.y = 0.0
        confidence_marker.pose.position.z = 2.0
        confidence_marker.scale.z = 0.3
        confidence_marker.color.r = 1.0
        confidence_marker.color.g = 1.0
        confidence_marker.color.b = 1.0
        confidence_marker.color.a = 1.0
        
        # Calculate average confidence for display
        active_sensors = [sensor for sensor, health in self.sensor_health.items() if health['active']]
        if active_sensors:
            avg_conf = sum(self.sensor_health[sensor]['confidence'] for sensor in active_sensors) / len(active_sensors)
            confidence_marker.text = f"Perception Confidence: {avg_conf:.2f}"
        else:
            confidence_marker.text = "Perception Confidence: 0.00"
        
        marker_array.markers.append(confidence_marker)
        self.marker_pub.publish(marker_array)
    
    def health_monitoring_callback(self):
        """Monitor sensor health and publish status"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Check for sensor timeouts
        for sensor_name, health in self.sensor_health.items():
            if current_time - health['last_update'] > self.sensor_timeout:
                health['active'] = False
                health['confidence'] = 0.0
        
        # Create health status message
        health_status = {
            'timestamp': current_time,
            'sensors': {}
        }
        
        for sensor_name, health in self.sensor_health.items():
            health_status['sensors'][sensor_name] = {
                'active': health['active'],
                'confidence': health['confidence'],
                'last_update': health['last_update']
            }
        
        # Publish health status
        health_msg = String()
        health_msg.data = str(health_status)
        self.health_pub.publish(health_msg)
        
        # Log warnings for failed sensors
        for sensor_name, health in self.sensor_health.items():
            if not health['active']:
                self.get_logger().warn(f"Sensor {sensor_name} is inactive or timed out")
            elif health['confidence'] < 0.5:
                self.get_logger().warn(f"Sensor {sensor_name} has low confidence: {health['confidence']:.2f}")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        fusion_node = AdvancedPerceptionFusion()
        rclpy.spin(fusion_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in perception fusion node: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
