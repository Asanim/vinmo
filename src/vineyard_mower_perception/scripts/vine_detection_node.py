#!/usr/bin/env python3
"""
Vine Detection Node for Vineyard Mower Robot

This node implements computer vision algorithms to detect grape vines
and provide proximity warnings to protect vegetation during navigation.

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
from threading import Lock

# ROS2 message types
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header, Float32, String, Bool
from visualization_msgs.msg import MarkerArray, Marker, ImageMarker
from vineyard_mower_interfaces.msg import VineProximity

# Computer vision and machine learning
from sklearn.cluster import KMeans
import math


class VineDetectionNode(Node):
    """
    Computer vision-based vine detection system for grape vine proximity monitoring
    and protection during autonomous vineyard navigation.
    """
    
    def __init__(self):
        super().__init__('vine_detection_node')
        
        # Initialize parameters
        self.declare_parameters()
        self.load_parameters()
        
        # Initialize utilities
        self.cv_bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.data_lock = Lock()
        
        # Camera intrinsics
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # Vine detection state
        self.latest_rgb = None
        self.latest_depth = None
        self.vine_locations = []
        self.closest_vine_distance = float('inf')
        
        # Color thresholds for vegetation detection (HSV)
        self.green_lower = np.array([35, 40, 40])
        self.green_upper = np.array([85, 255, 255])
        self.brown_lower = np.array([10, 50, 20])
        self.brown_upper = np.array([20, 255, 200])
        
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
        
        # Setup subscribers and publishers
        self.setup_subscribers()
        self.setup_publishers()
        
        # Processing timer
        self.detection_timer = self.create_timer(1.0 / self.detection_rate, self.detection_callback)
        
        self.get_logger().info("Vine Detection Node initialized")
    
    def declare_parameters(self):
        """Declare node parameters with default values"""
        self.declare_parameter('camera_namespace', 'front_camera')
        self.declare_parameter('detection_rate', 10.0)
        self.declare_parameter('safety_distance', 1.0)  # meters
        self.declare_parameter('warning_distance', 1.5)  # meters
        self.declare_parameter('max_detection_range', 5.0)  # meters
        self.declare_parameter('min_vine_area', 500)  # pixels
        self.declare_parameter('max_vine_area', 50000)  # pixels
        self.declare_parameter('vegetation_threshold', 0.3)  # fraction of green pixels
        self.declare_parameter('enable_color_detection', True)
        self.declare_parameter('enable_shape_detection', True)
        self.declare_parameter('enable_proximity_warnings', True)
        self.declare_parameter('debug_visualization', True)
    
    def load_parameters(self):
        """Load parameters from ROS parameter server"""
        self.camera_namespace = self.get_parameter('camera_namespace').get_parameter_value().string_value
        self.detection_rate = self.get_parameter('detection_rate').get_parameter_value().double_value
        self.safety_distance = self.get_parameter('safety_distance').get_parameter_value().double_value
        self.warning_distance = self.get_parameter('warning_distance').get_parameter_value().double_value
        self.max_detection_range = self.get_parameter('max_detection_range').get_parameter_value().double_value
        self.min_vine_area = self.get_parameter('min_vine_area').get_parameter_value().integer_value
        self.max_vine_area = self.get_parameter('max_vine_area').get_parameter_value().integer_value
        self.vegetation_threshold = self.get_parameter('vegetation_threshold').get_parameter_value().double_value
        self.enable_color_detection = self.get_parameter('enable_color_detection').get_parameter_value().bool_value
        self.enable_shape_detection = self.get_parameter('enable_shape_detection').get_parameter_value().bool_value
        self.enable_proximity_warnings = self.get_parameter('enable_proximity_warnings').get_parameter_value().bool_value
        self.debug_visualization = self.get_parameter('debug_visualization').get_parameter_value().bool_value
    
    def setup_subscribers(self):
        """Setup camera data subscribers"""
        # RGB image subscriber
        self.rgb_sub = self.create_subscription(
            Image,
            f'/{self.camera_namespace}/image',
            self.rgb_callback,
            self.sensor_qos
        )
        
        # Depth image subscriber
        self.depth_sub = self.create_subscription(
            Image,
            f'/{self.camera_namespace}/depth_image',
            self.depth_callback,
            self.sensor_qos
        )
        
        # Camera info subscriber
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            f'/{self.camera_namespace}/camera_info',
            self.camera_info_callback,
            self.reliable_qos
        )
    
    def setup_publishers(self):
        """Setup output publishers"""
        # Vine proximity information
        self.proximity_pub = self.create_publisher(
            Float32,
            '/obstacles/vine_proximity',
            self.reliable_qos
        )
        
        # Safety alert
        self.safety_alert_pub = self.create_publisher(
            Bool,
            '/safety/vine_safety_alert',
            self.reliable_qos
        )
        
        # Detected vine locations
        self.vine_markers_pub = self.create_publisher(
            MarkerArray,
            f'/{self.camera_namespace}/vine_markers',
            self.reliable_qos
        )
        
        # Debug visualization
        if self.debug_visualization:
            self.debug_image_pub = self.create_publisher(
                Image,
                f'/{self.camera_namespace}/vine_detection_debug',
                self.reliable_qos
            )
            
            self.mask_image_pub = self.create_publisher(
                Image,
                f'/{self.camera_namespace}/vegetation_mask',
                self.reliable_qos
            )
    
    def camera_info_callback(self, msg):
        """Update camera intrinsics"""
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)
        
        self.fx = self.camera_matrix[0, 0]
        self.fy = self.camera_matrix[1, 1]
        self.cx = self.camera_matrix[0, 2]
        self.cy = self.camera_matrix[1, 2]
    
    def rgb_callback(self, msg):
        """Store latest RGB image"""
        with self.data_lock:
            try:
                self.latest_rgb = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                self.latest_rgb_header = msg.header
            except Exception as e:
                self.get_logger().warn(f"RGB conversion error: {e}")
    
    def depth_callback(self, msg):
        """Store latest depth image"""
        with self.data_lock:
            try:
                if msg.encoding == '16UC1':
                    depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
                    self.latest_depth = depth_image.astype(np.float32) * 0.001  # Convert to meters
                else:
                    self.latest_depth = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
                self.latest_depth_header = msg.header
            except Exception as e:
                self.get_logger().warn(f"Depth conversion error: {e}")
    
    def detection_callback(self):
        """Main vine detection processing loop"""
        with self.data_lock:
            if self.latest_rgb is None or self.camera_matrix is None:
                return
            
            rgb_image = self.latest_rgb.copy()
            depth_image = self.latest_depth.copy() if self.latest_depth is not None else None
            header = self.latest_rgb_header
        
        # Detect vines in the image
        vine_detections = self.detect_vines(rgb_image, depth_image)
        
        # Calculate 3D positions and distances
        vine_positions_3d = self.calculate_vine_positions(vine_detections, depth_image)
        
        # Update vine locations and proximity
        self.update_vine_locations(vine_positions_3d)
        
        # Publish results
        self.publish_proximity_data(header)
        self.publish_vine_markers(vine_positions_3d, header)
        
        if self.debug_visualization:
            self.publish_debug_visualization(rgb_image, vine_detections, header)
    
    def detect_vines(self, rgb_image, depth_image=None):
        """Detect vines using color and shape analysis"""
        detections = []
        
        # Convert to HSV for better color segmentation
        hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
        
        # Color-based detection
        if self.enable_color_detection:
            vegetation_mask = self.detect_vegetation_color(hsv_image)
            color_detections = self.extract_vine_candidates_from_mask(vegetation_mask, rgb_image)
            detections.extend(color_detections)
        
        # Shape-based detection
        if self.enable_shape_detection:
            shape_detections = self.detect_vine_shapes(rgb_image)
            detections.extend(shape_detections)
        
        # Filter and refine detections
        filtered_detections = self.filter_vine_detections(detections, rgb_image, depth_image)
        
        return filtered_detections
    
    def detect_vegetation_color(self, hsv_image):
        """Detect vegetation using color thresholds"""
        # Create mask for green vegetation
        green_mask = cv2.inRange(hsv_image, self.green_lower, self.green_upper)
        
        # Create mask for brown vine trunks
        brown_mask = cv2.inRange(hsv_image, self.brown_lower, self.brown_upper)
        
        # Combine masks
        vegetation_mask = cv2.bitwise_or(green_mask, brown_mask)
        
        # Morphological operations to clean up mask
        kernel = np.ones((3, 3), np.uint8)
        vegetation_mask = cv2.morphologyEx(vegetation_mask, cv2.MORPH_CLOSE, kernel)
        vegetation_mask = cv2.morphologyEx(vegetation_mask, cv2.MORPH_OPEN, kernel)
        
        return vegetation_mask
    
    def extract_vine_candidates_from_mask(self, mask, rgb_image):
        """Extract vine candidates from vegetation mask"""
        candidates = []
        
        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            
            if self.min_vine_area <= area <= self.max_vine_area:
                # Get bounding rectangle
                x, y, w, h = cv2.boundingRect(contour)
                
                # Calculate region properties
                aspect_ratio = float(w) / h if h > 0 else 0
                extent = float(area) / (w * h) if w * h > 0 else 0
                
                # Calculate center point
                M = cv2.moments(contour)
                if M['m00'] != 0:
                    center_x = int(M['m10'] / M['m00'])
                    center_y = int(M['m01'] / M['m00'])
                else:
                    center_x, center_y = x + w // 2, y + h // 2
                
                # Vegetation percentage in bounding box
                roi_mask = mask[y:y+h, x:x+w]
                vegetation_ratio = np.sum(roi_mask > 0) / (w * h) if w * h > 0 else 0
                
                candidate = {
                    'type': 'color',
                    'center': (center_x, center_y),
                    'bbox': (x, y, w, h),
                    'area': area,
                    'aspect_ratio': aspect_ratio,
                    'extent': extent,
                    'vegetation_ratio': vegetation_ratio,
                    'contour': contour,
                    'confidence': vegetation_ratio
                }
                
                candidates.append(candidate)
        
        return candidates
    
    def detect_vine_shapes(self, rgb_image):
        """Detect vines using shape analysis (vertical structures)"""
        candidates = []
        
        # Convert to grayscale
        gray = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
        
        # Edge detection
        edges = cv2.Canny(gray, 50, 150)
        
        # Find vertical lines using HoughLinesP
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=50, minLineLength=30, maxLineGap=10)
        
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                
                # Calculate line properties
                length = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                angle = np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi
                
                # Check if line is roughly vertical (vine trunk-like)
                if abs(angle) > 70 or abs(angle - 180) < 20:  # Vertical or near-vertical
                    center_x = (x1 + x2) // 2
                    center_y = (y1 + y2) // 2
                    
                    candidate = {
                        'type': 'shape',
                        'center': (center_x, center_y),
                        'line': (x1, y1, x2, y2),
                        'length': length,
                        'angle': angle,
                        'confidence': min(1.0, length / 100.0)  # Confidence based on length
                    }
                    
                    candidates.append(candidate)
        
        return candidates
    
    def filter_vine_detections(self, detections, rgb_image, depth_image):
        """Filter and refine vine detections"""
        filtered = []
        
        for detection in detections:
            # Basic validation
            if detection['confidence'] < 0.3:
                continue
            
            # Check if detection is within valid depth range
            if depth_image is not None:
                center_x, center_y = detection['center']
                if 0 <= center_y < depth_image.shape[0] and 0 <= center_x < depth_image.shape[1]:
                    depth = depth_image[center_y, center_x]
                    if np.isnan(depth) or depth <= 0 or depth > self.max_detection_range:
                        continue
            
            # Add more sophisticated filtering here (ML models, etc.)
            
            filtered.append(detection)
        
        return filtered
    
    def calculate_vine_positions(self, detections, depth_image):
        """Calculate 3D positions of detected vines"""
        positions_3d = []
        
        if depth_image is None or self.camera_matrix is None:
            return positions_3d
        
        for detection in detections:
            center_x, center_y = detection['center']
            
            # Get depth at detection center
            if 0 <= center_y < depth_image.shape[0] and 0 <= center_x < depth_image.shape[1]:
                depth = depth_image[center_y, center_x]
                
                if not np.isnan(depth) and depth > 0:
                    # Convert pixel to 3D point in camera frame
                    x_cam = (center_x - self.cx) * depth / self.fx
                    y_cam = (center_y - self.cy) * depth / self.fy
                    z_cam = depth
                    
                    # Add detection information
                    position_3d = {
                        'position': (x_cam, y_cam, z_cam),
                        'distance': depth,
                        'detection': detection,
                        'pixel_location': (center_x, center_y)
                    }
                    
                    positions_3d.append(position_3d)
        
        return positions_3d
    
    def update_vine_locations(self, vine_positions_3d):
        """Update vine locations and calculate closest distance"""
        self.vine_locations = vine_positions_3d
        
        if vine_positions_3d:
            distances = [pos['distance'] for pos in vine_positions_3d]
            self.closest_vine_distance = min(distances)
        else:
            self.closest_vine_distance = float('inf')
    
    def publish_proximity_data(self, header):
        """Publish vine proximity information"""
        # Publish closest distance
        proximity_msg = Float32()
        proximity_msg.data = self.closest_vine_distance if self.closest_vine_distance != float('inf') else -1.0
        self.proximity_pub.publish(proximity_msg)
        
        # Publish safety alert if enabled
        if self.enable_proximity_warnings:
            safety_alert = Bool()
            safety_alert.data = self.closest_vine_distance < self.safety_distance
            self.safety_alert_pub.publish(safety_alert)
            
            # Log warnings
            if self.closest_vine_distance < self.safety_distance:
                self.get_logger().warn(f"VINE SAFETY ALERT: Closest vine at {self.closest_vine_distance:.2f}m")
            elif self.closest_vine_distance < self.warning_distance:
                self.get_logger().info(f"Vine proximity warning: {self.closest_vine_distance:.2f}m")
    
    def publish_vine_markers(self, vine_positions_3d, header):
        """Publish vine location markers for visualization"""
        marker_array = MarkerArray()
        
        for i, vine_pos in enumerate(vine_positions_3d):
            marker = Marker()
            marker.header = header
            marker.header.frame_id = f"{self.camera_namespace}_link"
            marker.ns = "detected_vines"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            # Set position
            pos = vine_pos['position']
            marker.pose.position.x = float(pos[0])
            marker.pose.position.y = float(pos[1])
            marker.pose.position.z = float(pos[2])
            marker.pose.orientation.w = 1.0
            
            # Set scale
            marker.scale.x = 0.2  # Diameter
            marker.scale.y = 0.2
            marker.scale.z = 1.0  # Height
            
            # Color based on distance
            distance = vine_pos['distance']
            if distance < self.safety_distance:
                marker.color.r = 1.0  # Red
                marker.color.g = 0.0
                marker.color.b = 0.0
            elif distance < self.warning_distance:
                marker.color.r = 1.0  # Orange
                marker.color.g = 0.5
                marker.color.b = 0.0
            else:
                marker.color.r = 0.0  # Green
                marker.color.g = 1.0
                marker.color.b = 0.0
            
            marker.color.a = 0.8
            marker.lifetime = rclpy.duration.Duration(seconds=1.0).to_msg()
            
            marker_array.markers.append(marker)
        
        self.vine_markers_pub.publish(marker_array)
    
    def publish_debug_visualization(self, rgb_image, detections, header):
        """Publish debug visualization images"""
        debug_image = rgb_image.copy()
        
        # Draw detection results
        for detection in detections:
            center = detection['center']
            confidence = detection['confidence']
            
            if detection['type'] == 'color':
                # Draw bounding box for color detections
                bbox = detection['bbox']
                x, y, w, h = bbox
                cv2.rectangle(debug_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(debug_image, f"Vine {confidence:.2f}", 
                           (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            
            elif detection['type'] == 'shape':
                # Draw line for shape detections
                line = detection['line']
                x1, y1, x2, y2 = line
                cv2.line(debug_image, (x1, y1), (x2, y2), (255, 0, 0), 2)
            
            # Draw center point
            cv2.circle(debug_image, center, 5, (0, 0, 255), -1)
        
        # Publish debug image
        try:
            debug_msg = self.cv_bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
            debug_msg.header = header
            self.debug_image_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().warn(f"Failed to publish debug image: {e}")
        
        # Publish vegetation mask
        if hasattr(self, 'latest_rgb') and self.latest_rgb is not None:
            hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
            vegetation_mask = self.detect_vegetation_color(hsv_image)
            
            try:
                mask_msg = self.cv_bridge.cv2_to_imgmsg(vegetation_mask, encoding='mono8')
                mask_msg.header = header
                self.mask_image_pub.publish(mask_msg)
            except Exception as e:
                self.get_logger().warn(f"Failed to publish vegetation mask: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        vine_detection_node = VineDetectionNode()
        rclpy.spin(vine_detection_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in vine detection node: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
