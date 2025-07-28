#!/usr/bin/env python3

"""
Vineyard Row Detector for Autonomous Navigation

This node processes LiDAR and camera data to detect vineyard rows,
calculate row orientations and spacing, and publish row detection markers.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, CameraInfo
from geometry_msgs.msg import Point, Pose
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA
import numpy as np
import math
from cv_bridge import CvBridge
import cv2


class VineyardRowDetector(Node):
    def __init__(self):
        super().__init__('vineyard_row_detector')
        
        # Declare parameters
        self.declare_parameter('use_sim_time', True)
        self.declare_parameter('row_spacing', 2.5)
        self.declare_parameter('detection_range', 10.0)
        self.declare_parameter('min_points_per_row', 5)
        self.declare_parameter('angle_tolerance', 0.2)  # radians
        self.declare_parameter('distance_threshold', 0.3)  # meters
        self.declare_parameter('publish_rate', 5.0)
        
        # Get parameters
        self.expected_row_spacing = self.get_parameter('row_spacing').get_parameter_value().double_value
        self.detection_range = self.get_parameter('detection_range').get_parameter_value().double_value
        self.min_points_per_row = self.get_parameter('min_points_per_row').get_parameter_value().integer_value
        self.angle_tolerance = self.get_parameter('angle_tolerance').get_parameter_value().double_value
        self.distance_threshold = self.get_parameter('distance_threshold').get_parameter_value().double_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        
        # CV Bridge for image processing
        self.bridge = CvBridge()
        
        # Subscribers
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )
        
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image',
            self.image_callback,
            10
        )
        
        # Publishers
        self.row_markers_pub = self.create_publisher(
            MarkerArray,
            '/vineyard/row_markers',
            10
        )
        
        self.detected_rows_pub = self.create_publisher(
            MarkerArray,
            '/vineyard/detected_rows',
            10
        )
        
        # Data storage
        self.latest_scan = None
        self.latest_image = None
        self.detected_rows = []
        
        # Processing timer
        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.process_and_publish
        )
        
        self.get_logger().info('Vineyard Row Detector started')
        self.get_logger().info(f'Expected row spacing: {self.expected_row_spacing}m')
        self.get_logger().info(f'Detection range: {self.detection_range}m')
    
    def laser_callback(self, msg):
        """Store latest laser scan data"""
        self.latest_scan = msg
    
    def image_callback(self, msg):
        """Store latest camera image"""
        self.latest_image = msg
    
    def process_and_publish(self):
        """Main processing loop"""
        if self.latest_scan is None:
            return
        
        # Detect rows from laser data
        rows = self.detect_rows_from_laser(self.latest_scan)
        
        # Enhanced detection with camera if available
        if self.latest_image is not None:
            camera_rows = self.detect_rows_from_camera(self.latest_image)
            rows = self.fuse_detections(rows, camera_rows)
        
        self.detected_rows = rows
        
        # Publish visualization markers
        self.publish_row_markers()
        self.publish_detected_rows()
    
    def detect_rows_from_laser(self, scan):
        """Detect vineyard rows from laser scan data"""
        if len(scan.ranges) == 0:
            return []
        
        # Convert laser scan to cartesian points
        points = []
        angle = scan.angle_min
        
        for r in scan.ranges:
            if scan.range_min <= r <= min(scan.range_max, self.detection_range):
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                points.append([x, y])
            angle += scan.angle_increment
        
        if len(points) < self.min_points_per_row:
            return []
        
        points = np.array(points)
        
        # Cluster points into potential rows using DBSCAN-like approach
        rows = self.cluster_points_into_rows(points)
        
        return rows
    
    def cluster_points_into_rows(self, points):
        """Cluster laser points into vineyard rows"""
        if len(points) == 0:
            return []
        
        rows = []
        remaining_points = points.copy()
        
        while len(remaining_points) >= self.min_points_per_row:
            # Find the most prominent line using RANSAC-like approach
            best_line = self.fit_line_ransac(remaining_points)
            
            if best_line is None:
                break
            
            # Extract points belonging to this line
            line_points = []
            new_remaining = []
            
            for point in remaining_points:
                distance = self.point_to_line_distance(point, best_line)
                if distance < self.distance_threshold:
                    line_points.append(point)
                else:
                    new_remaining.append(point)
            
            if len(line_points) >= self.min_points_per_row:
                rows.append({
                    'points': np.array(line_points),
                    'line': best_line,
                    'center': np.mean(line_points, axis=0)
                })
            
            remaining_points = np.array(new_remaining)
        
        return rows
    
    def fit_line_ransac(self, points, iterations=100):
        """Fit a line to points using RANSAC algorithm"""
        if len(points) < 2:
            return None
        
        best_line = None
        best_score = 0
        
        for _ in range(iterations):
            # Randomly select two points
            idx = np.random.choice(len(points), 2, replace=False)
            p1, p2 = points[idx]
            
            # Calculate line parameters (ax + by + c = 0)
            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]
            
            if abs(dx) < 1e-6 and abs(dy) < 1e-6:
                continue
            
            # Normal vector to the line
            a = -dy
            b = dx
            c = dy * p1[0] - dx * p1[1]
            
            # Normalize
            norm = math.sqrt(a*a + b*b)
            if norm < 1e-6:
                continue
            
            a /= norm
            b /= norm
            c /= norm
            
            # Count inliers
            score = 0
            for point in points:
                distance = abs(a * point[0] + b * point[1] + c)
                if distance < self.distance_threshold:
                    score += 1
            
            if score > best_score:
                best_score = score
                best_line = (a, b, c)
        
        return best_line
    
    def point_to_line_distance(self, point, line):
        """Calculate distance from point to line"""
        a, b, c = line
        return abs(a * point[0] + b * point[1] + c)
    
    def detect_rows_from_camera(self, image_msg):
        """Detect vineyard rows from camera image (simplified implementation)"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(f'Failed to convert image: {e}')
            return []
        
        # Convert to HSV for vegetation detection
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Define range for green vegetation
        lower_green = np.array([40, 40, 40])
        upper_green = np.array([80, 255, 255])
        
        # Create mask for vegetation
        mask = cv2.inRange(hsv, lower_green, upper_green)
        
        # Find contours (simplified row detection)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # This is a simplified implementation
        # In practice, you would need more sophisticated computer vision
        # to reliably detect row structures from camera images
        
        return []  # Return empty for now
    
    def fuse_detections(self, laser_rows, camera_rows):
        """Fuse laser and camera detections"""
        # For now, just use laser detections
        # In a full implementation, you would correlate and fuse
        # detections from both sensors
        return laser_rows
    
    def publish_row_markers(self):
        """Publish visualization markers for detected rows"""
        marker_array = MarkerArray()
        
        for i, row in enumerate(self.detected_rows):
            # Create line marker for each row
            marker = Marker()
            marker.header = Header()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = 'base_link'
            marker.ns = 'vineyard_rows'
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            
            # Set marker scale
            marker.scale.x = 0.1  # Line width
            
            # Set marker color (green for detected rows)
            marker.color = ColorRGBA()
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            
            # Add points to the line
            for point in row['points']:
                p = Point()
                p.x = float(point[0])
                p.y = float(point[1])
                p.z = 0.0
                marker.points.append(p)
            
            marker_array.markers.append(marker)
        
        self.row_markers_pub.publish(marker_array)
    
    def publish_detected_rows(self):
        """Publish detected row information"""
        marker_array = MarkerArray()
        
        for i, row in enumerate(self.detected_rows):
            # Create text marker showing row information
            marker = Marker()
            marker.header = Header()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = 'base_link'
            marker.ns = 'row_info'
            marker.id = i
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            
            # Position at row center
            marker.pose.position.x = float(row['center'][0])
            marker.pose.position.y = float(row['center'][1])
            marker.pose.position.z = 1.0  # Above ground
            
            marker.pose.orientation.w = 1.0
            
            # Set marker scale
            marker.scale.z = 0.5  # Text size
            
            # Set marker color (white text)
            marker.color = ColorRGBA()
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            
            # Set text
            marker.text = f'Row {i+1}\n{len(row["points"])} points'
            
            marker_array.markers.append(marker)
        
        self.detected_rows_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    
    vineyard_row_detector = VineyardRowDetector()
    
    try:
        rclpy.spin(vineyard_row_detector)
    except KeyboardInterrupt:
        pass
    finally:
        vineyard_row_detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
