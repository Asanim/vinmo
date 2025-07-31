#!/usr/bin/env python3
"""
Depth Obstacle Mapper for Vineyard Mower Robot

This node processes depth camera data to create detailed 3D obstacle maps
with ground plane filtering and height-aware obstacle classification.

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
from sklearn.cluster import DBSCAN
import open3d as o3d

# ROS2 message types
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import MarkerArray, Marker
import sensor_msgs_py.point_cloud2 as pc2


class DepthObstacleMapper(Node):
    """
    Advanced depth camera processing for 3D obstacle detection
    with ground plane filtering and height classification.
    """
    
    def __init__(self):
        super().__init__('depth_obstacle_mapper')
        
        # Initialize parameters
        self.declare_parameters()
        self.load_parameters()
        
        # Initialize utilities
        self.cv_bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Camera intrinsics (will be updated from camera_info)
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # Ground plane model (updated dynamically)
        self.ground_plane = None
        self.ground_confidence = 0.0
        
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
        
        self.get_logger().info("Depth Obstacle Mapper initialized")
    
    def declare_parameters(self):
        """Declare node parameters with default values"""
        self.declare_parameter('camera_namespace', 'front_camera')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('max_depth', 8.0)
        self.declare_parameter('min_depth', 0.3)
        self.declare_parameter('depth_scale', 0.001)  # Depth units to meters
        self.declare_parameter('ground_height_tolerance', 0.1)
        self.declare_parameter('min_obstacle_height', 0.15)
        self.declare_parameter('max_obstacle_height', 3.0)
        self.declare_parameter('cluster_tolerance', 0.05)
        self.declare_parameter('min_cluster_size', 10)
        self.declare_parameter('max_cluster_size', 5000)
        self.declare_parameter('processing_skip', 2)  # Process every nth pixel
        self.declare_parameter('ground_sample_ratio', 0.3)  # Fraction of points for ground estimation
    
    def load_parameters(self):
        """Load parameters from ROS parameter server"""
        self.camera_namespace = self.get_parameter('camera_namespace').get_parameter_value().string_value
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self.max_depth = self.get_parameter('max_depth').get_parameter_value().double_value
        self.min_depth = self.get_parameter('min_depth').get_parameter_value().double_value
        self.depth_scale = self.get_parameter('depth_scale').get_parameter_value().double_value
        self.ground_height_tolerance = self.get_parameter('ground_height_tolerance').get_parameter_value().double_value
        self.min_obstacle_height = self.get_parameter('min_obstacle_height').get_parameter_value().double_value
        self.max_obstacle_height = self.get_parameter('max_obstacle_height').get_parameter_value().double_value
        self.cluster_tolerance = self.get_parameter('cluster_tolerance').get_parameter_value().double_value
        self.min_cluster_size = self.get_parameter('min_cluster_size').get_parameter_value().integer_value
        self.max_cluster_size = self.get_parameter('max_cluster_size').get_parameter_value().integer_value
        self.processing_skip = self.get_parameter('processing_skip').get_parameter_value().integer_value
        self.ground_sample_ratio = self.get_parameter('ground_sample_ratio').get_parameter_value().double_value
    
    def setup_subscribers(self):
        """Setup depth camera subscribers"""
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
        
        # RGB image subscriber (for enhanced processing)
        self.rgb_sub = self.create_subscription(
            Image,
            f'/{self.camera_namespace}/image',
            self.rgb_callback,
            self.sensor_qos
        )
    
    def setup_publishers(self):
        """Setup output publishers"""
        # 3D obstacle point cloud
        self.obstacles_cloud_pub = self.create_publisher(
            PointCloud2,
            f'/{self.camera_namespace}/obstacle_cloud',
            self.reliable_qos
        )
        
        # Ground plane point cloud
        self.ground_cloud_pub = self.create_publisher(
            PointCloud2,
            f'/{self.camera_namespace}/ground_cloud',
            self.reliable_qos
        )
        
        # Classified obstacles (different heights)
        self.classified_obstacles_pub = self.create_publisher(
            MarkerArray,
            f'/{self.camera_namespace}/classified_obstacles',
            self.reliable_qos
        )
        
        # Processed depth image (for debugging)
        self.processed_depth_pub = self.create_publisher(
            Image,
            f'/{self.camera_namespace}/processed_depth',
            self.reliable_qos
        )
    
    def camera_info_callback(self, msg):
        """Update camera intrinsics from camera_info"""
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)
        
        # Cache intrinsic parameters for faster access
        self.fx = self.camera_matrix[0, 0]
        self.fy = self.camera_matrix[1, 1]
        self.cx = self.camera_matrix[0, 2]
        self.cy = self.camera_matrix[1, 2]
        
        self.get_logger().info(f"Updated camera intrinsics for {self.camera_namespace}")
    
    def rgb_callback(self, msg):
        """Store latest RGB image for enhanced processing"""
        try:
            self.latest_rgb = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"RGB conversion error: {e}")
    
    def depth_callback(self, msg):
        """Main depth processing callback"""
        if self.camera_matrix is None:
            self.get_logger().warn("Camera intrinsics not yet received")
            return
        
        try:
            # Convert depth image
            if msg.encoding == '16UC1':
                depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
                depth_image = depth_image.astype(np.float32) * self.depth_scale
            else:
                depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            
            # Process depth data
            self.process_depth_image(depth_image, msg.header)
            
        except Exception as e:
            self.get_logger().error(f"Depth processing error: {e}")
    
    def process_depth_image(self, depth_image, header):
        """Process depth image to extract obstacles and ground plane"""
        height, width = depth_image.shape
        
        # Generate 3D point cloud from depth image
        points_3d = []
        colors = []
        
        for v in range(0, height, self.processing_skip):
            for u in range(0, width, self.processing_skip):
                depth = depth_image[v, u]
                
                if (not np.isnan(depth) and 
                    self.min_depth < depth < self.max_depth):
                    
                    # Convert pixel to 3D point in camera frame
                    x_cam = (u - self.cx) * depth / self.fx
                    y_cam = (v - self.cy) * depth / self.fy
                    z_cam = depth
                    
                    points_3d.append([x_cam, y_cam, z_cam])
                    
                    # Add color information if RGB available
                    if hasattr(self, 'latest_rgb'):
                        try:
                            color = self.latest_rgb[v, u]
                            colors.append([color[2], color[1], color[0]])  # BGR to RGB
                        except:
                            colors.append([128, 128, 128])  # Default gray
                    else:
                        colors.append([128, 128, 128])
        
        if len(points_3d) < 100:  # Not enough points
            return
        
        points_3d = np.array(points_3d)
        colors = np.array(colors)
        
        # Transform points to base_link frame
        try:
            camera_frame = f"{self.camera_namespace}_link"
            transform = self.tf_buffer.lookup_transform(
                self.target_frame, camera_frame, header.stamp, timeout=rclpy.duration.Duration(seconds=0.1)
            )
            points_base = self.transform_points(points_3d, transform)
        except Exception as e:
            self.get_logger().warn(f"Transform lookup failed: {e}")
            points_base = points_3d  # Use camera frame if transform fails
        
        # Segment ground plane and obstacles
        ground_points, obstacle_points, ground_indices, obstacle_indices = self.segment_ground_obstacles(points_base)
        
        # Classify obstacles by height
        classified_obstacles = self.classify_obstacles_by_height(obstacle_points)
        
        # Publish results
        self.publish_point_clouds(ground_points, obstacle_points, colors, ground_indices, obstacle_indices, header)
        self.publish_classified_obstacles(classified_obstacles, header)
        self.publish_processed_depth_image(depth_image, ground_indices, obstacle_indices, header)
    
    def transform_points(self, points, transform):
        """Transform 3D points using TF2 transform"""
        # Extract rotation and translation
        t = transform.transform.translation
        r = transform.transform.rotation
        
        # Convert quaternion to rotation matrix
        rotation_matrix = self.quaternion_to_rotation_matrix(r)
        translation = np.array([t.x, t.y, t.z])
        
        # Apply transformation
        transformed_points = np.dot(points, rotation_matrix.T) + translation
        return transformed_points
    
    def quaternion_to_rotation_matrix(self, q):
        """Convert quaternion to rotation matrix"""
        w, x, y, z = q.w, q.x, q.y, q.z
        
        return np.array([
            [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
            [2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
            [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]
        ])
    
    def segment_ground_obstacles(self, points):
        """Segment points into ground plane and obstacles"""
        if len(points) < 100:
            return np.array([]), points, [], list(range(len(points)))
        
        # Use RANSAC to fit ground plane
        ground_plane, ground_indices = self.fit_ground_plane_ransac(points)
        
        if ground_plane is not None:
            self.ground_plane = ground_plane
            self.ground_confidence = len(ground_indices) / len(points)
            
            # Separate ground and obstacle points
            obstacle_indices = [i for i in range(len(points)) if i not in ground_indices]
            ground_points = points[ground_indices]
            obstacle_points = points[obstacle_indices]
            
            # Filter obstacles by height relative to ground
            valid_obstacle_indices = []
            for idx in obstacle_indices:
                point = points[idx]
                height_above_ground = self.distance_to_plane(point, ground_plane)
                
                if self.min_obstacle_height <= height_above_ground <= self.max_obstacle_height:
                    valid_obstacle_indices.append(idx)
            
            obstacle_points = points[valid_obstacle_indices]
            
            return ground_points, obstacle_points, ground_indices, valid_obstacle_indices
        else:
            # No ground plane found, treat all as obstacles
            return np.array([]), points, [], list(range(len(points)))
    
    def fit_ground_plane_ransac(self, points, max_iterations=1000, distance_threshold=0.05):
        """Fit ground plane using RANSAC algorithm"""
        best_plane = None
        best_inliers = []
        best_score = 0
        
        n_points = len(points)
        sample_size = max(int(n_points * self.ground_sample_ratio), 3)
        
        for _ in range(max_iterations):
            # Sample random points
            sample_indices = np.random.choice(n_points, 3, replace=False)
            sample_points = points[sample_indices]
            
            # Fit plane to sample points
            try:
                plane = self.fit_plane_to_points(sample_points)
                if plane is None:
                    continue
                
                # Find inliers
                distances = np.abs(self.distance_to_plane_vectorized(points, plane))
                inliers = np.where(distances < distance_threshold)[0]
                
                # Score based on number of inliers and how horizontal the plane is
                horizontal_score = abs(plane[2])  # Closer to 1 is more horizontal
                score = len(inliers) * horizontal_score
                
                if score > best_score:
                    best_score = score
                    best_plane = plane
                    best_inliers = inliers.tolist()
                    
            except Exception:
                continue
        
        return best_plane, best_inliers
    
    def fit_plane_to_points(self, points):
        """Fit plane to 3 or more points using least squares"""
        if len(points) < 3:
            return None
        
        # Center the points
        centroid = np.mean(points, axis=0)
        centered_points = points - centroid
        
        # SVD to find normal vector
        try:
            _, _, vt = np.linalg.svd(centered_points)
            normal = vt[-1]  # Last row is normal to plane
            
            # Ensure normal points upward (positive z component)
            if normal[2] < 0:
                normal = -normal
            
            # Plane equation: ax + by + cz + d = 0
            d = -np.dot(normal, centroid)
            
            return np.array([normal[0], normal[1], normal[2], d])
        except:
            return None
    
    def distance_to_plane(self, point, plane):
        """Calculate distance from point to plane"""
        a, b, c, d = plane
        return abs(a * point[0] + b * point[1] + c * point[2] + d) / np.sqrt(a**2 + b**2 + c**2)
    
    def distance_to_plane_vectorized(self, points, plane):
        """Calculate distances from multiple points to plane (vectorized)"""
        a, b, c, d = plane
        return (a * points[:, 0] + b * points[:, 1] + c * points[:, 2] + d) / np.sqrt(a**2 + b**2 + c**2)
    
    def classify_obstacles_by_height(self, obstacle_points):
        """Classify obstacles into height categories"""
        if len(obstacle_points) == 0:
            return {}
        
        # Height categories for vineyard environment
        categories = {
            'low': (0.15, 0.5),      # Small objects, debris
            'medium': (0.5, 1.5),    # Vine trunks, equipment
            'high': (1.5, 3.0)       # Overhead obstacles, trees
        }
        
        classified = {}
        
        for category, (min_h, max_h) in categories.items():
            # Filter points by height (z-coordinate)
            mask = (obstacle_points[:, 2] >= min_h) & (obstacle_points[:, 2] < max_h)
            category_points = obstacle_points[mask]
            
            if len(category_points) > 0:
                # Cluster points in this height category
                clusters = self.cluster_points(category_points)
                classified[category] = clusters
        
        return classified
    
    def cluster_points(self, points):
        """Cluster points using DBSCAN"""
        if len(points) < self.min_cluster_size:
            return []
        
        # Apply DBSCAN clustering
        clustering = DBSCAN(eps=self.cluster_tolerance, min_samples=5).fit(points)
        labels = clustering.labels_
        
        clusters = []
        for label in set(labels):
            if label == -1:  # Noise points
                continue
            
            cluster_points = points[labels == label]
            if self.min_cluster_size <= len(cluster_points) <= self.max_cluster_size:
                # Calculate cluster properties
                centroid = np.mean(cluster_points, axis=0)
                min_point = np.min(cluster_points, axis=0)
                max_point = np.max(cluster_points, axis=0)
                
                cluster_info = {
                    'points': cluster_points,
                    'centroid': centroid,
                    'min_bound': min_point,
                    'max_bound': max_point,
                    'size': len(cluster_points)
                }
                clusters.append(cluster_info)
        
        return clusters
    
    def publish_point_clouds(self, ground_points, obstacle_points, colors, ground_indices, obstacle_indices, header):
        """Publish ground and obstacle point clouds"""
        # Publish ground points
        if len(ground_points) > 0:
            ground_cloud = self.create_point_cloud(ground_points, colors[ground_indices], header, [0, 255, 0])
            self.ground_cloud_pub.publish(ground_cloud)
        
        # Publish obstacle points
        if len(obstacle_points) > 0:
            obstacle_cloud = self.create_point_cloud(obstacle_points, colors[obstacle_indices], header, [255, 0, 0])
            self.obstacles_cloud_pub.publish(obstacle_cloud)
    
    def create_point_cloud(self, points, colors, header, default_color=None):
        """Create PointCloud2 message from points and colors"""
        if default_color is not None:
            colors = np.full((len(points), 3), default_color, dtype=np.uint8)
        
        # Create structured array for point cloud
        cloud_data = []
        for i, point in enumerate(points):
            if i < len(colors):
                color = colors[i]
                # Pack RGB into single float
                rgb = (int(color[0]) << 16) | (int(color[1]) << 8) | int(color[2])
                cloud_data.append([point[0], point[1], point[2], rgb])
            else:
                cloud_data.append([point[0], point[1], point[2], 0])
        
        # Create PointCloud2 message
        fields = [
            pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='rgb', offset=12, datatype=pc2.PointField.UINT32, count=1),
        ]
        
        cloud_msg = pc2.create_cloud(header, fields, cloud_data)
        return cloud_msg
    
    def publish_classified_obstacles(self, classified_obstacles, header):
        """Publish classified obstacles as markers"""
        marker_array = MarkerArray()
        marker_id = 0
        
        # Color scheme for different height categories
        colors = {
            'low': ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.7),      # Yellow
            'medium': ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.7),   # Orange
            'high': ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.7)      # Red
        }
        
        for category, clusters in classified_obstacles.items():
            color = colors.get(category, ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.7))
            
            for cluster in clusters:
                marker = Marker()
                marker.header = header
                marker.ns = f"obstacles_{category}"
                marker.id = marker_id
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                
                # Set position to cluster centroid
                marker.pose.position.x = float(cluster['centroid'][0])
                marker.pose.position.y = float(cluster['centroid'][1])
                marker.pose.position.z = float(cluster['centroid'][2])
                marker.pose.orientation.w = 1.0
                
                # Set scale based on cluster bounds
                size = cluster['max_bound'] - cluster['min_bound']
                marker.scale.x = max(0.1, float(size[0]))
                marker.scale.y = max(0.1, float(size[1]))
                marker.scale.z = max(0.1, float(size[2]))
                
                marker.color = color
                marker.lifetime = rclpy.duration.Duration(seconds=1.0).to_msg()
                
                marker_array.markers.append(marker)
                marker_id += 1
        
        self.classified_obstacles_pub.publish(marker_array)
    
    def publish_processed_depth_image(self, depth_image, ground_indices, obstacle_indices, header):
        """Publish processed depth image for debugging"""
        # Create color-coded depth image
        height, width = depth_image.shape
        processed_image = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=255.0/self.max_depth), 
            cv2.COLORMAP_JET
        )
        
        # Overlay ground and obstacle classifications
        # This is a simplified visualization - in practice you'd need to map 3D points back to image
        
        try:
            processed_msg = self.cv_bridge.cv2_to_imgmsg(processed_image, encoding='bgr8')
            processed_msg.header = header
            self.processed_depth_pub.publish(processed_msg)
        except Exception as e:
            self.get_logger().warn(f"Failed to publish processed depth image: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        mapper_node = DepthObstacleMapper()
        rclpy.spin(mapper_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in depth obstacle mapper: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
