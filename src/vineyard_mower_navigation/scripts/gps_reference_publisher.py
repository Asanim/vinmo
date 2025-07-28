#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header
import tf2_ros
import tf2_geometry_msgs
from tf2_geometry_msgs import do_transform_pose_stamped
import math


class GPSReferencePublisher(Node):
    """
    GPS Reference Publisher for Vineyard Navigation
    
    This node provides GPS-assisted localization fallback for the vineyard mower.
    It can publish initial pose estimates based on GPS coordinates and provides
    a reference frame for outdoor navigation when AMCL tracking is lost.
    """

    def __init__(self):
        super().__init__('gps_reference_publisher')
        
        # Parameters
        self.declare_parameter('use_sim_time', True)
        self.declare_parameter('gps_frame', 'gps_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('gps_variance', 2.0)  # GPS accuracy in meters
        
        # Get parameters
        self.use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        self.gps_frame = self.get_parameter('gps_frame').get_parameter_value().string_value
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.gps_variance = self.get_parameter('gps_variance').get_parameter_value().double_value
        
        # Publishers
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )
        
        # Subscribers
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_callback,
            10
        )
        
        # TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # State variables
        self.gps_reference_set = False
        self.reference_lat = None
        self.reference_lon = None
        self.reference_x = 0.0
        self.reference_y = 0.0
        self.last_gps_pose_time = None
        
        # Timer for periodic operations
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        
        self.get_logger().info('GPS Reference Publisher initialized')
        self.get_logger().info(f'GPS variance set to: {self.gps_variance} meters')

    def gps_callback(self, msg):
        """Handle incoming GPS fix messages"""
        if msg.status.status < 0:  # No fix
            return
            
        # Set reference point on first valid GPS reading
        if not self.gps_reference_set:
            self.reference_lat = msg.latitude
            self.reference_lon = msg.longitude
            self.gps_reference_set = True
            self.get_logger().info(f'GPS reference set to: {self.reference_lat:.6f}, {self.reference_lon:.6f}')
            return
        
        # Convert GPS to local coordinates
        x, y = self.gps_to_local(msg.latitude, msg.longitude)
        
        # Create pose message
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = Header()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.map_frame
        
        # Set position
        pose_msg.pose.pose.position.x = x
        pose_msg.pose.pose.position.y = y
        pose_msg.pose.pose.position.z = 0.0
        
        # Set orientation (unknown from GPS alone)
        pose_msg.pose.pose.orientation.x = 0.0
        pose_msg.pose.pose.orientation.y = 0.0
        pose_msg.pose.pose.orientation.z = 0.0
        pose_msg.pose.pose.orientation.w = 1.0
        
        # Set covariance matrix
        # Position uncertainty based on GPS accuracy
        variance = self.gps_variance ** 2
        pose_msg.pose.covariance[0] = variance      # x-x
        pose_msg.pose.covariance[7] = variance      # y-y
        pose_msg.pose.covariance[14] = 0.01         # z-z (very certain about ground level)
        pose_msg.pose.covariance[21] = 1000.0       # roll-roll (unknown)
        pose_msg.pose.covariance[28] = 1000.0       # pitch-pitch (unknown)
        pose_msg.pose.covariance[35] = 1000.0       # yaw-yaw (unknown)
        
        # Store for potential publishing
        self.last_gps_pose = pose_msg
        self.last_gps_pose_time = self.get_clock().now()
        
    def gps_to_local(self, lat, lon):
        """Convert GPS coordinates to local x,y coordinates"""
        # Simple approximation for small areas
        # For more accuracy, consider using pyproj or similar
        
        # Calculate differences in degrees
        dlat = lat - self.reference_lat
        dlon = lon - self.reference_lon
        
        # Convert to meters (approximate)
        # 1 degree latitude ≈ 111,320 meters
        # 1 degree longitude ≈ 111,320 * cos(latitude) meters
        lat_rad = math.radians(self.reference_lat)
        
        x = dlon * 111320.0 * math.cos(lat_rad)
        y = dlat * 111320.0
        
        return x, y
    
    def timer_callback(self):
        """Periodic callback for publishing GPS-based pose estimates"""
        # This can be extended to publish periodic pose updates
        # or handle fallback localization scenarios
        pass
    
    def publish_initial_pose_from_gps(self):
        """Publish initial pose estimate based on latest GPS reading"""
        if hasattr(self, 'last_gps_pose') and self.last_gps_pose is not None:
            # Update timestamp
            self.last_gps_pose.header.stamp = self.get_clock().now().to_msg()
            
            # Publish to initialpose topic for AMCL
            self.initial_pose_pub.publish(self.last_gps_pose)
            
            self.get_logger().info('Published GPS-based initial pose estimate')
    
    def set_gps_reference(self, lat, lon):
        """Manually set GPS reference point"""
        self.reference_lat = lat
        self.reference_lon = lon
        self.gps_reference_set = True
        self.get_logger().info(f'GPS reference manually set to: {lat:.6f}, {lon:.6f}')


def main(args=None):
    rclpy.init(args=args)
    
    gps_reference_publisher = GPSReferencePublisher()
    
    try:
        rclpy.spin(gps_reference_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        gps_reference_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
            10
        )
        
        # Timer for publishing
        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_reference
        )
        
        self.get_logger().info(f'GPS Reference Publisher started')
        self.get_logger().info(f'Reference coordinates: {self.reference_lat:.6f}, {self.reference_lon:.6f}, {self.reference_alt:.2f}')
    
    def publish_reference(self):
        """Publish GPS reference point and pose"""
        current_time = self.get_clock().now().to_msg()
        
        # Publish GPS reference
        gps_msg = NavSatFix()
        gps_msg.header = Header()
        gps_msg.header.stamp = current_time
        gps_msg.header.frame_id = 'gps_link'
        
        gps_msg.latitude = self.reference_lat
        gps_msg.longitude = self.reference_lon
        gps_msg.altitude = self.reference_alt
        
        # Set covariance (9 elements for 3x3 matrix)
        gps_msg.position_covariance = [
            self.cov_x, 0.0, 0.0,
            0.0, self.cov_y, 0.0,
            0.0, 0.0, self.cov_z
        ]
        gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        gps_msg.status.status = NavSatFix.STATUS_FIX
        gps_msg.status.service = NavSatFix.SERVICE_GPS
        
        self.gps_ref_pub.publish(gps_msg)
        
        # Publish reference pose in map frame
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = Header()
        pose_msg.header.stamp = current_time
        pose_msg.header.frame_id = 'map'
        
        # Convert GPS to UTM-like local coordinates (simplified)
        # For vineyard navigation, we use the reference point as origin (0,0)
        pose_msg.pose.pose.position.x = 0.0
        pose_msg.pose.pose.position.y = 0.0
        pose_msg.pose.pose.position.z = 0.0
        
        # Identity quaternion (no rotation)
        pose_msg.pose.pose.orientation.x = 0.0
        pose_msg.pose.pose.orientation.y = 0.0
        pose_msg.pose.pose.orientation.z = 0.0
        pose_msg.pose.pose.orientation.w = 1.0
        
        # Set covariance (6x6 matrix, 36 elements)
        pose_msg.pose.covariance = [0.0] * 36
        pose_msg.pose.covariance[0] = self.cov_x   # x
        pose_msg.pose.covariance[7] = self.cov_y   # y
        pose_msg.pose.covariance[14] = self.cov_z  # z
        pose_msg.pose.covariance[21] = 0.1  # roll
        pose_msg.pose.covariance[28] = 0.1  # pitch
        pose_msg.pose.covariance[35] = 0.1  # yaw
        
        self.pose_ref_pub.publish(pose_msg)
    
    def gps_to_utm(self, lat, lon):
        """
        Simple GPS to UTM conversion for local coordinates
        This is a simplified version for vineyard navigation
        """
        # Earth radius in meters
        R = 6378137.0
        
        # Convert to radians
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        ref_lat_rad = math.radians(self.reference_lat)
        ref_lon_rad = math.radians(self.reference_lon)
        
        # Calculate relative distances
        dlat = lat_rad - ref_lat_rad
        dlon = lon_rad - ref_lon_rad
        
        # Convert to meters (approximate)
        x = R * dlon * math.cos(ref_lat_rad)
        y = R * dlat
        
        return x, y


def main(args=None):
    rclpy.init(args=args)
    
    gps_reference_publisher = GPSReferencePublisher()
    
    try:
        rclpy.spin(gps_reference_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        gps_reference_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
