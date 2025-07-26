#!/usr/bin/env python3

"""
GPS Reference Publisher for Vineyard Navigation

This node publishes GPS reference coordinates and provides coordinate frame
transformations for vineyard navigation.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Header
import math


class GPSReferencePublisher(Node):
    def __init__(self):
        super().__init__('gps_reference_publisher')
        
        # Declare parameters
        self.declare_parameter('latitude', 45.5017)
        self.declare_parameter('longitude', -122.6750)
        self.declare_parameter('altitude', 50.0)
        self.declare_parameter('use_sim_time', True)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('covariance_x', 1.0)
        self.declare_parameter('covariance_y', 1.0)
        self.declare_parameter('covariance_z', 1.0)
        
        # Get parameters
        self.reference_lat = self.get_parameter('latitude').get_parameter_value().double_value
        self.reference_lon = self.get_parameter('longitude').get_parameter_value().double_value
        self.reference_alt = self.get_parameter('altitude').get_parameter_value().double_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.cov_x = self.get_parameter('covariance_x').get_parameter_value().double_value
        self.cov_y = self.get_parameter('covariance_y').get_parameter_value().double_value
        self.cov_z = self.get_parameter('covariance_z').get_parameter_value().double_value
        
        # Publishers
        self.gps_ref_pub = self.create_publisher(
            NavSatFix, 
            '/gps/reference', 
            10
        )
        
        self.pose_ref_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/gps/reference_pose',
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
