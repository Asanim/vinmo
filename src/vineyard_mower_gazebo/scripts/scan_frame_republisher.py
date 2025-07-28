#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanFrameRepublisher(Node):
    def __init__(self):
        super().__init__('scan_frame_republisher')
        
        # Subscribe to the raw scan from Gazebo
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan_raw',
            self.scan_callback,
            10
        )
        
        # Publisher for the corrected scan
        self.publisher = self.create_publisher(LaserScan, '/scan', 10)
        
        self.get_logger().info('Scan frame republisher started')
    
    def scan_callback(self, msg):
        # Create new message with corrected frame
        corrected_msg = LaserScan()
        corrected_msg.header = msg.header
        corrected_msg.header.frame_id = 'lidar_link'  # Change to the correct frame
        corrected_msg.angle_min = msg.angle_min
        corrected_msg.angle_max = msg.angle_max
        corrected_msg.angle_increment = msg.angle_increment
        corrected_msg.time_increment = msg.time_increment
        corrected_msg.scan_time = msg.scan_time
        corrected_msg.range_min = msg.range_min
        corrected_msg.range_max = msg.range_max
        corrected_msg.ranges = msg.ranges
        corrected_msg.intensities = msg.intensities
        
        # Publish the corrected scan
        self.publisher.publish(corrected_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ScanFrameRepublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
