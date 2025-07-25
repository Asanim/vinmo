#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LaserScanFrameRemapper(Node):
    def __init__(self):
        super().__init__('laser_scan_frame_remapper')
        
        # Subscribe to the original scan with Gazebo frame
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan_raw',
            self.scan_callback,
            10
        )
        
        # Publisher for the remapped scan
        self.publisher = self.create_publisher(LaserScan, '/scan', 10)
        
        self.get_logger().info('Laser scan frame remapper started')
    
    def scan_callback(self, msg):
        # Create a new message with the same data but different frame
        remapped_msg = LaserScan()
        remapped_msg.header.stamp = msg.header.stamp
        remapped_msg.header.frame_id = 'lidar_link'  # Change to standard frame name
        remapped_msg.angle_min = msg.angle_min
        remapped_msg.angle_max = msg.angle_max
        remapped_msg.angle_increment = msg.angle_increment
        remapped_msg.time_increment = msg.time_increment
        remapped_msg.scan_time = msg.scan_time
        remapped_msg.range_min = msg.range_min
        remapped_msg.range_max = msg.range_max
        remapped_msg.ranges = msg.ranges
        remapped_msg.intensities = msg.intensities
        
        # Publish the remapped scan
        self.publisher.publish(remapped_msg)


def main(args=None):
    rclpy.init(args=args)
    
    laser_scan_frame_remapper = LaserScanFrameRemapper()
    
    try:
        rclpy.spin(laser_scan_frame_remapper)
    except KeyboardInterrupt:
        pass
    
    laser_scan_frame_remapper.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
