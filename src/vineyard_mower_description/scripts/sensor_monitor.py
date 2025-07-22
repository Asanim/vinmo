#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu, CameraInfo
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import time

class SensorMonitor(Node):
    def __init__(self):
        super().__init__('sensor_monitor')
        
        # Subscribers for all sensors
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.front_image_sub = self.create_subscription(Image, '/front_camera/image', self.front_image_callback, 10)
        self.front_depth_sub = self.create_subscription(Image, '/front_camera/depth_image', self.front_depth_callback, 10)
        self.rear_image_sub = self.create_subscription(Image, '/rear_camera/image', self.rear_image_callback, 10)
        self.rear_depth_sub = self.create_subscription(Image, '/rear_camera/depth_image', self.rear_depth_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/diff_drive_controller/odom', self.odom_callback, 10)
        
        # Track last received times
        self.last_scan = None
        self.last_front_image = None
        self.last_front_depth = None
        self.last_rear_image = None
        self.last_rear_depth = None
        self.last_imu = None
        self.last_odom = None
        
        # Timer for status reporting
        self.timer = self.create_timer(2.0, self.report_status)
        
        self.get_logger().info('Sensor monitor started. Monitoring all sensor topics...')

    def scan_callback(self, msg):
        self.last_scan = time.time()

    def front_image_callback(self, msg):
        self.last_front_image = time.time()

    def front_depth_callback(self, msg):
        self.last_front_depth = time.time()

    def rear_image_callback(self, msg):
        self.last_rear_image = time.time()

    def rear_depth_callback(self, msg):
        self.last_rear_depth = time.time()

    def imu_callback(self, msg):
        self.last_imu = time.time()

    def odom_callback(self, msg):
        self.last_odom = time.time()

    def report_status(self):
        current_time = time.time()
        
        sensors = {
            'LiDAR (/scan)': self.last_scan,
            'Front Camera': self.last_front_image,
            'Front Depth': self.last_front_depth,
            'Rear Camera': self.last_rear_image,
            'Rear Depth': self.last_rear_depth,
            'IMU': self.last_imu,
            'Odometry': self.last_odom
        }
        
        self.get_logger().info('=== Sensor Status Report ===')
        
        for sensor_name, last_time in sensors.items():
            if last_time is None:
                status_str = 'NO DATA'
            elif current_time - last_time < 5.0:
                status_str = f'ACTIVE ({current_time - last_time:.1f}s ago)'
            else:
                status_str = f'STALE ({current_time - last_time:.1f}s ago)'
            
            self.get_logger().info(f'{sensor_name:15}: {status_str}')

def main(args=None):
    rclpy.init(args=args)
    node = SensorMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
