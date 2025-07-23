#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
import time

class ClockMonitor(Node):
    def __init__(self):
        super().__init__('clock_monitor')
        
        # Subscribe to clock topic
        self.clock_sub = self.create_subscription(
            Clock,
            '/clock',
            self.clock_callback,
            10
        )
        
        # Timer to check clock reception
        self.timer = self.create_timer(2.0, self.check_clock)
        self.last_clock_time = None
        self.clock_received = False
        
        self.get_logger().info("Clock Monitor started. Checking for /clock topic...")

    def clock_callback(self, msg):
        self.last_clock_time = msg.clock
        self.clock_received = True
        self.get_logger().info(f"Clock received: {msg.clock.sec}.{msg.clock.nanosec}")

    def check_clock(self):
        if not self.clock_received:
            self.get_logger().warn("No clock messages received on /clock topic!")
            self.get_logger().warn("Make sure ros_gz_bridge is running with clock bridge")
        else:
            self.get_logger().info(f"Clock is working. Last time: {self.last_clock_time.sec}.{self.last_clock_time.nanosec}")

def main(args=None):
    rclpy.init(args=args)
    
    clock_monitor = ClockMonitor()
    
    try:
        rclpy.spin(clock_monitor)
    except KeyboardInterrupt:
        pass
    
    clock_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
