#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        
        # Create publisher
        self.publisher = self.create_publisher(String, 'hello_topic', 10)
        
        # Create timer to publish messages
        self.timer = self.create_timer(1.0, self.publish_message)
        self.counter = 0
        
        self.get_logger().info("Simple Publisher started. Publishing to 'hello_topic'")

    def publish_message(self):
        msg = String()
        msg.data = f"Hello World! Message #{self.counter}"
        self.publisher.publish(msg)
        self.get_logger().info(f"Published: '{msg.data}'")
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    
    publisher = SimplePublisher()
    
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
