#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        
        # Create subscriber
        self.subscription = self.create_subscription(
            String,
            'hello_topic',
            self.message_callback,
            10
        )
        
        self.get_logger().info("Simple Subscriber started. Listening to 'hello_topic'")

    def message_callback(self, msg):
        self.get_logger().info(f"Received: '{msg.data}'")

def main(args=None):
    rclpy.init(args=args)
    
    subscriber = SimpleSubscriber()
    
    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        pass
    
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
