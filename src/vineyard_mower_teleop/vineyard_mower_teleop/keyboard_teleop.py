#!/usr/bin/env python3
"""
Keyboard Teleoperation Node for Vineyard Mower
Publishes TwistStamped messages to control the differential drive robot
"""

import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

# Key mappings
KEY_BINDINGS = {
    'w': (1, 0, 0, 0),   # Forward
    's': (-1, 0, 0, 0),  # Backward
    'a': (0, 0, 0, 1),   # Turn left
    'd': (0, 0, 0, -1),  # Turn right
    'q': (1, 0, 0, 1),   # Forward + left
    'e': (1, 0, 0, -1),  # Forward + right
    'z': (-1, 0, 0, 1),  # Backward + left
    'c': (-1, 0, 0, -1), # Backward + right
    'x': (0, 0, 0, 0),   # Stop
}

HELP_TEXT = """
---------------------------
Vineyard Mower Keyboard Teleop
---------------------------
Moving around:
   q    w    e
   a    x    d
   z    s    c

w/s : increase/decrease linear velocity
a/d : increase/decrease angular velocity
q/e : forward + turn left/right
z/c : backward + turn left/right
x   : force stop

i/k : increase/decrease linear speed by 10%
j/l : increase/decrease angular speed by 10%

SPACE : emergency stop
ESC/CTRL-C : quit

Current target: linear={linear:.2f}, angular={angular:.2f}
---------------------------
"""


class KeyboardTeleopNode(Node):
    """Keyboard teleoperation node for differential drive robot"""

    def __init__(self):
        super().__init__('keyboard_teleop')
        
        # Declare parameters
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('angular_speed', 1.0)
        self.declare_parameter('linear_increment', 0.05)
        self.declare_parameter('angular_increment', 0.1)
        self.declare_parameter('publish_rate', 10.0)
        
        # Get parameters
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.linear_increment = self.get_parameter('linear_increment').value
        self.angular_increment = self.get_parameter('angular_increment').value
        publish_rate = self.get_parameter('publish_rate').value
        
        # Create publisher
        self.publisher = self.create_publisher(
            TwistStamped,
            '/diff_drive_controller/cmd_vel',
            10
        )
        
        # Create timer for publishing
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_velocity)
        
        # Initialize velocity targets
        self.target_linear = 0.0
        self.target_angular = 0.0
        
        # Terminal settings
        self.settings = None
        
        self.get_logger().info('Keyboard Teleop Node Started')
        self.get_logger().info(f'Linear speed: {self.linear_speed:.2f} m/s')
        self.get_logger().info(f'Angular speed: {self.angular_speed:.2f} rad/s')
        self.get_logger().info('Publishing to: /diff_drive_controller/cmd_vel')

    def get_key(self, timeout=0.1):
        """Get a key press from the terminal with timeout"""
        if self.settings is None:
            self.settings = termios.tcgetattr(sys.stdin)
        
        tty.setraw(sys.stdin.fileno())
        
        # Use select to check if input is available
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def publish_velocity(self):
        """Publish velocity command"""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = self.target_linear
        msg.twist.angular.z = self.target_angular
        self.publisher.publish(msg)

    def run(self):
        """Main loop for keyboard teleoperation"""
        print(HELP_TEXT.format(linear=self.linear_speed, angular=self.angular_speed))
        
        try:
            while rclpy.ok():
                key = self.get_key(timeout=0.1)
                
                if key == '\x03':  # CTRL-C
                    break
                elif key == '\x1b':  # ESC
                    break
                elif key == ' ':  # SPACE - emergency stop
                    self.target_linear = 0.0
                    self.target_angular = 0.0
                    self.get_logger().info('EMERGENCY STOP')
                elif key in KEY_BINDINGS:
                    # Get movement from key binding
                    linear_dir, _, _, angular_dir = KEY_BINDINGS[key]
                    self.target_linear = linear_dir * self.linear_speed
                    self.target_angular = angular_dir * self.angular_speed
                    
                    if key != 'x':
                        self.get_logger().info(
                            f'Command: linear={self.target_linear:.2f}, '
                            f'angular={self.target_angular:.2f}'
                        )
                elif key == 'i':
                    self.linear_speed = min(self.linear_speed * 1.1, 2.0)
                    self.get_logger().info(f'Linear speed: {self.linear_speed:.2f} m/s')
                elif key == 'k':
                    self.linear_speed = max(self.linear_speed * 0.9, 0.1)
                    self.get_logger().info(f'Linear speed: {self.linear_speed:.2f} m/s')
                elif key == 'j':
                    self.angular_speed = min(self.angular_speed * 1.1, 3.0)
                    self.get_logger().info(f'Angular speed: {self.angular_speed:.2f} rad/s')
                elif key == 'l':
                    self.angular_speed = max(self.angular_speed * 0.9, 0.1)
                    self.get_logger().info(f'Angular speed: {self.angular_speed:.2f} rad/s')
                
                # Spin once to process callbacks
                rclpy.spin_once(self, timeout_sec=0)
                
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
        finally:
            # Stop the robot
            self.target_linear = 0.0
            self.target_angular = 0.0
            self.publish_velocity()
            
            # Restore terminal settings
            if self.settings is not None:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main(args=None):
    rclpy.init(args=args)
    
    node = KeyboardTeleopNode()
    
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
