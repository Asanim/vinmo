#!/usr/bin/env python3
"""
Depth-based Obstacle Avoidance with Keyboard Override
Processes depth images to find free space and navigate autonomously
while allowing manual keyboard control override.
"""

import sys
import select
import termios
import tty
import threading
import numpy as np
import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped


class PIDController:
    """Simple PID controller for smooth angular control"""
    
    def __init__(self, kp=1.0, ki=0.0, kd=0.1, output_limit=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        
        self.integral = 0.0
        self.prev_error = 0.0
        
    def compute(self, error, dt):
        """Compute PID output"""
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        output = np.clip(output, -self.output_limit, self.output_limit)
        
        self.prev_error = error
        return output
    
    def reset(self):
        """Reset PID state"""
        self.integral = 0.0
        self.prev_error = 0.0


class DepthObstacleAvoidance(Node):
    """
    Autonomous obstacle avoidance using depth camera with keyboard override
    """
    
    def __init__(self):
        super().__init__('depth_obstacle_avoidance')
        
        # Declare parameters
        self.declare_parameter('safe_distance', 2.0)  # meters
        self.declare_parameter('min_obstacle_area', 500)  # pixels
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('angular_speed', 0.8)
        self.declare_parameter('pid_kp', 1.5)
        self.declare_parameter('pid_ki', 0.0)
        self.declare_parameter('pid_kd', 0.2)
        self.declare_parameter('manual_timeout', 2.0)  # seconds
        
        # Get parameters
        self.safe_distance = self.get_parameter('safe_distance').value
        self.min_obstacle_area = self.get_parameter('min_obstacle_area').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.manual_timeout = self.get_parameter('manual_timeout').value
        
        # Initialize PID controller
        self.pid = PIDController(
            kp=self.get_parameter('pid_kp').value,
            ki=self.get_parameter('pid_ki').value,
            kd=self.get_parameter('pid_kd').value,
            output_limit=self.angular_speed
        )
        
        # Subscribers
        self.depth_sub = self.create_subscription(
            Image,
            '/front_camera/depth_image',
            self.depth_callback,
            10
        )
        
        # Publisher
        self.cmd_vel_pub = self.create_publisher(
            TwistStamped,
            '/diff_drive_controller/cmd_vel',
            10
        )
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # State variables
        self.current_depth_image = None
        self.target_direction = 0.0  # Angular offset from center
        self.is_manual_mode = False
        self.last_manual_command_time = self.get_clock().now()
        self.manual_linear = 0.0
        self.manual_angular = 0.0
        
        # Terminal settings for keyboard input
        self.settings = None
        
        # Control timer
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        # Keyboard thread
        self.keyboard_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.keyboard_thread.start()
        
        self.get_logger().info('Depth Obstacle Avoidance Node Started')
        self.get_logger().info('Press W/A/S/D for manual control, SPACE to stop, ESC to quit')
        
    def depth_callback(self, msg):
        """Process incoming depth image"""
        try:
            # Convert ROS Image to OpenCV format
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            self.current_depth_image = depth_image
            
            # Process depth image
            self.process_depth_image(depth_image)
            
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {e}')
    
    def process_depth_image(self, depth_image):
        """
        Analyze depth image to find free space direction
        """
        if depth_image is None or depth_image.size == 0:
            return
        
        h, w = depth_image.shape
        
        # Replace NaN and inf values with max distance
        depth_image = np.nan_to_num(depth_image, nan=10.0, posinf=10.0, neginf=0.0)
        
        # Create obstacle mask (pixels closer than safe distance)
        obstacle_mask = (depth_image < self.safe_distance).astype(np.uint8) * 255
        
        # Apply morphological operations to reduce noise
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        obstacle_mask = cv2.morphologyEx(obstacle_mask, cv2.MORPH_CLOSE, kernel)
        obstacle_mask = cv2.morphologyEx(obstacle_mask, cv2.MORPH_OPEN, kernel)
        
        # Find contours of obstacles
        contours, _ = cv2.findContours(
            obstacle_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )
        
        # Filter contours by area
        valid_obstacles = [
            cnt for cnt in contours
            if cv2.contourArea(cnt) > self.min_obstacle_area
        ]
        
        # Find free space direction
        self.target_direction = self.find_free_space_direction(
            obstacle_mask, valid_obstacles, w, h
        )
        
        # Visualize
        self.visualize_depth_analysis(depth_image, obstacle_mask, valid_obstacles, w, h)
    
    def find_free_space_direction(self, obstacle_mask, contours, width, height):
        """
        Find the direction with most free space
        Returns angular offset from center (-1 to 1, normalized)
        """
        # Divide image into vertical slices
        num_slices = 10
        slice_width = width // num_slices
        
        # Calculate free space score for each slice
        free_space_scores = []
        for i in range(num_slices):
            x_start = i * slice_width
            x_end = x_start + slice_width
            
            # Count free pixels in lower half of image (more relevant for navigation)
            slice_region = obstacle_mask[height//2:, x_start:x_end]
            free_pixels = np.sum(slice_region == 0)
            
            # Weight center slices higher (prefer going straight)
            center_weight = 1.0 - abs(i - num_slices/2) / (num_slices/2) * 0.3
            score = free_pixels * center_weight
            
            free_space_scores.append(score)
        
        # Find slice with highest score
        if len(free_space_scores) > 0 and max(free_space_scores) > 0:
            best_slice = np.argmax(free_space_scores)
            # Convert to normalized direction (-1 left, 0 center, 1 right)
            direction = (best_slice - num_slices/2) / (num_slices/2)
            return direction
        
        return 0.0  # Default to center if no clear path
    
    def visualize_depth_analysis(self, depth_image, obstacle_mask, contours, width, height):
        """Display depth analysis with OpenCV"""
        # Normalize depth for visualization
        depth_viz = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
        depth_viz = depth_viz.astype(np.uint8)
        depth_viz = cv2.applyColorMap(depth_viz, cv2.COLORMAP_JET)
        
        # Draw obstacle contours
        cv2.drawContours(depth_viz, contours, -1, (0, 0, 255), 2)
        
        # Draw target direction indicator
        center_x = width // 2
        center_y = height // 2
        target_x = int(center_x + self.target_direction * width * 0.3)
        
        cv2.arrowedLine(
            depth_viz,
            (center_x, center_y),
            (target_x, int(height * 0.8)),
            (0, 255, 0),
            3,
            tipLength=0.3
        )
        
        # Draw mode indicator
        mode_text = "MANUAL" if self.is_manual_mode else "AUTO"
        color = (0, 165, 255) if self.is_manual_mode else (0, 255, 0)
        cv2.putText(
            depth_viz,
            mode_text,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            color,
            2
        )
        
        # Draw obstacle mask
        obstacle_viz = cv2.cvtColor(obstacle_mask, cv2.COLOR_GRAY2BGR)
        
        # Combine visualizations
        combined = np.hstack([depth_viz, obstacle_viz])
        
        # Display
        cv2.imshow('Depth Analysis | Obstacle Mask', combined)
        cv2.waitKey(1)
    
    def control_loop(self):
        """Main control loop - executes at 10Hz"""
        # Check if manual mode has timed out
        time_since_manual = (self.get_clock().now() - self.last_manual_command_time).nanoseconds / 1e9
        
        if self.is_manual_mode and time_since_manual > self.manual_timeout:
            self.is_manual_mode = False
            self.get_logger().info('Switching to AUTO mode')
            self.pid.reset()
        
        # Create velocity command
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        if self.is_manual_mode:
            # Use manual commands
            msg.twist.linear.x = self.manual_linear
            msg.twist.angular.z = self.manual_angular
        else:
            # Use autonomous navigation with PID control
            if self.current_depth_image is not None:
                # Compute angular velocity using PID
                angular_vel = self.pid.compute(self.target_direction, 0.1)
                
                msg.twist.linear.x = self.linear_speed
                msg.twist.angular.z = angular_vel
            else:
                # No depth data, stop
                msg.twist.linear.x = 0.0
                msg.twist.angular.z = 0.0
        
        # Publish command
        self.cmd_vel_pub.publish(msg)
    
    def get_key(self, timeout=0.05):
        """Get keyboard input with timeout"""
        if self.settings is None:
            self.settings = termios.tcgetattr(sys.stdin)
        
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def keyboard_loop(self):
        """Separate thread for keyboard input"""
        try:
            while rclpy.ok():
                key = self.get_key(timeout=0.05)
                
                if key == '\x1b' or key == '\x03':  # ESC or CTRL-C
                    self.get_logger().info('Shutting down...')
                    rclpy.shutdown()
                    break
                
                elif key in ['w', 's', 'a', 'd', 'x', ' ']:
                    self.is_manual_mode = True
                    self.last_manual_command_time = self.get_clock().now()
                    
                    if key == 'w':  # Forward
                        self.manual_linear = self.linear_speed
                        self.manual_angular = 0.0
                    elif key == 's':  # Backward
                        self.manual_linear = -self.linear_speed * 0.5
                        self.manual_angular = 0.0
                    elif key == 'a':  # Turn left
                        self.manual_linear = 0.0
                        self.manual_angular = self.angular_speed
                    elif key == 'd':  # Turn right
                        self.manual_linear = 0.0
                        self.manual_angular = -self.angular_speed
                    elif key == 'x' or key == ' ':  # Stop
                        self.manual_linear = 0.0
                        self.manual_angular = 0.0
                        self.is_manual_mode = False
                        self.get_logger().info('STOP - Switching to AUTO mode')
                
        except Exception as e:
            self.get_logger().error(f'Keyboard loop error: {e}')
        finally:
            if self.settings is not None:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
    
    def cleanup(self):
        """Cleanup on shutdown"""
        # Stop robot
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = 0.0
        msg.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)
        
        # Close OpenCV windows
        cv2.destroyAllWindows()
        
        # Restore terminal
        if self.settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main(args=None):
    rclpy.init(args=args)
    
    node = DepthObstacleAvoidance()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
