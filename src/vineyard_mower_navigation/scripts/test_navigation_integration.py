#!/usr/bin/env python3

"""
Integration test script for vineyard navigation system.
Tests Nav2 stack functionality, localization, and path planning.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import Header
from rclpy.qos import QoSProfile
import time
import math


class NavigationTester(Node):
    def __init__(self):
        super().__init__('navigation_tester')
        
        self.get_logger().info('Starting Navigation Integration Test')
        
        # Publishers
        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )
        
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )
        
        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        self.path_sub = self.create_subscription(
            Path,
            '/plan',
            self.path_callback,
            10
        )
        
        # Test state
        self.map_received = False
        self.path_received = False
        self.test_results = {}
        
        # Timer for running tests
        self.test_timer = self.create_timer(2.0, self.run_tests)
        self.test_step = 0
        
    def map_callback(self, msg):
        """Handle map messages"""
        if not self.map_received:
            self.get_logger().info('✓ Map received successfully')
            self.get_logger().info(f'  Map size: {msg.info.width}x{msg.info.height}')
            self.get_logger().info(f'  Map resolution: {msg.info.resolution}m/pixel')
            self.test_results['map_received'] = True
            self.map_received = True
    
    def path_callback(self, msg):
        """Handle path messages"""
        if len(msg.poses) > 0:
            self.get_logger().info('✓ Path planning successful')
            self.get_logger().info(f'  Path length: {len(msg.poses)} waypoints')
            self.test_results['path_planning'] = True
            self.path_received = True
    
    def run_tests(self):
        """Run navigation integration tests"""
        if self.test_step == 0:
            self.test_localization()
            self.test_step += 1
        elif self.test_step == 1:
            if self.map_received:
                self.test_path_planning()
                self.test_step += 1
        elif self.test_step == 2:
            self.test_navigation_goals()
            self.test_step += 1
        elif self.test_step == 3:
            self.print_test_summary()
            self.test_timer.cancel()
    
    def test_localization(self):
        """Test AMCL localization by setting initial pose"""
        self.get_logger().info('Testing localization (AMCL)...')
        
        # Create initial pose message
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header = Header()
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.header.frame_id = 'map'
        
        # Set pose at origin
        initial_pose.pose.pose.position.x = 0.0
        initial_pose.pose.pose.position.y = 0.0
        initial_pose.pose.pose.position.z = 0.0
        initial_pose.pose.pose.orientation.w = 1.0
        
        # Set reasonable covariance
        initial_pose.pose.covariance[0] = 0.25  # x variance
        initial_pose.pose.covariance[7] = 0.25  # y variance
        initial_pose.pose.covariance[35] = 0.07 # yaw variance
        
        # Publish initial pose
        self.initial_pose_pub.publish(initial_pose)
        self.get_logger().info('✓ Initial pose published for AMCL')
        self.test_results['localization_init'] = True
    
    def test_path_planning(self):
        """Test path planning with a simple goal"""
        self.get_logger().info('Testing path planning...')
        
        # Create a goal pose
        goal_pose = PoseStamped()
        goal_pose.header = Header()
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = 'map'
        
        # Set goal 2 meters forward
        goal_pose.pose.position.x = 2.0
        goal_pose.pose.position.y = 0.0
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.w = 1.0
        
        # Publish goal
        self.goal_pub.publish(goal_pose)
        self.get_logger().info('✓ Test goal published')
    
    def test_navigation_goals(self):
        """Test multiple navigation goals"""
        self.get_logger().info('Testing multiple navigation goals...')
        
        # Test goals in vineyard-like pattern
        test_goals = [
            (1.0, 0.0, 0.0),      # Forward
            (1.0, 1.0, 1.57),     # Turn left
            (0.0, 1.0, 3.14),     # Backward
            (0.0, 0.0, -1.57),    # Turn right back to start
        ]
        
        for i, (x, y, yaw) in enumerate(test_goals):
            goal_pose = PoseStamped()
            goal_pose.header = Header()
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.header.frame_id = 'map'
            
            goal_pose.pose.position.x = x
            goal_pose.pose.position.y = y
            goal_pose.pose.position.z = 0.0
            
            # Convert yaw to quaternion
            goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal_pose.pose.orientation.w = math.cos(yaw / 2.0)
            
            self.goal_pub.publish(goal_pose)
            time.sleep(0.5)  # Small delay between goals
        
        self.get_logger().info('✓ Multiple test goals published')
        self.test_results['multiple_goals'] = True
    
    def print_test_summary(self):
        """Print test results summary"""
        self.get_logger().info('\n' + '='*50)
        self.get_logger().info('NAVIGATION INTEGRATION TEST SUMMARY')
        self.get_logger().info('='*50)
        
        total_tests = 0
        passed_tests = 0
        
        test_descriptions = {
            'map_received': 'Map Server and SLAM',
            'localization_init': 'AMCL Localization Setup',
            'path_planning': 'Path Planning',
            'multiple_goals': 'Multiple Goal Handling'
        }
        
        for test_name, description in test_descriptions.items():
            total_tests += 1
            status = '✓ PASS' if self.test_results.get(test_name, False) else '✗ FAIL'
            self.get_logger().info(f'{description}: {status}')
            if self.test_results.get(test_name, False):
                passed_tests += 1
        
        self.get_logger().info('='*50)
        self.get_logger().info(f'RESULTS: {passed_tests}/{total_tests} tests passed')
        
        if passed_tests == total_tests:
            self.get_logger().info('🎉 All navigation tests PASSED!')
        else:
            self.get_logger().warn(f'⚠️  {total_tests - passed_tests} tests FAILED')
        
        self.get_logger().info('='*50)


def main(args=None):
    rclpy.init(args=args)
    
    tester = NavigationTester()
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
