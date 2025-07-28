#!/usr/bin/env python3
"""
Integration Test Runner

Runs automated integration tests for the vineyard web interface system.
Tests the interaction between ROS2 services, web interface, and simulation.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests
import json
import time
import threading

class IntegrationTestRunner(Node):
    def __init__(self):
        super().__init__('integration_test_runner')
        
        # Test results publisher
        self.results_publisher = self.create_publisher(String, '/integration_tests/results', 10)
        
        # Parameters
        self.declare_parameter('test_timeout', 30.0)
        self.declare_parameter('run_once', True)
        
        self.test_timeout = self.get_parameter('test_timeout').value
        self.run_once = self.get_parameter('run_once').value
        
        self.test_results = {}
        
        # Start tests after a delay to allow system startup
        self.timer = self.create_timer(10.0, self.run_integration_tests)
        
        self.get_logger().info('Integration Test Runner initialized')
        
    def run_integration_tests(self):
        """Run all integration tests"""
        if self.run_once:
            self.timer.cancel()  # Run only once
            
        self.get_logger().info('Starting integration tests...')
        
        tests = [
            ('test_rosbridge_connectivity', self.test_rosbridge_connectivity),
            ('test_backend_api', self.test_backend_api),
            ('test_costmap_service', self.test_costmap_service),
            ('test_path_planning_service', self.test_path_planning_service),
            ('test_web_ros_integration', self.test_web_ros_integration)
        ]
        
        for test_name, test_func in tests:
            try:
                self.get_logger().info(f'Running test: {test_name}')
                result = test_func()
                self.test_results[test_name] = {
                    'passed': result,
                    'timestamp': time.time(),
                    'error': None
                }
                self.get_logger().info(f'Test {test_name}: {"PASSED" if result else "FAILED"}')
            except Exception as e:
                self.test_results[test_name] = {
                    'passed': False,
                    'timestamp': time.time(),
                    'error': str(e)
                }
                self.get_logger().error(f'Test {test_name} ERROR: {e}')
                
        # Publish results
        self.publish_results()
        
        # Summary
        passed_tests = sum(1 for result in self.test_results.values() if result['passed'])
        total_tests = len(self.test_results)
        
        self.get_logger().info(f'Integration tests complete: {passed_tests}/{total_tests} passed')
        
    def test_rosbridge_connectivity(self):
        """Test ROSbridge WebSocket connectivity"""
        try:
            import websocket
            ws_url = "ws://localhost:9090"
            ws = websocket.create_connection(ws_url, timeout=5)
            
            # Test service call
            test_msg = {
                "op": "call_service",
                "service": "/rosapi/get_time",
                "type": "rosapi/GetTime"
            }
            ws.send(json.dumps(test_msg))
            response = ws.recv()
            ws.close()
            
            return json.loads(response).get('result') is not None
        except Exception:
            return False
            
    def test_backend_api(self):
        """Test backend API health endpoint"""
        try:
            response = requests.get('http://localhost:8000/api/health', timeout=5)
            return response.status_code == 200
        except Exception:
            return False
            
    def test_costmap_service(self):
        """Test costmap generation service"""
        try:
            # Check if service exists
            import subprocess
            result = subprocess.run(
                ['ros2', 'service', 'list'],
                capture_output=True,
                text=True,
                timeout=5
            )
            return '/costmap/generate_costmap' in result.stdout
        except Exception:
            return False
            
    def test_path_planning_service(self):
        """Test path planning service"""
        try:
            import subprocess
            result = subprocess.run(
                ['ros2', 'service', 'list'],
                capture_output=True,
                text=True,
                timeout=5
            )
            return '/path_planning/plan_path' in result.stdout
        except Exception:
            return False
            
    def test_web_ros_integration(self):
        """Test web interface to ROS integration"""
        try:
            # This would test actual web-to-ROS communication
            # For now, just check if both systems are responsive
            backend_ok = self.test_backend_api()
            rosbridge_ok = self.test_rosbridge_connectivity()
            return backend_ok and rosbridge_ok
        except Exception:
            return False
            
    def publish_results(self):
        """Publish test results"""
        results_msg = String()
        results_msg.data = json.dumps(self.test_results, indent=2)
        self.results_publisher.publish(results_msg)

def main(args=None):
    rclpy.init(args=args)
    
    test_runner = IntegrationTestRunner()
    
    try:
        rclpy.spin(test_runner)
    except KeyboardInterrupt:
        pass
    finally:
        test_runner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
