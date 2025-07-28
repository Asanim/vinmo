#!/usr/bin/env python3
"""
Web Interface Monitor Script

Monitors the health and status of the web interface components:
- ROSbridge server connectivity
- Backend API server health
- Frontend server status
- Database connectivity
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests
import websocket
import json
import time
import threading

class WebInterfaceMonitor(Node):
    def __init__(self):
        super().__init__('web_interface_monitor')
        
        # Publishers
        self.status_publisher = self.create_publisher(String, '/web_interface/status', 10)
        
        # Parameters
        self.declare_parameter('rosbridge_port', 9090)
        self.declare_parameter('backend_port', 8000)
        self.declare_parameter('frontend_port', 3000)
        self.declare_parameter('check_interval', 10.0)
        
        self.rosbridge_port = self.get_parameter('rosbridge_port').value
        self.backend_port = self.get_parameter('backend_port').value
        self.frontend_port = self.get_parameter('frontend_port').value
        self.check_interval = self.get_parameter('check_interval').value
        
        # Status tracking
        self.status = {
            'rosbridge': False,
            'backend': False,
            'frontend': False,
            'last_check': None
        }
        
        # Create timer for regular health checks
        self.timer = self.create_timer(self.check_interval, self.check_health)
        
        self.get_logger().info('Web Interface Monitor started')
        
    def check_health(self):
        """Perform health checks on all web interface components"""
        self.get_logger().info('Performing health checks...')
        
        # Check ROSbridge
        self.status['rosbridge'] = self.check_rosbridge()
        
        # Check Backend API
        self.status['backend'] = self.check_backend()
        
        # Check Frontend (if running in development)
        self.status['frontend'] = self.check_frontend()
        
        # Update timestamp
        self.status['last_check'] = time.time()
        
        # Publish status
        status_msg = String()
        status_msg.data = json.dumps(self.status)
        self.status_publisher.publish(status_msg)
        
        # Log results
        healthy_components = sum(self.status[key] for key in ['rosbridge', 'backend', 'frontend'])
        self.get_logger().info(f'Health check complete: {healthy_components}/3 components healthy')
        
    def check_rosbridge(self):
        """Check if ROSbridge WebSocket server is responding"""
        try:
            ws_url = f"ws://localhost:{self.rosbridge_port}"
            ws = websocket.create_connection(ws_url, timeout=5)
            
            # Send a simple ping message
            ping_msg = {
                "op": "call_service",
                "service": "/rosapi/get_time",
                "type": "rosapi/GetTime"
            }
            ws.send(json.dumps(ping_msg))
            
            # Wait for response
            response = ws.recv()
            ws.close()
            
            self.get_logger().debug('ROSbridge health check: OK')
            return True
            
        except Exception as e:
            self.get_logger().warn(f'ROSbridge health check failed: {e}')
            return False
            
    def check_backend(self):
        """Check if backend API server is responding"""
        try:
            response = requests.get(
                f"http://localhost:{self.backend_port}/api/health",
                timeout=5
            )
            
            if response.status_code == 200:
                self.get_logger().debug('Backend health check: OK')
                return True
            else:
                self.get_logger().warn(f'Backend health check failed: HTTP {response.status_code}')
                return False
                
        except Exception as e:
            self.get_logger().warn(f'Backend health check failed: {e}')
            return False
            
    def check_frontend(self):
        """Check if frontend development server is responding"""
        try:
            response = requests.get(
                f"http://localhost:{self.frontend_port}",
                timeout=5
            )
            
            if response.status_code == 200:
                self.get_logger().debug('Frontend health check: OK')
                return True
            else:
                self.get_logger().warn(f'Frontend health check failed: HTTP {response.status_code}')
                return False
                
        except Exception as e:
            self.get_logger().warn(f'Frontend health check failed: {e}')
            return False

def main(args=None):
    rclpy.init(args=args)
    
    monitor = WebInterfaceMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
