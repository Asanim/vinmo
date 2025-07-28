#!/usr/bin/env python3
"""
System Status Publisher

Publishes system status information to ROS topics for monitoring
and web interface display.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import psutil
import json
import subprocess
import os

class SystemStatusPublisher(Node):
    def __init__(self):
        super().__init__('system_status_publisher')
        
        # Publishers
        self.status_publisher = self.create_publisher(String, '/system/status', 10)
        self.diagnostics_publisher = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        
        # Parameters
        self.declare_parameter('publish_interval', 5.0)
        self.publish_interval = self.get_parameter('publish_interval').value
        
        # Create timer
        self.timer = self.create_timer(self.publish_interval, self.publish_status)
        
        self.get_logger().info('System Status Publisher started')
        
    def publish_status(self):
        """Publish system status"""
        
        # Gather system information
        status_data = {
            'timestamp': self.get_clock().now().nanoseconds,
            'system': self.get_system_info(),
            'ros': self.get_ros_info(),
            'services': self.get_service_status(),
            'resources': self.get_resource_usage()
        }
        
        # Publish as JSON string
        status_msg = String()
        status_msg.data = json.dumps(status_data, default=str)
        self.status_publisher.publish(status_msg)
        
        # Publish as diagnostics
        self.publish_diagnostics(status_data)
        
    def get_system_info(self):
        """Get basic system information"""
        return {
            'hostname': os.uname().nodename,
            'platform': os.uname().system,
            'architecture': os.uname().machine,
            'cpu_count': psutil.cpu_count(),
            'boot_time': psutil.boot_time()
        }
        
    def get_ros_info(self):
        """Get ROS-specific information"""
        try:
            # Get ROS domain ID
            ros_domain = os.environ.get('ROS_DOMAIN_ID', '0')
            
            # Check if rosbridge is running
            rosbridge_running = self.check_process('rosbridge')
            
            return {
                'domain_id': ros_domain,
                'rosbridge_running': rosbridge_running,
                'distro': os.environ.get('ROS_DISTRO', 'unknown')
            }
        except Exception as e:
            self.get_logger().warn(f'Error getting ROS info: {e}')
            return {'error': str(e)}
            
    def get_service_status(self):
        """Check status of key services"""
        services = {
            'costmap_service': self.check_ros_service('/costmap/generate_costmap'),
            'path_planning_service': self.check_ros_service('/path_planning/plan_path'),
            'rosbridge_websocket': self.check_process('rosbridge_websocket'),
        }
        return services
        
    def get_resource_usage(self):
        """Get system resource usage"""
        return {
            'cpu_percent': psutil.cpu_percent(interval=1),
            'memory': {
                'total': psutil.virtual_memory().total,
                'available': psutil.virtual_memory().available,
                'percent': psutil.virtual_memory().percent
            },
            'disk': {
                'total': psutil.disk_usage('/').total,
                'free': psutil.disk_usage('/').free,
                'percent': psutil.disk_usage('/').percent
            }
        }
        
    def check_process(self, process_name):
        """Check if a process is running"""
        try:
            for proc in psutil.process_iter(['name', 'cmdline']):
                if process_name in proc.info['name'] or \
                   any(process_name in arg for arg in proc.info['cmdline'] or []):
                    return True
            return False
        except Exception:
            return False
            
    def check_ros_service(self, service_name):
        """Check if a ROS service is available"""
        try:
            result = subprocess.run(
                ['ros2', 'service', 'list'],
                capture_output=True,
                text=True,
                timeout=5
            )
            return service_name in result.stdout
        except Exception:
            return False
            
    def publish_diagnostics(self, status_data):
        """Publish diagnostics information"""
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()
        
        # System diagnostics
        sys_diag = DiagnosticStatus()
        sys_diag.name = 'System'
        sys_diag.hardware_id = status_data['system']['hostname']
        
        cpu_percent = status_data['resources']['cpu_percent']
        memory_percent = status_data['resources']['memory']['percent']
        
        if cpu_percent > 90 or memory_percent > 90:
            sys_diag.level = DiagnosticStatus.ERROR
            sys_diag.message = 'High resource usage'
        elif cpu_percent > 70 or memory_percent > 70:
            sys_diag.level = DiagnosticStatus.WARN
            sys_diag.message = 'Moderate resource usage'
        else:
            sys_diag.level = DiagnosticStatus.OK
            sys_diag.message = 'System running normally'
            
        sys_diag.values = [
            KeyValue(key='CPU Usage (%)', value=str(cpu_percent)),
            KeyValue(key='Memory Usage (%)', value=str(memory_percent)),
            KeyValue(key='CPU Count', value=str(status_data['system']['cpu_count']))
        ]
        
        diag_array.status.append(sys_diag)
        
        # ROS diagnostics
        ros_diag = DiagnosticStatus()
        ros_diag.name = 'ROS System'
        ros_diag.hardware_id = status_data['system']['hostname']
        
        services_ok = sum(status_data['services'].values())
        total_services = len(status_data['services'])
        
        if services_ok == total_services:
            ros_diag.level = DiagnosticStatus.OK
            ros_diag.message = 'All services running'
        elif services_ok > 0:
            ros_diag.level = DiagnosticStatus.WARN
            ros_diag.message = f'{services_ok}/{total_services} services running'
        else:
            ros_diag.level = DiagnosticStatus.ERROR
            ros_diag.message = 'No services running'
            
        ros_diag.values = [
            KeyValue(key='Domain ID', value=status_data['ros']['domain_id']),
            KeyValue(key='ROS Distro', value=status_data['ros']['distro']),
            KeyValue(key='Services Running', value=f'{services_ok}/{total_services}')
        ]
        
        diag_array.status.append(ros_diag)
        
        self.diagnostics_publisher.publish(diag_array)

def main(args=None):
    rclpy.init(args=args)
    
    publisher = SystemStatusPublisher()
    
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
