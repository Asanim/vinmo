#!/usr/bin/env python3
"""
Sensor Health Monitor for Vineyard Mower Robot

This node monitors the health and performance of all perception sensors,
providing diagnostics and early warning for sensor failures.

Author: Vineyard Robotics Team
License: MIT
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import numpy as np
import json
from collections import deque
import time

# ROS2 message types
from sensor_msgs.msg import LaserScan, Image, CameraInfo
from std_msgs.msg import String, Float32, Bool
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


class SensorHealthMonitor(Node):
    """
    Comprehensive sensor health monitoring system that tracks sensor performance,
    detects anomalies, and provides predictive maintenance alerts.
    """
    
    def __init__(self):
        super().__init__('sensor_health_monitor')
        
        # Initialize parameters
        self.declare_parameters()
        self.load_parameters()
        
        # Sensor tracking data structures
        self.sensor_data = {
            'lidar': {
                'last_update': 0.0,
                'message_count': 0,
                'data_quality_history': deque(maxlen=100),
                'frequency_history': deque(maxlen=50),
                'error_count': 0,
                'status': 'OK'
            },
            'front_camera_rgb': {
                'last_update': 0.0,
                'message_count': 0,
                'data_quality_history': deque(maxlen=100),
                'frequency_history': deque(maxlen=50),
                'error_count': 0,
                'status': 'OK'
            },
            'front_camera_depth': {
                'last_update': 0.0,
                'message_count': 0,
                'data_quality_history': deque(maxlen=100),
                'frequency_history': deque(maxlen=50),
                'error_count': 0,
                'status': 'OK'
            },
            'rear_camera_rgb': {
                'last_update': 0.0,
                'message_count': 0,
                'data_quality_history': deque(maxlen=100),
                'frequency_history': deque(maxlen=50),
                'error_count': 0,
                'status': 'OK'
            },
            'rear_camera_depth': {
                'last_update': 0.0,
                'message_count': 0,
                'data_quality_history': deque(maxlen=100),
                'frequency_history': deque(maxlen=50),
                'error_count': 0,
                'status': 'OK'
            }
        }
        
        # Overall system health
        self.system_health = {
            'overall_status': 'OK',
            'active_sensors': 0,
            'failed_sensors': 0,
            'degraded_sensors': 0,
            'last_full_check': 0.0
        }
        
        # Initialize QoS profiles
        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5
        )
        
        self.reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Setup subscribers and publishers
        self.setup_subscribers()
        self.setup_publishers()
        
        # Setup timers
        self.health_check_timer = self.create_timer(1.0, self.health_check_callback)
        self.diagnostics_timer = self.create_timer(2.0, self.publish_diagnostics)
        
        self.get_logger().info("Sensor Health Monitor initialized")
    
    def declare_parameters(self):
        """Declare node parameters with default values"""
        self.declare_parameter('sensor_timeout', 2.0)  # seconds
        self.declare_parameter('quality_threshold', 0.7)  # minimum acceptable quality
        self.declare_parameter('frequency_tolerance', 0.2)  # 20% tolerance for expected frequency
        self.declare_parameter('min_lidar_points', 100)  # minimum valid LiDAR points
        self.declare_parameter('min_depth_valid_pixels', 0.5)  # minimum fraction of valid depth pixels
        self.declare_parameter('max_error_rate', 0.1)  # maximum acceptable error rate
        self.declare_parameter('degraded_threshold', 0.5)  # threshold for degraded status
        self.declare_parameter('expected_lidar_freq', 10.0)  # Hz
        self.declare_parameter('expected_camera_freq', 30.0)  # Hz
        self.declare_parameter('enable_predictive_alerts', True)
        self.declare_parameter('alert_cooldown', 30.0)  # seconds between repeated alerts
    
    def load_parameters(self):
        """Load parameters from ROS parameter server"""
        self.sensor_timeout = self.get_parameter('sensor_timeout').get_parameter_value().double_value
        self.quality_threshold = self.get_parameter('quality_threshold').get_parameter_value().double_value
        self.frequency_tolerance = self.get_parameter('frequency_tolerance').get_parameter_value().double_value
        self.min_lidar_points = self.get_parameter('min_lidar_points').get_parameter_value().integer_value
        self.min_depth_valid_pixels = self.get_parameter('min_depth_valid_pixels').get_parameter_value().double_value
        self.max_error_rate = self.get_parameter('max_error_rate').get_parameter_value().double_value
        self.degraded_threshold = self.get_parameter('degraded_threshold').get_parameter_value().double_value
        self.expected_lidar_freq = self.get_parameter('expected_lidar_freq').get_parameter_value().double_value
        self.expected_camera_freq = self.get_parameter('expected_camera_freq').get_parameter_value().double_value
        self.enable_predictive_alerts = self.get_parameter('enable_predictive_alerts').get_parameter_value().bool_value
        self.alert_cooldown = self.get_parameter('alert_cooldown').get_parameter_value().double_value
        
        # Alert tracking
        self.last_alerts = {}
    
    def setup_subscribers(self):
        """Setup sensor data subscribers for monitoring"""
        # LiDAR subscriber
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            self.sensor_qos
        )
        
        # Front camera subscribers
        self.front_rgb_sub = self.create_subscription(
            Image,
            '/front_camera/image',
            lambda msg: self.camera_rgb_callback(msg, 'front_camera_rgb'),
            self.sensor_qos
        )
        
        self.front_depth_sub = self.create_subscription(
            Image,
            '/front_camera/depth_image',
            lambda msg: self.camera_depth_callback(msg, 'front_camera_depth'),
            self.sensor_qos
        )
        
        # Rear camera subscribers
        self.rear_rgb_sub = self.create_subscription(
            Image,
            '/rear_camera/image',
            lambda msg: self.camera_rgb_callback(msg, 'rear_camera_rgb'),
            self.sensor_qos
        )
        
        self.rear_depth_sub = self.create_subscription(
            Image,
            '/rear_camera/depth_image',
            lambda msg: self.camera_depth_callback(msg, 'rear_camera_depth'),
            self.sensor_qos
        )
    
    def setup_publishers(self):
        """Setup health monitoring publishers"""
        # Sensor health status
        self.health_status_pub = self.create_publisher(
            String,
            '/sensors/sensor_health',
            self.reliable_qos
        )
        
        # System health summary
        self.system_health_pub = self.create_publisher(
            String,
            '/sensors/system_health',
            self.reliable_qos
        )
        
        # Individual sensor confidence scores
        self.confidence_pub = self.create_publisher(
            Float32,
            '/sensors/perception_confidence',
            self.reliable_qos
        )
        
        # Sensor failure alerts
        self.alert_pub = self.create_publisher(
            Bool,
            '/sensors/sensor_failure_alert',
            self.reliable_qos
        )
        
        # ROS diagnostics
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray,
            '/diagnostics',
            self.reliable_qos
        )
    
    def lidar_callback(self, msg):
        """Monitor LiDAR sensor health"""
        current_time = time.time()
        sensor_name = 'lidar'
        
        try:
            # Update timing information
            self.update_sensor_timing(sensor_name, current_time)
            
            # Assess data quality
            valid_points = sum(1 for r in msg.ranges 
                             if msg.range_min <= r <= msg.range_max and not np.isnan(r))
            total_points = len(msg.ranges)
            
            if total_points > 0:
                quality_score = valid_points / total_points
                # Additional quality metrics
                if valid_points < self.min_lidar_points:
                    quality_score *= 0.5  # Penalize low point count
                
                self.sensor_data[sensor_name]['data_quality_history'].append(quality_score)
            else:
                self.sensor_data[sensor_name]['error_count'] += 1
                self.sensor_data[sensor_name]['data_quality_history'].append(0.0)
            
        except Exception as e:
            self.get_logger().warn(f"LiDAR health monitoring error: {e}")
            self.sensor_data[sensor_name]['error_count'] += 1
    
    def camera_rgb_callback(self, msg, sensor_name):
        """Monitor RGB camera sensor health"""
        current_time = time.time()
        
        try:
            # Update timing information
            self.update_sensor_timing(sensor_name, current_time)
            
            # Basic image quality assessment
            # Note: Full image processing would be expensive, so we use header info
            quality_score = 1.0  # Base score
            
            # Check image dimensions
            if msg.width * msg.height < 100000:  # Very small image
                quality_score *= 0.7
            
            # Check encoding
            if msg.encoding not in ['rgb8', 'bgr8', 'mono8']:
                quality_score *= 0.8
            
            self.sensor_data[sensor_name]['data_quality_history'].append(quality_score)
            
        except Exception as e:
            self.get_logger().warn(f"RGB camera health monitoring error for {sensor_name}: {e}")
            self.sensor_data[sensor_name]['error_count'] += 1
    
    def camera_depth_callback(self, msg, sensor_name):
        """Monitor depth camera sensor health"""
        current_time = time.time()
        
        try:
            # Update timing information
            self.update_sensor_timing(sensor_name, current_time)
            
            # Assess depth data quality (simplified)
            quality_score = 1.0
            
            # Check image size
            if msg.width * msg.height < 50000:  # Small depth image
                quality_score *= 0.8
            
            # Check encoding
            if msg.encoding not in ['16UC1', '32FC1']:
                quality_score *= 0.7
            
            # Note: Full depth analysis would require converting the image
            # For now, we use a simplified quality assessment
            
            self.sensor_data[sensor_name]['data_quality_history'].append(quality_score)
            
        except Exception as e:
            self.get_logger().warn(f"Depth camera health monitoring error for {sensor_name}: {e}")
            self.sensor_data[sensor_name]['error_count'] += 1
    
    def update_sensor_timing(self, sensor_name, current_time):
        """Update sensor timing and frequency statistics"""
        sensor_info = self.sensor_data[sensor_name]
        
        # Calculate frequency if we have a previous update
        if sensor_info['last_update'] > 0:
            time_diff = current_time - sensor_info['last_update']
            if time_diff > 0:
                frequency = 1.0 / time_diff
                sensor_info['frequency_history'].append(frequency)
        
        sensor_info['last_update'] = current_time
        sensor_info['message_count'] += 1
    
    def health_check_callback(self):
        """Perform comprehensive sensor health check"""
        current_time = time.time()
        
        active_sensors = 0
        failed_sensors = 0
        degraded_sensors = 0
        
        for sensor_name, sensor_info in self.sensor_data.items():
            # Check if sensor is active (receiving data)
            time_since_update = current_time - sensor_info['last_update']
            
            if time_since_update > self.sensor_timeout:
                sensor_info['status'] = 'FAILED'
                failed_sensors += 1
                self.log_sensor_issue(sensor_name, 'TIMEOUT', f"No data for {time_since_update:.1f}s")
            else:
                active_sensors += 1
                
                # Assess sensor quality
                quality_score = self.calculate_sensor_quality(sensor_name)
                
                if quality_score < self.degraded_threshold:
                    sensor_info['status'] = 'DEGRADED'
                    degraded_sensors += 1
                    self.log_sensor_issue(sensor_name, 'DEGRADED', f"Quality: {quality_score:.2f}")
                elif quality_score < self.quality_threshold:
                    sensor_info['status'] = 'WARNING'
                    self.log_sensor_issue(sensor_name, 'WARNING', f"Quality: {quality_score:.2f}")
                else:
                    sensor_info['status'] = 'OK'
        
        # Update system health
        self.system_health.update({
            'active_sensors': active_sensors,
            'failed_sensors': failed_sensors,
            'degraded_sensors': degraded_sensors,
            'last_full_check': current_time
        })
        
        # Determine overall system status
        if failed_sensors > 0:
            self.system_health['overall_status'] = 'CRITICAL'
        elif degraded_sensors > active_sensors // 2:  # More than half degraded
            self.system_health['overall_status'] = 'DEGRADED'
        elif degraded_sensors > 0:
            self.system_health['overall_status'] = 'WARNING'
        else:
            self.system_health['overall_status'] = 'OK'
        
        # Publish health status
        self.publish_health_status()
        
        # Check for system-wide alerts
        if self.system_health['overall_status'] in ['CRITICAL', 'DEGRADED']:
            self.publish_failure_alert(True)
        else:
            self.publish_failure_alert(False)
    
    def calculate_sensor_quality(self, sensor_name):
        """Calculate overall quality score for a sensor"""
        sensor_info = self.sensor_data[sensor_name]
        
        # Data quality score (average of recent quality measurements)
        if sensor_info['data_quality_history']:
            data_quality = np.mean(list(sensor_info['data_quality_history']))
        else:
            data_quality = 0.0
        
        # Frequency stability score
        frequency_score = self.calculate_frequency_score(sensor_name)
        
        # Error rate score
        total_messages = sensor_info['message_count']
        if total_messages > 0:
            error_rate = sensor_info['error_count'] / total_messages
            error_score = max(0.0, 1.0 - error_rate / self.max_error_rate)
        else:
            error_score = 0.0
        
        # Weighted combination
        overall_quality = (data_quality * 0.5 + frequency_score * 0.3 + error_score * 0.2)
        
        return overall_quality
    
    def calculate_frequency_score(self, sensor_name):
        """Calculate frequency stability score"""
        sensor_info = self.sensor_data[sensor_name]
        
        if not sensor_info['frequency_history']:
            return 0.0
        
        # Determine expected frequency
        expected_freq = self.expected_camera_freq
        if sensor_name == 'lidar':
            expected_freq = self.expected_lidar_freq
        
        # Calculate frequency statistics
        frequencies = list(sensor_info['frequency_history'])
        mean_freq = np.mean(frequencies)
        freq_std = np.std(frequencies)
        
        # Score based on proximity to expected frequency and stability
        freq_error = abs(mean_freq - expected_freq) / expected_freq
        if freq_error < self.frequency_tolerance:
            freq_accuracy_score = 1.0 - freq_error / self.frequency_tolerance
        else:
            freq_accuracy_score = 0.0
        
        # Stability score (lower standard deviation is better)
        max_acceptable_std = expected_freq * 0.1  # 10% of expected frequency
        stability_score = max(0.0, 1.0 - freq_std / max_acceptable_std)
        
        return (freq_accuracy_score * 0.7 + stability_score * 0.3)
    
    def log_sensor_issue(self, sensor_name, issue_type, details):
        """Log sensor issues and handle alerting"""
        message = f"Sensor {sensor_name} - {issue_type}: {details}"
        
        if issue_type in ['FAILED', 'CRITICAL']:
            self.get_logger().error(message)
        elif issue_type in ['DEGRADED', 'WARNING']:
            self.get_logger().warn(message)
        else:
            self.get_logger().info(message)
        
        # Handle alert cooldown
        alert_key = f"{sensor_name}_{issue_type}"
        current_time = time.time()
        
        if (alert_key not in self.last_alerts or 
            current_time - self.last_alerts[alert_key] > self.alert_cooldown):
            
            self.last_alerts[alert_key] = current_time
            
            # Predictive maintenance alerts
            if self.enable_predictive_alerts and issue_type == 'WARNING':
                self.get_logger().info(f"Predictive maintenance alert: {message}")
    
    def publish_health_status(self):
        """Publish detailed sensor health status"""
        health_data = {
            'timestamp': time.time(),
            'system_health': self.system_health,
            'sensors': {}
        }
        
        for sensor_name, sensor_info in self.sensor_data.items():
            health_data['sensors'][sensor_name] = {
                'status': sensor_info['status'],
                'last_update': sensor_info['last_update'],
                'message_count': sensor_info['message_count'],
                'error_count': sensor_info['error_count'],
                'quality_score': self.calculate_sensor_quality(sensor_name),
                'time_since_update': time.time() - sensor_info['last_update']
            }
        
        # Publish as JSON string
        health_msg = String()
        health_msg.data = json.dumps(health_data, indent=2)
        self.health_status_pub.publish(health_msg)
        
        # Publish system health summary
        system_msg = String()
        system_msg.data = json.dumps(self.system_health, indent=2)
        self.system_health_pub.publish(system_msg)
        
        # Publish overall confidence score
        active_qualities = []
        for sensor_name, sensor_info in self.sensor_data.items():
            if sensor_info['status'] not in ['FAILED']:
                quality = self.calculate_sensor_quality(sensor_name)
                active_qualities.append(quality)
        
        if active_qualities:
            overall_confidence = np.mean(active_qualities)
        else:
            overall_confidence = 0.0
        
        confidence_msg = Float32()
        confidence_msg.data = overall_confidence
        self.confidence_pub.publish(confidence_msg)
    
    def publish_failure_alert(self, alert_active):
        """Publish sensor failure alert"""
        alert_msg = Bool()
        alert_msg.data = alert_active
        self.alert_pub.publish(alert_msg)
    
    def publish_diagnostics(self):
        """Publish ROS diagnostics messages"""
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()
        
        # Overall system diagnostic
        system_diag = DiagnosticStatus()
        system_diag.name = "vineyard_mower_perception_system"
        system_diag.hardware_id = "perception_sensors"
        
        if self.system_health['overall_status'] == 'OK':
            system_diag.level = DiagnosticStatus.OK
            system_diag.message = "All sensors operating normally"
        elif self.system_health['overall_status'] == 'WARNING':
            system_diag.level = DiagnosticStatus.WARN
            system_diag.message = f"Some sensors degraded ({self.system_health['degraded_sensors']} degraded)"
        else:
            system_diag.level = DiagnosticStatus.ERROR
            system_diag.message = f"Sensor failures detected ({self.system_health['failed_sensors']} failed)"
        
        # Add system-level key-value pairs
        system_diag.values.append(KeyValue(key="active_sensors", value=str(self.system_health['active_sensors'])))
        system_diag.values.append(KeyValue(key="failed_sensors", value=str(self.system_health['failed_sensors'])))
        system_diag.values.append(KeyValue(key="degraded_sensors", value=str(self.system_health['degraded_sensors'])))
        
        diag_array.status.append(system_diag)
        
        # Individual sensor diagnostics
        for sensor_name, sensor_info in self.sensor_data.items():
            sensor_diag = DiagnosticStatus()
            sensor_diag.name = f"vineyard_mower_{sensor_name}"
            sensor_diag.hardware_id = sensor_name
            
            if sensor_info['status'] == 'OK':
                sensor_diag.level = DiagnosticStatus.OK
                sensor_diag.message = "Sensor operating normally"
            elif sensor_info['status'] == 'WARNING':
                sensor_diag.level = DiagnosticStatus.WARN
                sensor_diag.message = "Sensor performance degraded"
            elif sensor_info['status'] == 'DEGRADED':
                sensor_diag.level = DiagnosticStatus.WARN
                sensor_diag.message = "Sensor significantly degraded"
            else:
                sensor_diag.level = DiagnosticStatus.ERROR
                sensor_diag.message = "Sensor failed or timed out"
            
            # Add sensor-specific key-value pairs
            sensor_diag.values.append(KeyValue(key="status", value=sensor_info['status']))
            sensor_diag.values.append(KeyValue(key="quality_score", value=f"{self.calculate_sensor_quality(sensor_name):.3f}"))
            sensor_diag.values.append(KeyValue(key="message_count", value=str(sensor_info['message_count'])))
            sensor_diag.values.append(KeyValue(key="error_count", value=str(sensor_info['error_count'])))
            sensor_diag.values.append(KeyValue(key="time_since_update", value=f"{time.time() - sensor_info['last_update']:.1f}s"))
            
            if sensor_info['frequency_history']:
                mean_freq = np.mean(list(sensor_info['frequency_history']))
                sensor_diag.values.append(KeyValue(key="frequency", value=f"{mean_freq:.1f} Hz"))
            
            diag_array.status.append(sensor_diag)
        
        self.diagnostics_pub.publish(diag_array)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        health_monitor = SensorHealthMonitor()
        rclpy.spin(health_monitor)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in sensor health monitor: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
