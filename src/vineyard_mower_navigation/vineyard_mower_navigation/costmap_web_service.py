#!/usr/bin/env python3
"""
Costmap Web Service Node
Implements the ROS services required for the web interface integration
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# Service imports
from vineyard_mower_interfaces.srv import GenerateCostmap, UpdateCostmapLayer, GetCostmapInfo

# Message imports
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header, String
from geometry_msgs.msg import Pose

import numpy as np
import json
from datetime import datetime
import threading
import time
import os
import yaml


class CostmapWebService(Node):
    """
    ROS2 service node that provides web interface integration for costmap generation
    """
    
    def __init__(self):
        super().__init__('costmap_web_service')
        
        # Use reentrant callback group for concurrent service calls
        self.callback_group = ReentrantCallbackGroup()
        
        # Current costmap state
        self.current_costmap = None
        self.costmap_info = {
            'has_costmap': False,
            'width': 0,
            'height': 0,
            'resolution': 0.1,
            'origin_x': 0.0,
            'origin_y': 0.0,
            'frame_id': 'map',
            'rows_detected': 0,
            'obstacles_detected': 0,
            'row_spacing': 0.0,
            'auto_update_enabled': False
        }
        
        # Processing state
        self.processing_jobs = {}
        self.job_counter = 0
        self.processing_lock = threading.Lock()
        
        # Parameters
        self.declare_parameters()
        
        # Services
        self.create_services()
        
        # Publishers
        self.create_publishers()
        
        # Subscribers (if needed)
        self.create_subscribers()
        
        self.get_logger().info('Costmap Web Service started')
        self.get_logger().info('Available services:')
        self.get_logger().info('  - /costmap/generate_costmap')
        self.get_logger().info('  - /costmap/update_costmap_layer') 
        self.get_logger().info('  - /costmap/get_costmap_info')
    
    def declare_parameters(self):
        """Declare node parameters"""
        self.declare_parameter('default_resolution', 0.1)
        self.declare_parameter('default_width', 1000)
        self.declare_parameter('default_height', 1000)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('costmap_topic', '/costmap/updates')
        self.declare_parameter('job_updates_topic', '/costmap/job_updates')
        
    def create_services(self):
        """Create ROS services"""
        self.generate_costmap_service = self.create_service(
            GenerateCostmap,
            '/costmap/generate_costmap',
            self.generate_costmap_callback,
            callback_group=self.callback_group
        )
        
        self.update_layer_service = self.create_service(
            UpdateCostmapLayer,
            '/costmap/update_costmap_layer',
            self.update_costmap_layer_callback,
            callback_group=self.callback_group
        )
        
        self.get_info_service = self.create_service(
            GetCostmapInfo,
            '/costmap/get_costmap_info',
            self.get_costmap_info_callback,
            callback_group=self.callback_group
        )
    
    def create_publishers(self):
        """Create ROS publishers"""
        costmap_topic = self.get_parameter('costmap_topic').get_parameter_value().string_value
        job_topic = self.get_parameter('job_updates_topic').get_parameter_value().string_value
        
        self.costmap_publisher = self.create_publisher(
            OccupancyGrid,
            costmap_topic,
            10
        )
        
        self.job_updates_publisher = self.create_publisher(
            String,
            job_topic,
            10
        )
    
    def create_subscribers(self):
        """Create ROS subscribers if needed"""
        # Add any necessary subscribers here
        pass
    
    def generate_costmap_callback(self, request, response):
        """
        Generate costmap service callback
        """
        self.get_logger().info(f'Generating costmap: lat={request.center_latitude}, '
                             f'lon={request.center_longitude}, zoom={request.zoom_level}')
        
        try:
            # Create a new processing job
            with self.processing_lock:
                self.job_counter += 1
                job_id = f'costmap_gen_{self.job_counter}'
                
                job_data = {
                    'id': job_id,
                    'type': 'costmap_generation',
                    'status': 'running',
                    'progress': 0,
                    'started_at': datetime.now().isoformat(),
                    'parameters': {
                        'center_latitude': request.center_latitude,
                        'center_longitude': request.center_longitude,
                        'zoom_level': request.zoom_level,
                        'use_local_image': request.use_local_image,
                        'image_path': request.image_path
                    }
                }
                self.processing_jobs[job_id] = job_data
            
            # Publish job start update
            self.publish_job_update(job_data)
            
            # Simulate costmap generation process
            costmap = self.simulate_costmap_generation(request, job_id)
            
            if costmap is not None:
                # Update job status
                with self.processing_lock:
                    self.processing_jobs[job_id]['status'] = 'completed'
                    self.processing_jobs[job_id]['progress'] = 100
                    self.processing_jobs[job_id]['completed_at'] = datetime.now().isoformat()
                
                # Store the generated costmap
                self.current_costmap = costmap
                self.update_costmap_info(costmap)
                
                # Publish the costmap
                self.publish_costmap(costmap)
                
                # Publish job completion update
                self.publish_job_update(self.processing_jobs[job_id])
                
                # Set response
                response.success = True
                response.message = f'Costmap generated successfully (job: {job_id})'
                response.costmap = costmap
                response.rows_detected = self.costmap_info['rows_detected']
                response.obstacles_detected = self.costmap_info['obstacles_detected']
                response.row_spacing = self.costmap_info['row_spacing']
                
            else:
                # Update job status
                with self.processing_lock:
                    self.processing_jobs[job_id]['status'] = 'failed'
                    self.processing_jobs[job_id]['error'] = 'Failed to generate costmap'
                
                self.publish_job_update(self.processing_jobs[job_id])
                
                response.success = False
                response.message = 'Failed to generate costmap'
                
        except Exception as e:
            self.get_logger().error(f'Error generating costmap: {str(e)}')
            response.success = False
            response.message = f'Error: {str(e)}'
        
        return response
    
    def update_costmap_layer_callback(self, request, response):
        """
        Update costmap layer service callback
        """
        self.get_logger().info(f'Updating costmap layer: {request.layer_name}')
        
        try:
            if self.current_costmap is None:
                response.success = False
                response.message = 'No costmap available to update'
                return response
            
            # Validate dimensions
            if len(request.layer_data) != request.width * request.height:
                response.success = False
                response.message = 'Layer data size does not match specified dimensions'
                return response
            
            # Update the costmap with new layer data
            # This is a simplified implementation - in practice you'd have more sophisticated layer merging
            layer_array = np.array(request.layer_data, dtype=np.int8).reshape(request.height, request.width)
            
            # Merge with existing costmap (simple overlay for demo)
            costmap_array = np.array(self.current_costmap.data, dtype=np.int8).reshape(
                self.current_costmap.info.height, self.current_costmap.info.width)
            
            # Simple merge strategy: take maximum value
            if (layer_array.shape == costmap_array.shape):
                merged_array = np.maximum(costmap_array, layer_array)
                self.current_costmap.data = merged_array.flatten().tolist()
                
                # Publish updated costmap
                self.publish_costmap(self.current_costmap)
                
                response.success = True
                response.message = f'Layer {request.layer_name} updated successfully'
            else:
                response.success = False
                response.message = 'Layer dimensions do not match costmap dimensions'
                
        except Exception as e:
            self.get_logger().error(f'Error updating costmap layer: {str(e)}')
            response.success = False
            response.message = f'Error: {str(e)}'
        
        return response
    
    def get_costmap_info_callback(self, request, response):
        """
        Get costmap info service callback
        """
        self.get_logger().info('Getting costmap info')
        
        try:
            # Return current costmap information
            response.has_costmap = self.costmap_info['has_costmap']
            response.width = self.costmap_info['width']
            response.height = self.costmap_info['height']
            response.resolution = self.costmap_info['resolution']
            response.origin_x = self.costmap_info['origin_x']
            response.origin_y = self.costmap_info['origin_y']
            response.frame_id = self.costmap_info['frame_id']
            response.rows_detected = self.costmap_info['rows_detected']
            response.obstacles_detected = self.costmap_info['obstacles_detected']
            response.row_spacing = self.costmap_info['row_spacing']
            response.auto_update_enabled = self.costmap_info['auto_update_enabled']
            
        except Exception as e:
            self.get_logger().error(f'Error getting costmap info: {str(e)}')
            # Set default values
            response.has_costmap = False
            response.width = 0
            response.height = 0
            response.resolution = 0.1
            response.origin_x = 0.0
            response.origin_y = 0.0
            response.frame_id = 'map'
            response.rows_detected = 0
            response.obstacles_detected = 0
            response.row_spacing = 0.0
            response.auto_update_enabled = False
        
        return response
    
    def simulate_costmap_generation(self, request, job_id):
        """
        Simulate costmap generation process
        This is a placeholder - replace with actual costmap generation logic
        """
        try:
            # Get parameters
            resolution = self.get_parameter('default_resolution').get_parameter_value().double_value
            width = self.get_parameter('default_width').get_parameter_value().integer_value
            height = self.get_parameter('default_height').get_parameter_value().integer_value
            frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
            
            # Simulate processing time with progress updates
            total_steps = 10
            for step in range(total_steps):
                time.sleep(0.5)  # Simulate processing time
                
                with self.processing_lock:
                    if job_id in self.processing_jobs:
                        self.processing_jobs[job_id]['progress'] = int((step + 1) * 100 / total_steps)
                        self.publish_job_update(self.processing_jobs[job_id])
            
            # Create a simple demo costmap
            costmap = OccupancyGrid()
            
            # Header
            costmap.header = Header()
            costmap.header.stamp = self.get_clock().now().to_msg()
            costmap.header.frame_id = frame_id
            
            # Map info
            costmap.info.resolution = resolution
            costmap.info.width = width
            costmap.info.height = height
            costmap.info.origin = Pose()
            costmap.info.origin.position.x = request.center_longitude - (width * resolution / 2)
            costmap.info.origin.position.y = request.center_latitude - (height * resolution / 2)
            costmap.info.origin.position.z = 0.0
            costmap.info.origin.orientation.w = 1.0
            
            # Generate demo data (simulate vineyard rows)
            data = np.zeros((height, width), dtype=np.int8)
            
            # Add some demo vineyard rows
            row_spacing = 30  # cells between rows
            row_width = 3    # width of each row
            num_rows = width // row_spacing
            
            for i in range(num_rows):
                row_center = i * row_spacing + row_spacing // 2
                if row_center + row_width < width:
                    # Create vine rows (occupied space)
                    data[:, row_center:row_center + row_width] = 80
                    
                    # Add some obstacles randomly
                    obstacle_positions = np.random.choice(height, size=height//20, replace=False)
                    for pos in obstacle_positions:
                        if row_center + row_width + 2 < width:
                            data[pos, row_center + row_width + 1] = 100
            
            costmap.data = data.flatten().tolist()
            
            self.get_logger().info(f'Generated demo costmap: {width}x{height} at {resolution}m/cell')
            
            return costmap
            
        except Exception as e:
            self.get_logger().error(f'Error in costmap generation simulation: {str(e)}')
            return None
    
    def update_costmap_info(self, costmap):
        """Update internal costmap information"""
        self.costmap_info.update({
            'has_costmap': True,
            'width': costmap.info.width,
            'height': costmap.info.height,
            'resolution': costmap.info.resolution,
            'origin_x': costmap.info.origin.position.x,
            'origin_y': costmap.info.origin.position.y,
            'frame_id': costmap.header.frame_id,
            'rows_detected': 5,  # Demo value
            'obstacles_detected': 12,  # Demo value  
            'row_spacing': 3.0,  # Demo value
            'auto_update_enabled': False
        })
    
    def publish_costmap(self, costmap):
        """Publish costmap to ROS topic"""
        try:
            self.costmap_publisher.publish(costmap)
            self.get_logger().info('Published costmap update')
        except Exception as e:
            self.get_logger().error(f'Error publishing costmap: {str(e)}')
    
    def publish_job_update(self, job_data):
        """Publish job update to ROS topic"""
        try:
            msg = String()
            msg.data = json.dumps(job_data)
            self.job_updates_publisher.publish(msg)
            self.get_logger().debug(f'Published job update: {job_data["id"]} - {job_data["status"]}')
        except Exception as e:
            self.get_logger().error(f'Error publishing job update: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = CostmapWebService()
        
        # Use MultiThreadedExecutor for concurrent service calls
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        
        node.get_logger().info('Costmap Web Service node started. Waiting for service calls...')
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            node.get_logger().info('Shutting down Costmap Web Service...')
        finally:
            executor.shutdown()
            node.destroy_node()
            
    except Exception as e:
        print(f'Error starting Costmap Web Service: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
