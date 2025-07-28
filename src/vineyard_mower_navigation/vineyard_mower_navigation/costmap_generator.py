#!/usr/bin/env python3
"""
Costmap Generation Module

This module converts vineyard detection results into ROS2 compatible costmaps
for navigation planning and obstacle avoidance.
"""

from __future__ import annotations
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import TransformException
import cv2
from typing import Dict, List, Tuple, Optional, Any
from dataclasses import dataclass
import logging
from .satellite_processor import VineyardDetectionResult
import yaml
import os
from datetime import datetime

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


@dataclass 
class CostmapConfig:
    """Configuration for costmap generation"""
    resolution: float = 0.1  # meters per pixel
    width: int = 1000  # cells
    height: int = 1000  # cells
    origin_x: float = 0.0  # meters
    origin_y: float = 0.0  # meters
    origin_z: float = 0.0  # meters
    frame_id: str = "map"
    
    # Cost values (0-100, where 100 is obstacle)
    free_space_cost: int = 0
    vine_row_cost: int = 90
    obstacle_cost: int = 100
    headland_cost: int = 30
    unknown_cost: int = 50
    
    # Inflation parameters
    inflation_radius: float = 1.0  # meters
    cost_scaling_factor: float = 10.0


class CostmapLayer:
    """Individual costmap layer"""
    
    def __init__(self, name: str, config: CostmapConfig):
        self.name = name
        self.config = config
        self.data = np.zeros((config.height, config.width), dtype=np.uint8)
        self.timestamp = datetime.now()
        
    def update_data(self, data: np.ndarray):
        """Update layer data"""
        self.data = data.astype(np.uint8)
        self.timestamp = datetime.now()
        
    def clear(self):
        """Clear layer data"""
        self.data.fill(0)
        self.timestamp = datetime.now()


class CostmapGenerator:
    """
    Generates ROS2 compatible costmaps from vineyard detection results
    """
    
    def __init__(self, config: CostmapConfig):
        self.config = config
        self.layers: Dict[str, CostmapLayer] = {}
        
        # Create default layers
        self._initialize_layers()
        
        logger.info(f"Initialized CostmapGenerator with resolution {config.resolution}m")
    
    def _initialize_layers(self):
        """Initialize costmap layers"""
        layer_names = ['static', 'vine_rows', 'obstacles', 'headlands', 'inflation']
        
        for name in layer_names:
            self.layers[name] = CostmapLayer(name, self.config)
    
    def generate_from_detection(self, 
                              detection_result: VineyardDetectionResult) -> OccupancyGrid:
        """
        Generate costmap from vineyard detection result
        
        Args:
            detection_result: Results from vineyard detection
            geo_bounds: Geographic bounds (lat_min, lat_max, lon_min, lon_max)
            
        Returns:
            ROS2 OccupancyGrid message
        """
        # Clear all layers
        for layer in self.layers.values():
            layer.clear()
        
        # Generate vine row costs
        self._generate_vine_row_costs(detection_result)
        
        # Generate obstacle costs
        self._generate_obstacle_costs(detection_result)
        
        # Generate headland costs
        self._generate_headland_costs(detection_result)
        
        # Apply inflation
        self._apply_inflation()
        
        # Combine layers
        combined_costmap = self._combine_layers()
        
        # Convert to OccupancyGrid message
        return self._to_occupancy_grid(combined_costmap)
    
    def _generate_vine_row_costs(self, detection_result: VineyardDetectionResult):
        """Generate costs for vine rows"""
        vine_layer = self.layers['vine_rows']
        
        for row_group in detection_result.vine_rows:
            for line in row_group:
                x1, y1, x2, y2 = line
                
                # Convert pixel coordinates to costmap coordinates
                x1_map, y1_map = self._pixel_to_costmap(x1, y1, detection_result)
                x2_map, y2_map = self._pixel_to_costmap(x2, y2, detection_result)
                
                # Draw line in costmap
                self._draw_line_in_costmap(vine_layer.data, 
                                         (x1_map, y1_map), (x2_map, y2_map),
                                         self.config.vine_row_cost)
    
    def _generate_obstacle_costs(self, detection_result: VineyardDetectionResult):
        """Generate costs for obstacles"""
        obstacle_layer = self.layers['obstacles']
        
        for obstacle in detection_result.obstacles:
            # Convert contour to costmap coordinates
            costmap_contour = []
            for point in obstacle:
                x, y = point[0]
                x_map, y_map = self._pixel_to_costmap(x, y, detection_result)
                costmap_contour.append([x_map, y_map])
            
            if costmap_contour:
                costmap_contour = np.array(costmap_contour, dtype=np.int32)
                
                # Fill contour in costmap
                cv2.fillPoly(obstacle_layer.data, [costmap_contour], 
                           self.config.obstacle_cost)
    
    def _generate_headland_costs(self, detection_result: VineyardDetectionResult):
        """Generate costs for headland areas"""
        headland_layer = self.layers['headlands']
        
        for headland in detection_result.headlands:
            # Convert headland polygon to costmap coordinates
            costmap_polygon = []
            for point in headland:
                x, y = point
                x_map, y_map = self._pixel_to_costmap(x, y, detection_result)
                costmap_polygon.append([x_map, y_map])
            
            if costmap_polygon:
                costmap_polygon = np.array(costmap_polygon, dtype=np.int32)
                
                # Fill polygon in costmap
                cv2.fillPoly(headland_layer.data, [costmap_polygon],
                           self.config.headland_cost)
    
    def _pixel_to_costmap(self, pixel_x: float, pixel_y: float, 
                         detection_result: VineyardDetectionResult) -> Tuple[int, int]:
        """
        Convert pixel coordinates to costmap coordinates
        
        Args:
            pixel_x, pixel_y: Pixel coordinates from detection
            detection_result: Detection result with bounds info
            
        Returns:
            Costmap cell coordinates
        """
        # Scale from detection image to world coordinates
        world_x = pixel_x * detection_result.pixel_to_meter_ratio
        world_y = pixel_y * detection_result.pixel_to_meter_ratio
        
        # Convert world coordinates to costmap cells
        costmap_x = int((world_x - self.config.origin_x) / self.config.resolution)
        costmap_y = int((world_y - self.config.origin_y) / self.config.resolution)
        
        # Clamp to costmap bounds
        costmap_x = max(0, min(costmap_x, self.config.width - 1))
        costmap_y = max(0, min(costmap_y, self.config.height - 1))
        
        return costmap_x, costmap_y
    
    def _draw_line_in_costmap(self, costmap: np.ndarray, 
                            start: Tuple[int, int], end: Tuple[int, int],
                            cost: int, thickness: int = 3):
        """Draw a line in the costmap with specified cost"""
        cv2.line(costmap, start, end, cost, thickness)
    
    def _apply_inflation(self):
        """Apply inflation around obstacles"""
        inflation_layer = self.layers['inflation']
        combined_obstacles = np.maximum(self.layers['vine_rows'].data,
                                      self.layers['obstacles'].data)
        
        # Create inflation kernel
        inflation_radius_cells = int(self.config.inflation_radius / self.config.resolution)
        
        # Distance transform for smooth cost gradient
        obstacle_mask = (combined_obstacles > 0).astype(np.uint8)
        distances = cv2.distanceTransform(1 - obstacle_mask, 
                                        cv2.DIST_L2, cv2.DIST_MASK_PRECISE)
        
        # Apply exponential decay for inflation costs
        inflation_costs = np.zeros_like(distances, dtype=np.uint8)
        mask = (distances > 0) & (distances <= inflation_radius_cells)
        
        inflation_costs[mask] = (
            self.config.cost_scaling_factor * 
            np.exp(-self.config.cost_scaling_factor * distances[mask] / inflation_radius_cells)
        ).astype(np.uint8)
        
        inflation_layer.update_data(inflation_costs)
    
    def _combine_layers(self) -> np.ndarray:
        """Combine all layers into final costmap"""
        combined = np.zeros((self.config.height, self.config.width), dtype=np.uint8)
        
        # Take maximum cost from all layers
        for layer_name in ['vine_rows', 'obstacles', 'headlands', 'inflation']:
            combined = np.maximum(combined, self.layers[layer_name].data)
        
        return combined
    
    def _to_occupancy_grid(self, costmap: np.ndarray) -> OccupancyGrid:
        """Convert costmap to ROS2 OccupancyGrid message"""
        grid = OccupancyGrid()
        
        # Header
        grid.header = Header()
        grid.header.stamp = rclpy.clock.Clock().now().to_msg()
        grid.header.frame_id = self.config.frame_id
        
        # Map metadata
        grid.info.resolution = self.config.resolution
        grid.info.width = self.config.width
        grid.info.height = self.config.height
        
        # Origin pose
        grid.info.origin.position.x = self.config.origin_x
        grid.info.origin.position.y = self.config.origin_y
        grid.info.origin.position.z = self.config.origin_z
        grid.info.origin.orientation.w = 1.0
        
        # Convert costmap to occupancy grid format
        # ROS occupancy grid: -1 = unknown, 0 = free, 100 = occupied
        occupancy_data = []
        
        for y in range(self.config.height):
            for x in range(self.config.width):
                cost = costmap[y, x]
                if cost == 0:
                    occupancy_data.append(0)  # Free
                elif cost >= self.config.obstacle_cost:
                    occupancy_data.append(100)  # Occupied
                else:
                    # Scale intermediate costs to 0-99 range
                    scaled_cost = int((cost / self.config.obstacle_cost) * 99)
                    occupancy_data.append(scaled_cost)
        
        grid.data = occupancy_data
        return grid
    
    def update_layer(self, layer_name: str, data: np.ndarray):
        """Update a specific costmap layer"""
        if layer_name in self.layers:
            self.layers[layer_name].update_data(data)
        else:
            logger.warning(f"Unknown layer: {layer_name}")
    
    def get_layer(self, layer_name: str) -> Optional[np.ndarray]:
        """Get data from a specific layer"""
        if layer_name in self.layers:
            return self.layers[layer_name].data.copy()
        return None
    
    def save_costmap(self, costmap: np.ndarray, filepath: str):
        """Save costmap as image file"""
        # Normalize for visualization
        normalized = (costmap / 100.0 * 255).astype(np.uint8)
        cv2.imwrite(filepath, normalized)
        logger.info(f"Costmap saved to {filepath}")
    
    def load_config_from_file(self, config_file: str) -> bool:
        """Load configuration from YAML file"""
        try:
            with open(config_file, 'r') as f:
                config_data = yaml.safe_load(f)
            
            # Update configuration
            for key, value in config_data.items():
                if hasattr(self.config, key):
                    setattr(self.config, key, value)
            
            logger.info(f"Configuration loaded from {config_file}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to load configuration: {e}")
            return False


class CostmapPublisher(Node):
    """
    ROS2 node for publishing costmaps
    """
    
    def __init__(self, config: Optional[CostmapConfig] = None):
        super().__init__('costmap_publisher')
        
        self.config = config or CostmapConfig()
        self.costmap_generator = CostmapGenerator(self.config)
        
        # Publishers
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.global_costmap_pub = self.create_publisher(
            OccupancyGrid, 'global_costmap', qos_profile)
        self.local_costmap_pub = self.create_publisher(
            OccupancyGrid, 'local_costmap', qos_profile)
        
        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Timer for periodic updates
        self.create_timer(1.0, self.update_costmaps)
        
        self.get_logger().info("CostmapPublisher initialized")
    
    def publish_global_costmap(self, detection_result: VineyardDetectionResult):
        """Publish global costmap from detection result"""
        costmap = self.costmap_generator.generate_from_detection(detection_result)
        self.global_costmap_pub.publish(costmap)
        self.get_logger().info("Published global costmap")
    
    def publish_local_costmap(self, sensor_data: Optional[Any] = None):
        """Publish local costmap with real-time sensor data"""
        # Create a smaller local costmap around robot position
        local_config = CostmapConfig(
            width=200,  # 20m x 20m at 0.1m resolution
            height=200,
            resolution=self.config.resolution
        )
        
        # Get robot position
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
            
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
            
            # Update local costmap origin
            local_config.origin_x = robot_x - 10.0  # Center robot in local map
            local_config.origin_y = robot_y - 10.0
            
        except TransformException as e:
            self.get_logger().warn(f"Could not get robot transform: {e}")
            return
        
        # Generate local costmap (would include sensor data in practice)
        local_generator = CostmapGenerator(local_config)
        
        # Create empty detection result for local map
        empty_result = VineyardDetectionResult(
            vine_rows=[], row_orientations=[], row_spacing=0.0,
            obstacles=[], headlands=[],
            image_bounds=(0, 200, 0, 200),
            pixel_to_meter_ratio=local_config.resolution
        )
        
        local_costmap = local_generator.generate_from_detection(empty_result)
        self.local_costmap_pub.publish(local_costmap)
    
    def update_costmaps(self):
        """Periodic costmap updates"""
        # This would be called by external detection results in practice
        pass


def create_sample_config() -> str:
    """Create a sample configuration file"""
    config = {
        'resolution': 0.1,
        'width': 1000,
        'height': 1000,
        'origin_x': 0.0,
        'origin_y': 0.0,
        'origin_z': 0.0,
        'frame_id': 'map',
        'free_space_cost': 0,
        'vine_row_cost': 90,
        'obstacle_cost': 100,
        'headland_cost': 30,
        'unknown_cost': 50,
        'inflation_radius': 1.0,
        'cost_scaling_factor': 10.0
    }
    
    config_path = '/tmp/costmap_config.yaml'
    with open(config_path, 'w') as f:
        yaml.dump(config, f, default_flow_style=False)
    
    return config_path


def main(args=None):
    """Test the costmap generator"""
    rclpy.init(args=args)
    
    # Create sample configuration
    config_file = create_sample_config()
    logger.info(f"Sample configuration created at {config_file}")
    
    # Create costmap publisher node
    node = CostmapPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
