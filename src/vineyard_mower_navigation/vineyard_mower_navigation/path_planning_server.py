#!/usr/bin/env python3
"""
ROS2 Action Server for Vineyard Path Planning

This module provides a ROS2 action server interface for the vineyard
path planning system, integrating with the costmap generation system.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import numpy as np
from typing import Optional, Dict, Any
import logging
import threading
import json
import os
from datetime import datetime

# Import custom interfaces (to be created)
from vineyard_mower_interfaces.action import PlanPath
from vineyard_mower_interfaces.srv import GenerateCostmap, GetCostmapInfo
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Header, String
from visualization_msgs.msg import Marker, MarkerArray

from .path_planner import VineyardPathPlanner, PathPlanningConfig, VineyardPath
from .satellite_processor import VineyardDetectionResult
from .costmap_generator import CostmapGenerator

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class PathPlanningActionServer(Node):
    """
    ROS2 Action Server for vineyard path planning
    """
    
    def __init__(self):
        super().__init__('path_planning_action_server')
        
        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('config_file', ''),
                ('vehicle_width', 1.5),
                ('vehicle_length', 2.0),
                ('min_turning_radius', 2.0),
                ('row_spacing', 2.5),
                ('max_velocity', 2.0),
                ('waypoint_density', 1.0),
                ('save_paths', True),
                ('path_save_directory', '/tmp/vineyard_paths/'),
                ('visualization_topic', 'path_visualization')
            ]
        )
        
        # Get parameters
        self.config_file = self.get_parameter('config_file').value
        self.save_paths = self.get_parameter('save_paths').value
        self.save_directory = self.get_parameter('path_save_directory').value
        self.visualization_topic = self.get_parameter('visualization_topic').value
        
        # Create save directory
        if self.save_paths:
            os.makedirs(self.save_directory, exist_ok=True)
        
        # Initialize path planner configuration
        config = PathPlanningConfig(
            vehicle_width=self.get_parameter('vehicle_width').value,
            vehicle_length=self.get_parameter('vehicle_length').value,
            min_turning_radius=self.get_parameter('min_turning_radius').value,
            row_spacing=self.get_parameter('row_spacing').value,
            max_velocity=self.get_parameter('max_velocity').value,
            waypoint_density=self.get_parameter('waypoint_density').value
        )
        
        # Initialize path planner
        self.path_planner = VineyardPathPlanner(config)
        
        # Load config file if provided
        if self.config_file and os.path.exists(self.config_file):
            self.path_planner.load_config_from_file(self.config_file)
        
        # State management
        self.current_path: Optional[VineyardPath] = None
        self.planning_lock = threading.Lock()
        
        # QoS profiles
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Action server
        self._action_server = ActionServer(
            self,
            PlanPath,
            'plan_vineyard_path',
            self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        
        # Service clients
        self.costmap_client = self.create_client(GenerateCostmap, 'generate_costmap')
        self.costmap_info_client = self.create_client(GetCostmapInfo, 'get_costmap_info')
        
        # Publishers
        self.path_pub = self.create_publisher(Path, 'planned_path', qos_profile)
        self.visualization_pub = self.create_publisher(
            MarkerArray, self.visualization_topic, qos_profile)
        self.status_pub = self.create_publisher(String, 'path_planning_status', qos_profile)
        
        # Subscribers
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, 'global_costmap', self.costmap_callback, qos_profile)
        
        self.get_logger().info("PathPlanningActionServer initialized")
        self.publish_status("initialized")
    
    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action"""
        self.get_logger().info("Received path planning goal request")
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action"""
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT
    
    async def execute_callback(self, goal_handle):
        """Execute the path planning action"""
        self.get_logger().info("Executing path planning action")
        self.publish_status("planning")
        
        goal = goal_handle.request
        feedback_msg = PlanPath.Feedback()
        result = PlanPath.Result()
        
        try:
            with self.planning_lock:
                # Step 1: Acquire costmap data
                feedback_msg.status = "acquiring_costmap"
                feedback_msg.progress = 10
                goal_handle.publish_feedback(feedback_msg)
                
                if goal.use_satellite_imagery:
                    # Generate costmap from satellite imagery
                    success = await self.generate_costmap_from_satellite(goal)
                    if not success:
                        result.success = False
                        result.message = "Failed to generate costmap from satellite imagery"
                        return result
                else:
                    # Use existing costmap
                    if not self.wait_for_costmap():
                        result.success = False
                        result.message = "No costmap data available"
                        return result
                
                # Step 2: Extract vineyard structure
                feedback_msg.status = "extracting_vineyard_structure"
                feedback_msg.progress = 30
                goal_handle.publish_feedback(feedback_msg)
                
                detection_result = self.extract_vineyard_from_goal(goal)
                
                # Step 3: Plan coverage path
                feedback_msg.status = "planning_coverage_path"
                feedback_msg.progress = 50
                goal_handle.publish_feedback(feedback_msg)
                
                vineyard_path = self.path_planner.plan_vineyard_coverage(detection_result)
                
                # Step 4: Optimize path
                if goal.optimize_path:
                    feedback_msg.status = "optimizing_path"
                    feedback_msg.progress = 70
                    goal_handle.publish_feedback(feedback_msg)
                    
                    vineyard_path = self.path_planner.optimize_path(vineyard_path)
                
                # Step 5: Convert to ROS messages and publish
                feedback_msg.status = "publishing_results"
                feedback_msg.progress = 90
                goal_handle.publish_feedback(feedback_msg)
                
                ros_path = self.convert_to_ros_path(vineyard_path)
                self.path_pub.publish(ros_path)
                
                # Visualize path
                self.visualize_path(vineyard_path)
                
                # Save path if requested
                if self.save_paths:
                    self.save_planned_path(vineyard_path)
                
                # Store current path
                self.current_path = vineyard_path
                
                # Prepare result
                result.success = True
                result.message = f"Successfully planned path with {len(vineyard_path.segments)} segments"
                result.path = ros_path
                result.total_distance = vineyard_path.total_distance
                result.total_time = vineyard_path.total_time
                result.coverage_percentage = vineyard_path.coverage_percentage
                result.num_waypoints = len(vineyard_path.get_all_waypoints())
                
                feedback_msg.status = "completed"
                feedback_msg.progress = 100
                goal_handle.publish_feedback(feedback_msg)
                
                self.get_logger().info(f"Path planning completed: {result.message}")
                self.publish_status("completed")
                
                goal_handle.succeed()
                return result
                
        except Exception as e:
            error_msg = f"Path planning failed: {str(e)}"
            self.get_logger().error(error_msg)
            self.publish_status("error")
            
            result.success = False
            result.message = error_msg
            goal_handle.abort()
            return result
    
    async def generate_costmap_from_satellite(self, goal) -> bool:
        """Generate costmap from satellite imagery"""
        if not self.costmap_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Costmap generation service not available")
            return False
        
        # Create costmap generation request
        request = GenerateCostmap.Request()
        request.center_latitude = goal.center_latitude
        request.center_longitude = goal.center_longitude
        request.zoom_level = goal.zoom_level
        request.use_local_image = goal.use_local_image
        request.image_path = goal.image_path
        
        try:
            future = self.costmap_client.call_async(request)
            response = await future
            
            if response.success:
                self.get_logger().info(f"Costmap generated: {response.message}")
                return True
            else:
                self.get_logger().error(f"Costmap generation failed: {response.message}")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Costmap generation service call failed: {e}")
            return False
    
    def wait_for_costmap(self, timeout: float = 10.0) -> bool:
        """Wait for costmap data to be available"""
        # This would check if we have recent costmap data
        # For now, we'll assume it's available if the service responds
        if not self.costmap_info_client.wait_for_service(timeout_sec=timeout):
            return False
        
        try:
            request = GetCostmapInfo.Request()
            future = self.costmap_info_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            
            if future.result() and future.result().has_costmap:
                return True
                
        except Exception as e:
            self.get_logger().error(f"Failed to get costmap info: {e}")
        
        return False
    
    def extract_vineyard_from_goal(self, goal) -> VineyardDetectionResult:
        """Extract vineyard detection result from goal parameters"""
        # This would normally come from the costmap service
        # For now, create a synthetic detection result based on goal parameters
        
        vine_rows = self._extract_vine_rows_from_goal(goal)
        row_spacing = self._get_row_spacing_from_goal(goal)
        
        return VineyardDetectionResult(
            vine_rows=vine_rows,
            row_orientations=[0.0] * len(vine_rows),
            row_spacing=row_spacing,
            obstacles=[],
            headlands=[],
            image_bounds=(0, 200, 0, 200),
            pixel_to_meter_ratio=0.1
        )
    
    def _extract_vine_rows_from_goal(self, goal) -> list:
        """Extract vine rows from goal parameters"""
        vine_rows = []
        
        if hasattr(goal, 'vineyard_rows') and goal.vineyard_rows:
            vine_rows = self._process_provided_rows(goal.vineyard_rows)
        else:
            vine_rows = self._create_default_rows()
        
        return vine_rows
    
    def _process_provided_rows(self, vineyard_rows) -> list:
        """Process provided vineyard rows from goal"""
        vine_rows = []
        for row_data in vineyard_rows:
            row_lines = []
            for i in range(0, len(row_data.points), 2):
                if i + 1 < len(row_data.points):
                    p1 = row_data.points[i]
                    p2 = row_data.points[i + 1]
                    line = np.array([p1.x, p1.y, p2.x, p2.y])
                    row_lines.append(line)
            if row_lines:
                vine_rows.append(np.array(row_lines))
        return vine_rows
    
    def _create_default_rows(self) -> list:
        """Create default vineyard structure"""
        vine_rows = []
        for i in range(5):
            y = 20 + i * 40
            row_lines = np.array([[10, y, 190, y], [12, y + 20, 188, y + 20]])
            vine_rows.append(row_lines)
        return vine_rows
    
    def _get_row_spacing_from_goal(self, goal) -> float:
        """Get row spacing from goal parameters"""
        return goal.row_spacing if hasattr(goal, 'row_spacing') else 4.0
    
    def convert_to_ros_path(self, vineyard_path: VineyardPath) -> Path:
        """Convert VineyardPath to ROS Path message"""
        ros_path = Path()
        ros_path.header = Header()
        ros_path.header.stamp = self.get_clock().now().to_msg()
        ros_path.header.frame_id = "map"
        
        # Convert all waypoints to PoseStamped
        waypoints = vineyard_path.get_all_waypoints()
        
        for wp in waypoints:
            pose_stamped = PoseStamped()
            pose_stamped.header = ros_path.header
            
            # Position
            pose_stamped.pose.position.x = wp.x
            pose_stamped.pose.position.y = wp.y
            pose_stamped.pose.position.z = 0.0
            
            # Orientation (convert from yaw to quaternion)
            pose_stamped.pose.orientation.x = 0.0
            pose_stamped.pose.orientation.y = 0.0
            pose_stamped.pose.orientation.z = np.sin(wp.theta / 2.0)
            pose_stamped.pose.orientation.w = np.cos(wp.theta / 2.0)
            
            ros_path.poses.append(pose_stamped)
        
        return ros_path
    
    def visualize_path(self, vineyard_path: VineyardPath):
        """Create visualization markers for the path"""
        marker_array = MarkerArray()
        marker_id = 0
        
        # Create markers for each segment
        for i, segment in enumerate(vineyard_path.segments):
            # Path line marker
            line_marker = Marker()
            line_marker.header.frame_id = "map"
            line_marker.header.stamp = self.get_clock().now().to_msg()
            line_marker.ns = "path_segments"
            line_marker.id = marker_id
            marker_id += 1
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            
            # Set color based on segment type
            if segment.segment_type.value == "row_traversal":
                line_marker.color.r = 0.0
                line_marker.color.g = 1.0
                line_marker.color.b = 0.0
            elif segment.segment_type.value == "headland_turn":
                line_marker.color.r = 1.0
                line_marker.color.g = 0.5
                line_marker.color.b = 0.0
            else:  # connector
                line_marker.color.r = 0.0
                line_marker.color.g = 0.0
                line_marker.color.b = 1.0
            
            line_marker.color.a = 0.8
            line_marker.scale.x = 0.2
            
            # Add waypoints to line
            for wp in segment.waypoints:
                point = Point()
                point.x = wp.x
                point.y = wp.y
                point.z = 0.1
                line_marker.points.append(point)
            
            marker_array.markers.append(line_marker)
        
        # Add waypoint markers
        waypoints = vineyard_path.get_all_waypoints()
        for i, wp in enumerate(waypoints):
            waypoint_marker = Marker()
            waypoint_marker.header.frame_id = "map"
            waypoint_marker.header.stamp = self.get_clock().now().to_msg()
            waypoint_marker.ns = "waypoints"
            waypoint_marker.id = marker_id
            marker_id += 1
            waypoint_marker.type = Marker.SPHERE
            waypoint_marker.action = Marker.ADD
            
            waypoint_marker.pose.position.x = wp.x
            waypoint_marker.pose.position.y = wp.y
            waypoint_marker.pose.position.z = 0.1
            
            waypoint_marker.pose.orientation.w = 1.0
            
            waypoint_marker.scale.x = 0.3
            waypoint_marker.scale.y = 0.3
            waypoint_marker.scale.z = 0.3
            
            waypoint_marker.color.r = 1.0
            waypoint_marker.color.g = 1.0
            waypoint_marker.color.b = 0.0
            waypoint_marker.color.a = 0.6
            
            marker_array.markers.append(waypoint_marker)
        
        # Publish visualization
        self.visualization_pub.publish(marker_array)
        self.get_logger().info(f"Published path visualization with {len(marker_array.markers)} markers")
    
    def save_planned_path(self, vineyard_path: VineyardPath):
        """Save the planned path to file"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"vineyard_path_{timestamp}.json"
        filepath = os.path.join(self.save_directory, filename)
        
        if self.path_planner.save_path(vineyard_path, filepath):
            self.get_logger().info(f"Path saved to {filepath}")
        else:
            self.get_logger().error(f"Failed to save path to {filepath}")
    
    def costmap_callback(self, msg: OccupancyGrid):
        """Callback for costmap updates"""
        # Update the path planner with new costmap data
        costmap_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        
        self.path_planner.set_costmap(costmap_data, msg.info.resolution, origin)
        self.get_logger().debug("Updated path planner with new costmap data")
    
    def publish_status(self, status: str):
        """Publish current status"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        # Create the action server
        path_planning_server = PathPlanningActionServer()
        
        # Use MultiThreadedExecutor for handling multiple callbacks
        executor = MultiThreadedExecutor()
        executor.add_node(path_planning_server)
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
        
    finally:
        if 'path_planning_server' in locals():
            path_planning_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
