#!/usr/bin/env python3
"""
Vineyard Path Planning Module

This module implements coverage path planning algorithms optimized for vineyard
navigation, including boustrophedon patterns, turning maneuvers, and integration
with costmap data.
"""

import numpy as np
import cv2
from typing import List, Tuple, Optional, Dict, Any
from dataclasses import dataclass
from enum import Enum
import logging
from shapely.geometry import Point, LineString, Polygon, MultiPolygon
from shapely.ops import unary_union
import networkx as nx
from scipy.spatial.distance import cdist
from scipy.optimize import minimize
import yaml
import json
from datetime import datetime

from .satellite_processor import VineyardDetectionResult
from .costmap_generator import CostmapGenerator

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class TurnType(Enum):
    """Types of turning maneuvers"""
    U_TURN = "u_turn"
    OMEGA_TURN = "omega_turn"
    STRAIGHT_THROUGH = "straight_through"


class PathSegmentType(Enum):
    """Types of path segments"""
    ROW_TRAVERSAL = "row_traversal"
    HEADLAND_TURN = "headland_turn"
    CONNECTOR = "connector"


@dataclass
class Waypoint:
    """Individual waypoint in the path"""
    x: float
    y: float
    theta: float  # Orientation in radians
    velocity: float = 1.0  # Target velocity m/s
    action: str = "move"  # Action type (move, pause, etc.)
    segment_type: PathSegmentType = PathSegmentType.ROW_TRAVERSAL
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert waypoint to dictionary"""
        return {
            'x': self.x,
            'y': self.y,
            'theta': self.theta,
            'velocity': self.velocity,
            'action': self.action,
            'segment_type': self.segment_type.value
        }


@dataclass
class PathSegment:
    """A segment of the complete path"""
    waypoints: List[Waypoint]
    segment_type: PathSegmentType
    start_row: Optional[int] = None
    end_row: Optional[int] = None
    turn_type: Optional[TurnType] = None
    estimated_time: float = 0.0
    estimated_distance: float = 0.0


@dataclass
class VineyardPath:
    """Complete vineyard coverage path"""
    segments: List[PathSegment]
    total_distance: float
    total_time: float
    coverage_percentage: float
    vineyard_bounds: Polygon
    
    def get_all_waypoints(self) -> List[Waypoint]:
        """Get all waypoints in order"""
        waypoints = []
        for segment in self.segments:
            waypoints.extend(segment.waypoints)
        return waypoints
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert path to dictionary for serialization"""
        return {
            'segments': [
                {
                    'waypoints': [wp.to_dict() for wp in seg.waypoints],
                    'segment_type': seg.segment_type.value,
                    'start_row': seg.start_row,
                    'end_row': seg.end_row,
                    'turn_type': seg.turn_type.value if seg.turn_type else None,
                    'estimated_time': seg.estimated_time,
                    'estimated_distance': seg.estimated_distance
                }
                for seg in self.segments
            ],
            'total_distance': self.total_distance,
            'total_time': self.total_time,
            'coverage_percentage': self.coverage_percentage,
            'vineyard_bounds': list(self.vineyard_bounds.exterior.coords)
        }


@dataclass
class PathPlanningConfig:
    """Configuration for path planning"""
    # Vehicle constraints
    vehicle_width: float = 1.5  # meters
    vehicle_length: float = 2.0  # meters
    min_turning_radius: float = 2.0  # meters
    max_velocity: float = 2.0  # m/s
    min_velocity: float = 0.5  # m/s
    
    # Coverage parameters
    row_spacing: float = 2.5  # meters between rows
    overlap_percentage: float = 0.1  # 10% overlap
    gap_tolerance: float = 0.5  # meters
    headland_width: float = 5.0  # meters
    
    # Path optimization
    optimize_for_time: bool = True
    avoid_obstacles: bool = True
    smooth_turns: bool = True
    waypoint_density: float = 1.0  # waypoints per meter
    
    # Turn parameters
    preferred_turn_type: TurnType = TurnType.U_TURN
    turn_speed_factor: float = 0.5  # Slow down factor for turns
    
    # Cost weights for optimization
    distance_weight: float = 1.0
    time_weight: float = 1.5
    obstacle_weight: float = 3.0
    smoothness_weight: float = 0.5


class VineyardPathPlanner:
    """
    path planning system for vineyard coverage
    """
    
    def __init__(self, config: Optional[PathPlanningConfig] = None):
        """
        Initialize the path planner
        
        Args:
            config: Path planning configuration
        """
        self.config = config or PathPlanningConfig()
        self.costmap_data: Optional[np.ndarray] = None
        self.costmap_resolution: float = 0.1
        self.costmap_origin: Tuple[float, float] = (0.0, 0.0)
        
        logger.info("VineyardPathPlanner initialized")
    
    def set_costmap(self, costmap_data: np.ndarray, resolution: float, 
                   origin: Tuple[float, float]):
        """
        Set the costmap data for path planning
        
        Args:
            costmap_data: 2D array of cost values
            resolution: meters per pixel
            origin: Origin coordinates (x, y) in meters
        """
        self.costmap_data = costmap_data
        self.costmap_resolution = resolution
        self.costmap_origin = origin
        logger.info(f"Costmap set with resolution {resolution}m, origin {origin}")
    
    def plan_vineyard_coverage(self, detection_result: VineyardDetectionResult,
                             vineyard_boundary: Optional[Polygon] = None) -> VineyardPath:
        """
        Plan complete vineyard coverage path
        
        Args:
            detection_result: Results from vineyard detection
            vineyard_boundary: Optional boundary polygon
            
        Returns:
            Complete vineyard coverage path
        """
        logger.info("Planning vineyard coverage path")
        
        # Extract vineyard structure
        vine_rows = self._extract_vine_rows(detection_result)
        
        # Create vineyard boundary if not provided
        if vineyard_boundary is None:
            vineyard_boundary = self._create_vineyard_boundary(vine_rows)
        
        # Generate coverage pattern
        coverage_rows = self._generate_coverage_rows(vine_rows, vineyard_boundary)
        
        # Optimize row sequence
        optimized_sequence = self._optimize_row_sequence(coverage_rows)
        
        # Generate path segments
        path_segments = self._generate_path_segments(optimized_sequence)
        
        # Calculate path metrics
        total_distance, total_time, coverage_percentage = self._calculate_path_metrics(
            path_segments, vineyard_boundary)
        
        # Create complete path
        vineyard_path = VineyardPath(
            segments=path_segments,
            total_distance=total_distance,
            total_time=total_time,
            coverage_percentage=coverage_percentage,
            vineyard_bounds=vineyard_boundary
        )
        
        logger.info("Path planned: %d segments, %.1fm, %.1fs, %.1f%% coverage",
                   len(path_segments), total_distance, total_time, coverage_percentage)
        
        return vineyard_path
    
    def _extract_vine_rows(self, detection_result: VineyardDetectionResult) -> List[LineString]:
        """Extract vine rows as LineString objects"""
        vine_rows = []
        
        for row_group in detection_result.vine_rows:
            # Convert each row group to LineString
            for line in row_group:
                x1, y1, x2, y2 = line
                # Convert pixel coordinates to world coordinates
                world_x1 = x1 * detection_result.pixel_to_meter_ratio
                world_y1 = y1 * detection_result.pixel_to_meter_ratio
                world_x2 = x2 * detection_result.pixel_to_meter_ratio
                world_y2 = y2 * detection_result.pixel_to_meter_ratio
                
                row_line = LineString([(world_x1, world_y1), (world_x2, world_y2)])
                vine_rows.append(row_line)
        
        return vine_rows
    
    def _create_vineyard_boundary(self, vine_rows: List[LineString]) -> Polygon:
        """Create vineyard boundary from vine rows"""
        if not vine_rows:
            # Default boundary
            return Polygon([(0, 0), (100, 0), (100, 100), (0, 100)])
        
        # Get bounding box of all vine rows
        all_coords = []
        for row in vine_rows:
            all_coords.extend(list(row.coords))
        
        coords_array = np.array(all_coords)
        min_x, min_y = np.min(coords_array, axis=0)
        max_x, max_y = np.max(coords_array, axis=0)
        
        # Add headland buffer
        buffer = self.config.headland_width
        boundary = Polygon([
            (min_x - buffer, min_y - buffer),
            (max_x + buffer, min_y - buffer),
            (max_x + buffer, max_y + buffer),
            (min_x - buffer, max_y + buffer)
        ])
        
        return boundary
    
    def _generate_coverage_rows(self, vine_rows: List[LineString], 
                              boundary: Polygon) -> List[LineString]:
        """Generate coverage rows with proper spacing"""
        if not vine_rows:
            return self._generate_boustrophedon_pattern(boundary)
        
        # Sort vine rows by position (assuming roughly parallel)
        sorted_rows = self._sort_parallel_lines(vine_rows)
        
        # Generate coverage rows between vine rows
        coverage_rows = []
        
        for i in range(len(sorted_rows)):
            # Add traversal along the vine row
            coverage_rows.append(sorted_rows[i])
            
            # Add coverage between this row and the next
            if i < len(sorted_rows) - 1:
                between_rows = self._generate_between_row_coverage(
                    sorted_rows[i], sorted_rows[i + 1])
                coverage_rows.extend(between_rows)
        
        return coverage_rows
    
    def _generate_boustrophedon_pattern(self, boundary: Polygon) -> List[LineString]:
        """Generate boustrophedon coverage pattern for irregular boundaries"""
        coverage_rows = []
        
        # Get boundary extent
        min_x, min_y, max_x, max_y = boundary.bounds
        
        # Generate parallel lines
        current_y = min_y + self.config.row_spacing / 2
        row_direction = 1  # 1 for left-to-right, -1 for right-to-left
        
        while current_y < max_y:
            # Create line across the boundary
            if row_direction == 1:
                line = LineString([(min_x, current_y), (max_x, current_y)])
            else:
                line = LineString([(max_x, current_y), (min_x, current_y)])
            
            # Clip line to boundary
            clipped = boundary.intersection(line)
            
            if isinstance(clipped, LineString):
                coverage_rows.append(clipped)
            elif hasattr(clipped, 'geoms'):  # MultiLineString
                coverage_rows.extend(list(clipped.geoms))
            
            current_y += self.config.row_spacing
            row_direction *= -1  # Alternate direction
        
        return coverage_rows
    
    def _sort_parallel_lines(self, lines: List[LineString]) -> List[LineString]:
        """Sort parallel lines by their perpendicular distance"""
        if not lines:
            return []
        
        # Calculate centroid of each line
        centroids = [line.centroid for line in lines]
        
        # Find the direction perpendicular to the lines
        first_line = lines[0]
        line_vector = np.array(first_line.coords[-1]) - np.array(first_line.coords[0])
        perp_vector = np.array([-line_vector[1], line_vector[0]])
        perp_vector = perp_vector / np.linalg.norm(perp_vector)
        
        # Project centroids onto perpendicular direction
        projections = []
        for centroid in centroids:
            point = np.array([centroid.x, centroid.y])
            projection = np.dot(point, perp_vector)
            projections.append(projection)
        
        # Sort lines by projection
        sorted_indices = np.argsort(projections)
        return [lines[i] for i in sorted_indices]
    
    def _generate_between_row_coverage(self, row1: LineString, 
                                     row2: LineString) -> List[LineString]:
        """Generate coverage lines between two vine rows"""
        coverage_lines = []
        
        # Calculate distance between rows
        distance = row1.distance(row2)
        
        if distance <= self.config.row_spacing * (1 + self.config.overlap_percentage):
            # Rows are close enough, no intermediate coverage needed
            return coverage_lines
        
        # Generate intermediate lines
        num_intermediate = int(distance / self.config.row_spacing) - 1
        
        for i in range(1, num_intermediate + 1):
            # Interpolate between the two rows
            alpha = i / (num_intermediate + 1)
            
            # Create intermediate line by interpolating points
            coords1 = list(row1.coords)
            coords2 = list(row2.coords)
            
            if len(coords1) == len(coords2):
                intermediate_coords = []
                for c1, c2 in zip(coords1, coords2):
                    x = c1[0] * (1 - alpha) + c2[0] * alpha
                    y = c1[1] * (1 - alpha) + c2[1] * alpha
                    intermediate_coords.append((x, y))
                
                coverage_lines.append(LineString(intermediate_coords))
        
        return coverage_lines
    
    def _optimize_row_sequence(self, coverage_rows: List[LineString]) -> List[LineString]:
        """Optimize the sequence of row traversal"""
        if len(coverage_rows) <= 1:
            return coverage_rows
        
        # Create distance matrix between row endpoints
        endpoints = []
        for row in coverage_rows:
            coords = list(row.coords)
            endpoints.append((coords[0], coords[-1]))  # Start and end points
        
        # Solve traveling salesman-like problem
        optimized_sequence = self._solve_coverage_sequence(coverage_rows, endpoints)
        
        return optimized_sequence
    
    def _solve_coverage_sequence(self, rows: List[LineString], 
                               endpoints: List[Tuple[Tuple[float, float], Tuple[float, float]]]) -> List[LineString]:
        """Solve for optimal coverage sequence"""
        n = len(rows)
        if n <= 1:
            return rows
        
        # For small problems, use exact solution
        if n <= 8:
            return self._exact_sequence_solution(rows, endpoints)
        
        # For larger problems, use greedy heuristic
        return self._greedy_sequence_solution(rows, endpoints)
    
    def _exact_sequence_solution(self, rows: List[LineString], 
                               endpoints: List[Tuple[Tuple[float, float], Tuple[float, float]]]) -> List[LineString]:
        """Exact solution for small sequence optimization problems"""
        # This is a simplified version - in practice, you'd use dynamic programming
        # or other exact algorithms for the traveling salesman problem
        
        # For now, use a simple greedy approach
        return self._greedy_sequence_solution(rows, endpoints)
    
    def _greedy_sequence_solution(self, rows: List[LineString], 
                                endpoints: List[Tuple[Tuple[float, float], Tuple[float, float]]]) -> List[LineString]:
        """Greedy solution for sequence optimization"""
        if not rows:
            return []
        
        # Start with the first row
        sequence = [rows[0]]
        remaining = list(range(1, len(rows)))
        current_end = endpoints[0][1]  # End point of first row
        
        while remaining:
            # Find the closest next row
            min_distance = float('inf')
            next_idx = -1
            next_start_idx = -1
            
            for i in remaining:
                # Try both orientations of the next row
                start_dist = np.linalg.norm(
                    np.array(current_end) - np.array(endpoints[i][0]))
                end_dist = np.linalg.norm(
                    np.array(current_end) - np.array(endpoints[i][1]))
                
                if start_dist < min_distance:
                    min_distance = start_dist
                    next_idx = i
                    next_start_idx = 0
                
                if end_dist < min_distance:
                    min_distance = end_dist
                    next_idx = i
                    next_start_idx = 1
            
            # Add the next row (possibly reversed)
            next_row = rows[next_idx]
            if next_start_idx == 1:
                # Reverse the row
                coords = list(next_row.coords)
                next_row = LineString(coords[::-1])
            
            sequence.append(next_row)
            remaining.remove(next_idx)
            current_end = list(next_row.coords)[-1]
        
        return sequence
    
    def _generate_path_segments(self, coverage_rows: List[LineString]) -> List[PathSegment]:
        """Generate path segments with turns between rows"""
        segments = []
        
        for i, row in enumerate(coverage_rows):
            # Generate row traversal segment
            row_segment = self._create_row_segment(row, i)
            segments.append(row_segment)
            
            # Generate turn segment to next row
            if i < len(coverage_rows) - 1:
                turn_segment = self._create_turn_segment(
                    row, coverage_rows[i + 1])
                if turn_segment:
                    segments.append(turn_segment)
        
        return segments
    
    def _create_row_segment(self, row: LineString, row_index: int) -> PathSegment:
        """Create a path segment for traversing a row"""
        waypoints = []
        
        # Generate waypoints along the row
        total_length = row.length
        num_waypoints = max(2, int(total_length * self.config.waypoint_density))
        
        for i in range(num_waypoints):
            alpha = i / (num_waypoints - 1) if num_waypoints > 1 else 0
            point = row.interpolate(alpha, normalized=True)
            
            # Calculate orientation
            if i < num_waypoints - 1:
                next_point = row.interpolate((i + 1) / (num_waypoints - 1), normalized=True)
                theta = np.arctan2(next_point.y - point.y, next_point.x - point.x)
            else:
                theta = waypoints[-1].theta if waypoints else 0.0
            
            waypoint = Waypoint(
                x=point.x,
                y=point.y,
                theta=theta,
                velocity=self.config.max_velocity,
                segment_type=PathSegmentType.ROW_TRAVERSAL
            )
            waypoints.append(waypoint)
        
        return PathSegment(
            waypoints=waypoints,
            segment_type=PathSegmentType.ROW_TRAVERSAL,
            start_row=row_index,
            end_row=row_index,
            estimated_distance=total_length,
            estimated_time=total_length / self.config.max_velocity
        )
    
    def _create_turn_segment(self, from_row: LineString, to_row: LineString) -> Optional[PathSegment]:
        """Create a turn segment between two rows"""
        from_end = Point(list(from_row.coords)[-1])
        to_start = Point(list(to_row.coords)[0])
        
        # Determine turn type
        turn_type = self._determine_turn_type(from_row, to_row)
        
        # Generate turn waypoints
        turn_waypoints = self._generate_turn_waypoints(
            from_end, to_start, turn_type)
        
        if not turn_waypoints:
            return None
        
        # Calculate turn metrics
        total_distance = sum(
            np.linalg.norm([wp2.x - wp1.x, wp2.y - wp1.y])
            for wp1, wp2 in zip(turn_waypoints[:-1], turn_waypoints[1:])
        )
        
        turn_speed = self.config.max_velocity * self.config.turn_speed_factor
        estimated_time = total_distance / turn_speed
        
        return PathSegment(
            waypoints=turn_waypoints,
            segment_type=PathSegmentType.HEADLAND_TURN,
            turn_type=turn_type,
            estimated_distance=total_distance,
            estimated_time=estimated_time
        )
    
    def _determine_turn_type(self, from_row: LineString, to_row: LineString) -> TurnType:
        """Determine the best turn type between two rows"""
        from_end = np.array(list(from_row.coords)[-1])
        to_start = np.array(list(to_row.coords)[0])
        
        # Calculate vectors
        from_vector = np.array(list(from_row.coords)[-1]) - np.array(list(from_row.coords)[-2])
        to_vector = np.array(list(to_row.coords)[1]) - np.array(list(to_row.coords)[0])
        
        # Normalize vectors
        from_vector = from_vector / np.linalg.norm(from_vector)
        to_vector = to_vector / np.linalg.norm(to_vector)
        
        # Calculate angle between vectors
        dot_product = np.dot(from_vector, to_vector)
        angle = np.arccos(np.clip(dot_product, -1.0, 1.0))
        
        # Decide turn type based on angle and distance
        distance = np.linalg.norm(to_start - from_end)
        
        if angle > np.pi * 0.75:  # > 135 degrees, opposite direction
            if distance < self.config.min_turning_radius * 4:
                return TurnType.U_TURN
            else:
                return TurnType.OMEGA_TURN
        else:
            return self.config.preferred_turn_type
    
    def _generate_turn_waypoints(self, from_point: Point, to_point: Point,
                               turn_type: TurnType) -> List[Waypoint]:
        """Generate waypoints for turn maneuver"""
        waypoints = []
        
        if turn_type == TurnType.U_TURN:
            waypoints = self._generate_u_turn_waypoints(from_point, to_point)
        elif turn_type == TurnType.OMEGA_TURN:
            waypoints = self._generate_omega_turn_waypoints(from_point, to_point)
        else:  # STRAIGHT_THROUGH
            waypoints = self._generate_straight_waypoints(from_point, to_point)
        
        return waypoints
    
    def _generate_u_turn_waypoints(self, from_point: Point, to_point: Point) -> List[Waypoint]:
        """Generate waypoints for U-turn maneuver"""
        waypoints = []
        
        # Calculate U-turn path
        center_x = (from_point.x + to_point.x) / 2
        center_y = (from_point.y + to_point.y) / 2
        
        # Create arc waypoints
        radius = max(self.config.min_turning_radius, 
                    np.linalg.norm([to_point.x - from_point.x, to_point.y - from_point.y]) / 2)
        
        num_waypoints = max(3, int(np.pi * radius * self.config.waypoint_density))
        
        for i in range(num_waypoints):
            alpha = i / (num_waypoints - 1) * np.pi
            
            x = center_x + radius * np.cos(alpha + np.pi)
            y = center_y + radius * np.sin(alpha + np.pi)
            theta = alpha + np.pi / 2
            
            waypoint = Waypoint(
                x=x, y=y, theta=theta,
                velocity=self.config.max_velocity * self.config.turn_speed_factor,
                segment_type=PathSegmentType.HEADLAND_TURN
            )
            waypoints.append(waypoint)
        
        return waypoints
    
    def _generate_omega_turn_waypoints(self, from_point: Point, to_point: Point) -> List[Waypoint]:
        """Generate waypoints for omega-turn maneuver"""
        waypoints = []
        
        # Move away from vineyard, turn around, come back
        direction = np.array([to_point.x - from_point.x, to_point.y - from_point.y])
        direction = direction / np.linalg.norm(direction)
        
        # Perpendicular direction (toward headland)
        perp_direction = np.array([-direction[1], direction[0]])
        
        # Move to headland
        headland_distance = self.config.headland_width
        headland_point = np.array([from_point.x, from_point.y]) + perp_direction * headland_distance
        
        # Turn around point
        turn_point = headland_point + direction * self.config.min_turning_radius
        
        # Return point
        return_point = np.array([to_point.x, to_point.y]) + perp_direction * headland_distance
        
        # Generate waypoints for the omega path
        points = [
            (from_point.x, from_point.y),
            tuple(headland_point),
            tuple(turn_point),
            tuple(return_point),
            (to_point.x, to_point.y)
        ]
        
        for i, (x, y) in enumerate(points):
            if i < len(points) - 1:
                next_x, next_y = points[i + 1]
                theta = np.arctan2(next_y - y, next_x - x)
            else:
                theta = waypoints[-1].theta if waypoints else 0.0
            
            waypoint = Waypoint(
                x=x, y=y, theta=theta,
                velocity=self.config.max_velocity * self.config.turn_speed_factor,
                segment_type=PathSegmentType.HEADLAND_TURN
            )
            waypoints.append(waypoint)
        
        return waypoints
    
    def _generate_straight_waypoints(self, from_point: Point, to_point: Point) -> List[Waypoint]:
        """Generate waypoints for straight connection"""
        waypoints = []
        
        distance = np.linalg.norm([to_point.x - from_point.x, to_point.y - from_point.y])
        num_waypoints = max(2, int(distance * self.config.waypoint_density))
        
        for i in range(num_waypoints):
            alpha = i / (num_waypoints - 1) if num_waypoints > 1 else 0
            
            x = from_point.x + alpha * (to_point.x - from_point.x)
            y = from_point.y + alpha * (to_point.y - from_point.y)
            theta = np.arctan2(to_point.y - from_point.y, to_point.x - from_point.x)
            
            waypoint = Waypoint(
                x=x, y=y, theta=theta,
                velocity=self.config.max_velocity * self.config.turn_speed_factor,
                segment_type=PathSegmentType.CONNECTOR
            )
            waypoints.append(waypoint)
        
        return waypoints
    
    def _calculate_path_metrics(self, segments: List[PathSegment], 
                              boundary: Polygon) -> Tuple[float, float, float]:
        """Calculate total distance, time, and coverage percentage"""
        total_distance = sum(seg.estimated_distance for seg in segments)
        total_time = sum(seg.estimated_time for seg in segments)
        
        # Calculate coverage percentage
        covered_area = 0.0
        boundary_area = boundary.area
        
        for segment in segments:
            if segment.segment_type == PathSegmentType.ROW_TRAVERSAL:
                # Approximate covered area as path length * vehicle width
                segment_length = segment.estimated_distance
                covered_area += segment_length * self.config.vehicle_width
        
        coverage_percentage = min(100.0, (covered_area / boundary_area) * 100.0) if boundary_area > 0 else 0.0
        
        return total_distance, total_time, coverage_percentage
    
    def optimize_path(self, path: VineyardPath) -> VineyardPath:
        """Optimize existing path for better performance"""
        logger.info("Optimizing vineyard path")
        
        # Apply various optimization techniques
        optimized_segments = []
        
        for segment in path.segments:
            # Smooth waypoints
            smoothed_segment = self._smooth_segment(segment)
            
            # Optimize velocities
            velocity_optimized = self._optimize_segment_velocities(smoothed_segment)
            
            optimized_segments.append(velocity_optimized)
        
        # Recalculate metrics
        total_distance, total_time, coverage_percentage = self._calculate_path_metrics(
            optimized_segments, path.vineyard_bounds)
        
        optimized_path = VineyardPath(
            segments=optimized_segments,
            total_distance=total_distance,
            total_time=total_time,
            coverage_percentage=coverage_percentage,
            vineyard_bounds=path.vineyard_bounds
        )
        
        logger.info(f"Path optimized: {total_time:.1f}s -> improved performance")
        return optimized_path
    
    def _smooth_segment(self, segment: PathSegment) -> PathSegment:
        """Apply smoothing to segment waypoints"""
        if len(segment.waypoints) < 3:
            return segment
        
        # Apply simple moving average smoothing to positions
        smoothed_waypoints = []
        waypoints = segment.waypoints
        
        for i, wp in enumerate(waypoints):
            if i == 0 or i == len(waypoints) - 1:
                # Keep first and last waypoints unchanged
                smoothed_waypoints.append(wp)
            else:
                # Smooth position
                prev_wp = waypoints[i - 1]
                next_wp = waypoints[i + 1]
                
                smoothed_x = (prev_wp.x + wp.x + next_wp.x) / 3
                smoothed_y = (prev_wp.y + wp.y + next_wp.y) / 3
                
                # Recalculate orientation
                if i < len(waypoints) - 1:
                    theta = np.arctan2(next_wp.y - smoothed_y, next_wp.x - smoothed_x)
                else:
                    theta = wp.theta
                
                smoothed_wp = Waypoint(
                    x=smoothed_x, y=smoothed_y, theta=theta,
                    velocity=wp.velocity, action=wp.action,
                    segment_type=wp.segment_type
                )
                smoothed_waypoints.append(smoothed_wp)
        
        return PathSegment(
            waypoints=smoothed_waypoints,
            segment_type=segment.segment_type,
            start_row=segment.start_row,
            end_row=segment.end_row,
            turn_type=segment.turn_type,
            estimated_time=segment.estimated_time,
            estimated_distance=segment.estimated_distance
        )
    
    def _optimize_segment_velocities(self, segment: PathSegment) -> PathSegment:
        """Optimize velocities for segment waypoints"""
        if not segment.waypoints:
            return segment
        
        optimized_waypoints = []
        
        for i, wp in enumerate(segment.waypoints):
            velocity = self._calculate_waypoint_velocity(segment, i, wp)
            
            optimized_wp = Waypoint(
                x=wp.x, y=wp.y, theta=wp.theta,
                velocity=velocity, action=wp.action,
                segment_type=wp.segment_type
            )
            optimized_waypoints.append(optimized_wp)
        
        return PathSegment(
            waypoints=optimized_waypoints,
            segment_type=segment.segment_type,
            start_row=segment.start_row,
            end_row=segment.end_row,
            turn_type=segment.turn_type,
            estimated_time=segment.estimated_time,
            estimated_distance=segment.estimated_distance
        )
    
    def _calculate_waypoint_velocity(self, segment: PathSegment, index: int, waypoint: Waypoint) -> float:
        """Calculate optimal velocity for a single waypoint"""
        base_velocity = waypoint.velocity
        
        if segment.segment_type == PathSegmentType.HEADLAND_TURN:
            return base_velocity * self.config.turn_speed_factor
        
        # Check curvature for regular waypoints
        curvature_factor = self._calculate_curvature_factor(segment.waypoints, index)
        velocity = base_velocity * curvature_factor
        
        return np.clip(velocity, self.config.min_velocity, self.config.max_velocity)
    
    def _calculate_curvature_factor(self, waypoints: List[Waypoint], index: int) -> float:
        """Calculate velocity reduction factor based on path curvature"""
        if index == 0 or index >= len(waypoints) - 1:
            return 1.0
        
        prev_wp = waypoints[index - 1]
        curr_wp = waypoints[index]
        next_wp = waypoints[index + 1]
        
        # Calculate direction vectors
        v1 = np.array([curr_wp.x - prev_wp.x, curr_wp.y - prev_wp.y])
        v2 = np.array([next_wp.x - curr_wp.x, next_wp.y - curr_wp.y])
        
        # Check for zero-length vectors
        if np.linalg.norm(v1) == 0 or np.linalg.norm(v2) == 0:
            return 1.0
        
        # Normalize vectors
        v1_norm = v1 / np.linalg.norm(v1)
        v2_norm = v2 / np.linalg.norm(v2)
        
        # Calculate angle change
        dot_product = np.dot(v1_norm, v2_norm)
        angle_change = np.arccos(np.clip(dot_product, -1.0, 1.0))
        
        # Reduce velocity for sharp turns
        if angle_change > np.pi / 4:  # > 45 degrees
            return 1 - angle_change / np.pi
        
        return 1.0
    
    def save_path(self, path: VineyardPath, filepath: str) -> bool:
        """Save path to file"""
        try:
            path_data = path.to_dict()
            path_data['timestamp'] = datetime.now().isoformat()
            path_data['config'] = {
                'vehicle_width': self.config.vehicle_width,
                'vehicle_length': self.config.vehicle_length,
                'min_turning_radius': self.config.min_turning_radius,
                'row_spacing': self.config.row_spacing,
                'overlap_percentage': self.config.overlap_percentage
            }
            
            with open(filepath, 'w') as f:
                json.dump(path_data, f, indent=2)
            
            logger.info(f"Path saved to {filepath}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to save path: {e}")
            return False
    
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


def main():
    """Test the path planner"""
    # Create test configuration
    config = PathPlanningConfig(
        vehicle_width=1.5,
        row_spacing=2.5,
        waypoint_density=0.5
    )
    
    planner = VineyardPathPlanner(config)
    
    # Create sample vineyard detection result
    from .satellite_processor import VineyardDetectionResult
    
    # Sample vine rows
    vine_rows = [
        np.array([[10, 20, 190, 25], [12, 80, 188, 85]]),
        np.array([[10, 60, 190, 65], [12, 120, 188, 125]]),
        np.array([[10, 100, 190, 105], [12, 160, 188, 165]])
    ]
    
    detection_result = VineyardDetectionResult(
        vine_rows=vine_rows,
        row_orientations=[0.0, 0.0, 0.0],
        row_spacing=4.0,
        obstacles=[],
        headlands=[],
        image_bounds=(0, 200, 0, 200),
        pixel_to_meter_ratio=0.1
    )
    
    # Plan coverage path
    path = planner.plan_vineyard_coverage(detection_result)
    
    logger.info(f"Planned path with {len(path.segments)} segments")
    logger.info(f"Total distance: {path.total_distance:.1f}m")
    logger.info(f"Total time: {path.total_time:.1f}s")
    logger.info(f"Coverage: {path.coverage_percentage:.1f}%")
    
    # Save path
    planner.save_path(path, "/tmp/test_vineyard_path.json")


if __name__ == "__main__":
    main()
