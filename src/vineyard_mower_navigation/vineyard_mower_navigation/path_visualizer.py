#!/usr/bin/env python3
"""
Path Visualization Tool

This module provides visualization tools for vineyard paths,
including interactive displays and analysis capabilities.
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
from typing import List, Optional, Tuple, Dict, Any
import json
import cv2
import logging

from .path_planner import VineyardPath, PathSegment, Waypoint, PathSegmentType

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class PathVisualizer:
    """
    Interactive visualization tool for vineyard paths
    """
    
    def __init__(self, figsize: Tuple[int, int] = (12, 8)):
        """
        Initialize the path visualizer
        
        Args:
            figsize: Figure size for plots
        """
        self.figsize = figsize
        self.fig = None
        self.ax = None
        self.current_path: Optional[VineyardPath] = None
        
        # Color scheme
        self.colors = {
            'row_traversal': '#2E8B57',      # Sea Green
            'headland_turn': '#FF6347',      # Tomato
            'connector': '#4169E1',          # Royal Blue
            'waypoint': '#FFD700',           # Gold
            'boundary': '#8B4513',           # Saddle Brown
            'obstacle': '#DC143C',           # Crimson
            'vine_row': '#228B22'            # Forest Green
        }
        
        logger.info("PathVisualizer initialized")
    
    def visualize_path(self, vineyard_path: VineyardPath, 
                      show_waypoints: bool = True,
                      show_directions: bool = True,
                      save_path: Optional[str] = None) -> plt.Figure:
        """
        Create a comprehensive visualization of the vineyard path
        
        Args:
            vineyard_path: Path to visualize
            show_waypoints: Whether to show individual waypoints
            show_directions: Whether to show direction arrows
            save_path: Optional path to save the figure
            
        Returns:
            Matplotlib figure
        """
        self.current_path = vineyard_path
        
        # Create figure and axis
        self.fig, self.ax = plt.subplots(1, 1, figsize=self.figsize)
        
        # Plot vineyard boundary
        self._plot_boundary(vineyard_path.vineyard_bounds)
        
        # Plot path segments
        self._plot_path_segments(vineyard_path.segments, show_waypoints, show_directions)
        
        # Add path information
        self._add_path_info(vineyard_path)
        
        # Set axis properties
        self._configure_axes()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            logger.info(f"Path visualization saved to {save_path}")
        
        return self.fig
    
    def _plot_boundary(self, boundary):
        """Plot vineyard boundary"""
        if boundary is None:
            return
        
        # Extract boundary coordinates
        coords = list(boundary.exterior.coords)
        xs, ys = zip(*coords)
        
        self.ax.plot(xs, ys, color=self.colors['boundary'], 
                    linewidth=2, linestyle='--', alpha=0.7, label='Vineyard Boundary')
    
    def _plot_path_segments(self, segments: List[PathSegment], 
                          show_waypoints: bool, show_directions: bool):
        """Plot all path segments"""
        for i, segment in enumerate(segments):
            self._plot_single_segment(segment, i, show_waypoints, show_directions)
    
    def _plot_single_segment(self, segment: PathSegment, segment_index: int,
                           show_waypoints: bool, show_directions: bool):
        """Plot a single path segment"""
        if not segment.waypoints:
            return
        
        # Get segment color
        color = self.colors.get(segment.segment_type.value, '#000000')
        
        # Extract coordinates
        xs = [wp.x for wp in segment.waypoints]
        ys = [wp.y for wp in segment.waypoints]
        
        # Plot segment path
        label = f"{segment.segment_type.value.replace('_', ' ').title()}"
        if segment_index == 0:  # Only add label to legend once per type
            self.ax.plot(xs, ys, color=color, linewidth=2, alpha=0.8, label=label)
        else:
            self.ax.plot(xs, ys, color=color, linewidth=2, alpha=0.8)
        
        # Plot waypoints
        if show_waypoints:
            self.ax.scatter(xs, ys, color=self.colors['waypoint'], 
                          s=20, alpha=0.6, zorder=5)
        
        # Plot direction arrows
        if show_directions and len(segment.waypoints) > 1:
            self._add_direction_arrows(segment.waypoints, color)
        
        # Add turn type annotation for turns
        if segment.segment_type == PathSegmentType.HEADLAND_TURN and segment.turn_type:
            mid_idx = len(segment.waypoints) // 2
            mid_wp = segment.waypoints[mid_idx]
            self.ax.annotate(segment.turn_type.value.replace('_', ' ').title(),
                           (mid_wp.x, mid_wp.y), xytext=(5, 5), 
                           textcoords='offset points', fontsize=8,
                           bbox={'boxstyle': 'round,pad=0.3', 'facecolor': 'white', 'alpha': 0.7})
    
    def _add_direction_arrows(self, waypoints: List[Waypoint], color: str):
        """Add direction arrows along the path"""
        # Add arrows at regular intervals
        arrow_interval = max(1, len(waypoints) // 10)  # ~10 arrows per segment
        
        for i in range(0, len(waypoints) - 1, arrow_interval):
            wp1 = waypoints[i]
            wp2 = waypoints[i + 1]
            
            # Calculate arrow direction
            dx = wp2.x - wp1.x
            dy = wp2.y - wp1.y
            
            if abs(dx) > 1e-6 or abs(dy) > 1e-6:  # Avoid zero-length arrows
                self.ax.arrow(wp1.x, wp1.y, dx * 0.3, dy * 0.3,
                            head_width=0.5, head_length=0.3, 
                            fc=color, ec=color, alpha=0.7)
    
    def _add_path_info(self, vineyard_path: VineyardPath):
        """Add path information text box"""
        info_text = [
            f"Total Distance: {vineyard_path.total_distance:.1f} m",
            f"Total Time: {vineyard_path.total_time:.1f} s",
            f"Coverage: {vineyard_path.coverage_percentage:.1f}%",
            f"Segments: {len(vineyard_path.segments)}",
            f"Waypoints: {len(vineyard_path.get_all_waypoints())}"
        ]
        
        # Create text box
        text_str = '\n'.join(info_text)
        bbox_props = {'boxstyle': 'round,pad=0.5', 'facecolor': 'lightgray', 'alpha': 0.8}
        self.ax.text(0.02, 0.98, text_str, transform=self.ax.transAxes, 
                    fontsize=10, verticalalignment='top', bbox=bbox_props)
    
    def _configure_axes(self):
        """Configure axis properties"""
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlabel('X Position (m)', fontsize=12)
        self.ax.set_ylabel('Y Position (m)', fontsize=12)
        self.ax.set_title('Vineyard Coverage Path Plan', fontsize=14, fontweight='bold')
        self.ax.legend(loc='upper right')
    
    def create_animated_traversal(self, vineyard_path: VineyardPath, 
                                save_path: Optional[str] = None) -> FuncAnimation:
        """
        Create an animated visualization of path traversal
        
        Args:
            vineyard_path: Path to animate
            save_path: Optional path to save animation
            
        Returns:
            Animation object
        """
        self.current_path = vineyard_path
        
        # Create figure
        self.fig, self.ax = plt.subplots(1, 1, figsize=self.figsize)
        
        # Plot static elements
        self._plot_boundary(vineyard_path.vineyard_bounds)
        
        # Plot path segments (static)
        for segment in vineyard_path.segments:
            if not segment.waypoints:
                continue
            
            color = self.colors.get(segment.segment_type.value, '#000000')
            xs = [wp.x for wp in segment.waypoints]
            ys = [wp.y for wp in segment.waypoints]
            self.ax.plot(xs, ys, color=color, linewidth=1, alpha=0.3)
        
        # Initialize robot marker
        all_waypoints = vineyard_path.get_all_waypoints()
        if not all_waypoints:
            logger.error("No waypoints found for animation")
            return None
        
        robot_marker, = self.ax.plot([], [], 'ro', markersize=8, label='Robot')
        trail_line, = self.ax.plot([], [], 'r-', alpha=0.5, linewidth=2, label='Trail')
        
        self._configure_axes()
        
        # Animation data
        trail_x, trail_y = [], []
        
        def animate(frame):
            if frame >= len(all_waypoints):
                return robot_marker, trail_line
            
            wp = all_waypoints[frame]
            
            # Update robot position
            robot_marker.set_data([wp.x], [wp.y])
            
            # Update trail
            trail_x.append(wp.x)
            trail_y.append(wp.y)
            trail_line.set_data(trail_x, trail_y)
            
            return robot_marker, trail_line
        
        # Create animation
        anim = FuncAnimation(self.fig, animate, frames=len(all_waypoints),
                           interval=100, blit=True, repeat=True)
        
        if save_path:
            anim.save(save_path, writer='pillow', fps=10)
            logger.info(f"Animation saved to {save_path}")
        
        return anim
    
    def compare_paths(self, paths: Dict[str, VineyardPath], 
                     save_path: Optional[str] = None) -> plt.Figure:
        """
        Compare multiple paths side by side
        
        Args:
            paths: Dictionary of path names to VineyardPath objects
            save_path: Optional path to save the figure
            
        Returns:
            Matplotlib figure
        """
        num_paths = len(paths)
        if num_paths == 0:
            logger.error("No paths provided for comparison")
            return None
        
        # Create subplots
        cols = min(3, num_paths)
        rows = (num_paths + cols - 1) // cols
        
        fig, axes = plt.subplots(rows, cols, figsize=(5 * cols, 4 * rows))
        if num_paths == 1:
            axes = [axes]
        elif rows == 1:
            axes = axes.reshape(1, -1)
        
        # Plot each path
        for idx, (name, path) in enumerate(paths.items()):
            row = idx // cols
            col = idx % cols
            ax = axes[row, col] if rows > 1 else axes[col]
            
            self._plot_single_path_on_axis(ax, path, name)
        
        # Hide unused subplots
        for idx in range(num_paths, rows * cols):
            row = idx // cols
            col = idx % cols
            ax = axes[row, col] if rows > 1 else axes[col]
            ax.set_visible(False)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            logger.info(f"Path comparison saved to {save_path}")
        
        return fig
    
    def _plot_single_path_on_axis(self, ax, vineyard_path: VineyardPath, title: str):
        """Plot a single path on a given axis"""
        # Plot boundary
        if vineyard_path.vineyard_bounds:
            coords = list(vineyard_path.vineyard_bounds.exterior.coords)
            xs, ys = zip(*coords)
            ax.plot(xs, ys, color=self.colors['boundary'], 
                   linewidth=1, linestyle='--', alpha=0.7)
        
        # Plot segments
        for segment in vineyard_path.segments:
            if not segment.waypoints:
                continue
            
            color = self.colors.get(segment.segment_type.value, '#000000')
            xs = [wp.x for wp in segment.waypoints]
            ys = [wp.y for wp in segment.waypoints]
            ax.plot(xs, ys, color=color, linewidth=2, alpha=0.8)
        
        # Configure axis
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.set_title(f"{title}\n{vineyard_path.total_distance:.1f}m, "
                    f"{vineyard_path.total_time:.1f}s, "
                    f"{vineyard_path.coverage_percentage:.1f}%", fontsize=10)
    
    def generate_path_report(self, vineyard_path: VineyardPath, 
                           output_file: str) -> bool:
        """
        Generate a detailed path analysis report
        
        Args:
            vineyard_path: Path to analyze
            output_file: Output file path
            
        Returns:
            Success status
        """
        try:
            report_data = self._analyze_path(vineyard_path)
            
            # Create visualizations
            fig = self.visualize_path(vineyard_path, show_waypoints=True, show_directions=True)
            
            # Save visualization
            viz_path = output_file.replace('.json', '_visualization.png')
            plt.savefig(viz_path, dpi=300, bbox_inches='tight')
            plt.close(fig)
            
            # Add visualization path to report
            report_data['visualization_file'] = viz_path
            
            # Save report
            with open(output_file, 'w') as f:
                json.dump(report_data, f, indent=2)
            
            logger.info(f"Path report generated: {output_file}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to generate path report: {e}")
            return False
    
    def _analyze_path(self, vineyard_path: VineyardPath) -> Dict[str, Any]:
        """Analyze path and generate metrics"""
        waypoints = vineyard_path.get_all_waypoints()
        
        # Basic metrics
        metrics = {
            'total_distance': vineyard_path.total_distance,
            'total_time': vineyard_path.total_time,
            'coverage_percentage': vineyard_path.coverage_percentage,
            'num_segments': len(vineyard_path.segments),
            'num_waypoints': len(waypoints),
            'average_velocity': vineyard_path.total_distance / vineyard_path.total_time if vineyard_path.total_time > 0 else 0
        }
        
        # Segment analysis
        segment_stats = {}
        for seg_type in PathSegmentType:
            segments_of_type = [s for s in vineyard_path.segments if s.segment_type == seg_type]
            
            if segments_of_type:
                total_distance = sum(s.estimated_distance for s in segments_of_type)
                total_time = sum(s.estimated_time for s in segments_of_type)
                
                segment_stats[seg_type.value] = {
                    'count': len(segments_of_type),
                    'total_distance': total_distance,
                    'total_time': total_time,
                    'percentage_of_path': (total_distance / vineyard_path.total_distance) * 100 if vineyard_path.total_distance > 0 else 0
                }
        
        # Velocity analysis
        velocities = [wp.velocity for wp in waypoints]
        velocity_stats = {
            'min_velocity': min(velocities) if velocities else 0,
            'max_velocity': max(velocities) if velocities else 0,
            'avg_velocity': np.mean(velocities) if velocities else 0,
            'std_velocity': np.std(velocities) if velocities else 0
        }
        
        return {
            'timestamp': vineyard_path.segments[0].waypoints[0].action if vineyard_path.segments else 'unknown',
            'basic_metrics': metrics,
            'segment_analysis': segment_stats,
            'velocity_analysis': velocity_stats,
            'vineyard_bounds': list(vineyard_path.vineyard_bounds.exterior.coords) if vineyard_path.vineyard_bounds else []
        }


def main():
    """Test the path visualizer"""
    # This would normally load a real path from file
    # For testing, create a simple synthetic path
    
    from .path_planner import VineyardPathPlanner, PathPlanningConfig
    from .satellite_processor import VineyardDetectionResult
    
    # Create test configuration and planner
    config = PathPlanningConfig(vehicle_width=1.5, row_spacing=2.5)
    planner = VineyardPathPlanner(config)
    
    # Create synthetic detection result
    vine_rows = [
        np.array([[10, 20, 190, 25]]),
        np.array([[10, 60, 190, 65]]),
        np.array([[10, 100, 190, 105]])
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
    
    # Plan path
    path = planner.plan_vineyard_coverage(detection_result)
    
    # Create visualizer and generate visualization
    visualizer = PathVisualizer()
    
    # Static visualization
    visualizer.visualize_path(path, save_path='/tmp/test_path_visualization.png')
    plt.show()
    
    # Generate report
    visualizer.generate_path_report(path, '/tmp/test_path_report.json')
    
    logger.info("Path visualization test completed")


if __name__ == "__main__":
    main()
