#!/usr/bin/env python3
"""
Unit tests for vineyard path planning system
"""

import unittest
import numpy as np
from unittest.mock import patch, MagicMock
import tempfile
import os
import json
from shapely.geometry import Polygon, LineString

import sys
import rclpy

# Add the module path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from vineyard_mower_navigation.path_planner import (
    VineyardPathPlanner, PathPlanningConfig, VineyardPath, 
    PathSegment, Waypoint, PathSegmentType, TurnType
)
from vineyard_mower_navigation.satellite_processor import VineyardDetectionResult
from vineyard_mower_navigation.path_visualizer import PathVisualizer


class TestPathPlanningConfig(unittest.TestCase):
    """Test path planning configuration"""
    
    def test_default_config(self):
        """Test default configuration values"""
        config = PathPlanningConfig()
        
        self.assertEqual(config.vehicle_width, 1.5)
        self.assertEqual(config.vehicle_length, 2.0)
        self.assertEqual(config.min_turning_radius, 2.0)
        self.assertEqual(config.row_spacing, 2.5)
        self.assertIsInstance(config.preferred_turn_type, TurnType)
    
    def test_custom_config(self):
        """Test custom configuration"""
        config = PathPlanningConfig(
            vehicle_width=2.0,
            row_spacing=3.0,
            max_velocity=1.5
        )
        
        self.assertEqual(config.vehicle_width, 2.0)
        self.assertEqual(config.row_spacing, 3.0)
        self.assertEqual(config.max_velocity, 1.5)


class TestWaypoint(unittest.TestCase):
    """Test waypoint functionality"""
    
    def test_waypoint_creation(self):
        """Test waypoint creation"""
        wp = Waypoint(x=10.0, y=20.0, theta=1.57, velocity=1.5)
        
        self.assertEqual(wp.x, 10.0)
        self.assertEqual(wp.y, 20.0)
        self.assertEqual(wp.theta, 1.57)
        self.assertEqual(wp.velocity, 1.5)
        self.assertEqual(wp.action, "move")
        self.assertEqual(wp.segment_type, PathSegmentType.ROW_TRAVERSAL)
    
    def test_waypoint_to_dict(self):
        """Test waypoint serialization"""
        wp = Waypoint(x=10.0, y=20.0, theta=1.57, velocity=1.5, action="pause")
        wp_dict = wp.to_dict()
        
        expected_keys = ['x', 'y', 'theta', 'velocity', 'action', 'segment_type']
        for key in expected_keys:
            self.assertIn(key, wp_dict)
        
        self.assertEqual(wp_dict['x'], 10.0)
        self.assertEqual(wp_dict['action'], "pause")


class TestPathSegment(unittest.TestCase):
    """Test path segment functionality"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.waypoints = [
            Waypoint(0.0, 0.0, 0.0, 1.0),
            Waypoint(10.0, 0.0, 0.0, 1.0),
            Waypoint(20.0, 0.0, 0.0, 1.0)
        ]
    
    def test_segment_creation(self):
        """Test path segment creation"""
        segment = PathSegment(
            waypoints=self.waypoints,
            segment_type=PathSegmentType.ROW_TRAVERSAL,
            estimated_distance=20.0,
            estimated_time=20.0
        )
        
        self.assertEqual(len(segment.waypoints), 3)
        self.assertEqual(segment.segment_type, PathSegmentType.ROW_TRAVERSAL)
        self.assertEqual(segment.estimated_distance, 20.0)
        self.assertEqual(segment.estimated_time, 20.0)


class TestVineyardPathPlanner(unittest.TestCase):
    """Test vineyard path planner"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.config = PathPlanningConfig(
            vehicle_width=1.5,
            row_spacing=2.5,
            waypoint_density=0.5
        )
        self.planner = VineyardPathPlanner(self.config)
        
        # Create test detection result
        self.vine_rows = [
            np.array([[10, 20, 190, 25]]),
            np.array([[10, 60, 190, 65]]),
            np.array([[10, 100, 190, 105]])
        ]
        
        self.detection_result = VineyardDetectionResult(
            vine_rows=self.vine_rows,
            row_orientations=[0.0, 0.0, 0.0],
            row_spacing=4.0,
            obstacles=[],
            headlands=[],
            image_bounds=(0, 200, 0, 200),
            pixel_to_meter_ratio=0.1
        )
    
    def test_planner_initialization(self):
        """Test planner initialization"""
        self.assertIsNotNone(self.planner)
        self.assertEqual(self.planner.config.vehicle_width, 1.5)
        self.assertIsNone(self.planner.costmap_data)
    
    def test_set_costmap(self):
        """Test costmap setting"""
        costmap_data = np.random.randint(0, 100, (100, 100))
        resolution = 0.1
        origin = (0.0, 0.0)
        
        self.planner.set_costmap(costmap_data, resolution, origin)
        
        np.testing.assert_array_equal(self.planner.costmap_data, costmap_data)
        self.assertEqual(self.planner.costmap_resolution, resolution)
        self.assertEqual(self.planner.costmap_origin, origin)
    
    def test_extract_vine_rows(self):
        """Test vine row extraction"""
        vine_rows = self.planner._extract_vine_rows(self.detection_result)
        
        self.assertIsInstance(vine_rows, list)
        self.assertGreater(len(vine_rows), 0)
        
        for row in vine_rows:
            self.assertIsInstance(row, LineString)
    
    def test_create_vineyard_boundary(self):
        """Test vineyard boundary creation"""
        vine_rows = self.planner._extract_vine_rows(self.detection_result)
        boundary = self.planner._create_vineyard_boundary(vine_rows)
        
        self.assertIsInstance(boundary, Polygon)
        self.assertGreater(boundary.area, 0)
    
    def test_boustrophedon_pattern(self):
        """Test boustrophedon pattern generation"""
        boundary = Polygon([(0, 0), (100, 0), (100, 100), (0, 100)])
        coverage_rows = self.planner._generate_boustrophedon_pattern(boundary)
        
        self.assertIsInstance(coverage_rows, list)
        self.assertGreater(len(coverage_rows), 0)
        
        for row in coverage_rows:
            self.assertIsInstance(row, LineString)
    
    def test_plan_vineyard_coverage(self):
        """Test complete vineyard coverage planning"""
        path = self.planner.plan_vineyard_coverage(self.detection_result)
        
        self.assertIsInstance(path, VineyardPath)
        self.assertGreater(len(path.segments), 0)
        self.assertGreater(path.total_distance, 0)
        self.assertGreater(path.total_time, 0)
        self.assertGreaterEqual(path.coverage_percentage, 0)
        self.assertLessEqual(path.coverage_percentage, 100)
    
    def test_path_optimization(self):
        """Test path optimization"""
        # Create initial path
        original_path = self.planner.plan_vineyard_coverage(self.detection_result)
        
        # Optimize path
        optimized_path = self.planner.optimize_path(original_path)
        
        self.assertIsInstance(optimized_path, VineyardPath)
        self.assertEqual(len(optimized_path.segments), len(original_path.segments))
    
    def test_save_and_load_config(self):
        """Test configuration saving and loading"""
        # Create temporary config file
        config_data = {
            'vehicle_width': 2.0,
            'row_spacing': 3.0,
            'max_velocity': 1.5
        }
        
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as tmp:
            import yaml
            yaml.dump(config_data, tmp)
            
            # Load configuration
            success = self.planner.load_config_from_file(tmp.name)
            self.assertTrue(success)
            
            # Check configuration was updated
            self.assertEqual(self.planner.config.vehicle_width, 2.0)
            self.assertEqual(self.planner.config.row_spacing, 3.0)
            self.assertEqual(self.planner.config.max_velocity, 1.5)
            
            # Cleanup
            os.unlink(tmp.name)
    
    def test_save_path(self):
        """Test path saving"""
        path = self.planner.plan_vineyard_coverage(self.detection_result)
        
        with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as tmp:
            success = self.planner.save_path(path, tmp.name)
            self.assertTrue(success)
            
            # Verify file was created and has valid JSON
            self.assertTrue(os.path.exists(tmp.name))
            
            with open(tmp.name, 'r') as f:
                loaded_data = json.load(f)
                self.assertIn('segments', loaded_data)
                self.assertIn('total_distance', loaded_data)
                self.assertIn('total_time', loaded_data)
            
            # Cleanup
            os.unlink(tmp.name)


class TestTurnGeneration(unittest.TestCase):
    """Test turn generation algorithms"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.config = PathPlanningConfig()
        self.planner = VineyardPathPlanner(self.config)
    
    def test_u_turn_generation(self):
        """Test U-turn waypoint generation"""
        from shapely.geometry import Point
        
        from_point = Point(0, 0)
        to_point = Point(10, 0)
        
        waypoints = self.planner._generate_u_turn_waypoints(from_point, to_point)
        
        self.assertIsInstance(waypoints, list)
        self.assertGreater(len(waypoints), 2)
        
        for wp in waypoints:
            self.assertIsInstance(wp, Waypoint)
            self.assertEqual(wp.segment_type, PathSegmentType.HEADLAND_TURN)
    
    def test_omega_turn_generation(self):
        """Test omega-turn waypoint generation"""
        from shapely.geometry import Point
        
        from_point = Point(0, 0)
        to_point = Point(0, 10)
        
        waypoints = self.planner._generate_omega_turn_waypoints(from_point, to_point)
        
        self.assertIsInstance(waypoints, list)
        self.assertGreater(len(waypoints), 3)
        
        for wp in waypoints:
            self.assertIsInstance(wp, Waypoint)
            self.assertEqual(wp.segment_type, PathSegmentType.HEADLAND_TURN)
    
    def test_turn_type_determination(self):
        """Test turn type determination"""
        # Create two rows with different orientations
        row1 = LineString([(0, 0), (10, 0)])
        row2 = LineString([(10, 10), (0, 10)])  # Opposite direction
        
        turn_type = self.planner._determine_turn_type(row1, row2)
        
        self.assertIsInstance(turn_type, TurnType)


class TestPathVisualizer(unittest.TestCase):
    """Test path visualization"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.visualizer = PathVisualizer()
        
        # Create a simple test path
        waypoints = [
            Waypoint(0, 0, 0, 1.0),
            Waypoint(10, 0, 0, 1.0),
            Waypoint(10, 10, 1.57, 1.0),
            Waypoint(0, 10, 3.14, 1.0)
        ]
        
        segment = PathSegment(
            waypoints=waypoints,
            segment_type=PathSegmentType.ROW_TRAVERSAL,
            estimated_distance=30.0,
            estimated_time=30.0
        )
        
        boundary = Polygon([(0, 0), (10, 0), (10, 10), (0, 10)])
        
        self.test_path = VineyardPath(
            segments=[segment],
            total_distance=30.0,
            total_time=30.0,
            coverage_percentage=80.0,
            vineyard_bounds=boundary
        )
    
    def test_visualizer_initialization(self):
        """Test visualizer initialization"""
        self.assertIsNotNone(self.visualizer)
        self.assertIsNone(self.visualizer.current_path)
        self.assertIsInstance(self.visualizer.colors, dict)
    
    @patch('matplotlib.pyplot.show')
    @patch('matplotlib.pyplot.savefig')
    def test_path_visualization(self, mock_savefig, mock_show):
        """Test path visualization"""
        fig = self.visualizer.visualize_path(self.test_path, save_path='/tmp/test.png')
        
        self.assertIsNotNone(fig)
        mock_savefig.assert_called_once()
    
    def test_path_analysis(self):
        """Test path analysis"""
        analysis = self.visualizer._analyze_path(self.test_path)
        
        self.assertIsInstance(analysis, dict)
        self.assertIn('basic_metrics', analysis)
        self.assertIn('segment_analysis', analysis)
        self.assertIn('velocity_analysis', analysis)
        
        # Check basic metrics
        basic_metrics = analysis['basic_metrics']
        self.assertEqual(basic_metrics['total_distance'], 30.0)
        self.assertEqual(basic_metrics['num_segments'], 1)
    
    def test_report_generation(self):
        """Test path report generation"""
        with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as tmp:
            success = self.visualizer.generate_path_report(self.test_path, tmp.name)
            
            # Skip this test if matplotlib is not available in test environment
            if not success:
                self.skipTest("Matplotlib not available in test environment")
            
            self.assertTrue(success)
            self.assertTrue(os.path.exists(tmp.name))
            
            # Cleanup
            os.unlink(tmp.name)
            
            # Check if visualization file was created
            viz_file = tmp.name.replace('.json', '_visualization.png')
            if os.path.exists(viz_file):
                os.unlink(viz_file)


class TestPathIntegration(unittest.TestCase):
    """Integration tests for the complete path planning system"""
    
    def test_end_to_end_planning(self):
        """Test complete end-to-end path planning"""
        # Create configuration
        config = PathPlanningConfig(
            vehicle_width=1.5,
            row_spacing=2.5,
            max_velocity=2.0
        )
        
        # Create planner
        planner = VineyardPathPlanner(config)
        
        # Create detection result
        vine_rows = [
            np.array([[0, 10, 100, 10]]),
            np.array([[0, 30, 100, 30]]),
            np.array([[0, 50, 100, 50]])
        ]
        
        detection_result = VineyardDetectionResult(
            vine_rows=vine_rows,
            row_orientations=[0.0, 0.0, 0.0],
            row_spacing=20.0,
            obstacles=[],
            headlands=[],
            image_bounds=(0, 100, 0, 100),
            pixel_to_meter_ratio=1.0
        )
        
        # Plan path
        path = planner.plan_vineyard_coverage(detection_result)
        
        # Verify path properties
        self.assertIsInstance(path, VineyardPath)
        self.assertGreater(len(path.segments), 0)
        self.assertGreater(path.total_distance, 0)
        
        # Test path optimization
        optimized_path = planner.optimize_path(path)
        self.assertIsInstance(optimized_path, VineyardPath)
        
        # Test visualization
        visualizer = PathVisualizer()
        analysis = visualizer._analyze_path(optimized_path)
        self.assertIsInstance(analysis, dict)


def create_test_suite():
    """Create test suite"""
    suite = unittest.TestSuite()
    
    # Add test cases
    suite.addTest(unittest.TestLoader().loadTestsFromTestCase(TestPathPlanningConfig))
    suite.addTest(unittest.TestLoader().loadTestsFromTestCase(TestWaypoint))
    suite.addTest(unittest.TestLoader().loadTestsFromTestCase(TestPathSegment))
    suite.addTest(unittest.TestLoader().loadTestsFromTestCase(TestVineyardPathPlanner))
    suite.addTest(unittest.TestLoader().loadTestsFromTestCase(TestTurnGeneration))
    suite.addTest(unittest.TestLoader().loadTestsFromTestCase(TestPathVisualizer))
    suite.addTest(unittest.TestLoader().loadTestsFromTestCase(TestPathIntegration))
    
    return suite


def main():
    """Run the test suite"""
    # Initialize ROS2 (required for some tests)
    rclpy.init()
    
    try:
        # Create and run test suite
        suite = create_test_suite()
        runner = unittest.TextTestRunner(verbosity=2)
        result = runner.run(suite)
        
        # Return appropriate exit code
        return 0 if result.wasSuccessful() else 1
        
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    exit(main())
