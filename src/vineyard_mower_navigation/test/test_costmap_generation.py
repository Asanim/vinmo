#!/usr/bin/env python3
"""
Test suite for costmap generation system
"""

import unittest
import numpy as np
import cv2
import os
import tempfile
import yaml
from unittest.mock import patch, MagicMock

import sys
import rclpy
from rclpy.node import Node

# Add the module path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from vineyard_mower_navigation.satellite_processor import (
    SatelliteImageProcessor, VineyardDetector, VineyardDetectionResult
)
from vineyard_mower_navigation.costmap_generator import (
    CostmapGenerator, CostmapConfig, CostmapLayer
)


class TestSatelliteProcessor(unittest.TestCase):
    """Test satellite imagery processing"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.processor = SatelliteImageProcessor()
        self.test_image = self.create_test_image()
    
    def create_test_image(self) -> np.ndarray:
        """Create a synthetic test image with vine rows"""
        image = np.zeros((400, 600, 3), dtype=np.uint8)
        
        # Fill with grass-like color
        image[:, :] = (34, 139, 34)  # Forest green
        
        # Draw vine rows (darker lines)
        for i in range(5):
            y = 50 + i * 70
            cv2.line(image, (0, y), (600, y), (20, 60, 20), 10)
        
        # Add some obstacles (buildings/equipment)
        cv2.rectangle(image, (100, 100), (150, 150), (100, 100, 100), -1)
        cv2.rectangle(image, (450, 200), (500, 250), (80, 80, 80), -1)
        
        return image
    
    def test_load_local_image(self):
        """Test loading local image"""
        # Create temporary image file
        with tempfile.NamedTemporaryFile(suffix='.jpg', delete=False) as tmp:
            cv2.imwrite(tmp.name, self.test_image)
            
            # Test loading
            loaded_image = self.processor.load_local_image(tmp.name)
            self.assertIsNotNone(loaded_image)
            self.assertEqual(loaded_image.shape, self.test_image.shape)
            
            # Cleanup
            os.unlink(tmp.name)
    
    def test_preprocess_image(self):
        """Test image preprocessing"""
        processed = self.processor.preprocess_image(self.test_image)
        
        # Check that preprocessing returns valid image
        self.assertIsNotNone(processed)
        self.assertEqual(processed.shape, self.test_image.shape)
        self.assertEqual(processed.dtype, np.uint8)
    
    @patch('requests.get')
    def test_fetch_satellite_image(self, mock_get):
        """Test satellite image fetching"""
        # Mock API response
        mock_response = MagicMock()
        mock_response.content = cv2.imencode('.jpg', self.test_image)[1].tobytes()
        mock_response.raise_for_status.return_value = None
        mock_get.return_value = mock_response
        
        # Set API key
        self.processor.api_key = "test_key"
        
        # Test fetching
        image = self.processor.fetch_satellite_image(37.7749, -122.4194)
        self.assertIsNotNone(image)
        self.assertEqual(len(image.shape), 3)


class TestVineyardDetector(unittest.TestCase):
    """Test vineyard structure detection"""
    
    def setUp(self):
        """Set up test fixtures"""
        config = {
            'pixel_to_meter_ratio': 0.1,
            'hough_threshold': 30,
            'hough_min_line_length': 50
        }
        self.detector = VineyardDetector(config)
        self.test_image = self.create_test_vineyard_image()
    
    def create_test_vineyard_image(self) -> np.ndarray:
        """Create synthetic vineyard image"""
        image = np.ones((300, 400, 3), dtype=np.uint8) * 100  # Gray background
        
        # Draw parallel vine rows
        for i in range(4):
            y = 50 + i * 60
            cv2.line(image, (20, y), (380, y), (0, 0, 0), 3)
        
        return image
    
    def test_detect_vine_rows(self):
        """Test vine row detection"""
        result = self.detector.detect_vine_rows(self.test_image)
        
        # Check result structure
        self.assertIsInstance(result, VineyardDetectionResult)
        self.assertIsInstance(result.vine_rows, list)
        self.assertIsInstance(result.row_orientations, list)
        self.assertIsInstance(result.row_spacing, float)
        
        # Should detect some vine rows
        self.assertGreater(len(result.vine_rows), 0)
    
    def test_visualization(self):
        """Test detection visualization"""
        result = self.detector.detect_vine_rows(self.test_image)
        
        with tempfile.NamedTemporaryFile(suffix='.jpg', delete=False) as tmp:
            vis_image = self.detector.visualize_detection(
                self.test_image, result, tmp.name)
            
            # Check visualization was created
            self.assertTrue(os.path.exists(tmp.name))
            self.assertEqual(vis_image.shape, self.test_image.shape)
            
            # Cleanup
            os.unlink(tmp.name)


class TestCostmapGenerator(unittest.TestCase):
    """Test costmap generation"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.config = CostmapConfig(
            resolution=0.1,
            width=200,
            height=200
        )
        self.generator = CostmapGenerator(self.config)
        self.detection_result = self.create_test_detection_result()
    
    def create_test_detection_result(self) -> VineyardDetectionResult:
        """Create test detection result"""
        # Create some test vine rows
        vine_rows = [
            np.array([[10, 20, 190, 25], [12, 80, 188, 85]]),
            np.array([[10, 100, 190, 105], [12, 160, 188, 165]])
        ]
        
        # Create test obstacles
        obstacles = [
            np.array([[50, 50], [70, 50], [70, 70], [50, 70]])
        ]
        
        return VineyardDetectionResult(
            vine_rows=vine_rows,
            row_orientations=[0.0, 0.0],
            row_spacing=8.0,
            obstacles=obstacles,
            headlands=[],
            image_bounds=(0, 200, 0, 200),
            pixel_to_meter_ratio=0.1
        )
    
    def test_layer_initialization(self):
        """Test costmap layer initialization"""
        self.assertIn('static', self.generator.layers)
        self.assertIn('vine_rows', self.generator.layers)
        self.assertIn('obstacles', self.generator.layers)
        self.assertIn('headlands', self.generator.layers)
        self.assertIn('inflation', self.generator.layers)
    
    def test_costmap_generation(self):
        """Test costmap generation from detection result"""
        costmap = self.generator.generate_from_detection(self.detection_result)
        
        # Check costmap structure
        self.assertEqual(costmap.info.width, self.config.width)
        self.assertEqual(costmap.info.height, self.config.height)
        self.assertEqual(costmap.info.resolution, self.config.resolution)
        self.assertEqual(len(costmap.data), self.config.width * self.config.height)
    
    def test_layer_update(self):
        """Test updating individual layers"""
        # Create test layer data
        rng = np.random.default_rng(42)  # Fixed seed for reproducible tests
        test_data = rng.integers(0, 100, (self.config.height, self.config.width), dtype=np.uint8)
        
        # Update layer
        self.generator.update_layer('static', test_data)
        
        # Check layer was updated
        layer_data = self.generator.get_layer('static')
        np.testing.assert_array_equal(layer_data, test_data)
    
    def test_config_loading(self):
        """Test configuration loading from file"""
        config_data = {
            'resolution': 0.2,
            'width': 500,
            'height': 500,
            'vine_row_cost': 95
        }
        
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as tmp:
            yaml.dump(config_data, tmp)
            
            # Load configuration
            success = self.generator.load_config_from_file(tmp.name)
            self.assertTrue(success)
            
            # Check configuration was updated
            self.assertEqual(self.generator.config.resolution, 0.2)
            self.assertEqual(self.generator.config.vine_row_cost, 95)
            
            # Cleanup
            os.unlink(tmp.name)


class TestCostmapLayer(unittest.TestCase):
    """Test costmap layer functionality"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.config = CostmapConfig(width=100, height=100)
        self.layer = CostmapLayer('test_layer', self.config)
    
    def test_layer_initialization(self):
        """Test layer initialization"""
        self.assertEqual(self.layer.name, 'test_layer')
        self.assertEqual(self.layer.data.shape, (100, 100))
        self.assertEqual(self.layer.data.dtype, np.uint8)
        self.assertTrue(np.all(self.layer.data == 0))
    
    def test_data_update(self):
        """Test layer data update"""
        rng = np.random.default_rng(42)
        test_data = rng.integers(0, 255, (100, 100))
        
        self.layer.update_data(test_data)
        
        np.testing.assert_array_equal(self.layer.data, test_data.astype(np.uint8))
    
    def test_clear_layer(self):
        """Test layer clearing"""
        # Fill with random data
        rng = np.random.default_rng(42)
        test_data = rng.integers(0, 255, (100, 100))
        self.layer.update_data(test_data)
        
        # Clear
        self.layer.clear()
        
        # Check all zeros
        self.assertTrue(np.all(self.layer.data == 0))


class TestIntegration(unittest.TestCase):
    """Integration tests for the complete system"""
    
    def test_end_to_end_processing(self):
        """Test complete pipeline from image to costmap"""
        # Create test image
        test_image = np.ones((200, 300, 3), dtype=np.uint8) * 128
        
        # Draw some vine rows
        for i in range(3):
            y = 50 + i * 50
            cv2.line(test_image, (10, y), (290, y), (0, 0, 0), 2)
        
        # Initialize components
        detector = VineyardDetector({'pixel_to_meter_ratio': 0.1})
        config = CostmapConfig(width=300, height=200, resolution=0.1)
        generator = CostmapGenerator(config)
        
        # Process image
        detection_result = detector.detect_vine_rows(test_image)
        costmap = generator.generate_from_detection(detection_result)
        
        # Verify results
        self.assertIsNotNone(costmap)
        self.assertEqual(costmap.info.width, 300)
        self.assertEqual(costmap.info.height, 200)
        self.assertGreater(len([x for x in costmap.data if x > 0]), 0)  # Some non-zero costs


def create_test_suite():
    """Create test suite"""
    suite = unittest.TestSuite()
    
    # Add test cases
    suite.addTest(unittest.TestLoader().loadTestsFromTestCase(TestSatelliteProcessor))
    suite.addTest(unittest.TestLoader().loadTestsFromTestCase(TestVineyardDetector))
    suite.addTest(unittest.TestLoader().loadTestsFromTestCase(TestCostmapGenerator))
    suite.addTest(unittest.TestLoader().loadTestsFromTestCase(TestCostmapLayer))
    suite.addTest(unittest.TestLoader().loadTestsFromTestCase(TestIntegration))
    
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
