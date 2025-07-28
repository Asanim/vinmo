#!/usr/bin/env python3
"""
Satellite Imagery Processing Module

This module handles satellite imagery acquisition, preprocessing,
and vineyard structure detection from satellite/aerial imagery.
"""

import cv2
import numpy as np
import requests
import os
from typing import Tuple, List, Optional, Dict, Any
import logging
from dataclasses import dataclass
from urllib.parse import urlencode
import base64
from io import BytesIO
from PIL import Image
import json

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


@dataclass
class VineyardDetectionResult:
    """Results from vineyard structure detection"""
    vine_rows: List[np.ndarray]  # Line segments representing vine rows
    row_orientations: List[float]  # Orientation angles in radians
    row_spacing: float  # Average spacing between rows in meters
    obstacles: List[np.ndarray]  # Detected obstacle regions
    headlands: List[np.ndarray]  # Turning/headland areas
    image_bounds: Tuple[float, float, float, float]  # lat_min, lat_max, lon_min, lon_max
    pixel_to_meter_ratio: float  # Conversion factor


class SatelliteImageProcessor:
    """
    Handles satellite imagery acquisition and preprocessing
    """
    
    def __init__(self, api_key: Optional[str] = None):
        """
        Initialize the satellite image processor
        
        Args:
            api_key: Google Maps API key for satellite imagery
        """
        self.api_key = api_key or os.getenv('GOOGLE_MAPS_API_KEY')
        self.base_url = "https://maps.googleapis.com/maps/api/staticmap"
        
    def fetch_satellite_image(self, 
                            center_lat: float, 
                            center_lon: float,
                            zoom: int = 18,
                            size: Tuple[int, int] = (640, 640),
                            maptype: str = "satellite") -> Optional[np.ndarray]:
        """
        Fetch satellite image from Google Maps API
        
        Args:
            center_lat: Center latitude
            center_lon: Center longitude  
            zoom: Zoom level (1-20)
            size: Image size in pixels
            maptype: Map type (satellite, hybrid, etc.)
            
        Returns:
            numpy array of the image or None if failed
        """
        if not self.api_key:
            logger.error("Google Maps API key not provided")
            return None
            
        params = {
            'center': f"{center_lat},{center_lon}",
            'zoom': zoom,
            'size': f"{size[0]}x{size[1]}",
            'maptype': maptype,
            'key': self.api_key
        }
        
        url = f"{self.base_url}?{urlencode(params)}"
        
        try:
            response = requests.get(url, timeout=30)
            response.raise_for_status()
            
            # Convert to OpenCV format
            image = Image.open(BytesIO(response.content))
            image_array = np.array(image)
            
            # Convert RGB to BGR for OpenCV
            if len(image_array.shape) == 3:
                image_array = cv2.cvtColor(image_array, cv2.COLOR_RGB2BGR)
                
            logger.info(f"Successfully fetched satellite image: {image_array.shape}")
            return image_array
            
        except Exception as e:
            logger.error(f"Failed to fetch satellite image: {e}")
            return None
    
    def load_local_image(self, image_path: str) -> Optional[np.ndarray]:
        """
        Load local satellite/aerial image
        
        Args:
            image_path: Path to the image file
            
        Returns:
            numpy array of the image or None if failed
        """
        try:
            image = cv2.imread(image_path)
            if image is None:
                logger.error(f"Could not load image from {image_path}")
                return None
            logger.info(f"Successfully loaded local image: {image.shape}")
            return image
        except Exception as e:
            logger.error(f"Error loading image: {e}")
            return None
    
    def preprocess_image(self, image: np.ndarray, 
                        enhance_contrast: bool = True,
                        gaussian_blur: bool = True,
                        bilateral_filter: bool = True) -> np.ndarray:
        """
        Preprocess satellite image for better feature detection
        
        Args:
            image: Input image
            enhance_contrast: Apply CLAHE contrast enhancement
            gaussian_blur: Apply Gaussian blur for noise reduction
            bilateral_filter: Apply bilateral filter for edge preservation
            
        Returns:
            Preprocessed image
        """
        processed = image.copy()
        
        # Convert to LAB color space for better contrast enhancement
        if len(processed.shape) == 3:
            lab = cv2.cvtColor(processed, cv2.COLOR_BGR2LAB)
            
            if enhance_contrast:
                # Apply CLAHE to L channel
                clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
                lab[:, :, 0] = clahe.apply(lab[:, :, 0])
                
            processed = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
        
        if bilateral_filter:
            # Bilateral filter for noise reduction while preserving edges
            processed = cv2.bilateralFilter(processed, 9, 75, 75)
            
        if gaussian_blur:
            # Light Gaussian blur for final smoothing
            processed = cv2.GaussianBlur(processed, (3, 3), 0)
            
        return processed


class VineyardDetector:
    """
    Computer vision algorithms for vineyard structure detection
    """
    
    def __init__(self, config: Optional[Dict[str, Any]] = None):
        """
        Initialize vineyard detector with configuration
        
        Args:
            config: Detection parameters configuration
        """
        default_config = {
            'hough_threshold': 50,
            'hough_min_line_length': 100,
            'hough_max_line_gap': 20,
            'canny_low_threshold': 50,
            'canny_high_threshold': 150,
            'morphology_kernel_size': 5,
            'min_row_length': 50,  # meters
            'max_row_spacing': 5.0,  # meters
            'angle_tolerance': 0.17,  # ~10 degrees in radians
            'pixel_to_meter_ratio': 0.1  # Default ratio, should be calibrated
        }
        
        self.config = {**default_config, **(config or {})}
        logger.info(f"Initialized VineyardDetector with config: {self.config}")
    
    def detect_vine_rows(self, image: np.ndarray) -> VineyardDetectionResult:
        """
        Detect vineyard structure from preprocessed satellite image
        
        Args:
            image: Preprocessed satellite image
            
        Returns:
            VineyardDetectionResult with detected features
        """
        # Convert to grayscale for edge detection
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image.copy()
            
        # Edge detection
        edges = cv2.Canny(gray, 
                         self.config['canny_low_threshold'],
                         self.config['canny_high_threshold'])
        
        # Morphological operations to connect broken lines
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, 
                                         (self.config['morphology_kernel_size'],
                                          self.config['morphology_kernel_size']))
        edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
        
        # Hough line detection
        lines = cv2.HoughLinesP(edges,
                               rho=1,
                               theta=np.pi/180,
                               threshold=self.config['hough_threshold'],
                               minLineLength=self.config['hough_min_line_length'],
                               maxLineGap=self.config['hough_max_line_gap'])
        
        if lines is None:
            logger.warning("No lines detected in image")
            return self._empty_detection_result(image.shape)
        
        # Filter and group lines into vine rows
        vine_rows = self._group_parallel_lines(lines)
        
        # Calculate row orientations and spacing
        orientations = self._calculate_orientations(vine_rows)
        row_spacing = self._calculate_row_spacing(vine_rows)
        
        # Detect obstacles and headlands
        obstacles = self._detect_obstacles(image, vine_rows)
        headlands = self._detect_headlands(image, vine_rows)
        
        return VineyardDetectionResult(
            vine_rows=vine_rows,
            row_orientations=orientations,
            row_spacing=row_spacing,
            obstacles=obstacles,
            headlands=headlands,
            image_bounds=(0, image.shape[0], 0, image.shape[1]),  # Pixel coordinates
            pixel_to_meter_ratio=self.config['pixel_to_meter_ratio']
        )
    
    def _empty_detection_result(self, image_shape: Tuple[int, ...]) -> VineyardDetectionResult:
        """Create empty detection result"""
        return VineyardDetectionResult(
            vine_rows=[],
            row_orientations=[],
            row_spacing=0.0,
            obstacles=[],
            headlands=[],
            image_bounds=(0, image_shape[0], 0, image_shape[1]),
            pixel_to_meter_ratio=self.config['pixel_to_meter_ratio']
        )
    
    def _group_parallel_lines(self, lines: np.ndarray) -> List[np.ndarray]:
        """
        Group parallel lines that likely represent vine rows
        
        Args:
            lines: Array of detected lines from HoughLinesP
            
        Returns:
            List of grouped line segments representing vine rows
        """
        if len(lines) == 0:
            return []
            
        # Calculate angles for all lines
        angles = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            angle = np.arctan2(y2 - y1, x2 - x1)
            angles.append(angle)
        
        angles = np.array(angles)
        
        # Group lines by similar angles
        grouped_lines = []
        used = np.zeros(len(lines), dtype=bool)
        
        for i, angle in enumerate(angles):
            if used[i]:
                continue
                
            # Find lines with similar angles
            angle_diff = np.abs(angles - angle)
            angle_diff = np.minimum(angle_diff, np.pi - angle_diff)  # Handle angle wraparound
            similar_lines = angle_diff < self.config['angle_tolerance']
            
            if np.sum(similar_lines) >= 2:  # At least 2 lines to form a group
                group_lines = lines[similar_lines]
                grouped_lines.append(group_lines.reshape(-1, 4))
                used[similar_lines] = True
        
        return grouped_lines
    
    def _calculate_orientations(self, vine_rows: List[np.ndarray]) -> List[float]:
        """Calculate orientation angles for each vine row group"""
        orientations = []
        
        for row_group in vine_rows:
            angles = []
            for line in row_group:
                x1, y1, x2, y2 = line
                angle = np.arctan2(y2 - y1, x2 - x1)
                angles.append(angle)
            
            # Use median angle for robustness
            median_angle = np.median(angles)
            orientations.append(median_angle)
        
        return orientations
    
    def _calculate_row_spacing(self, vine_rows: List[np.ndarray]) -> float:
        """Calculate average spacing between vine rows"""
        if len(vine_rows) < 2:
            return 0.0
        
        # Calculate perpendicular distances between parallel row groups
        spacings = []
        
        for i in range(len(vine_rows)):
            for j in range(i + 1, len(vine_rows)):
                # Get representative points from each row group
                row1_center = self._get_row_center_line(vine_rows[i])
                row2_center = self._get_row_center_line(vine_rows[j])
                
                if row1_center is not None and row2_center is not None:
                    # Calculate perpendicular distance
                    distance = self._perpendicular_distance(row1_center, row2_center)
                    spacings.append(distance * self.config['pixel_to_meter_ratio'])
        
        return np.mean(spacings) if spacings else 0.0
    
    def _get_row_center_line(self, row_lines: np.ndarray) -> Optional[np.ndarray]:
        """Get center line representing a vine row group"""
        if len(row_lines) == 0:
            return None
        
        # Calculate centroid of all line segments
        all_points = row_lines.reshape(-1, 2)
        centroid = np.mean(all_points, axis=0)
        
        # Use the longest line in the group as reference
        lengths = []
        for line in row_lines:
            x1, y1, x2, y2 = line
            length = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            lengths.append(length)
        
        longest_line = row_lines[np.argmax(lengths)]
        return longest_line
    
    def _perpendicular_distance(self, line1: np.ndarray, line2: np.ndarray) -> float:
        """Calculate perpendicular distance between two lines"""
        x1, y1, x2, y2 = line1
        x3, y3, x4, y4 = line2
        
        # Get midpoints
        mid1 = np.array([(x1 + x2) / 2, (y1 + y2) / 2])
        mid2 = np.array([(x3 + x4) / 2, (y3 + y4) / 2])
        
        # Calculate distance between midpoints
        return np.linalg.norm(mid2 - mid1)
    
    def _detect_obstacles(self, image: np.ndarray, vine_rows: List[np.ndarray]) -> List[np.ndarray]:
        """
        Detect obstacles and non-navigable areas
        
        Args:
            image: Input image
            vine_rows: Detected vine rows
            
        Returns:
            List of obstacle regions (contours)
        """
        # Convert to HSV for better color segmentation
        if len(image.shape) == 3:
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        else:
            hsv = cv2.cvtColor(image, cv2.COLOR_GRAY2HSV)
        
        # Define color ranges for obstacles (buildings, equipment, etc.)
        # These ranges may need tuning based on specific imagery
        lower_obstacle = np.array([0, 0, 0])     # Dark objects
        upper_obstacle = np.array([180, 255, 100])
        
        mask = cv2.inRange(hsv, lower_obstacle, upper_obstacle)
        
        # Morphological operations to clean up the mask
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Filter contours by size
        min_area = 100  # Minimum obstacle area in pixels
        obstacles = [cnt for cnt in contours if cv2.contourArea(cnt) > min_area]
        
        return obstacles
    
    def _detect_headlands(self, image: np.ndarray, vine_rows: List[np.ndarray]) -> List[np.ndarray]:
        """
        Detect headland/turning areas at the ends of vine rows
        
        Args:
            image: Input image
            vine_rows: Detected vine rows
            
        Returns:
            List of headland regions
        """
        headlands = []
        
        if not vine_rows:
            return headlands
        
        # Get image dimensions
        height, width = image.shape[:2]
        
        # For each vine row group, detect areas at the ends
        for row_group in vine_rows:
            # Find the extremes of the row group
            all_points = row_group.reshape(-1, 2)
            
            # Get bounding box
            min_x, min_y = np.min(all_points, axis=0)
            max_x, max_y = np.max(all_points, axis=0)
            
            # Define headland regions at both ends
            headland_width = min(50, width // 10)  # Adaptive width
            
            # Top headland
            if min_y > headland_width:
                top_headland = np.array([
                    [min_x, 0],
                    [max_x, 0], 
                    [max_x, min_y],
                    [min_x, min_y]
                ])
                headlands.append(top_headland)
            
            # Bottom headland  
            if max_y < height - headland_width:
                bottom_headland = np.array([
                    [min_x, max_y],
                    [max_x, max_y],
                    [max_x, height],
                    [min_x, height]
                ])
                headlands.append(bottom_headland)
        
        return headlands
    
    def visualize_detection(self, image: np.ndarray, 
                          result: VineyardDetectionResult,
                          save_path: Optional[str] = None) -> np.ndarray:
        """
        Visualize detection results on the image
        
        Args:
            image: Original image
            result: Detection results
            save_path: Optional path to save the visualization
            
        Returns:
            Annotated image
        """
        vis_image = image.copy()
        
        # Draw vine rows
        colors = [(0, 255, 0), (255, 0, 0), (0, 0, 255), (255, 255, 0), (255, 0, 255)]
        
        for i, row_group in enumerate(result.vine_rows):
            color = colors[i % len(colors)]
            for line in row_group:
                x1, y1, x2, y2 = line
                cv2.line(vis_image, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
        
        # Draw obstacles
        for obstacle in result.obstacles:
            cv2.drawContours(vis_image, [obstacle], -1, (0, 0, 255), 2)
        
        # Draw headlands
        for headland in result.headlands:
            cv2.drawContours(vis_image, [headland], -1, (255, 255, 0), 2)
        
        # Add text information
        info_text = [
            f"Rows detected: {len(result.vine_rows)}",
            f"Row spacing: {result.row_spacing:.2f}m",
            f"Obstacles: {len(result.obstacles)}",
            f"Headlands: {len(result.headlands)}"
        ]
        
        for i, text in enumerate(info_text):
            cv2.putText(vis_image, text, (10, 30 + i * 25), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        if save_path:
            cv2.imwrite(save_path, vis_image)
            logger.info(f"Visualization saved to {save_path}")
        
        return vis_image


def main():
    """Test the satellite imagery processing"""
    # Initialize processor
    processor = SatelliteImageProcessor()
    detector = VineyardDetector()
    
    # Test with a local image (you would need to provide this)
    test_image_path = "/path/to/test/vineyard/image.jpg"
    
    if os.path.exists(test_image_path):
        # Load and process image
        image = processor.load_local_image(test_image_path)
        if image is not None:
            processed_image = processor.preprocess_image(image)
            
            # Detect vineyard structure
            result = detector.detect_vine_rows(processed_image)
            
            # Visualize results
            vis_image = detector.visualize_detection(processed_image, result, 
                                                   "/tmp/vineyard_detection.jpg")
            
            logger.info(f"Detection complete: {len(result.vine_rows)} vine rows detected")
    else:
        logger.info("Test image not found. Please provide a test vineyard image.")


if __name__ == "__main__":
    main()
