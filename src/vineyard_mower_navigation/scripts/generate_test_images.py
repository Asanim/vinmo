#!/usr/bin/env python3
"""
Generate sample vineyard images for testing costmap generation
"""

import cv2
import numpy as np
import os
import argparse
from typing import Tuple


def generate_simple_vineyard(width: int = 800, height: int = 600, 
                           num_rows: int = 6, row_spacing: int = 80) -> np.ndarray:
    """Generate a simple synthetic vineyard image"""
    
    # Create RNG for reproducible generation
    rng = np.random.default_rng(42)
    
    # Create base image with grass-like background
    image = np.zeros((height, width, 3), dtype=np.uint8)
    image[:, :] = (34, 139, 34)  # Forest green background
    
    # Add some texture/noise to make it more realistic
    noise = rng.normal(0, 10, (height, width, 3))
    image = np.clip(image.astype(np.float32) + noise, 0, 255).astype(np.uint8)
    
    # Calculate starting position to center the rows
    start_y = (height - (num_rows - 1) * row_spacing) // 2
    
    # Draw vine rows as darker parallel lines
    for i in range(num_rows):
        y = start_y + i * row_spacing
        
        # Main vine row line
        cv2.line(image, (50, y), (width - 50, y), (20, 60, 20), 8)
        
        # Add some variation to make rows look more natural
        for x in range(50, width - 50, 20):
            # Add small perpendicular segments (vine posts)
            cv2.line(image, (x, y - 3), (x, y + 3), (139, 69, 19), 2)
            
            # Add some random vegetation
            if rng.random() > 0.7:
                cv2.circle(image, (x + rng.integers(-5, 5), 
                                 y + rng.integers(-10, 10)), 
                          rng.integers(2, 5), (0, 100, 0), -1)
    
    return image


def generate_complex_vineyard(width: int = 1000, height: int = 800) -> np.ndarray:
    """Generate a more complex vineyard with obstacles and variations"""
    
    # Base vineyard
    image = generate_simple_vineyard(width, height, num_rows=8, row_spacing=90)
    
    # Add buildings/structures (obstacles)
    # Storage building
    cv2.rectangle(image, (100, 100), (200, 180), (120, 120, 120), -1)
    cv2.rectangle(image, (100, 100), (200, 180), (80, 80, 80), 2)
    
    # Equipment shed
    cv2.rectangle(image, (width - 250, height - 200), (width - 100, height - 100), 
                 (100, 100, 100), -1)
    cv2.rectangle(image, (width - 250, height - 200), (width - 100, height - 100), 
                 (60, 60, 60), 2)
    
    # Water tank (circular obstacle)
    cv2.circle(image, (width // 2, 80), 30, (150, 150, 150), -1)
    cv2.circle(image, (width // 2, 80), 30, (100, 100, 100), 2)
    
    # Add some irrigation equipment
    for i in range(3):
        x = 200 + i * 250
        y = height - 50
        cv2.rectangle(image, (x, y), (x + 20, y + 30), (80, 80, 80), -1)
    
    # Add headland areas (lighter colored areas at ends)
    # Top headland
    overlay = image.copy()
    cv2.rectangle(overlay, (0, 0), (width, 80), (100, 160, 100), -1)
    image = cv2.addWeighted(image, 0.7, overlay, 0.3, 0)
    
    # Bottom headland
    overlay = image.copy()
    cv2.rectangle(overlay, (0, height - 80), (width, height), (100, 160, 100), -1)
    image = cv2.addWeighted(image, 0.7, overlay, 0.3, 0)
    
    return image


def generate_irregular_vineyard(width: int = 900, height: int = 700) -> np.ndarray:
    """Generate vineyard with irregular/curved rows"""
    
    # Create RNG for reproducible generation
    rng = np.random.default_rng(42)
    
    # Base image
    image = np.zeros((height, width, 3), dtype=np.uint8)
    image[:, :] = (34, 139, 34)  # Forest green background
    
    # Add noise
    noise = rng.normal(0, 8, (height, width, 3))
    image = np.clip(image.astype(np.float32) + noise, 0, 255).astype(np.uint8)
    
    # Generate curved vine rows
    num_rows = 7
    for i in range(num_rows):
        base_y = 100 + i * 80
        
        points = []
        for x in range(50, width - 50, 10):
            # Add some curvature
            y_offset = int(20 * np.sin(x * 0.01) * (i % 2 * 2 - 1))
            y = base_y + y_offset
            points.append((x, y))
        
        # Draw curved line
        points = np.array(points, dtype=np.int32)
        for j in range(len(points) - 1):
            cv2.line(image, tuple(points[j]), tuple(points[j + 1]), (20, 60, 20), 6)
    
    # Add some obstacles
    cv2.ellipse(image, (300, 200), (40, 25), 45, 0, 360, (100, 100, 100), -1)
    cv2.rectangle(image, (600, 400), (700, 480), (120, 120, 120), -1)
    
    return image


def generate_test_images(output_dir: str):
    """Generate a set of test images"""
    
    os.makedirs(output_dir, exist_ok=True)
    
    # Generate different types of vineyard images
    images = {
        'simple_vineyard.jpg': generate_simple_vineyard(),
        'complex_vineyard.jpg': generate_complex_vineyard(),
        'irregular_vineyard.jpg': generate_irregular_vineyard(),
        'small_vineyard.jpg': generate_simple_vineyard(400, 300, 4, 60),
        'large_vineyard.jpg': generate_complex_vineyard(1200, 1000)
    }
    
    for filename, image in images.items():
        filepath = os.path.join(output_dir, filename)
        cv2.imwrite(filepath, image)
        print(f"Generated: {filepath}")
    
    # Generate a real-world style image (more realistic)
    realistic_image = generate_realistic_vineyard()
    realistic_path = os.path.join(output_dir, 'realistic_vineyard.jpg')
    cv2.imwrite(realistic_path, realistic_image)
    print(f"Generated: {realistic_path}")


def generate_realistic_vineyard(width: int = 1024, height: int = 768) -> np.ndarray:
    """Generate more realistic looking vineyard from aerial perspective"""
    
    # Create RNG for reproducible generation
    rng = np.random.default_rng(42)
    
    # Create base terrain
    image = np.zeros((height, width, 3), dtype=np.uint8)
    
    # Create varied ground colors (soil/grass mix)
    for y in range(height):
        for x in range(width):
            # Vary colors based on position
            base_color = np.array([30 + rng.integers(-10, 10),
                                 100 + rng.integers(-20, 20), 
                                 30 + rng.integers(-10, 10)])
            image[y, x] = np.clip(base_color, 0, 255)
    
    # Add some Gaussian blur for smoother terrain
    image = cv2.GaussianBlur(image, (5, 5), 0)
    
    # Draw vine rows with more realistic appearance
    num_rows = 12
    row_spacing = 55
    start_y = 60
    
    for i in range(num_rows):
        y = start_y + i * row_spacing
        
        # Draw vine canopy (wider, darker green areas)
        for x in range(80, width - 80, 5):
            # Add some randomness to vine density
            if rng.random() > 0.1:
                # Main vine canopy
                cv2.ellipse(image, (x, y), 
                           (rng.integers(8, 15), rng.integers(3, 6)),
                           rng.integers(-10, 10), 0, 360,
                           (10 + rng.integers(0, 30), 
                            80 + rng.integers(0, 40), 
                            10 + rng.integers(0, 20)), -1)
        
        # Draw trellis posts
        for x in range(100, width - 100, 40):
            cv2.line(image, (x, y - 8), (x, y + 8), (101, 67, 33), 2)
    
    # Add realistic obstacles
    # Equipment shed
    shed_corners = np.array([[200, 150], [280, 150], [285, 200], [195, 200]], dtype=np.int32)
    cv2.fillPoly(image, [shed_corners], (120, 120, 140))
    cv2.polylines(image, [shed_corners], True, (80, 80, 80), 2)
    
    # Water tank
    cv2.circle(image, (width - 150, 100), 25, (180, 180, 180), -1)
    cv2.circle(image, (width - 150, 100), 25, (140, 140, 140), 2)
    
    # Road/path
    cv2.rectangle(image, (0, height - 40), (width, height), (160, 160, 160), -1)
    
    # Add some shadows and highlights for depth
    shadow_overlay = image.copy()
    cv2.rectangle(shadow_overlay, (0, 0), (width, height), (0, 0, 0), -1)
    
    # Apply shadows near obstacles
    cv2.circle(shadow_overlay, (width - 150, 125), 30, (255, 255, 255), -1)
    cv2.rectangle(shadow_overlay, (195, 200), (290, 220), (255, 255, 255), -1)
    
    # Blend shadows
    image = cv2.addWeighted(image, 0.9, shadow_overlay, 0.1, 0)
    
    return image


def main():
    """Main function with command line interface"""
    parser = argparse.ArgumentParser(description='Generate test vineyard images')
    parser.add_argument('--output-dir', '-o', default='/tmp/test_vineyard_images',
                       help='Output directory for generated images')
    parser.add_argument('--type', choices=['simple', 'complex', 'irregular', 'realistic', 'all'],
                       default='all', help='Type of vineyard image to generate')
    parser.add_argument('--width', type=int, default=800, help='Image width')
    parser.add_argument('--height', type=int, default=600, help='Image height')
    
    args = parser.parse_args()
    
    os.makedirs(args.output_dir, exist_ok=True)
    
    if args.type == 'all':
        generate_test_images(args.output_dir)
    else:
        if args.type == 'simple':
            image = generate_simple_vineyard(args.width, args.height)
        elif args.type == 'complex':
            image = generate_complex_vineyard(args.width, args.height)
        elif args.type == 'irregular':
            image = generate_irregular_vineyard(args.width, args.height)
        elif args.type == 'realistic':
            image = generate_realistic_vineyard(args.width, args.height)
        
        filename = f'{args.type}_vineyard.jpg'
        filepath = os.path.join(args.output_dir, filename)
        cv2.imwrite(filepath, image)
        print(f"Generated: {filepath}")
    
    print(f"Test images saved to: {args.output_dir}")


if __name__ == '__main__':
    main()
