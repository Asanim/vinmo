#!/usr/bin/env python3
"""
Generate a terrain heightmap for vineyard simulation
Creates a simple heightmap with subtle variations suitable for vineyard terrain
"""

import numpy as np
from PIL import Image
import os

def generate_vineyard_terrain(width=512, height=512, output_path="terrain.png"):
    """Generate a realistic vineyard terrain heightmap"""
    
    # Create base terrain with subtle variations
    terrain = np.random.normal(128, 10, (height, width))
    
    # Add gentle slopes and undulations
    x = np.linspace(0, 4*np.pi, width)
    y = np.linspace(0, 4*np.pi, height)
    X, Y = np.meshgrid(x, y)
    
    # Add large-scale undulations
    undulation = 20 * np.sin(X/2) * np.cos(Y/2)
    terrain += undulation
    
    # Add smaller scale variations
    small_variations = 5 * np.sin(X*2) * np.cos(Y*2)
    terrain += small_variations
    
    # Create drainage patterns (subtle channels)
    for i in range(3):
        center_x = np.random.randint(width//4, 3*width//4)
        center_y = np.random.randint(height//4, 3*height//4)
        
        # Create a subtle channel
        for y in range(height):
            for x in range(width):
                dist_to_channel = abs(x - center_x - (y - center_y) * 0.1)
                if dist_to_channel < 20:
                    terrain[y, x] -= 3 * (20 - dist_to_channel) / 20
    
    # Ensure values are in valid range
    terrain = np.clip(terrain, 0, 255)
    
    # Convert to grayscale image
    terrain_image = Image.fromarray(terrain.astype(np.uint8), mode='L')
    terrain_image.save(output_path)
    print(f"Terrain heightmap saved to {output_path}")

if __name__ == "__main__":
    # Generate terrain for vineyard
    script_dir = os.path.dirname(os.path.abspath(__file__))
    models_dir = os.path.join(script_dir, "..", "models")
    output_path = os.path.join(models_dir, "terrain.png")
    
    generate_vineyard_terrain(output_path=output_path)
