#!/usr/bin/env python3

"""
Simple heightmap generator for vineyard terrain
Creates a basic heightmap texture for Gazebo terrain.
"""

import numpy as np
from PIL import Image
import os

def create_heightmap(width=512, height=512, output_file="terrain.png"):
    """Create a simple heightmap with gentle variations"""
    # Create base terrain with gentle slopes
    heightmap = np.zeros((height, width), dtype=np.uint8)
    
    # Add some gentle undulations
    x = np.linspace(0, 2*np.pi, width)
    y = np.linspace(0, 2*np.pi, height)
    X, Y = np.meshgrid(x, y)
    
    # Create gentle rolling terrain
    terrain = 128 + 20 * np.sin(X/2) * np.cos(Y/3) + 10 * np.sin(X*2) * np.sin(Y*2)
    
    # Clamp values to valid range
    terrain = np.clip(terrain, 0, 255).astype(np.uint8)
    
    # Create PIL image and save
    img = Image.fromarray(terrain, mode='L')
    img.save(output_file)
    print(f"Created heightmap: {output_file}")

if __name__ == "__main__":
    # Create heightmap in models directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    models_dir = os.path.join(os.path.dirname(script_dir), "models")
    output_path = os.path.join(models_dir, "terrain.png")
    
    try:
        create_heightmap(output_file=output_path)
    except ImportError:
        print("PIL/Pillow not available. Creating simple placeholder...")
        # Create a simple flat terrain file as placeholder
        with open(output_path.replace('.png', '.txt'), 'w') as f:
            f.write("# Placeholder terrain file\n")
            f.write("# Install PIL/Pillow to generate actual heightmap\n")
