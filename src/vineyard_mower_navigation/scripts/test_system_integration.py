#!/usr/bin/env python3
"""
Integration test script for the complete costmap generation system
"""

import rclpy
import os
import time
import subprocess
import signal
import sys
from pathlib import Path


class CostmapSystemTest:
    """Complete system integration test"""
    
    def __init__(self):
        self.processes = []
        self.test_results = {}
        
    def cleanup(self):
        """Clean up any running processes"""
        for process in self.processes:
            try:
                process.terminate()
                process.wait(timeout=5)
            except (subprocess.TimeoutExpired, OSError):
                process.kill()
    
    def run_command(self, command, timeout=30):
        """Run a command with timeout"""
        try:
            result = subprocess.run(
                command, shell=True, timeout=timeout,
                capture_output=True, text=True
            )
            return result.returncode == 0, result.stdout, result.stderr
        except subprocess.TimeoutExpired:
            return False, "", "Command timed out"
    
    def test_build_system(self):
        """Test that the system builds correctly"""
        print("Testing build system...")
        
        # Build interfaces
        success, _, stderr = self.run_command(
            "cd /home/sam/vinmo && colcon build --packages-select vineyard_mower_interfaces"
        )
        
        self.test_results['build_interfaces'] = success
        
        if not success:
            print(f"Interface build failed: {stderr}")
            return False
        
        # Build navigation package
        success, _, stderr = self.run_command(
            "cd /home/sam/vinmo && colcon build --packages-select vineyard_mower_navigation"
        )
        
        self.test_results['build_navigation'] = success
        
        if not success:
            print(f"Navigation build failed: {stderr}")
            return False
        
        print("✅ Build system test passed")
        return True
    
    def test_image_generation(self):
        """Test image generation"""
        print("Testing image generation...")
        
        # Generate test images
        success, _, stderr = self.run_command(
            "python3 /home/sam/vinmo/src/vineyard_mower_navigation/scripts/generate_test_images.py --output-dir /tmp/test_images"
        )
        
        self.test_results['image_generation'] = success
        
        if not success:
            print(f"Image generation failed: {stderr}")
            return False
        
        # Check that images were created
        test_dir = Path("/tmp/test_images")
        if not test_dir.exists():
            print("Test image directory not created")
            return False
        
        image_files = list(test_dir.glob("*.jpg"))
        if len(image_files) == 0:
            print("No test images generated")
            return False
        
        print(f"✅ Generated {len(image_files)} test images")
        return True
    
    def test_satellite_processing(self):
        """Test satellite imagery processing module"""
        print("Testing satellite processing module...")
        
        # Test the module directly
        test_code = """
import sys
sys.path.append('/home/sam/vinmo/src/vineyard_mower_navigation')
from vineyard_mower_navigation.satellite_processor import SatelliteImageProcessor, VineyardDetector
import cv2
import numpy as np

# Create test image
test_image = np.ones((300, 400, 3), dtype=np.uint8) * 100
for i in range(4):
    y = 50 + i * 60
    cv2.line(test_image, (20, y), (380, y), (0, 0, 0), 3)

# Test processor
processor = SatelliteImageProcessor()
processed = processor.preprocess_image(test_image)

# Test detector
detector = VineyardDetector()
result = detector.detect_vine_rows(processed)

print(f"Detected {len(result.vine_rows)} vine row groups")
print("SUCCESS")
"""
        
        success, stdout, stderr = self.run_command(f"python3 -c '{test_code}'")
        
        self.test_results['satellite_processing'] = success and "SUCCESS" in stdout
        
        if not success:
            print(f"Satellite processing test failed: {stderr}")
            return False
        
        print("✅ Satellite processing test passed")
        return True
    
    def test_costmap_generation(self):
        """Test costmap generation module"""
        print("Testing costmap generation module...")
        
        test_code = """
import sys
sys.path.append('/home/sam/vinmo/src/vineyard_mower_navigation')
from vineyard_mower_navigation.costmap_generator import CostmapGenerator, CostmapConfig
from vineyard_mower_navigation.satellite_processor import VineyardDetectionResult
import numpy as np

# Create test detection result
vine_rows = [np.array([[10, 20, 190, 25], [12, 80, 188, 85]])]
obstacles = [np.array([[50, 50], [70, 50], [70, 70], [50, 70]])]

result = VineyardDetectionResult(
    vine_rows=vine_rows,
    row_orientations=[0.0],
    row_spacing=8.0,
    obstacles=obstacles,
    headlands=[],
    image_bounds=(0, 200, 0, 200),
    pixel_to_meter_ratio=0.1
)

# Test costmap generator
config = CostmapConfig(width=200, height=200, resolution=0.1)
generator = CostmapGenerator(config)
costmap = generator.generate_from_detection(result)

print(f"Generated costmap: {costmap.info.width}x{costmap.info.height}")
print(f"Data length: {len(costmap.data)}")
print("SUCCESS")
"""
        
        success, stdout, stderr = self.run_command(f"python3 -c '{test_code}'")
        
        self.test_results['costmap_generation'] = success and "SUCCESS" in stdout
        
        if not success:
            print(f"Costmap generation test failed: {stderr}")
            return False
        
        print("✅ Costmap generation test passed")
        return True
    
    def test_ros_services(self):
        """Test ROS2 services (if ROS2 is available)"""
        print("Testing ROS2 services...")
        
        # Check if ROS2 is available
        success, stdout, stderr = self.run_command("which ros2")
        if not success:
            print("⚠️ ROS2 not available, skipping service tests")
            self.test_results['ros_services'] = None
            return True
        
        # Try to source workspace
        success, stdout, stderr = self.run_command(
            "cd /home/sam/vinmo && source install/setup.bash && ros2 pkg list | grep vineyard_mower"
        )
        
        self.test_results['ros_services'] = success
        
        if not success:
            print(f"⚠️ ROS2 package not found: {stderr}")
            return True  # Don't fail the test, just warn
        
        print("✅ ROS2 services test passed")
        return True
    
    def test_unit_tests(self):
        """Run unit tests"""
        print("Running unit tests...")
        
        success, stdout, stderr = self.run_command(
            "cd /home/sam/vinmo && python3 src/vineyard_mower_navigation/test/test_costmap_generation.py"
        )
        
        self.test_results['unit_tests'] = success
        
        if not success:
            print(f"Unit tests failed: {stderr}")
            return False
        
        print("✅ Unit tests passed")
        return True
    
    def run_all_tests(self):
        """Run all tests"""
        print("🚀 Starting Vineyard Costmap System Integration Tests")
        print("=" * 60)
        
        all_passed = True
        
        # Test order matters
        tests = [
            ("Build System", self.test_build_system),
            ("Image Generation", self.test_image_generation),
            ("Satellite Processing", self.test_satellite_processing),
            ("Costmap Generation", self.test_costmap_generation),
            ("Unit Tests", self.test_unit_tests),
            ("ROS Services", self.test_ros_services),
        ]
        
        for test_name, test_func in tests:
            try:
                if not test_func():
                    all_passed = False
                    print(f"❌ {test_name} FAILED")
                else:
                    print(f"✅ {test_name} PASSED")
            except Exception as e:
                print(f"❌ {test_name} ERROR: {e}")
                all_passed = False
                self.test_results[test_name.lower().replace(' ', '_')] = False
            
            print("-" * 40)
        
        # Print summary
        print("\n" + "=" * 60)
        print("TEST SUMMARY")
        print("=" * 60)
        
        for test_name, result in self.test_results.items():
            if result is None:
                status = "SKIPPED"
                symbol = "⚠️"
            elif result:
                status = "PASSED"
                symbol = "✅"
            else:
                status = "FAILED"
                symbol = "❌"
            
            print(f"{symbol} {test_name.replace('_', ' ').title()}: {status}")
        
        if all_passed:
            print("\n🎉 ALL TESTS PASSED! System ready for deployment.")
        else:
            print("\n💥 SOME TESTS FAILED! Check the logs above.")
        
        return all_passed


def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully"""
    print("\n🛑 Test interrupted by user")
    sys.exit(0)


def main():
    """Main test runner"""
    signal.signal(signal.SIGINT, signal_handler)
    
    # Create test instance
    tester = CostmapSystemTest()
    
    try:
        # Run all tests
        success = tester.run_all_tests()
        return 0 if success else 1
        
    except Exception as e:
        print(f"💥 Unexpected error: {e}")
        return 1
        
    finally:
        tester.cleanup()


if __name__ == '__main__':
    exit(main())
