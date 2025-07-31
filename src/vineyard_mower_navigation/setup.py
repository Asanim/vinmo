#!/usr/bin/env python3

from setuptools import setup, find_packages

package_name = 'vineyard_mower_navigation'
share_prefix = 'share/'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        (share_prefix + 'ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (share_prefix + package_name, ['package.xml']),
        (share_prefix + package_name + '/launch', [
            'launch/costmap_generation.launch.py',
            'launch/vineyard_web_system.launch.py'
        ]),
        (share_prefix + package_name + '/config', ['config/costmap_config.yaml']),
        ('lib/' + package_name, [
            'scripts/gps_reference_publisher.py',
            'scripts/laser_scan_frame_remapper.py', 
            'scripts/test_costmap_client.py',
            'scripts/test_navigation_integration.py',
            'scripts/test_system_integration.py',
            'scripts/vineyard_row_detector.py',
            'scripts/generate_test_images.py'
        ]),
    ],
    install_requires=[
        'setuptools',
        'opencv-python',
        'numpy',
        'requests',
        'pillow',
        'pyyaml',
    ],
    zip_safe=True,
    maintainer='Sam',
    maintainer_email='sam@example.com',
    description='Navigation, teleoperation and SLAM for the vineyard mowing robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'costmap_service = vineyard_mower_navigation.costmap_service:main',
            'costmap_web_service = vineyard_mower_navigation.costmap_web_service:main',
            'costmap_publisher = vineyard_mower_navigation.costmap_generator:main',
        ],
    },
)
