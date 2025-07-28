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
        (share_prefix + package_name + '/launch', ['launch/costmap_generation.launch.py']),
        (share_prefix + package_name + '/config', ['config/costmap_config.yaml']),
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
            'costmap_publisher = vineyard_mower_navigation.costmap_generator:main',
            'test_costmap_client = scripts.test_costmap_client:main',
            'generate_test_images = scripts.generate_test_images:main',
        ],
    },
)
