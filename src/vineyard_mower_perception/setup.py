from setuptools import setup

package_name = 'vineyard_mower_perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vineyard Robotics Team',
    maintainer_email='developer@vineyard-robotics.com',
    description='Multi-sensor perception and obstacle detection for vineyard mower robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'advanced_perception_fusion = vineyard_mower_perception.advanced_perception_fusion:main',
            'depth_obstacle_mapper = vineyard_mower_perception.depth_obstacle_mapper:main',
            'vine_detection_node = vineyard_mower_perception.vine_detection_node:main',
            'sensor_health_monitor = vineyard_mower_perception.sensor_health_monitor:main',
        ],
    },
)
