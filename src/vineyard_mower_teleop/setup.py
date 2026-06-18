from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'vineyard_mower_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sam',
    maintainer_email='samjt775@gmail.com',
    description='Teleoperation and obstacle avoidance for vineyard mower',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'keyboard_teleop = vineyard_mower_teleop.keyboard_teleop_control:main',
            'depth_obstacle_avoidance = vineyard_mower_teleop.depth_obstacle_avoidance:main',
        ],
    },
)
