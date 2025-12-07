from setuptools import setup
import os
from glob import glob

package_name = 'pupper_apriltag'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join(package_name, 'launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='max',
    maintainer_email='you@example.com',
    description='AprilTag pipeline launcher',
    license='MIT',

    # ✔ ADD THIS SECTION
    entry_points={
        'console_scripts': [
            'camera_viewer = pupper_apriltag.camera_viewer:main',
            'fast_tag_pose = pupper_apriltag.fast_tag_pose:main',
            'highspeed_oak_node = pupper_apriltag.highspeed_oak_node:main',
            'simple_sub = pupper_apriltag.simple_sub:main',
            
        ],
    },
)
