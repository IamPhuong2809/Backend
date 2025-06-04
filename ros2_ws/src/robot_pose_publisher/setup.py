from setuptools import setup
import os
from glob import glob

package_name = 'robot_pose_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Publish robot pose from TF',
    license='Apache License 2.0',
    
    # Thêm phần entry_points vào đây
    entry_points={
        'console_scripts': [
            'robot_pose_publisher = robot_pose_publisher.robot_pose_publisher:main',
        ],
    },
)
