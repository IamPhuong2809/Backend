from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'aruco_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'opencv-python', 'numpy', 'pyrealsense2', 'sensor_msgs', 'cv_bridge'],
    zip_safe=True,
    maintainer='phuongnguyen',
    maintainer_email='nminhphuong2809@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'aruco_node = aruco_node.ArucoNode:main',
            'aruco_scan = aruco_node.ArucoScan:main',
        ],
    },
)
