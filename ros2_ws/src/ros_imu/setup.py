from setuptools import find_packages, setup

package_name = 'ros_imu'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools',
                      'pyserial'],
    zip_safe=True,
    maintainer='horizon',
    maintainer_email='nminhphuong2809@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            "ros_imu = ros_imu.Node_imu:main"
        ],
    },
)
