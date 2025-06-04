from setuptools import find_packages, setup

package_name = 'ammr_mobile_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='horizon',
    maintainer_email='nminhphuong2809@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            "ammr_mobile_node = ammr_mobile_node.mobile_robot:main"
        ],
    },
)
