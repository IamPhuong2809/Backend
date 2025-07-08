from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_rplidar_dir = get_package_share_directory('rplidar_ros')
    pkg_camera_dir = get_package_share_directory('realsense2_camera')
    pkg_rosbridge_dir = get_package_share_directory('rosbridge_server')

    return LaunchDescription([
        # Include Python launch files
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_rplidar_dir, 'launch', 'rplidar_a2m8_launch.py')
            ),
            launch_arguments={'serial_port': '/dev/ttyUSB0'}.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_camera_dir, 'launch', 'rs_launch.py')
            )
        ),

        # Include XML launch file (correct way)
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                os.path.join(pkg_rosbridge_dir, 'launch', 'rosbridge_websocket_launch.xml')
            )
        ),
    ])
