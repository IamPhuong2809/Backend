from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_pose_publisher',
            executable='robot_pose_publisher',
            name='robot_pose_publisher_node',
            output='screen'
        ),
    ])
