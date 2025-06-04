from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("ammr").to_dict()

    # MTC Demo node
    pick_place_demo = Node(
        package="ammr_moveit_controller",
        executable="ammr_mtc_demo",
        output="screen",
        parameters=[
            moveit_config,
        ],
    )
    return LaunchDescription([pick_place_demo])