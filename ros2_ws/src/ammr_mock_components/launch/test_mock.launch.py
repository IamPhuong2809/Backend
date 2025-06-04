from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("jetbot_mock_control"), "urdf", "jetbot.urdf.xacro"]
            ),
            " ",
            "use_mock_hardware:=true",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("jetbot_mock_control"),
            "config",
            "my_controllers.yaml",
        ]
    )

    # Control node với debug logging
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
        arguments=["--ros-args", "--log-level", "INFO"],
    )

    return LaunchDescription([control_node])