from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="true",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )

    # Initialize Arguments
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    use_rviz = LaunchConfiguration("use_rviz")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ammr_mock_components"), "urdf", "robot.urdf.xacro"]
            ),
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("ammr_mock_components"),
            "config",
            "my_controllers.yaml",
        ]
    )

    # Control node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    # Robot state publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        condition=IfCondition(use_rviz),
    )

    # Controller spawners - spawn joint_state_broadcaster first
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="both",
    )

    # Spawn arm_controller after joint_state_broadcaster
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="both",
    )

    # Spawn gripper_controller after joint_state_broadcaster
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
        output="both",
    )

    # Spawn diff_base_controller after joint_state_broadcaster
    diff_base_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_base_controller", "--controller-manager", "/controller_manager"],
        output="both",
    )

    # Delay other controllers to start after joint_state_broadcaster
    delay_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    delay_gripper_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )

    delay_diff_base_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_base_controller_spawner],
        )
    )

    # Alternative approach using TimerAction for delays (uncomment if preferred)
    # arm_controller_spawner_delayed = TimerAction(
    #     period=2.0,
    #     actions=[arm_controller_spawner],
    # )
    
    # gripper_controller_spawner_delayed = TimerAction(
    #     period=3.0,
    #     actions=[gripper_controller_spawner],
    # )
    
    # diff_base_controller_spawner_delayed = TimerAction(
    #     period=4.0,
    #     actions=[diff_base_controller_spawner],
    # )

    nodes = [
        control_node,
        robot_state_pub_node,
        rviz_node,
        joint_state_broadcaster_spawner,
        delay_arm_controller,
        delay_gripper_controller,
        delay_diff_base_controller,
    ]

    # If using TimerAction approach instead, use this nodes list:
    # nodes = [
    #     control_node,
    #     robot_state_pub_node,
    #     rviz_node,
    #     joint_state_broadcaster_spawner,
    #     arm_controller_spawner_delayed,
    #     gripper_controller_spawner_delayed,
    #     diff_base_controller_spawner_delayed,
    # ]

    return LaunchDescription(declared_arguments + nodes)