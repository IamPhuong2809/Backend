import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory, get_package_share_path
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_controller_path          = os.path.join(get_package_share_directory("ammr_moveit_config"),  # Update with your package name
                                                        "config", "moveit_controllers.yaml",
                                                )
    moveit_urdf_path                = os.path.join(get_package_share_directory("ammr_moveit_config"),  # Update with your package name
                                                        "config", "ammr_main.urdf.xacro",
                                                )
    moveit_srdf_path                = os.path.join(get_package_share_directory("ammr_moveit_config"),  # Update with your package name
                                                        "config", "ammr_main.srdf",
                                                )
    kinematics_path                 = os.path.join(get_package_share_directory("ammr_moveit_config"),  # Update with your package name
                                                        "config", "kinematics.yaml",
                                                )
    rviz_path                       = os.path.join(get_package_share_directory("ammr_moveit_config"),  # Update with your package name
                                                        "config", "moveit.rviz",
                                                )
    initial_positions_file_path     = os.path.join(get_package_share_directory("ammr_moveit_config"),  # Update with your package name
                                                        "config", "initial_positions.yaml",
                                                )
    ros2_controllers_path           = os.path.join(get_package_share_directory("ammr_moveit_config"),  # Update with your package name
                                                        "config", "ros2_controllers.yaml",  # Make sure this file exists and is correctly configured
                                                )


    ros2_control_hardware_type = DeclareLaunchArgument(
        "ros2_control_hardware_type",
        default_value="mock_components",
        description="ROS 2 control hardware interface type to use for the launch file -- possible values: [mock_components, isaac]",
    )

    # Load the robot configuration
    moveit_config = (
        MoveItConfigsBuilder(
            "ammr",                                                         # Change this to the name of your robot (e.g., "your_robot_name")
            # package_name                        = "ammr_moveit_config"      # Replace with your MoveIt 2 config package name
        )
        .robot_description(file_path            = moveit_urdf_path,
                           mappings             = {"ros2_control_hardware_type": 
                                                LaunchConfiguration("ros2_control_hardware_type")},
        )   # Pass the mappings for the robot description
        .robot_description_semantic(file_path   = moveit_srdf_path)
        .trajectory_execution(file_path         = moveit_controller_path)   # Update with correct controller config file
        .robot_description_kinematics(file_path = kinematics_path)
        .planning_scene_monitor(
            publish_robot_description_semantic  = True,
        )
        .planning_pipelines(pipelines           = ["ompl", "chomp", "pilz_industrial_motion_planner"])                 # These are common planners; update if necessary
        .to_moveit_configs()
    )

    # Start the move_group node/action server for planning and execution
    move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability"
    }

    move_group_node = Node(
        package                                 = "moveit_ros_move_group",
        executable                              = "move_group",
        output                                  = "screen",
        parameters                              = [moveit_config.to_dict(),
                                                move_group_capabilities,],
        arguments                               = ["--ros-args", "--log-level", "info"],

    )

    # Launch RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits
        ],
    )

    ## SETUP STATIC TF
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        # arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "odom", "base_link"],
    )

    ## PUBLISHING ROBOT STATE 
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[moveit_config.robot_description]
    )

    ammr_moveit_controller_node = Node(
        package='ammr_moveit_controller',
        executable='ammr_moveit_controller',
        output='log'   
    )

    ## LAUNCH ROS2 CONTROL NODE
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path,
                    moveit_config.robot_description],
        remappings=[
            ("/controller_manager/robot_description",
             "/robot_description"),
        ],
        output="screen",
    )

    ## SPAWN CONTROLLER [JS_BROADCASTER / MANIPULATOR_CONTROLLER / GRIPPER_CONTROLLER]
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    # Arm controller
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", 
                   "/controller_manager"],
    )
    # Gripper controller
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "-c", 
                   "/controller_manager"],
    )
    # Diffrential base controller
    diff_base_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_base_controller", "-c", 
                   "/controller_manager"],
    )


    return LaunchDescription([
        ros2_control_hardware_type,
        # rviz_node,
        static_tf_node,
        robot_state_publisher_node,
        ammr_moveit_controller_node,
        move_group_node,
        ros2_control_node,
        # joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
        diff_base_controller_spawner,
    ])
