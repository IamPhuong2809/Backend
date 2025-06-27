import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory, get_package_share_path



def generate_launch_description():
    moveit_controller_path          = os.path.join(get_package_share_directory("my_robot_hw"),  # Update with your package name
                                                        "config", "moveit_controllers.yaml",
                                                )
    moveit_urdf_path                = os.path.join(get_package_share_directory("my_robot_hw"),  # Update with your package name
                                                        "urdf", "ammr_main.urdf.xacro",
                                                )
    moveit_srdf_path                = os.path.join(get_package_share_directory("my_robot_hw"),  # Update with your package name
                                                        "config", "ammr_main.srdf",
                                                )
    kinematics_path                 = os.path.join(get_package_share_directory("my_robot_hw"),  # Update with your package name
                                                        "config", "kinematics.yaml",
                                                )
    rviz_path                       = os.path.join(get_package_share_directory("my_robot_hw"),  # Update with your package name
                                                        "config", "moveit.rviz",
                                                )
    initial_positions_file_path     = os.path.join(get_package_share_directory("my_robot_hw"),  # Update with your package name
                                                        "config", "initial_positions.yaml",
                                                )
    ros2_controllers_path           = os.path.join(get_package_share_directory("my_robot_hw"),  # Update with your package name
                                                        "config", "ros2_controllers.yaml",  # Make sure this file exists and is correctly configured
                                                )

    # Load the robot configuration
    moveit_config = (
        MoveItConfigsBuilder(
            "ammr",                                                         # Change this to the name of your robot (e.g., "your_robot_name")
        )
        .robot_description(file_path            = moveit_urdf_path,
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

    # MoveGroupInterface demo executable
    ammr_moveit = Node(
        name="ammr_moveit_controller",
        package="ammr_moveit_controller",
        executable="move_robot_service",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([ammr_moveit])