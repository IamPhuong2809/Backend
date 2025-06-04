import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'my_robot_brightup'

    # 1. robot_state_publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
            )
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 2. twist_mux
    twist_mux_params = os.path.join(
        get_package_share_directory(package_name), 'config', 'twist_mux.yaml'
    )
    twist_mux = Node(
        package='twist_mux', executable='twist_mux',
        parameters=[twist_mux_params, {'use_sim_time': True}],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')]
    )

    # 3. Gazebo world launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
            )
        ])
    )

    # 4. spawn_entity and controllers delayed until robot_description is available
    spawn_entity = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
        output='screen'
    )
    diff_drive_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['diff_cont'],
    )
    joint_broad_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_broad'],
    )
    delayed_actions = TimerAction(
        period=5.0,
        actions=[spawn_entity, diff_drive_spawner, joint_broad_spawner]
    )

    return LaunchDescription([
        rsp,
        twist_mux,
        gazebo,
        delayed_actions,
        # Include node publish pose
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('robot_pose_publisher') + '/launch/robot_pose_publisher.launch.py'
            ])
        ),
    ])

