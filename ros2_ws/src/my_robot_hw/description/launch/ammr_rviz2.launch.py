import os
import xacro
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():
    ## Directory path
    ammr_dir = get_package_share_path('my_robot_hw')
    urdf_path = os.path.join(ammr_dir, 'urdf', 'ammr_main.urdf.xacro')
    rviz_config_file= os.path.join(ammr_dir, 'config', 'moveit.rviz')
    robot_description = xacro.process_file(urdf_path).toxml()
    # Launching GUI
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='True'
    )
    show_gui = LaunchConfiguration('gui')

    ## Joint State and Robot State Publisher Node 
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True,'robot_description' : robot_description}]
    )
    joint_state_publisher_node = Node(
        condition=UnlessCondition(show_gui),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )
    joint_state_publisher_gui_node = Node(
        condition=IfCondition(show_gui),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )
    ## Launch RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        gui_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])

