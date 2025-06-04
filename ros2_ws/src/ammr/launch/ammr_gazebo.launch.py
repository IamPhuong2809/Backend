import os
import xacro
from ament_index_python.packages import get_package_share_path, get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command

from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    
    urdf_path                       = os.path.join(get_package_share_directory("ammr"),
                                    "urdf", "ammr_main.urdf.xacro",
                                    )   
    gazebo_ros_path                 = os.path.join(get_package_share_directory("gazebo_ros"),  
                                    "launch",
                                    )
    headless_mode = LaunchConfiguration('headless', default='true')
    robot_description_content = Command(['xacro ', urdf_path])
    robot_description = ParameterValue(robot_description_content, value_type=None)
   


    ## Load Gazebo server & Client and Spawn robot in Gazebo
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_ros_path, '/gzserver.launch.py']),
                                    launch_arguments={'headless': headless_mode}.items()

    )
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_ros_path, '/gzclient.launch.py']),
                                    launch_arguments={'headless': headless_mode}.items()

    )

    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description', 
                   '-entity', 'ammr'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('headless', 
            default_value='true', 
            description='Run Gazebo in headless mode'
        ),
        gazebo_server,
        gazebo_client,
        spawn_entity_node,

    ])

