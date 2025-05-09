from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():
    pkg_share = FindPackageShare('bolt_visu').find('bolt_visu')
    urdf_file = os.path.join(pkg_share, 'urdf', 'bolt.urdf.xacro')
    world_file = 'empty.sdf'

    robot_description = ParameterValue(
        Command([
            f"bash -c 'xacro {urdf_file} | sed \"s|package://bolt_visu|{pkg_share}|g\"'"
        ]),
        value_type=str
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments={'gz_args': world_file}.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'bolt', '-topic', 'robot_description'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity
    ])
