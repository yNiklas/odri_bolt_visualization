from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():
    pkg_share = FindPackageShare('bolt_visu').find('bolt_visu')
    urdf_file = os.path.join(pkg_share, 'urdf', 'bolt.urdf.xacro')
    world_file = 'empty.sdf'
    gz_controller_file = os.path.join(pkg_share, 'config', 'bolt_gz_controller.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

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
        launch_arguments=[('gz_args', ['-r', '-v 4 empty.sdf'])]
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
        arguments=[
            '-name', 'bolt',
            '-topic', 'robot_description',
            '-z', '0.5'
            ],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', "--controller-manager", "/controller_manager"]
    )
    joint_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_position_controller',
            '--controller-manager',
            '/controller_manager'
        ]
    )
    controller_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[gz_controller_file]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        controller_control_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[joint_state_broadcaster_spawner]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[joint_position_controller_spawner]
            )
        ),
        spawn_entity,
        DeclareLaunchArgument('use_sim_time', default_value=use_sim_time)
    ])
