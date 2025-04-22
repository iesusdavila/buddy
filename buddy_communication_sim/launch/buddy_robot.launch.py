#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    gz_args = LaunchConfiguration('gz_args', default='')

    pkg_share = FindPackageShare('buddy_communication_sim').find('buddy_communication_sim')
    # urdf_file_name = 'buddy_description_sim.urdf'  # Archivo URDF para simulación

    # urdf_path = os.path.join(
    #     get_package_share_directory('buddy_description'),
    #     'urdf',
    #     urdf_file_name)
    # with open(urdf_path, "r") as infp:
    #     robot_description_content = infp.read()

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('buddy_description'),
                 'urdf', 'buddy_description_sim.xacro.urdf']
            ),
        ]
    )
    controller_config_path = os.path.join(pkg_share, 'config', 'buddy_controllers.yaml')
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('buddy_communication_sim'),
            'config',
            'buddy_controllers.yaml',
        ]
    )

    # rsp = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     output='screen',
    #     parameters=[
    #         {'robot_description': robot_description_content}, 
    #         {'use_sim_time': True}
    #     ])

    # ros2_control_node = Node(
    #         package='controller_manager',
    #         executable='ros2_control_node',
    #         parameters=[
    #             controller_config_path,
    #             {'use_sim_time': True},
    #         ],
    #         output='screen',
    #         remappings=[
    #             ("~/robot_description", "/robot_description"),
    #         ],
    #     )

    # spawner_joint_state = Node(
    #         package="controller_manager",
    #         executable="spawner",
    #         arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    #         output="screen",
    #     )
    
    # spawner = Node(
    #         package="controller_manager",
    #         executable="spawner",
    #         arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
    #         output="screen",
    #     )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'buddy',
                    '-x', '0.0', '-y', '0.0', '-z', '0.0',
                    '-allow_renaming', 'true']
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',],
    )

    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_trajectory_controller',
            '--param-file',
            controller_config_path,],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    return LaunchDescription([
        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', [gz_args, ' -r -v 1 empty.sdf'])]),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[joint_trajectory_controller_spawner],
            )
        ),
        bridge,
        node_robot_state_publisher,
        gz_spawn_entity,
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
    ])