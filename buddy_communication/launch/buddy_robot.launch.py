#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction

def generate_launch_description():
    pkg_share = FindPackageShare('buddy_communication').find('buddy_communication')
    urdf_file_name = 'buddy_description.urdf'


    urdf_path = os.path.join(
        get_package_share_directory('buddy_description'),
        'urdf',
        urdf_file_name)
    with open(urdf_path, "r") as infp:
        robot_description_content = infp.read()
    controller_config_path = os.path.join(pkg_share, 'config', 'buddy_controllers.yaml')

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description_content}, 
            {'use_sim_time': False}
        ])

    ros2_control_node = Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': robot_description_content},
                controller_config_path
            ],
            output='screen',
            # arguments=['--ros-args', '--log-level', 'debug']
        )
    
    spawner_joint_state = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen",
        )
    
    spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
            output="screen",
        )

    control_timer = TimerAction(period=5.0, actions=[ros2_control_node, spawner_joint_state, spawner])

    return LaunchDescription([
        rsp,
        control_timer,
    ])
