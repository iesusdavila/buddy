#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf_file_name = 'coco_description.urdf'

    print("urdf_file_name : {}".format(urdf_file_name))

    urdf = os.path.join(
        get_package_share_directory('coco_description'),
        'urdf',
        urdf_file_name)

    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    rsp_params = {'robot_description': robot_desc}

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(get_package_share_directory('coco_bringup'), 'config', 'coco_mimic.rviz')]),
        Node(
            package="usb_cam",
            executable="usb_cam_node_exe",
            name="usb_cam_node_exe",
            output="screen",
        ),
        Node(
            package='coco_arm_mimic',
            executable='body_points_detector.py',
            name='body_points_detector',
            output='screen'
        ),
        Node(
            package='coco_arm_mimic',
            executable='body_tracker_node',
            name='body_tracker_node',
            output='screen'
        ),
        Node(
            package='coco_arm_mimic',
            executable='coco_controller',
            name='coco_controller',
            output='screen'
        ),
    ])