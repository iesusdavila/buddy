#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(
            package="usb_cam",
            executable="usb_cam_node_exe",
            name="usb_cam_node_exe",
            output="screen",
        ),
        Node(
            package='coco_filters',
            executable='face_landmark_detector.py',
            name='face_landmark_detector',
            output='screen'
        ),
        Node(
            package='coco_filters',
            executable='animal_filter_mask',
            name='animal_filter_mask',
            output='screen'
        ),
    ])