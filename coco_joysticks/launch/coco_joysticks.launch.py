#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            output="screen",
            parameters=[{"deadzone": 0.05, "autorepeat_rate": 10.0}],
        ),
        Node(
            package="coco_joysticks",
            executable="coco_controller",
            name="coco_controller",
            output="screen",
        ),
    ])