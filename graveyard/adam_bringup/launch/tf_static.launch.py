#!/usr/bin/env python3

"""
Launch file for static TF transforms.
This file publishes the static transform from base_footprint to base_link.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_to_link',
            arguments=['0.0', '0.0', '0.0282', '0', '0', '0', 'base_footprint', 'base_link']
        ),
    ])
