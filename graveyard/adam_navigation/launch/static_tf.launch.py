#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
import math

def generate_launch_description():
    radius = 0.16
    height = 0.2418

    angle_left = math.radians(120)
    angle_right = math.radians(-120)

    return LaunchDescription([
        # base_footprint -> base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_to_link',
            arguments=['0.0', '0.0', '0.0282', '0', '0', '0', 'base_footprint', 'base_link']
        ),

        # base_link -> oak_front_frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_to_front',
            arguments=[
                f'{radius}', '0.0', f'{height}',  # x y z
                '0', '0', '0',                    # roll pitch yaw
                'base_link', 'oak_front_frame'
            ]
        ),

        # base_link -> oak_left_frame (120° yaw)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_to_left',
            arguments=[
                f'{radius * math.cos(angle_left)}',
                f'{radius * math.sin(angle_left)}',
                f'{height}',
                f'{angle_left}', '0', '0',       # yaw in radians
                'base_link', 'oak_left_frame'
            ]
        ),

        # base_link -> oak_right_frame (-120° yaw)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_to_right',
            arguments=[
                f'{radius * math.cos(angle_right)}',
                f'{radius * math.sin(angle_right)}',
                f'{height}',
                f'{angle_right}', '0', '0',
                'base_link', 'oak_right_frame'
            ]
        )
    ])
