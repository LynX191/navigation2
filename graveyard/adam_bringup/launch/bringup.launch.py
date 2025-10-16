#!/usr/bin/env python3

"""
Main bringup launch file for the ADAM robot.
This launch file includes:
1. Motor drivers and hardware interfaces
2. Multi-camera setup with OAK-D cameras
3. Robot description (URDF)

Usage:
    ros2 launch adam_bringup bringup.launch.py
    
Optional arguments:
    'all:=true'    - Launch all cameras (default: false)
    'front:=true'          - Launch front camera (default: false)
    'left:=true'           - Launch left camera (default: false)
    'right:=true'          - Launch right camera (default: false)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def declare_arguments():
    """Declare all launch arguments in one place."""
    return [
        DeclareLaunchArgument('all', default_value='false', description='Launch all cameras'),
        DeclareLaunchArgument('front', default_value='false', description='Launch front camera'),
        DeclareLaunchArgument('left', default_value='false', description='Launch left camera'),
        DeclareLaunchArgument('right', default_value='false', description='Launch right camera')
    ]

def include_launch_file(package_name, launch_file_name, arguments=None):
    """Helper function to include a launch file dynamically."""
    try:
        package_path = get_package_share_directory(package_name)
        launch_file_path = os.path.join(package_path, 'launch', launch_file_name)
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file_path),
            launch_arguments=arguments.items() if arguments else None
        )
    except Exception as e:
        print(f"Warning: Failed to include {launch_file_name} from {package_name}: {e}")
        return None

def generate_launch_description():
    """Generate the main launch description."""
    # Declare arguments
    declared_args = declare_arguments()

    # Include drivers
    drivers_launch = include_launch_file('adam_bringup', 'triorb_robot.launch.py')

    # Include static TF transforms
    tf_static_launch = include_launch_file('adam_bringup', 'tf_static.launch.py')
    
    # Include map server
    map_server_launch = include_launch_file('adam_bringup', 'map_server.launch.py')

    # Include multi-camera setup
    cameras_launch = include_launch_file(
        'adam_bringup', 'multi_camera.launch.py',
        arguments={
            'all': LaunchConfiguration('all'),
            'front': LaunchConfiguration('front'),
            'left': LaunchConfiguration('left'),
            'right': LaunchConfiguration('right')
        }
    )

    # Create launch actions
    launch_actions = declared_args
    if drivers_launch:
        launch_actions.append(drivers_launch)
    if cameras_launch:
        launch_actions.append(cameras_launch)
    if tf_static_launch:
        launch_actions.append(tf_static_launch)
    if map_server_launch:
        launch_actions.append(map_server_launch)

    return LaunchDescription(launch_actions)
