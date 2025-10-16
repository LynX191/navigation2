#!/usr/bin/env python3

"""
Flexible multi-camera launcher for OAK-D Pro using depthai_ros_driver.

Automatically scans the config/cameras/ folder for camera configurations.
Each camera gets its own namespace and can be toggled on/off via launch arguments.
Also publishes static TF transforms based on tf_static.yaml in each camera's folder.

Use:
    ros2 launch adam_bringup multi_camera.launch.py all:=true
    ros2 launch adam_bringup multi_camera.launch.py left:=true front:=true

Frame relationships:
- parent_frame: The frame to which the camera is attached (e.g., base_link)
- child_frame: The frame representing the camera's position and orientation (e.g., oak_front_frame)
"""

import os
import yaml
import math
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def create_camera_launch(camera_name, config_path, camera_launch_path, enable_expr, parent_frame):
    """Creates an IncludeLaunchDescription for a single camera."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch_path),
        condition=IfCondition(enable_expr),
        launch_arguments={
            'params_file': config_path,
            'name': 'oak_'+camera_name,
            'parent_frame': parent_frame,
            'namespace': ''
        }.items()
    )

def create_static_transform_node(camera_name, tf_config_path, enable_expr):
    """Creates a static transform publisher node based on tf_static.yaml config."""
    try:
        with open(tf_config_path, 'r') as f:
            tf_config = yaml.safe_load(f)

        transform = tf_config['transform']
        translation = transform['translation']
        rotation = transform['rotation']
        parent_frame = transform['parent_frame']
        child_frame = transform['child_frame']

        # Note: tf2_ros static_transform_publisher takes arguments in the order:
        # x y z yaw pitch roll parent_frame child_frame
        # or
        # x y z qx qy qz qw parent_frame child_frame
        return Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'tf_static_to_{camera_name}',
            namespace=camera_name,  # Add to camera namespace
            condition=IfCondition(enable_expr),
            arguments=[
                str(translation['x']), 
                str(translation['y']), 
                str(translation['z']),
                str(rotation['yaw']),     # Yaw (rotation around Z)
                str(rotation['pitch']),   # Pitch (rotation around Y)
                str(rotation['roll']),    # Roll (rotation around X)
                parent_frame, 
                child_frame
            ]
        )
    except (FileNotFoundError, yaml.YAMLError, KeyError) as e:
        return None

def generate_launch_description():
    # Base paths
    bringup_dir = get_package_share_directory('adam_bringup')
    config_dir = os.path.join(bringup_dir, 'config', 'cameras')
    camera_launch_path = os.path.join(
        get_package_share_directory('depthai_ros_driver'),
        'launch', 'camera.launch.py'
    )

    # Discover all camera folders in config/cameras/
    camera_folders = [f.name for f in os.scandir(config_dir) if f.is_dir()]

    # Declare common 'all' argument
    all_arg = DeclareLaunchArgument('all', default_value='false', description='Launch all cameras')
    all_config = LaunchConfiguration('all')

    # Generate launch arguments and launch actions per camera
    declared_args = [all_arg]
    launch_actions = []

    for camera_name in camera_folders:
        # Define paths for each camera
        params_path = os.path.join(config_dir, camera_name, 'params.yaml')
        tf_path = os.path.join(config_dir, camera_name, 'tf_static.yaml')
        
        # Skip if params file doesn't exist
        if not os.path.exists(params_path):
            launch_actions.append(LogInfo(msg=f"Warning: No params.yaml found for camera '{camera_name}', skipping"))
            continue
            
        # Create launch argument for this camera
        cam_arg = DeclareLaunchArgument(
            camera_name, 
            default_value='false', 
            description=f'Enable {camera_name} camera'
        )
        
        # Create conditional expression for enabling this camera
        enable_expr = PythonExpression([
            "'", LaunchConfiguration(camera_name), "' == 'true' or '", all_config, "' == 'true'"
        ])

        # Simplify default parent_frame and child_frame logic
        parent_frame = "base_link"  # Default if tf_static.yaml doesn't exist
        child_frame = f"oak_{camera_name}_frame"  # Default child frame

        if os.path.exists(tf_path):
            try:
                with open(tf_path, 'r') as f:
                    tf_config = yaml.safe_load(f)
                    parent_frame = tf_config['transform']['parent_frame']
                    child_frame = tf_config['transform']['child_frame']
            except (FileNotFoundError, yaml.YAMLError, KeyError) as e:
                launch_actions.append(LogInfo(msg=f"Warning: Error loading TF config for {camera_name}: {e}"))

        # Create static transform publisher if tf_static.yaml exists
        tf_node = create_static_transform_node(camera_name, tf_path, enable_expr)
        
        # Create camera launch with parent frame set to the child frame from TF
        camera_launch = create_camera_launch(
            camera_name, 
            params_path, 
            camera_launch_path, 
            enable_expr,
            child_frame  # This will be 'oak_front_frame', 'oak_left_frame', etc.
        )

        # Add to our lists
        declared_args.append(cam_arg)
        launch_actions.append(camera_launch)
        if tf_node:
            launch_actions.append(tf_node)

    return LaunchDescription(declared_args + launch_actions)
