#!/usr/bin/env python3

"""
Multi-camera point cloud and obstacle detection launcher.

Launches:
- point_cloud_xyz
- obstacles_detection

For each camera: front, left, right, or all.

Usage examples:
    ros2 launch adam_bringup obstacles_detection.launch.py all:=true
    ros2 launch adam_bringup obstacles_detection.launch.py front:=true left:=true
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def create_nodes_for_camera(camera_name, enable_condition):
    """Create point_cloud_xyz and obstacles_detection nodes for a given camera."""
    depth_image_topic = f"/oak_{camera_name}/stereo/image_raw"
    depth_info_topic = f"/oak_{camera_name}/stereo/camera_info"

    parameters = {
        'frame_id': 'base_footprint',
        'Grid/RayTracing': 'false',
        'Grid/3D': 'true',
        'Grid/RangeMax': '5.0',
        'Grid/NormalsSegmentation': 'true',
        'Grid/MaxGroundHeight': '0.08',
        'Grid/MaxObstacleHeight': '0.5',
        'Grid/CellSize': '0.05',
        'Grid/ClusterRadius': '0.1',
        'Grid/DepthDecimation': '4',
        'Grid/DepthRoiRatios': '0.0 0.2 0.0 0.4',
        'Grid/FlatObstacleDetected': 'true',
        'Grid/FootprintHeight': '0.0',
        'Grid/FootprintLength': '0.0',
        'Grid/FootprintWidth': '0.0',
        'Grid/GroundIsObstacle': 'false',
        'Grid/MapFrameProjection': 'false',
        'Grid/MaxGroundAngle': '45',
        'Grid/MinClusterSize': '25',
        'Grid/MinGroundHeight': '0.0',
        'Grid/NoiseFilteringMinNeighbors': '5',
        'Grid/NoiseFilteringRadius': '0.08',
        'Grid/NormalK': '20',
        'Grid/PreVoxelFiltering': 'true',
        'Grid/RangeMin': '0.0',
        'Grid/Scan2dUnknownSpaceFilled': 'false',
        'Grid/ScanDecimation': '1'
    }

    pointcloud_node = Node(
        package='rtabmap_util',
        executable='point_cloud_xyz',
        name=f'point_cloud_xyz_{camera_name}',
        output='screen',
        parameters=[{'decimation': 1, 'max_depth': 5.0, 'voxel_size': 0.02}],
        remappings=[
            ('depth/image', depth_image_topic),
            ('depth/camera_info', depth_info_topic)
        ],
        condition=IfCondition(enable_condition)
    )

    obstacles_node = Node(
        package='rtabmap_util',
        executable='obstacles_detection',
        name=f'obstacles_detection_{camera_name}',
        output='screen',
        parameters=[parameters],
        condition=IfCondition(enable_condition)
    )

    return [pointcloud_node, obstacles_node]


def generate_launch_description():
    # Assume cameras are: front, left, right (folder names under config/cameras/)
    bringup_dir = get_package_share_directory('adam_bringup')
    config_dir = os.path.join(bringup_dir, 'config', 'cameras')
    camera_folders = [f.name for f in os.scandir(config_dir) if f.is_dir()]

    declared_args = [
        DeclareLaunchArgument('all', default_value='false', description='Launch all cameras')
    ]
    all_config = LaunchConfiguration('all')

    launch_actions = []

    for camera_name in camera_folders:
        cam_arg = DeclareLaunchArgument(
            camera_name,
            default_value='false',
            description=f'Enable {camera_name} camera'
        )

        enable_expr = PythonExpression([
            "'", LaunchConfiguration(camera_name), "' == 'true' or '", all_config, "' == 'true'"
        ])

        declared_args.append(cam_arg)

        nodes = create_nodes_for_camera(camera_name, enable_expr)
        launch_actions.extend(nodes)

    return LaunchDescription(declared_args + launch_actions)
