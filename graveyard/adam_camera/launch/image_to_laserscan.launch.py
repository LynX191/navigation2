# import os

# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch_ros.actions import Node


# def generate_launch_description():
#     param_config = os.path.join(
#         get_package_share_directory('adam_camera'), 'config', 'image_to_laserscan.yaml')
#     return LaunchDescription([
#         # base_footprint -> base_link
#         Node(
#             package='tf2_ros',
#             executable='static_transform_publisher',
#             name='tf_base_to_link',
#             arguments=['0.2', '0.0', '0.2418', '0', '0', '0', 'base_link', 'camera_scan_frame']
#         ),

#         Node(
#             package='depthimage_to_laserscan',
#             executable='depthimage_to_laserscan_node',
#             name='depthimage_to_laserscan',
#             remappings=[('depth', '/oak/stereo/image_raw'),
#                         ('depth_camera_info', '/oak/stereo/camera_info')],
#             parameters=[param_config])
#     ])

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node

def create_depth_node(name, topic_prefix):
    return Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name=f'depthimage_to_laserscan_{name}',
        remappings=[
            ('depth', f'/{topic_prefix}/stereo/image_raw'),
            ('depth_camera_info', f'/{topic_prefix}/stereo/camera_info'),
            ('scan', f'/{name}/scan'),
        ],
        parameters=[{
            'scan_time': 0.033,
            'range_min': 3.0,
            'range_max': 10.0,
            'scan_height': 1,
            'output_frame': f'{topic_prefix}_frame'
        }],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration(name), "' == 'true' or '", LaunchConfiguration('all'), "' == 'true'"])
        )
    )

def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument('front', default_value='false', description='Launch front camera'),
        DeclareLaunchArgument('right', default_value='false', description='Launch right camera'),
        DeclareLaunchArgument('left', default_value='false', description='Launch left camera'),
        DeclareLaunchArgument('all', default_value='false', description='Launch all cameras'),
    ]

    # Return a flat list of actions
    return LaunchDescription(declared_args + [
        create_depth_node('front', 'oak_front'),
        create_depth_node('right', 'oak_right'),
        create_depth_node('left', 'oak_left')
    ])
