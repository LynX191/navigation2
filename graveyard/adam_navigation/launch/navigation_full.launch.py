import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    # Get the package share directories (plain strings are fine here)
    nav_dir = get_package_share_directory('adam_navigation')
    adam_camera_dir = get_package_share_directory('adam_camera')

    # Launch configurations
    omni = LaunchConfiguration('omni')
    obstacle_avoidance = LaunchConfiguration('obstacle_avoidance')
    params_name = LaunchConfiguration('params_name')

    # Arguments
    declare_omni_cmd = DeclareLaunchArgument(
        'omni',
        default_value='false',
        description='Use omnidirectional mode'
    )

    declare_params_name_arg = DeclareLaunchArgument(
        'params_name',
        default_value='navigation_omni.yaml',
        description='YAML file name located under the config/ directory (include .yaml)'
    )

    # Allow passing a full path directly, otherwise derive from params_name
    declare_params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([nav_dir, 'config', params_name]),
        description='Full path to the params YAML. If set, overrides params_name.'
    )

    declare_obstacle_avoidance_cmd = DeclareLaunchArgument(
        'obstacle_avoidance',
        default_value='false',
        description='Enable obstacle avoidance'
    )

    # Include the main navigation launch, forwarding the params_file (full path or derived)
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_dir, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={
            'params_file': LaunchConfiguration('params_file')
        }.items()
    )

    # Optional extra launches conditioned by obstacle_avoidance
    obstacle_detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(adam_camera_dir, 'launch', 'obstacles_detection.launch.py')
        ),
        condition=IfCondition(obstacle_avoidance),
        launch_arguments={'all': 'true'}.items(),
    )

    image_to_laserscan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(adam_camera_dir, 'launch', 'image_to_laserscan.launch.py')
        ),
        condition=IfCondition(obstacle_avoidance),
        launch_arguments={'all': 'true'}.items(),
    )

    ld = LaunchDescription()
    ld.add_action(declare_omni_cmd)
    ld.add_action(declare_params_name_arg)
    ld.add_action(declare_params_file_arg)
    ld.add_action(declare_obstacle_avoidance_cmd)
    ld.add_action(nav_launch)
    ld.add_action(obstacle_detection_launch)
    ld.add_action(image_to_laserscan_launch)
    return ld