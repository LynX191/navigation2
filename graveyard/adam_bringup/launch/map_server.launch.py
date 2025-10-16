from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directory dynamically
    package_dir = get_package_share_directory('adam_bringup')

    # Node declaration
    robot_node = Node(
        package='adam_bringup',
        executable='map_server_node',
        name='map_server_node',
        output='screen',
    )

    # Return LaunchDescription with all actions
    return LaunchDescription([
        robot_node
    ])