from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directory dynamically
    package_dir = get_package_share_directory('adam_bringup')
    default_config_path = os.path.join(package_dir, 'config', 'motor_params.yaml')
    
    # Declare launch arguments
    robot_port_arg = DeclareLaunchArgument(
        'robot_port',
        default_value='/dev/triorb_robot',
        description='The serial port for the robot'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_path,
        description='Path to parameter configuration file.'
    )

    # Define launch configuration substitutions
    robot_port = LaunchConfiguration('robot_port')
    config_file = LaunchConfiguration('config_file')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='guider',
        description='Top-level namespace')

    # Node declaration
    robot_node = Node(
        package='adam_bringup',
        executable='triorb_robot_node',
        name='triorb_robot_node',
        output='both',
        parameters=[config_file],
        remappings=[
            ('/cmd_vel', 'cmd_vel'),
            ('/odom', 'odom')
        ],
        arguments=[f'--robot_port={robot_port}']
    )

    # Log info message
    log_info = LogInfo(
        msg="Launching TriOrbRobotNode..."
    )

    # Return LaunchDescription with all actions
    return LaunchDescription([
        robot_port_arg,
        config_file_arg,
        declare_namespace_cmd,
        log_info,
        robot_node
    ])