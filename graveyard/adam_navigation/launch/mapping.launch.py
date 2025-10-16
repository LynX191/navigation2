from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to rtabmap_launch
    rtabmap_launch_file = os.path.join(
        get_package_share_directory('rtabmap_launch'),
        'launch',
        'rtabmap.launch.py'
    )

    database_path_arg = DeclareLaunchArgument(
        'database_path',
        default_value='/home/emage/ws_adam/database/last.db',
        description='Path to the RTAB-Map database file'
    )
    database_path = LaunchConfiguration('database_path')

    return LaunchDescription([
        database_path_arg,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rtabmap_launch_file),
            launch_arguments={
                'rtabmap_args': '--delete_db_on_start',
                'rgb_topic': '/oak/rgb/image_raw',
                'depth_topic': '/oak/stereo/image_raw',
                'camera_info_topic': '/oak/rgb/camera_info',
                'frame_id': 'base_footprint',
                'approx_sync': 'true',
                'wait_imu_to_init': 'true',
                'imu_topic': '/oak/imu/data',
                'database_path': database_path,
                'cfg': '/home/emage/ws_adam/adam_nav/src/adam_navigation/config/rtabmap_tunning.ini',
                'namespace': '/'
            }.items()
        )
    ])
