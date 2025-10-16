import os
import launch
import launch_ros
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

package_name = 'adam_localize'

def generate_launch_description():
    # === RVIZ CONFIGURATION ===
    package_path = get_package_share_directory(package_name)
    rviz_config_file = os.path.join(package_path, 'config', 'rviz.rviz')

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2 with config'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # === LAUNCH ARGUMENTS ===
    localization_mode_arg = DeclareLaunchArgument(
        'localization_mode',
        default_value='false',
        description='Set to true for localization mode (no mapping)'
    )

    load_previous_arg = DeclareLaunchArgument(
        'load_previous',
        default_value='false',
        description='If true AND not in localization mode, loads previous map without deleting DB'
    )

    database_path_arg = DeclareLaunchArgument(
        'database_path',
        default_value="/home/emage/ws_adam/database/abdel.db",
        description='Where is the map saved/loaded.'
    )
    database_path = LaunchConfiguration('database_path')
    
    # === INCLUDE SYNC LAUNCH ===
    sync_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('adam_bringup'),
                'launch',
                'multi_camera_sync.launch.py'
            )
        )
    )
    rtabmap_config_path = os.path.join(package_path, 'config', 'rtabmap_tunning.ini')
    
    # === RGBD ODOMETRY NODE ===
    rgbd_odometry_node = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        parameters=[{
            "rgbd_cameras": 0,  # Input is rgbd_images from multi_camera_sync
            "frame_id": 'base_footprint',
            "odom_frame_id": 'odom',
            "publish_tf": True,
            "subscribe_rgbd": True,
            "config_path": rtabmap_config_path,
            "qos": 1,
            "topic_queue_size": 1000,
        }],
        remappings=[
            ('rgbd_images', '/rgbd_sync'),
        ]
    )

    # === FUNCTION TO CONFIGURE SLAM NODE ===
    def configure_slam_node(context):
        localization_mode = context.launch_configurations.get('localization_mode', 'false')
        load_previous = context.launch_configurations.get('load_previous', 'false')

        if localization_mode == 'true':
            incremental_memory = 'false'
            incremental_dictionary = 'false'
            init_wm_with_all_nodes = 'true'
            delete_db = []
        else:
            if load_previous == 'true':
                incremental_memory = 'true'
                incremental_dictionary = 'true'
                init_wm_with_all_nodes = 'true'
                delete_db = []
            else:
                incremental_memory = 'true'
                incremental_dictionary = 'true'
                init_wm_with_all_nodes = 'false'
                delete_db = ['--delete_db_on_start']

        return incremental_memory, incremental_dictionary, init_wm_with_all_nodes, delete_db

    # === SLAM NODE ===
    slam_opaque_function = OpaqueFunction(
        function=lambda context: [
            Node(
                package='rtabmap_slam',
                executable='rtabmap',
                name='rtabmap',
                output='screen',
                parameters=[{
                    "rgbd_cameras": 0,
                    "subscribe_depth": False,
                    "subscribe_rgbd": True,
                    "subscribe_rgb": False,
                    "subscribe_odom": True,
                    "subscribe_odom_info": True,
                    "frame_id": 'base_footprint',
                    "map_frame_id": 'map',
                    "publish_tf": True,
                    "approx_sync": True,
                    "approx_sync_max_interval": 0.1,
                    "database_path": database_path,
                    "config_path": rtabmap_config_path,
                    "topic_queue_size": 100,
                    "Mem/IncrementalMemory": configure_slam_node(context)[0],
                    "Kp/IncrementalDictionary": configure_slam_node(context)[1],
                    "Mem/InitWMWithAllNodes": configure_slam_node(context)[2],
                    
                }],
                remappings=[
                    ('rgbd_images', '/rgbd_sync'),
                    ('imu', '/oak_right/imu/data'),
                ],
                arguments=configure_slam_node(context)[3]
            )
        ]
    )

    # === VISUALIZATION NODE ===
    rtabmap_viz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[{
            "rgbd_cameras": 0,
            "frame_id": 'base_footprint',
            "map_frame_id": 'map',
            "subscribe_depth": False,
            "subscribe_rgbd": True,
            "subscribe_odom": True,
            "subscribe_odom_info": True,
            "approx_sync": True,
            "config_path": rtabmap_config_path,
        }],
        remappings=[
            ('rgbd_images', '/rgbd_sync'),
        ]
    )

    # === RETURN LAUNCH DESCRIPTION ===
    return launch.LaunchDescription([
        localization_mode_arg,
        load_previous_arg,
        database_path_arg,
        rviz_arg,
        sync_launch,
        rgbd_odometry_node,
        slam_opaque_function,
        rtabmap_viz_node,
        # rviz_node,
    ])
