from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():

    parameters={
        'frame_id':'base_link',
        # 'Reg/Force3DoF':'true',
        'Grid/RayTracing':'false', # Fill empty space
        'Grid/3D':'true', # Use 2D occupancy
        'Grid/RangeMax':'5.0',
        'Grid/NormalsSegmentation':'true', # Use passthrough filter to detect obstacles
        'Grid/MaxGroundHeight':'0.08', # All points above 5 cm are obstacles
        'Grid/MaxObstacleHeight':'0.5',  # All points over 1 meter are ignored
        'Grid/CellSize': '0.05',
        'Grid/ClusterRadius': '0.1',
        'Grid/DepthDecimation': '4',
        'Grid/DepthRoiRatios': '0.0 0.0 1.0 1.0',
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

    return LaunchDescription([

        # Nodes to launch

        Node(
            package='rtabmap_util', executable='point_cloud_xyz', output='screen',
            parameters=[{'decimation': 1,
                         'max_depth': 5.0,
                         'voxel_size': 0.02}],
            remappings=[('depth/image', '/oak_front/stereo/image_raw'),
                        ('depth/camera_info', '/oak_front/stereo/camera_info')
                       ]),
        Node(
            package='rtabmap_util', executable='obstacles_detection', output='screen',
            parameters=[parameters],
           ),
    ])