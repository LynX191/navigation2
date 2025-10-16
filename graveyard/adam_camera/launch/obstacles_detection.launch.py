# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration, PythonExpression
# from launch.conditions import IfCondition
# from launch_ros.actions import Node


# def generate_launch_description():
#     # Declare launch arguments
#     declared_args = [
#         DeclareLaunchArgument('all', default_value='false'),
#         DeclareLaunchArgument('front', default_value='false'),
#         DeclareLaunchArgument('left', default_value='false'),
#         DeclareLaunchArgument('right', default_value='false'),
#     ]

#     # Get values
#     all_cams = LaunchConfiguration('all')
#     use_front = LaunchConfiguration('front')
#     use_left = LaunchConfiguration('left')
#     use_right = LaunchConfiguration('right')

#     parameters = {
#         'frame_id': 'base_link',
#         'Grid/RayTracing': 'false',
#         'Grid/3D': 'true',
#         'Grid/RangeMax': '5.0',
#         'Grid/NormalsSegmentation': 'true',
#         'Grid/MaxGroundHeight': '0.08',
#         'Grid/MaxObstacleHeight': '0.5',
#         'Grid/CellSize': '0.05',
#         'Grid/ClusterRadius': '0.1',
#         'Grid/DepthDecimation': '4',
#         'Grid/DepthRoiRatios': '0.0 0.2 0.0 0.4',
#         'Grid/FlatObstacleDetected': 'true',
#         'Grid/FootprintHeight': '0.0',
#         'Grid/FootprintLength': '0.0',
#         'Grid/FootprintWidth': '0.0',
#         'Grid/GroundIsObstacle': 'false',
#         'Grid/MapFrameProjection': 'false',
#         'Grid/MaxGroundAngle': '45',
#         'Grid/MinClusterSize': '25',
#         'Grid/MinGroundHeight': '0.0',
#         'Grid/NoiseFilteringMinNeighbors': '5',
#         'Grid/NoiseFilteringRadius': '0.08',
#         'Grid/NormalK': '20',
#         'Grid/PreVoxelFiltering': 'true',
#         'Grid/RangeMin': '0.0',
#         'Grid/Scan2dUnknownSpaceFilled': 'false',
#         'Grid/ScanDecimation': '1'
#     }

#     nodes = []

#     def camera_nodes(cam):
#         return [
#             Node(
#                 package='rtabmap_util',
#                 executable='point_cloud_xyz',
#                 name=f'{cam}_pc',
#                 output='screen',
#                 parameters=[{
#                     'decimation': 1,
#                     'max_depth': 5.0,
#                     'voxel_size': 0.02
#                 }],
#                 remappings=[
#                     ('depth/image', f'/{cam}/stereo/image_raw'),
#                     ('depth/camera_info', f'/{cam}/stereo/camera_info'),
#                     ('cloud', f'/{cam}/cloud')
#                 ]
#             ),
#             Node(
#                 package='rtabmap_util',
#                 executable='obstacles_detection',
#                 name=f'{cam}_obstacles',
#                 output='screen',
#                 parameters=[parameters],
#                 remappings=[
#                     ('cloud', f'/{cam}/cloud'),
#                     ('obstacles', f'/{cam}/obstacles'),
#                     ('ground', f'/{cam}/ground')
#                 ]
#             )
#         ]

#     # Dynamically add camera nodes based on arguments
#     for cam, config in [('oak_front', use_front), ('oak_left', use_left), ('oak_right', use_right)]:
#         nodes.append(
#             Node(
#                 package='launch_ros',
#                 executable='launch_ros_substitution',
#                 condition=IfCondition(
#                     PythonExpression([
#                         '"("', ' + ', LaunchConfiguration('front'), ' + ', '") == true or (', LaunchConfiguration('all'), ' == true)"'
#                     ])
#                 ),
#                 output='log',
#                 arguments=[],
#             )
#         )
#         nodes += camera_nodes(cam)

#     return LaunchDescription(declared_args + nodes)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    declared_args = [
        DeclareLaunchArgument('all', default_value='false'),
        DeclareLaunchArgument('front', default_value='false'),
        DeclareLaunchArgument('left', default_value='false'),
        DeclareLaunchArgument('right', default_value='false'),
    ]

    # Launch configurations
    all_cams = LaunchConfiguration('all')
    front = LaunchConfiguration('front')
    left = LaunchConfiguration('left')
    right = LaunchConfiguration('right')

    # RTAB-Map obstacle detection parameters
    parameters = {
        'frame_id': 'base_footprint',
        'Grid/RayTracing': 'false',
        'Grid/3D': 'true',
        'Grid/RangeMax': '7.5',
        'Grid/NormalsSegmentation': 'true',
        'Grid/MaxGroundHeight': '0.04',
        'Grid/MaxObstacleHeight': '0.3',
        'Grid/CellSize': '0.05',
        'Grid/ClusterRadius': '0.1',
        'Grid/DepthDecimation': '1',
        'Grid/DepthRoiRatios': '',
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

    def camera_nodes(cam_name, flag_name):
        return [
            Node(
                package='rtabmap_util',
                executable='point_cloud_xyz',
                name=f'{cam_name}_pc',
                output='screen',
                parameters=[{
                    'decimation': 2,
                    'max_depth': 3.0,
                    'voxel_size': 0.03,
                    'noise_filter_min_neighbors': 5,
                    'noise_filter_radius': 0.05,
                    # 'normal_k': 50,
                    'normal_radius': 0.0,
                    # 'filter_nans': 'false'
                }],
                remappings=[
                    ('depth/image', f'/{cam_name}/stereo/image_raw'),
                    ('depth/camera_info', f'/{cam_name}/stereo/camera_info'),
                    ('cloud', f'/{cam_name}/cloud')
                ],
                condition=IfCondition(
                    PythonExpression(["'", LaunchConfiguration(flag_name), "' == 'true' or '", LaunchConfiguration('all'), "' == 'true'"])
                )
            ),
            Node(
                package='rtabmap_util',
                executable='obstacles_detection',
                name=f'{cam_name}_obstacles',
                output='screen',
                parameters=[parameters],
                remappings=[
                    ('cloud', f'/{cam_name}/cloud'),
                    ('obstacles', f'/{cam_name}/obstacles'),
                    ('ground', f'/{cam_name}/ground')
                ],
                condition=IfCondition(
                    PythonExpression(["'", LaunchConfiguration(flag_name), "' == 'true' or '", LaunchConfiguration('all'), "' == 'true'"])
                )
            )
        ]


    return LaunchDescription(declared_args +
        camera_nodes('oak_front', 'front') +
        camera_nodes('oak_left', 'left') +
        camera_nodes('oak_right', 'right')
    )
