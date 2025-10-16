# File: multi_camera_sync.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cameras = ["oak_front", "oak_left", "oak_right"]

    nodes = []

    # Per-camera RGB + Depth sync
    for cam in cameras:
        nodes.append(
            Node(
                package="adam_bringup",
                executable="rgb_sync",
                name=f"{cam}_rgb_sync",
                output="screen",
                parameters=[{
                    "rgb_topic": f"/{cam}/rgb/image_raw",
                    "depth_topic": f"/{cam}/stereo/image_raw",
                    "rgb_info_topic": f"/{cam}/rgb/camera_info",
                    "depth_info_topic": f"/{cam}/stereo/camera_info",
                    "output_topic": f"/{cam}/rgbd_image",
                }]
            )
        )

    # Global RGBD sync (combining front, left, right)
    nodes.append(
        Node(
            package="adam_bringup",
            executable="rgbd_sync",
            name="rgbd_global_sync",
            output="screen",
            parameters=[{
                "num_inputs": 3,
                "input_topics_0": "/oak_front/rgbd_image",
                "input_topics_1": "/oak_left/rgbd_image",
                "input_topics_2": "/oak_right/rgbd_image",
                "output_topic": "/rgbd_sync",
            }]
        )
    )
    return LaunchDescription(nodes)
