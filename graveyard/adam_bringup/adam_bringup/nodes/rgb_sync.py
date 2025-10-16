#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from message_filters import Subscriber, ApproximateTimeSynchronizer

from sensor_msgs.msg import Image, CameraInfo
from rtabmap_msgs.msg import RGBDImage


class RGBDSyncNode(Node):
    def __init__(self):
        super().__init__('rgbd_sync_node')

        # Declare parameters
        self.declare_parameter('rgb_topic', '/rgb/image_raw')
        self.declare_parameter('depth_topic', '/stereo/image_raw')
        self.declare_parameter('rgb_info_topic', '/rgb/camera_info')
        self.declare_parameter('depth_info_topic', '/stereo/camera_info')
        self.declare_parameter('output_topic', '/rgbd_image')
        self.declare_parameter('slop', 0.5)  # synchronization tolerance in seconds

        # Get parameters
        self.rgb_topic = self.get_parameter('rgb_topic').get_parameter_value().string_value
        self.depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        self.rgb_info_topic = self.get_parameter('rgb_info_topic').get_parameter_value().string_value
        self.depth_info_topic = self.get_parameter('depth_info_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.slop = self.get_parameter('slop').get_parameter_value().double_value

        # QoS settings
        self.qos = 10

        # Subscribers using message_filters
        self.rgb_sub = Subscriber(self, Image, self.rgb_topic)
        self.depth_sub = Subscriber(self, Image, self.depth_topic)
        self.rgb_info_sub = Subscriber(self, CameraInfo, self.rgb_info_topic)
        self.depth_info_sub = Subscriber(self, CameraInfo, self.depth_info_topic)

        # Synchronizer
        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub, self.rgb_info_sub, self.depth_info_sub],
            queue_size=30,
            slop=self.slop
        )
        self.sync.registerCallback(self.synced_callback)

        # Publisher
        self.rgbd_pub = self.create_publisher(RGBDImage, self.output_topic, self.qos)

        self.get_logger().info(f"RGBD Sync Node started, publishing to {self.output_topic}")

    def synced_callback(self, rgb_msg, depth_msg, rgb_info_msg, depth_info_msg):
        # Use the RGB message's timestamp for synchronization
        sync_stamp = rgb_msg.header.stamp

        rgbd_msg = RGBDImage()
        rgbd_msg.header.stamp = sync_stamp
        rgbd_msg.header.frame_id = rgb_msg.header.frame_id  # Keep same frame ID

        rgbd_msg.rgb = rgb_msg
        rgbd_msg.depth = depth_msg
        rgbd_msg.rgb_camera_info = rgb_info_msg
        rgbd_msg.depth_camera_info = depth_info_msg

        self.rgbd_pub.publish(rgbd_msg)

        # self.get_logger().info(
        #     f"Published RGBD with header timestamp {sync_stamp.sec}.{sync_stamp.nanosec}"
        # )



# ------------- Main Entry Point -------------
def main(args=None):
    rclpy.init(args=args)
    node = RGBDSyncNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('RGBD Sync Node shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
