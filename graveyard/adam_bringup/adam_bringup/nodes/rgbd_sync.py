#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from rtabmap_msgs.msg import RGBDImage, RGBDImages

from message_filters import Subscriber, ApproximateTimeSynchronizer


class RGBDMultiSyncNode(Node):
    def __init__(self):
        super().__init__('rgbd_multi_sync_node')

        # Parameters
        self.declare_parameter('num_inputs', 2)
        self.declare_parameter('input_topics_0', '/oak_front/rgbd_image')
        self.declare_parameter('input_topics_1', '/oak_left/rgbd_image')
        self.declare_parameter('input_topics_2', '/oak_right/rgbd_image')
        self.declare_parameter('output_topic', '/rgbd_sync')
        self.declare_parameter('slop', 0.05)  # seconds tolerance for sync
        self.declare_parameter('queue_size', 100)

        # Load parameters
        self.num_inputs = self.get_parameter('num_inputs').get_parameter_value().integer_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.slop = self.get_parameter('slop').get_parameter_value().double_value
        self.queue_size = self.get_parameter('queue_size').get_parameter_value().integer_value

        if self.num_inputs not in [2, 3]:
            self.get_logger().error('num_inputs must be 2 or 3.')
            raise ValueError('Invalid num_inputs')

        self.input_topics = [
            self.get_parameter(f'input_topics_{i}').get_parameter_value().string_value
            for i in range(self.num_inputs)
        ]

        self.get_logger().info(f"Subscribing to: {self.input_topics}")

        # Subscribers with message_filters
        self.subscribers = [
            Subscriber(self, RGBDImage, topic)
            for topic in self.input_topics
        ]

        # Synchronizer
        self.sync = ApproximateTimeSynchronizer(
            self.subscribers,
            queue_size=self.queue_size,
            slop=self.slop
        )
        self.sync.registerCallback(self.synced_callback)

        # QoS settings
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=30
        )

        # Publisher with QoS
        self.rgbd_publisher = self.create_publisher(
            RGBDImages,
            self.output_topic,
            qos_profile
        )

        self.get_logger().info(f"RGBD Multi Sync Node started with output: {self.output_topic}")

    def synced_callback(self, *msgs):
        # Use current time as the synchronized timestamp
        latest_stamp = self.get_clock().now().to_msg()

        synced_msgs = []
        for msg in msgs:
            new_msg = RGBDImage()

            # Shallow copy of the input message
            new_msg = msg

            # Update all timestamps to the same
            new_msg.header.stamp = latest_stamp
            new_msg.rgb.header.stamp = latest_stamp
            new_msg.depth.header.stamp = latest_stamp
            new_msg.rgb_camera_info.header.stamp = latest_stamp
            new_msg.depth_camera_info.header.stamp = latest_stamp

            synced_msgs.append(new_msg)

        rgbd_images_msg = RGBDImages()
        rgbd_images_msg.header.stamp = latest_stamp
        rgbd_images_msg.header.frame_id = msgs[0].header.frame_id  # Use first msg frame_id

        rgbd_images_msg.rgbd_images = synced_msgs

        self.rgbd_publisher.publish(rgbd_images_msg)

        # self.get_logger().info(
        #     f"Published RGBDImages with stamp {latest_stamp.sec}.{latest_stamp.nanosec} "
        #     f"and {len(msgs)} inputs"
        # )


# ---------------- Main Entry ----------------
def main(args=None):
    rclpy.init(args=args)
    node = RGBDMultiSyncNode()

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('RGBD Multi Sync Node shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
