#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from rtabmap_msgs.msg import RGBDImage

from message_filters import Subscriber, ApproximateTimeSynchronizer


class RGBDPerTopicSyncNode(Node):
    def __init__(self):
        super().__init__('rgbd_per_topic_sync_node')

        # Parameters
        self.declare_parameter('num_inputs', 2)
        self.declare_parameter('input_topics_0', '/oak_front/rgbd_image')
        self.declare_parameter('input_topics_1', '/oak_left/rgbd_image')
        self.declare_parameter('input_topics_2', '/oak_right/rgbd_image')
        self.declare_parameter('slop', 0.5)  # seconds tolerance for sync
        self.declare_parameter('queue_size', 100)

        # Load parameters
        self.num_inputs = self.get_parameter('num_inputs').get_parameter_value().integer_value
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

        # Create one publisher per input topic
        self.rgbd_publishers = {}
        for topic in self.input_topics:
            out_topic = topic + "_synced"
            pub = self.create_publisher(RGBDImage, out_topic, qos_profile)
            self.rgbd_publishers[topic] = pub
            self.get_logger().info(f"Publisher created for {out_topic}")

        self.get_logger().info("RGBD Per Topic Sync Node started.")

    def synced_callback(self, *msgs):
        # Use current time as the synchronized timestamp
        latest_stamp = self.get_clock().now().to_msg()

        for msg, topic in zip(msgs, self.input_topics):
            new_msg = RGBDImage()

            # Shallow copy of the input message
            new_msg = msg

            # Update timestamps
            new_msg.header.stamp = latest_stamp
            new_msg.rgb.header.stamp = latest_stamp
            new_msg.depth.header.stamp = latest_stamp
            new_msg.rgb_camera_info.header.stamp = latest_stamp
            new_msg.depth_camera_info.header.stamp = latest_stamp

            # Publish to corresponding output topic
            self.rgbd_publishers[topic].publish(new_msg)

        self.get_logger().info(
            f"Published {len(msgs)} synced RGBDImages with timestamp {latest_stamp.sec}.{latest_stamp.nanosec}"
        )


# ---------------- Main Entry ----------------
def main(args=None):
    rclpy.init(args=args)
    node = RGBDPerTopicSyncNode()

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('RGBD Per Topic Sync Node shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
