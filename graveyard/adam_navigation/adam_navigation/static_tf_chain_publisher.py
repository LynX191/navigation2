#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class StaticTFChainPublisher(Node):
    def __init__(self):
        super().__init__('static_tf_chain_publisher')
        self.br = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_transforms)

    def broadcast_transforms(self):
        now = self.get_clock().now().to_msg()

        # base_footprint -> base_link
        t1 = TransformStamped()
        t1.header.stamp = now
        t1.header.frame_id = 'base_footprint'
        t1.child_frame_id = 'base_link'
        t1.transform.translation.x = 0.0
        t1.transform.translation.y = 0.0
        t1.transform.translation.z = 0.0282  # base_link is slightly above the ground
        t1.transform.rotation.x = 0.0
        t1.transform.rotation.y = 0.0
        t1.transform.rotation.z = 0.0
        t1.transform.rotation.w = 1.0

        # base_link -> oak-d-base-frame
        t2 = TransformStamped()
        t2.header.stamp = now
        t2.header.frame_id = 'base_link'
        t2.child_frame_id = 'oak-d-base-frame'
        t2.transform.translation.x = 0.16  # Adjust according to your camera's pose
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = 0.2418
        t2.transform.rotation.x = 0.0
        t2.transform.rotation.y = 0.0
        t2.transform.rotation.z = 0.0
        t2.transform.rotation.w = 1.0

        self.br.sendTransform(t1)
        self.br.sendTransform(t2)

def main():
    rclpy.init()
    node = StaticTFChainPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
