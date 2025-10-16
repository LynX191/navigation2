#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class ZRangeChecker(Node):
    def __init__(self):
        super().__init__('z_range_checker')
        self.sub = self.create_subscription(PointCloud2, '/obstacles', self.callback, 10)
        self.min_z = float('inf')
        self.max_z = float('-inf')

    def callback(self, msg):
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        count = 0
        for p in points:
            z = p[2]
            if z < self.min_z:
                self.min_z = z
            if z > self.max_z:
                self.max_z = z
            count += 1
        self.get_logger().info(f'Points processed: {count}')
        self.get_logger().info(f'Z range: min = {self.min_z}, max = {self.max_z}')
        rclpy.shutdown()

def main():
    rclpy.init()
    node = ZRangeChecker()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
