#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.duration import Duration

class CmdVelSafetyWatchdog(Node):
    def __init__(self):
        super().__init__('cmd_vel_safety')
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.last_cmd_time = self.get_clock().now()
        self.last_cmd = Twist()
        self.timeout_duration = Duration(seconds=0.5)
        self.timer = self.create_timer(0.1, self.watchdog_check)
        self.stopped = False

    def cmd_callback(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()
        self.last_cmd = msg
        # Only reset stopped if actual motion is commanded
        if abs(msg.linear.x) > 1e-3 or abs(msg.angular.z) > 1e-3 or abs(msg.linear.y) > 1e-3:
            self.stopped = False

    def watchdog_check(self):
        now = self.get_clock().now()
        time_since_last_cmd = now - self.last_cmd_time
        if time_since_last_cmd > self.timeout_duration and not self.stopped:
            if abs(self.last_cmd.linear.x) > 1e-3 or abs(self.last_cmd.angular.z) > 1e-3 or abs(self.last_cmd.linear.y) > 1e-3:
                self.get_logger().warn("⚠️ No velocity commands received recently. Stopping the robot.")
                stop_cmd = Twist()
                self.cmd_pub.publish(stop_cmd)
                self.stopped = True


def main():
    rclpy.init()
    node = CmdVelSafetyWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
