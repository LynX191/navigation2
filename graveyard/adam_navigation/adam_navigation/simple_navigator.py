#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from tf_transformations import euler_from_quaternion
import math


class SmartNavigator(Node):
    def __init__(self):
        super().__init__('smart_navigator')

        self.goal_pose = None
        self.current_pose = None
        self.goal_reached = False
        self.initial_rotation_done = False
        self.final_rotation_done = False

        self.linear_speed = 0.15
        self.angular_speed = 0.2
        self.max_angular_speed = 0.2

        self.goal_tolerance = 0.15  # meters
        self.angle_tolerance = math.radians(5)  # radians

        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/localization_pose',
            self.pose_callback,
            10
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose

    def goal_callback(self, msg):
        self.goal_pose = msg.pose
        self.goal_reached = False
        self.initial_rotation_done = False
        self.final_rotation_done = False
        self.get_logger().info("🎯 New goal received")

    def control_loop(self):
        if not self.current_pose or not self.goal_pose:
            return

        dx = self.goal_pose.position.x - self.current_pose.position.x
        dy = self.goal_pose.position.y - self.current_pose.position.y
        distance = math.hypot(dx, dy)

        robot_yaw = self.get_yaw(self.current_pose.orientation)
        angle_to_goal = math.atan2(dy, dx)
        heading_error = self.normalize_angle(angle_to_goal - robot_yaw)

        goal_yaw = self.get_yaw(self.goal_pose.orientation)
        rotate_to_goal_error = self.normalize_angle(goal_yaw - robot_yaw)

        cmd = Twist()

        if not self.goal_reached:
            if distance > self.goal_tolerance:
                # Step 1: Initial rotation to face goal
                # if not self.initial_rotation_done:
                if abs(heading_error) > math.radians(20):
                    cmd.linear.x = 0.0
                    cmd.angular.z = max(-self.angular_speed, min(self.angular_speed, 1.0 * heading_error))
                    self.cmd_pub.publish(cmd)
                    return
                    # else:
                    #     self.initial_rotation_done = True

                # Step 2: Move forward with light angular correction
                cmd.linear.x = self.linear_speed

                # Only correct angularly if heading error is significant
                if abs(heading_error) > math.radians(5):  # ignore micro-drifts
                    correction = 0.5 * heading_error  # reduce gain
                    cmd.angular.z = max(-0.1, min(0.1 , correction))
                else:
                    cmd.angular.z = 0.0
            else:
                self.goal_reached = True
                self.get_logger().info("📍 Position reached. Rotating to match goal heading...")

        elif not self.final_rotation_done:
            # Rotate to match goal heading
            if abs(rotate_to_goal_error) > self.angle_tolerance:
                cmd.angular.z = max(-self.angular_speed, min(self.angular_speed, 1.5 * rotate_to_goal_error))
                cmd.linear.x = 0.0
            else:
                cmd.angular.z = 0.0
                cmd.linear.x = 0.0
                self.final_rotation_done = True
                self.get_logger().info("✅ Goal heading aligned.")

        self.cmd_pub.publish(cmd)

    def get_yaw(self, orientation):
        q = (orientation.x, orientation.y, orientation.z, orientation.w)
        _, _, yaw = euler_from_quaternion(q)
        return yaw

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main():
    rclpy.init()
    node = SmartNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
