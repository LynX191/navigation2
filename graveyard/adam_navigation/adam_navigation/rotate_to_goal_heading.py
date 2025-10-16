#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
from rclpy.action import ActionServer
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from action_msgs.msg import GoalStatusArray
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool
from tf_transformations import euler_from_quaternion


class RotateToGoalHeading(Node):
    def __init__(self):
        super().__init__('rotate_to_goal_heading')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_status_pub = self.create_publisher(Bool, '/goal_reached', 10)
        # self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/localization_pose', self.localization_callback, 10)
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.status_callback, 10)
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            self.execute_goal,
            goal_callback=self.handle_goal
        )
        self.timer = self.create_timer(0.1, self.control_loop)

        self.goal_yaw = None
        self.current_yaw = None
        self.goal_reached = False
        self.rotating = False
        self.is_goal_reached = False
        self.yaw_tolerance = 0.1
        self.rotation_speed = 0.5

    def handle_goal(self, goal_request):
        pose: PoseStamped = goal_request.pose
        q = pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.goal_yaw = yaw
        self.goal_reached = False
        self.rotating = False
        self.is_goal_reached = False
        # self.get_logger().info(f"New goal received: Yaw = {math.degrees(yaw):.2f}°")
        # return GoalResponse.ACCEPT  # or GoalResponse.REJECT

    def execute_goal(self, goal_handle):
        pass

    # def goal_pose_callback(self, msg: PoseStamped):
    #     q = msg.pose.orientation
    #     _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
    #     self.goal_yaw = yaw
    #     self.goal_reached = False
    #     self.rotating = False
    #     self.get_logger().info(f"New goal received: Yaw = {math.degrees(yaw):.2f}°")

    def localization_callback(self, msg: PoseWithCovarianceStamped):
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_yaw = yaw
    
    def status_callback(self, msg: GoalStatusArray):
        if not msg.status_list:
            return

        latest_status = msg.status_list[-1]

        # Only act on new SUCCEEDED status
        if latest_status.status == 4 and not self.goal_reached:
            self.goal_reached = True
            self.rotating = True
            # self.get_logger().info("✅ Goal reached. Starting heading alignment.")

    def control_loop(self):
        if self.rotating and self.current_yaw is not None and self.goal_yaw is not None:
            error = self.normalize_angle(self.goal_yaw - self.current_yaw)
            if abs(error) < self.yaw_tolerance:
                self.cmd_vel_pub.publish(Twist())
                self.rotating = False
                self.get_logger().info("Alignment complete.")
                self.is_goal_reached = True
                self.publish_goal_reached(self.is_goal_reached)
                return

            # Adaptive gain zone: slow earlier by shaping the curve
            k_p = 0.17
            raw_speed = k_p * error

            # Instead of hard clamp, shape it with tanh or scaled sigmoid
            shaped_speed = self.rotation_speed * math.tanh(raw_speed / self.rotation_speed)
            cmd = Twist()
            cmd.angular.z = shaped_speed
            self.cmd_vel_pub.publish(cmd)

        if not self.is_goal_reached:
            self.publish_goal_reached(False)

    def publish_goal_reached(self, reached: bool):
        msg = Bool()
        msg.data = reached
        self.goal_status_pub.publish(msg)

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))


def main():
    rclpy.init()
    node = RotateToGoalHeading()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()