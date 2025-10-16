import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer, TransformBroadcaster
from geometry_msgs.msg import TransformStamped, PointStamped, Twist
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data
import math
import numpy as np
import re
from visualization_msgs.msg import Marker

def pose_to_matrix(x, y, theta):
    c, s = math.cos(theta), math.sin(theta)
    return np.array([
        [c, -s, x],
        [s,  c, y],
        [0,  0, 1]
    ])

def matrix_to_pose(mat):
    x = mat[0, 2]
    y = mat[1, 2]
    theta = math.atan2(mat[1, 0], mat[0, 0])
    return x, y, theta

class BaseLinkPoseVector(Node):
    def __init__(self):
        super().__init__('map_server')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.depth_1 = 0.7  # meters
        self.angle_1 = 1.4  # degrees
        self.shift_x_1 = -0.01  # meters
        
        self.depth_2 = 0.7  # meters
        self.angle_2 = 1.4  # degrees
        self.shift_x_2 = -0.01  # meters

        self.last_t1_in_map = None
        self.last_t2_in_map = None
        self.new_imaging_data_1 = False   
        self.new_imaging_data_2 = False   

        self.report_timer = self.create_timer(0.2, self.calculate_and_report)

        self.imaging_control_sub_t1 = self.create_subscription(
            String,
            '/t1/comm/imaging_control',
            self.imaging_control_callback_t1,
            qos_profile_sensor_data
        )

        self.imaging_control_sub_t2 = self.create_subscription(
            String,
            '/t2/comm/imaging_control',
            self.imaging_control_callback_t2,
            qos_profile_sensor_data
        )
        self.guider_transporter_pub = self.create_publisher(String, '/guider/comm/guider_transporter', 10)
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)

    def imaging_control_callback_t1(self, msg):
        try:
            pattern = r'([-\d.]+)\^([-\d.]+)\^\(([-\d.]+),\s*([-\d.]+)\)'
            m = re.match(pattern, msg.data.strip())
            if m:
                self.depth_1 = float(m.group(1))
                self.angle_1 = float(m.group(2))
                self.shift_x_1 = float(m.group(3))
                self.new_imaging_data_1 = True  # Mark that new imaging data has been received
        except Exception:
            pass
        
    def imaging_control_callback_t2(self, msg):
        try:
            pattern = r'([-\d.]+)\^([-\d.]+)\^\(([-\d.]+),\s*([-\d.]+)\)'
            m = re.match(pattern, msg.data.strip())
            if m:
                self.depth_2 = float(m.group(1))
                self.angle_2 = float(m.group(2))
                self.shift_x_2 = float(m.group(3))
                self.new_imaging_data_2 = True  # Mark that new imaging data has been received
        except Exception:
            pass
        
    def calculate_and_report(self):
        try:
            # 1. Get transform: map -> base_link
            from_frame = 'map'
            to_frame = 'base_link'
            now = rclpy.time.Time()
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                from_frame, to_frame, now, timeout=rclpy.duration.Duration(seconds=1.0)
            )
            x_base = trans.transform.translation.x
            y_base = trans.transform.translation.y
            q = trans.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            theta_base = math.atan2(siny_cosp, cosy_cosp)
            T_map_to_base = pose_to_matrix(x_base, y_base, theta_base)

            # --- Compute t1 pose in map frame (ONLY if new imaging data received) ---
            if self.new_imaging_data_1:
                T_t1_to_guilder_base = np.eye(3)
                T_t1_to_guilder_base[0, 2] = self.shift_x_1
                T_t1_to_guilder_base[1, 2] = self.depth_1
                angle_rad = math.radians(self.angle_1)
                T_t1_to_guilder_base[0, 0] = math.cos(angle_rad)
                T_t1_to_guilder_base[0, 1] = -math.sin(angle_rad)
                T_t1_to_guilder_base[1, 0] = math.sin(angle_rad)
                T_t1_to_guilder_base[1, 1] = math.cos(angle_rad)
                trans_mat = np.array([
                    [0., -1., 0.],
                    [1., 0., 0.],
                    [0., 0., 1.]])
                T_guilder_base_to_t1 = np.linalg.inv(T_t1_to_guilder_base)
                T_map_to_t1 = T_map_to_base @ np.linalg.inv(trans_mat) @ T_guilder_base_to_t1
                t1_map_x, t1_map_y, t1_map_theta = matrix_to_pose(T_map_to_t1)
                self.last_t1_in_map = (t1_map_x, t1_map_y, t1_map_theta)
                self.new_imaging_data_1 = False  # Reset after saving

            # --- Use last_t1_in_map for visualization and tf broadcast ---
            if self.last_t1_in_map is not None:
                t1_map_x, t1_map_y, t1_map_theta = self.last_t1_in_map
                # Broadcast tf for t1 in map frame
                t1_tf = TransformStamped()
                t1_tf.header.stamp = self.get_clock().now().to_msg()
                t1_tf.header.frame_id = "map"
                t1_tf.child_frame_id = "t1"
                t1_tf.transform.translation.x = t1_map_x
                t1_tf.transform.translation.y = t1_map_y
                t1_tf.transform.translation.z = 0.0
                t1_tf.transform.rotation.x = 0.0
                t1_tf.transform.rotation.y = 0.0
                t1_tf.transform.rotation.z = math.sin(t1_map_theta/2)
                t1_tf.transform.rotation.w = math.cos(t1_map_theta/2)
                self.tf_broadcaster.sendTransform(t1_tf)                
                self.publish_rectangle("T1", t1_map_x, t1_map_y, t1_map_theta, color=(0.2,0.8,0.2), scale=(0.55,0.75), id_num=1)
                self.publish_label("T1", t1_map_x, t1_map_y, t1_map_theta, id_num=1)

                # --- Calculate movement needed for t1 to see base_link ---
                dx = x_base - t1_map_x
                dy = y_base - t1_map_y
                distance = math.hypot(dx, dy)
                angle_to_base = math.atan2(dy, dx)
                angle_diff = angle_to_base - t1_map_theta
                angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))  # normalize

                # Optionally, log or publish this info
                # self.get_logger().info(
                #     f"t1 needs to move {distance:.2f}m and rotate {math.degrees(-angle_diff + math.pi/2):.2f} deg to see base_link"
                # )
                msg = String()
                msg.data = f"t1^{distance:.2f}^{math.degrees(-angle_diff + math.pi/2):.2f}"
                self.guider_transporter_pub.publish(msg)
                            
            # --- Compute t1 pose in map frame (ONLY if new imaging data received) ---
            if self.new_imaging_data_2:
                T_t2_center_to_t1_rear = np.eye(3)
                T_t2_center_to_t1_rear[0, 2] = self.shift_x_2
                T_t2_center_to_t1_rear[1, 2] = self.depth_2
                angle_rad = math.radians(self.angle_2)
                T_t2_center_to_t1_rear[0, 0] = math.cos(angle_rad)
                T_t2_center_to_t1_rear[0, 1] = -math.sin(angle_rad)
                T_t2_center_to_t1_rear[1, 0] = math.sin(angle_rad)
                T_t2_center_to_t1_rear[1, 1] = math.cos(angle_rad)
                
                T_t1_rear_to_t1_center = np.array([
                       [1., 0., 0.],
                       [0., 1., 0.392],
                       [0., 0., 1.]
                ])   
                T_t2_center_to_t1_center = T_t2_center_to_t1_rear @ T_t1_rear_to_t1_center
                T_t1_center_to_t2_center = np.linalg.inv(T_t2_center_to_t1_center)

                T_map_to_t2 = T_map_to_t1 @ T_t1_center_to_t2_center
                t2_map_x, t2_map_y, t2_map_theta = matrix_to_pose(T_map_to_t2)
                self.last_t2_in_map = (t2_map_x, t2_map_y, t2_map_theta)
                self.new_imaging_data_2 = False
                self.get_logger().info(f"T_t2_center_to_t1_rear\n{T_t2_center_to_t1_rear}")

            if self.last_t2_in_map is not None:
                t2_map_x, t2_map_y, t2_map_theta = self.last_t2_in_map
                # Broadcast tf for t2 in map frame
                t2_tf = TransformStamped()
                t2_tf.header.stamp = self.get_clock().now().to_msg()
                t2_tf.header.frame_id = "map"
                t2_tf.child_frame_id = "t2"
                t2_tf.transform.translation.x = t2_map_x
                t2_tf.transform.translation.y = t2_map_y
                t2_tf.transform.translation.z = 0.0
                t2_tf.transform.rotation.x = 0.0
                t2_tf.transform.rotation.y = 0.0
                t2_tf.transform.rotation.z = math.sin(t2_map_theta/2)
                t2_tf.transform.rotation.w = math.cos(t2_map_theta/2)
                self.tf_broadcaster.sendTransform(t2_tf)
                self.publish_rectangle("T2", t2_map_x, t2_map_y, t2_map_theta, color=(0.8,0.2,0.2), scale=(0.55,0.75), id_num=2)
                self.publish_label("T2", t2_map_x, t2_map_y, t2_map_theta, id_num=2)

                # --- Calculate movement needed for t1 to see base_link ---
                dx = t1_map_x - t2_map_x
                dy = t1_map_y - t2_map_y
                distance = math.hypot(dx, dy)
                angle_to_base = math.atan2(dy, dx)
                angle_diff = angle_to_base - t2_map_theta
                angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))  # normalize

                # Optionally, log or publish this info
                # self.get_logger().info(
                #     f"t1 needs to move {distance:.2f}m and rotate {math.degrees(-angle_diff + math.pi/2):.2f} deg to see base_link"
                # )
                msg = String()
                msg.data = f"t2^{distance:.2f}^{math.degrees(-angle_diff + math.pi/2):.2f}"
                self.guider_transporter_pub.publish(msg)

        except Exception as e:
            pass
            # self.get_logger().warn(f"Transform not available or error: {e}")

    def publish_rectangle(self, name, x, y, theta, color=(0.2, 0.8, 0.2), scale=(0.5, 0.3), namespace="rects", id_num=0):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = id_num
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.1
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = math.sin(theta/2)
        marker.pose.orientation.w = math.cos(theta/2)
        marker.scale.x = scale[0]  # width
        marker.scale.y = scale[1]  # height
        marker.scale.z = 0.05      # thickness
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 0.6
        self.marker_pub.publish(marker)

    def publish_label(self, name, x, y, theta, namespace="labels", id_num=0, color=(1.0, 1.0, 1.0)):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = id_num
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.2  # above rectangle
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.15  # text height
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0
        marker.text = name
        self.marker_pub.publish(marker)
def main(args=None):
    rclpy.init(args=args)
    node = BaseLinkPoseVector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
