import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
from rtabmap_msgs.msg import Info
from tf2_ros.transform_broadcaster import TransformBroadcaster
import logging
from triorb_core import robot as TriOrbRobot
from triorb_core import TriOrbDrive3Pose
from nav_msgs.msg import Path
import time
import os
import math
from rclpy.action import ActionServer, CancelResponse, GoalResponse, ActionClient
from adam_message.action import ExecuteTask
import threading
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from tf_transformations import euler_from_quaternion
from rclpy.executors import MultiThreadedExecutor
# Configure logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] [%(filename)s:%(lineno)d] %(message)s")


RESET = '\033[0m'
BOLD = '\033[1m'
YELLOW = '\033[93m'
CYAN = '\033[96m'
GREEN = '\033[92m'
RED = '\033[91m'
MAGENTA = '\033[95m'
WHITE = '\033[97m'

ID_NAME_MAP = {
    "t1": "Transporter 1",
    "t2": "Transporter 2",
    "guider": "Guider"
}

def rad(int):
    """Convert degrees to radians"""
    return int * (math.pi / 180.0)

def adam_info(msg):
    print(f"{WHITE}[{time.time():.4f}][INFO]{msg}{RESET}", flush=True)

def adam_warn(msg):
    print(f"{YELLOW}[{time.time():.4f}][WARN]{msg}{RESET}", flush=True)

def adam_error(msg):
    print(f"{RED}[{time.time():.4f}][ERROR]{BOLD}{msg}{RESET}", flush=True)

class TriOrbRobotNode(Node):
    def __init__(self):
        super().__init__('triorb_robot_node')

        # Declare parameters
        self.declare_parameter('robot_port', '/dev/triorb_robot')
        self.declare_parameter('acc', 500)
        self.declare_parameter('dec', 500)
        self.declare_parameter('std_vel', 0.25)
        self.declare_parameter('std_rot', 0.5)
        self.declare_parameter('torque', 500)
        self.declare_parameter('cmd_vel_timeout', 0.2)  # Timeout in seconds (e.g., 0.2 seconds)
        self.reentrant_group = ReentrantCallbackGroup()

        # Get parameters
        robot_port = self.get_parameter('robot_port').get_parameter_value().string_value
        acc = self.get_parameter('acc').get_parameter_value().integer_value
        dec = self.get_parameter('dec').get_parameter_value().integer_value
        std_vel = self.get_parameter('std_vel').get_parameter_value().double_value
        std_rot = self.get_parameter('std_rot').get_parameter_value().double_value
        torque = self.get_parameter('torque').get_parameter_value().integer_value
        cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').get_parameter_value().double_value
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_theta = 0.0 
        self.prev_time = self.get_clock().now()
        self.target_yaw = None      # Target yaw in radians
        self.rotating = False       # Flag: are we trying to rotate to a target?
        self.is_goal_reached = True

        self.robot_cmd_try = 3
        adam_info(f"Initializing the TriOrb Robot on port: {robot_port}")
        self.transporter_id = "guider"
        self._setup_action_server()
        self.count_lost_t1 = 0
        self.count_lost_t2 = 0

        # Parameters for robot state
        self.last_time = self.get_clock().now()
        self.vel_x, self.vel_y, self.omega = 0.0, 0.0, 0.0
        self.t1_followed = True
        self.t2_followed = True
        self.t1_bypass_followed = False
        self.t2_bypass_followed = False
        self.robot_command_lock = threading.Lock()
        self.lost_odom_flag = False
        self.is_lost_loop_closure = False
        self.slow_speed_wait_for_loop_closure = 1.0
        self._cancel_t1_follow_timer = None
        self._cancel_t2_follow_timer = None

        self.qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        # Initialize the robot
        try:
            self.robot = TriOrbRobot(robot_port)
            adam_info("Robot initialized successfully!")
            adam_info(f"Robot Info: {self.robot.get_info()}")

            # Use these parameters to configure the robot
            self.robot.write_config({
                "acc": acc,
                "dec": dec,
                "std-vel": std_vel,
                "std-rot": std_rot,
                "torque": torque,
            })
            adam_info(f"Robot Config : {self.robot.read_config()}")  # Read current configuration
            
            # Resetting Odometry origin to 0
            for _ in range(self.robot_cmd_try):
                self.robot.reset_origin()

            # Exciting the motors
            for _ in range(self.robot_cmd_try):
                self.robot.wakeup()
            time.sleep(2)
        except Exception as e:
            self.get_logger().error(f"Failed to initialize the robot: {e}")
            self.robot = None 
            
            
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.ui_control_sub = self.create_subscription( 
            String, 
            'guider/comm/ui_control', 
            self.ui_control_callback, 
            self.qos,
            callback_group=self.reentrant_group)
        
        self.transporter_guider_sub_t1 = self.create_subscription( 
            String, 
            't1/comm/transporter_guider', 
            self.t1_guider_callback, 
            self.qos,
            callback_group=self.reentrant_group)
        
        self.transporter_guider_sub_t2 = self.create_subscription( 
            String, 
            't2/comm/transporter_guider', 
            self.t2_guider_callback, 
            self.qos,
            callback_group=self.reentrant_group)


        self.goal_reached_sub = self.create_subscription( 
            Bool, 
            '/guider_goal_reached_', 
            self.goal_reached_callback, 
            self.qos,
            callback_group=self.reentrant_group)

        self.rtabmap_info_subscription = None
        # self.rtabmap_info_subscription = self.create_subscription(
        #     Info,
        #     '/info',
        #     self.rtabmap_info_callback,
        #     self.qos,
        #     callback_group=self.reentrant_group
        # ) 
        self.localization_pose_subscription = None

    def cmd_vel_callback(self, msg):
        """Callback for /cmd_vel to set robot velocity."""
        # Update last time a command was received
        self.last_time = self.get_clock().now()

        # Convert ROS frame velocities to robot frame velocities
        vel_x = -msg.linear.y
        vel_y = msg.linear.x
        omega = -msg.angular.z
        omega = max(min(omega, 0.25), -0.25)  # Clamp omega to [-0.25, 0.25]
        adam_info(f"Received cmd_vel: linear({vel_x}, {vel_y}), angular({omega})")
        if not self.is_goal_reached:
            if (abs(vel_x) <= 0.0001 and abs(self.prev_x) > 0.01) or (abs(vel_y) <= 0.0001 and abs(self.prev_y) > 0.01):
                adam_info(f"GOAL not REACHED, ignore stop")
                return
        self.prev_x = vel_x
        self.prev_y = vel_y
        self.prev_theta = omega
        try:
            # Setting Velocity relative to robot base / base_link
            for _ in range(self.robot_cmd_try):
                if self.lost_odom_flag:
                    self.safe_robot_command("set_vel_relative", 0, 0, 0, 350, 350)
                    return
                if (self.t1_followed or self.t1_bypass_followed) and (self.t2_followed or self.t2_bypass_followed):                    
                    self.safe_robot_command("set_vel_relative", vel_x, vel_y, omega, 350, 350)
                    return
                self.safe_robot_command("set_vel_relative", 0, 0, 0, 350, 350)
                return
        except Exception as e:
            self.get_logger().error(f"Error setting velocity: {e}")

    def ui_control_callback(self, msg):
        """Callback for UI control messages to set robot velocity."""
        # Example data: "0.5,0.0,1.57" (x,y,theta)
        next_pos = msg.data.split(',')
        if len(next_pos) != 3:
            self.get_logger().error(f"Invalid UI control message format: {msg.data}")
            return
        try:
            # Parse the position and orientation from the message
            x = float(next_pos[0])
            y = float(next_pos[1])
            theta = float(next_pos[2])
            # Move to the specified position
            adam_info(f"UI Control - Setting position: x:{x:.2f}, y:{y:.2f}, theta:{theta:.2f}")
            
        except ValueError as e:
            self.get_logger().error(f"Error parsing UI control message: {e}")
            return

    def t1_guider_callback(self, msg):
        split_msg = msg.data.split('^')
        if len(split_msg) != 2:
            # Handle error
            return
        task = split_msg[0]
        value = split_msg[1]
        if task == "followed":
            if value == "1":
                self.t1_followed = True
                # Cancel any pending timer
                if self._cancel_t1_follow_timer is not None:
                    self._cancel_t1_follow_timer.cancel()
                    self._cancel_t1_follow_timer = None
            elif value == "0":
                # Start a timer that sets to False after 2 seconds
                if self._cancel_t1_follow_timer is not None:
                    self._cancel_t1_follow_timer.cancel()
                self._cancel_t1_follow_timer = threading.Timer(2, self.set_t1_followed_false)
                self._cancel_t1_follow_timer.start()
            elif value == "2":
                self.t1_bypass_followed = True

    def set_t1_followed_false(self):
        self.t1_followed = False
        self._cancel_t1_follow_timer = None

    def t2_guider_callback(self, msg):
        split_msg = msg.data.split('^')
        if len(split_msg) != 2:
            # Handle error
            return
        task = split_msg[0]
        value = split_msg[1]
        if task == "followed":
            if value == "1":
                self.t2_followed = True
                # Cancel any pending timer
                if self._cancel_t2_follow_timer is not None:
                    self._cancel_t2_follow_timer.cancel()
                    self._cancel_t2_follow_timer = None
            elif value == "0":
                # Start a timer that sets to False after 2 seconds
                if self._cancel_t2_follow_timer is not None:
                    self._cancel_t2_follow_timer.cancel()
                self._cancel_t2_follow_timer = threading.Timer(2, self.set_t2_followed_false)
                self._cancel_t2_follow_timer.start()
            elif value == "2":
                self.t2_bypass_followed = True

    def set_t2_followed_false(self):
        self.t2_followed = False
        self._cancel_t2_follow_timer = None

    def localization_callback(self, msg: PoseWithCovarianceStamped):
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.theta = yaw
        # adam_info(f"Localization updated: x={msg.pose.pose.position.x:.2f}, y={msg.pose.pose.position.y:.2f}, theta={self.theta:.2f} rad")

    def rtabmap_info_callback(self, msg):
        if msg.proximity_detection_id > 0:
            adam_info(f'LOOP CLOSURE DETECTED! ref_id={msg.ref_id}, loop_closure_id={msg.loop_closure_id} proximity_detection_id={msg.proximity_detection_id}')
            self.is_lost_loop_closure = False
            return   
        
    def goal_reached_callback(self, msg):
        adam_info(f' GOAL REACHED: {msg.data}')
        self.is_goal_reached = msg.data
        if self.is_goal_reached:
            adam_info(f"GOAL REACHED, stop robot")
            self.safe_robot_command("set_vel_relative", 0, 0, 0, 200, 200)
    
    def _setup_action_server(self):
        """Setup action server for UI communication"""
        action_name = f'guider_execute_task'
        
        self._action_server = ActionServer(
            self,
            ExecuteTask,
            action_name,
            execute_callback=self.execute_action_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.reentrant_group
        )
        
        self._current_goal_handle = None
        self._is_executing_action = False
        
        adam_info(f'🎯 Action server created: {action_name}')
        
    # ============ ACTION SERVER INTERFACE ============
    
    def goal_callback(self, goal_request):
        """Accept or reject action goal"""
        adam_info(f'🎯 {ID_NAME_MAP[self.transporter_id]}: Received action goal: {goal_request.step}')
        
        if self._is_executing_action:
            adam_warn(f'{ID_NAME_MAP[self.transporter_id]}: Already executing a task, rejecting goal')
            return GoalResponse.REJECT
            
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Handle action cancellation"""
        adam_info(f'🛑 {ID_NAME_MAP[self.transporter_id]}: Received cancel request')
        return CancelResponse.ACCEPT

    def execute_action_callback(self, goal_handle):
        """Execute action task"""
        adam_info(f'⚡ {ID_NAME_MAP[self.transporter_id]}: Executing action task: {goal_handle.request.step}')
        
        self._current_goal_handle = goal_handle
        self._is_executing_action = True
        
        request = goal_handle.request
        step = request.step
        value = request.value
        
        feedback_msg = ExecuteTask.Feedback()
        
        try:            
            success = self._execute_task(step, value, goal_handle, feedback_msg)
            
            result = ExecuteTask.Result()
            result.success = success
            result.message = f"{ID_NAME_MAP[self.transporter_id]}: Task {step} {'completed' if success else 'failed'}"
            result.duration = 5.0
            
            if success:
                goal_handle.succeed()
                adam_info(f'✅ {ID_NAME_MAP[self.transporter_id]}: Task {step} completed successfully')
            else:
                goal_handle.abort()
                adam_error(f'❌ {ID_NAME_MAP[self.transporter_id]}: Task {step} failed')
            
            return result
            
        except Exception as e:
            adam_error(f'💥 {ID_NAME_MAP[self.transporter_id]}: Task execution error: {str(e)}')
            result = ExecuteTask.Result()
            result.success = False
            result.message = f"{ID_NAME_MAP[self.transporter_id]}: Task failed with error: {str(e)}"
            result.duration = 0.0
            goal_handle.abort()
            return result
        
        finally:
            self._is_executing_action = False
            self._current_goal_handle = None

    def _execute_task(self, step, value, goal_handle=None, feedback_msg=None):
        """Execute task using robot methods with real-time completion"""
        adam_info(f'🚀 {ID_NAME_MAP[self.transporter_id]}: Starting {step} with value "{value}"')
        
        success_flag = [False]
        completion_flag = [False]
        
        def execute_command():
            try:
                if step == "Position Step":
                    parts = value.split('_')
                    if len(parts) != 3:
                        adam_error(f"Invalid Position Step value: {value}")
                        success_flag[0] = False
                        completion_flag[0] = True
                        return
                    x, y, theta = map(float, parts)
                    success_flag[0] = self.position_step(x, y, theta)
                elif step == "Lost Odom":
                    success_flag[0] = self.lost_odom(value)
                elif step == "Set velocity":
                    parts = value.split('_')
                    if len(parts) != 4:
                        adam_error(f"Invalid Set Velocity value: {value}")
                        success_flag[0] = False
                        completion_flag[0] = True
                        return
                    x, y, theta, duration = map(float, parts)
                    success_flag[0] = self.set_velocity(x, y, theta, duration)
                elif step == "Rotate Step":
                    parts = value.split('_')
                    if len(parts) != 4:
                        adam_error(f"Invalid Rotate Step value: {value}")
                        success_flag[0] = False
                        completion_flag[0] = True
                        return
                    qx, qy, qz, qw = map(float, parts)
                    success_flag[0] = self.rotate_to_orientation(qx, qy, qz, qw)
                elif step == "Get Loop Closure":
                    self.is_lost_loop_closure = True
                    success_flag[0] = self.rotate_to_get_loop_closure()
                else:
                    adam_warn(f"Unknown step: {step}")
                    success_flag[0] = False
                    
                completion_flag[0] = True
                
            except Exception as e:
                adam_error(f"Error in command execution: {e}")
                success_flag[0] = False
                completion_flag[0] = True
        
        task_thread = threading.Thread(target=execute_command)
        task_thread.start()
        
        if goal_handle and feedback_msg:
            start_time = time.time()
            
            while not completion_flag[0]:
                if goal_handle.is_cancel_requested:
                    adam_info(f'🛑 {ID_NAME_MAP[self.transporter_id]}: Task canceled by client')
                    task_thread.join(timeout=1.0)
                    return False
                
                elapsed_time = time.time() - start_time
                
                if step in ["Rotate", "Stop"]:
                    progress = min(int(elapsed_time * 180), 90)
                elif step in ["Go to position"]:
                    progress = min(int(elapsed_time * 90), 90)
                else:
                    progress = min(int(elapsed_time * 6), 90)
                
                feedback_msg.progress_percent = progress
                feedback_msg.current_status = f"{ID_NAME_MAP[self.transporter_id]}: {step}... {progress}%"
                feedback_msg.elapsed_time = elapsed_time
                
                goal_handle.publish_feedback(feedback_msg)
                
                if int(elapsed_time) % 2 == 0 and elapsed_time - int(elapsed_time) < 0.1:
                    adam_info(f'📊 {ID_NAME_MAP[self.transporter_id]}: {step} progress: {progress}%')
                
                time.sleep(0.1)
            
            final_elapsed = time.time() - start_time
            feedback_msg.progress_percent = 100
            feedback_msg.current_status = f"{ID_NAME_MAP[self.transporter_id]}: {step} completed"
            feedback_msg.elapsed_time = final_elapsed
            goal_handle.publish_feedback(feedback_msg)
        
        task_thread.join()
        
        adam_info(f'🎉 {ID_NAME_MAP[self.transporter_id]}: {step} execution completed')
        return success_flag[0]
    
    def position_step(self, x, y, theta, duration=3):
        """
        Move the robot to target position using y-movement and rotation sequence.
        """
        
        # Step 1: Move in y-direction first
        duration_xy = max(abs(x),abs(y), 0.2) / 0.2
        duration_theta = max(abs(theta), 0.2) / 0.1
        if abs(y) > 0:
            self.safe_robot_command("set_vel_relative", 0, y/duration_xy, 0)
            time.sleep(duration)
            self.safe_robot_command("set_vel_relative", 0, 0, 0)
        if abs(theta) > 0:
            self.safe_robot_command("set_vel_relative", 0, 0, theta/duration_theta)
            time.sleep(duration_theta)
            self.safe_robot_command("set_vel_relative", 0, 0, 0)
        if abs(x) > 0:            
            if abs(theta - (-math.pi/2)) < 0.1:  # -90 degrees
                y_direction = -x  # If we want x=-1, we need y=+1
            elif abs(theta - (math.pi/2)) < 0.1:  # +90 degrees
                # After +90° rotation: +y → +x, -y → -x
                y_direction = x   # If we want x=-1, we need y=-1
            else:
                # For other angles, use the sign of x
                y_direction = -abs(x) if x < 0 else abs(x)
            self.safe_robot_command("set_vel_relative", 0, y_direction/duration_xy, 0)
            time.sleep(duration)
            self.safe_robot_command("set_vel_relative", 0, 0, 0)
        return True
    
    def rotate_to_orientation(self, qx, qy, qz, qw, max_speed=0.4, min_speed=0.1, tolerance=0.1, timeout=60):
        """
        Rotate robot in place to match the given quaternion orientation (map frame).
        - max_speed: maximum angular velocity (rad/s)
        - min_speed: minimum angular velocity (rad/s) (used when close to target)
        """

        if self.localization_pose_subscription is None:
            self.localization_pose_subscription = self.create_subscription(PoseWithCovarianceStamped, '/localization_pose', self.localization_callback, self.qos,
                callback_group=self.reentrant_group)
            
        import tf_transformations
        import math
        
        # Convert quaternion to yaw, normalize to [-pi, pi]
        _, _, target_yaw = tf_transformations.euler_from_quaternion([qx, qy, qz, qw])
        adam_info(f"Rotating to yaw {target_yaw:.2f} rad")

        start_time = time.time()
        while True:
            current_yaw = getattr(self, 'theta', 5.0)

            error = self.normalize_angle(target_yaw - current_yaw)
            if abs(error) < tolerance:
                if not self.safe_robot_command("set_vel_relative", 0, 0, 0):
                    return False
                adam_info(f"Reached target yaw: {current_yaw:.2f} rad (target: {target_yaw:.2f})")
                self.destroy_subscription(self.localization_pose_subscription)
                self.localization_pose_subscription = None
                return True
            # Adaptive gain zone: slow earlier by shaping the curve
            k_p = 0.20
            raw_speed = k_p * error

            # Instead of hard clamp, shape it with tanh or scaled sigmoid
            shaped_speed = max_speed * math.tanh(raw_speed / max_speed)
            if not self.safe_robot_command("set_vel_relative", 0, 0, -shaped_speed):
                return False

            # Check timeout
            if time.time() - start_time > timeout:
                self.safe_robot_command("set_vel_relative", 0, 0, 0)
                adam_error("Timeout rotating to orientation")
                return False

            time.sleep(0.1)

    @staticmethod
    def normalize_angle(angle):
        import math
        return math.atan2(math.sin(angle), math.cos(angle))
    
    def set_velocity(self, x, y, theta, duration=1):
        """
        Set robot velocity for a specified duration.
        Args:
            x: Velocity in X direction
            y: Velocity in Y direction
            theta: Angular velocity (rotation)
            duration: Duration to apply the velocity
        """
        try:
            adam_info(f"🎯 {ID_NAME_MAP[self.transporter_id]} Setting velocity: x={x}, y={y}, theta={theta}, duration={duration}")
            self.safe_robot_command("set_vel_relative", x, y, theta, 350, 350)
            time.sleep(duration)
            self.safe_robot_command("set_vel_relative", 0, 0, 0, 350, 350)
            adam_info(f"✅ {ID_NAME_MAP[self.transporter_id]} Velocity set successfully")
            return True
        except Exception as e:
            adam_error(f"❌ {ID_NAME_MAP[self.transporter_id]} Error setting velocity: {e}")
            return False
    
    def rotate_to_get_loop_closure(self):
        # To resubscribe:
        if self.rtabmap_info_subscription is None:
            self.rtabmap_info_subscription = self.create_subscription(
                Info,
                '/info',
                self.rtabmap_info_callback,
                self.qos,
                callback_group=self.reentrant_group
            )        
        while True:
            m_is_lost_loop_closure = self.is_lost_loop_closure
            if not m_is_lost_loop_closure: 
                # To unsubscribe:
                if self.rtabmap_info_subscription is not None:
                    self.destroy_subscription(self.rtabmap_info_subscription)
                    self.rtabmap_info_subscription = None
                self.safe_robot_command("set_vel_relative", 0, 0, 0, 350, 350)
                return True
            else:                
                self.safe_robot_command("set_vel_relative", 0, 0, 0.1, 350, 350)
                pass
            time.sleep(1)

    def lost_odom(self, value=None):
        """Reset robot odometry"""
        try:
            adam_info(f"🔄 {ID_NAME_MAP[self.transporter_id]} Guider odometry status change {value}")
            if value == "true" or value == "True":
                self.lost_odom_flag = True
            elif value == "false" or value == "False":
                self.lost_odom_flag = False
            return True
        except Exception as e:
            adam_error(f"❌ {ID_NAME_MAP[self.transporter_id]} Error resetting odometry: {e}")
            return False
        
    def safe_robot_command(self, command, *args):
        """Thread-safe robot command execution"""
        try:
            with self.robot_command_lock:
                return self.mock_robot_command(command, *args)
        except Exception as e:
            adam_error(f"💥 Thread-safe robot command error: {e}")
            return False

    def mock_robot_command(self, command, *args):
        """Execute robot command - real or simulated"""
        if self.robot is None:
            adam_info(f"🎮 {ID_NAME_MAP[self.transporter_id]} SIM: {command}({args})")
            time.sleep(0.1)
            if command == "get_pos":
                x_mock,y_mock, w_mock = args
                from types import SimpleNamespace
                mock_pose = SimpleNamespace(x=x_mock, y=y_mock, w=w_mock)
                return [mock_pose]
            else:
                adam_info(f"Robot SIM command: {command}")
                return True
        else:
            try:
                # adam_info(f"🤖 {ID_NAME_MAP[self.transporter_id]} REAL: {command}({args})")
                if command == "set_vel_relative":
                    self.robot.set_vel_relative(*args)
                elif command == "set_lifter_move":
                    self.robot.set_lifter_move(*args)
                elif command == "set_vel_absolute":
                    self.robot.set_vel_absolute(*args)
                elif command == "brake":
                    self.robot.brake()
                elif command == "sleep":
                    self.robot.sleep()
                elif command == "wakeup":
                    self.robot.wakeup()
                elif command == "get_pos":
                    self.robot.get_pos()
                elif command == "reset_origin":
                    self.robot.reset_origin()
                elif command == "join":
                    self.robot.join()
                else:
                    adam_warn(f"Unknown robot command: {command}")
                return True
            except Exception as e:
                adam_error(f"💥 Robot command error: {e}")
                return False

    def shutdown_node(self, msg):
        """Shutdown the node gracefully with a custom message."""
        self.get_logger().error(msg)
        self.destroy_node()
        try:
            rclpy.shutdown()  # Safe shutdown attempt
        except RuntimeError:
            pass  # Ignore error if rclpy is not initialized
        exit(1)  # Optionally, exit with a non-zero status to indicate failure

def main(args=None):
    rclpy.init(args=args)
    robot_node = TriOrbRobotNode()
    executor = MultiThreadedExecutor()
    executor.add_node(robot_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
