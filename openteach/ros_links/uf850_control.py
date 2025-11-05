import numpy as np
import time
from copy import deepcopy as copy

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from threading import Thread

from sensor_msgs.msg import JointState
from xarm_msgs.msg import RobotMsg
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from std_srvs.srv import Trigger
# from moveit_commander import MoveGroupCommander
from xarm_msgs.srv import GetFloat32List
from openteach.constants import SCALE_FACTOR
from scipy.spatial.transform import Rotation as R

HAND_JOINT_STATE_TOPIC = '/allegroHand/joint_states'
HAND_COMMANDED_JOINT_STATE_TOPIC = '/allegroHand/commanded_joint_states'

UF850_JOINT_STATE_TOPIC = 'joint_states'
UF850_ROBOT_STATE_TOPIC = 'robot_states'

HW_NS = 'ufactory'

TWIST_RATE_HZ = 200.0
FRAME_ID = 'link_base'

JJ_RATE_HZ = 200.0
JJ_MAX_DELTA_PER_STEP = 0.015
JJ_TOLERANCE = 0.003
JJ_TIMEOUT_SEC = 5.0
JJ_FRAME_ID = 'link_base'



class DexArmControl():
    def __init__(self, record_type=None, robot_type='both'):
        # Initialize rclpy and node
        if not rclpy.ok():
            rclpy.init(args=None)
        self._node = Node('dex_arm', automatically_declare_parameters_from_overrides=True)
        self._node.get_logger().info('dex_arm (ROS 2 Humble) init start')
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self._node)
        self._cbg = ReentrantCallbackGroup()

        # Set Parameters
        def param(name, default):
            if not self._node.has_parameter(name):
                self._node.declare_parameter(name, default)
                return default
            return self._node.get_parameter(name).value
        
        ros_queue_size_ = int(param('ros_queue_size', 1)) # For Qos
        self._xarm_ns = str(param('xarm.hw_ns', HW_NS))
        self._joint_states_topic = str(param('joint_states_topic', f'/{self._xarm_ns}/{UF850_JOINT_STATE_TOPIC}'))
        self._robot_states_topic = str(param('robot_states_topic', f'/{self._xarm_ns}/{UF850_ROBOT_STATE_TOPIC}'))
        cartesian_command_in_topic_ = str(param('moveit_servo.cartesian_command_in_topic', '/servo_server/delta_twist_cmds'))
        joint_command_in_topic_ = str(param('moveit_servo.joint_command_in_topic', '/servo_server/delta_joint_cmds'))
        self._command_frame = str(param('moveit_servo.planning_frame', FRAME_ID))

        self._node.get_logger().info(f'DexArmControl params: _xarm_ns={self._xarm_ns}, _joint_states_topic={self._joint_states_topic}, _robot_states_topic={self._robot_states_topic}, cartesian_command_in_topic_={cartesian_command_in_topic_}, joint_command_in_topic_={joint_command_in_topic_}, _command_frame={self._command_frame}')

        # Internal state storage
        self.uf850_joint_state = None  # latest sensor_msgs/JointState
        self.uf850_robot_state = None  # latest xarm_msgs/msg/RoboMsg.msg
        self._current_mode = None
        self._pose_future = None          # track in-flight request
        self.uf850_tcp_position_aa = None # latest cached pose dict
        self._twist_active = False
        self._twist_deadline = None
        self._twist_cmd = np.zeros(6, dtype=float)
        self._jog_active = False
        self._jog_session = None  # dict set by move_arm_joint

        # persistent timers (never canceled)
        self._twist_timer = self._node.create_timer(
            1.0/float(TWIST_RATE_HZ),
            self._publish_twist_tick,
            callback_group=self._cbg
        )
        self._jog_timer = self._node.create_timer(
            1.0/float(JJ_RATE_HZ),
            self._publish_jog_tick,
            callback_group=self._cbg
        )

        # QoS compatible with MoveIt Servo
        qos = QoSProfile(depth=ros_queue_size_)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.history = HistoryPolicy.KEEP_LAST

        # ── Publishers ────────────────────────────────────────────────────────
        self._twist_pub = self._node.create_publisher(TwistStamped, cartesian_command_in_topic_, qos)
        self._joint_pub = self._node.create_publisher(JointJog, joint_command_in_topic_, qos)

        # ── Services ─────────────────────────────────────────────────────────
        self._servo_start_cli = self._node.create_client(Trigger, '/servo_server/start_servo')
        self._servo_stop_cli = self._node.create_client(Trigger, '/servo_server/stop_servo')
        self._get_pos_aa_cli = self._node.create_client(GetFloat32List, f'/{self._xarm_ns}/get_position_aa')

        # ── Subscriptions ─────────────────────────────────────────────────────
        self._uf850_joint_state_sub = self._node.create_subscription(
            JointState, self._joint_states_topic, self._callback_uf850_joint_state, qos
        )
        self._uf850_robot_state_sub = self._node.create_subscription(
            RobotMsg, self._robot_states_topic, self._callback_uf850_robot_state, qos
        )

        self._spin_thread = Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

        # Start Servo server if available (non-blocking)
        if self._servo_start_cli.wait_for_service(timeout_sec=0.5):
            self._servo_start_cli.call_async(Trigger.Request())

        # ── Blocking check for first robot state ───────────────────────────────
        start_time = time.time()
        timeout = 5.0  # seconds
        self._node.get_logger().info("Waiting for /robot_states message...")

        while (self.uf850_robot_state) is None and (time.time() - start_time) < timeout:
            continue
        if self.uf850_robot_state is None:
            self._node.get_logger().warn("No /robot_states received...")
        else:
            self._node.get_logger().info("Received first /robot_states message.")

        # ── Blocking check for first joint state ───────────────────────────────
        start_time = time.time()
        timeout = 5.0  # seconds
        self._node.get_logger().info("Waiting for /joint_states message...")

        while (self.uf850_joint_state) is None and (time.time() - start_time) < timeout:
            continue
        if self.uf850_joint_state is None:
            self._node.get_logger().warn("No /joint_states received...")
        else:
            self._node.get_logger().info("Received first /joint_states message.")

    # Rostopic callback functions
    def _callback_uf850_joint_state(self, joint_state: JointState):
        self.uf850_joint_state = joint_state
    
    def _callback_uf850_robot_state(self, robot_state: RobotMsg):
        self.uf850_robot_state = robot_state

    # ── Get Arm Cartesian State (RPY) ────────────────────────────────────────────────────────
    #   return raw state [x, y, z, roll, pitch, yaw]
    def get_arm_cartesian_coords(self):
        if self.uf850_robot_state is None:
            return None
        
        raw_cartesian_state = copy(self.uf850_robot_state)
        return raw_cartesian_state.pose
    
    #   return formatted state with timestamp
    #   dict['position': np.array(x, y, z), 'orientation': np.array(roll, pitch, yaw)]
    def get_arm_cartesian_state(self):
        if self.uf850_robot_state is None:
            return None
        
        raw_cartesian_state = copy(self.uf850_robot_state)

        cartesian_state = dict( 
            position = np.array([
                raw_cartesian_state.pose[0], raw_cartesian_state.pose[1], raw_cartesian_state.pose[2]
            ], dtype = np.float32),
            orientation = np.array([
                raw_cartesian_state.pose[3], raw_cartesian_state.pose[4], raw_cartesian_state.pose[5]
            ], dtype=np.float32),
            timestamp = raw_cartesian_state.header.stamp.secs + (raw_cartesian_state.header.stamp.nsecs * 1e-9)
        )
        return cartesian_state
    
    # ── Get Arm Cartesian State (Axis Angle) ────────────────────────────────────────────────────────
    def get_arm_cartesian_coords_aa(self):
        if self.uf850_robot_state is None:
            return None
        
        raw_cartesian_state = copy(self.uf850_robot_state)
        x, y, z = raw_cartesian_state.pose[0], raw_cartesian_state.pose[1], raw_cartesian_state.pose[2]
        roll, pitch, yaw = raw_cartesian_state.pose[3], raw_cartesian_state.pose[4], raw_cartesian_state.pose[5]
        oritentation_aa = self.uf850_pose_rpy_to_aa(roll, pitch, yaw)
        return np.array([x, y, z, oritentation_aa[0], oritentation_aa[1], oritentation_aa[2]], dtype=np.float32)
        self._node.get_logger().info(
            f"coords_aa: {roll}, {pitch}, {yaw}"
        )
        # return np.array([x, y, z, roll, pitch, yaw], dtype=np.float32)

    def get_arm_cartesian_state_aa(self):
        if self.uf850_robot_state is None:
            return None
        
        raw_cartesian_state = copy(self.uf850_robot_state)
        x, y, z = raw_cartesian_state.pose[0], raw_cartesian_state.pose[1], raw_cartesian_state.pose[2]
        roll, pitch, yaw = raw_cartesian_state.pose[3], raw_cartesian_state.pose[4], raw_cartesian_state.pose[5]
        oritentation_aa = self.uf850_pose_rpy_to_aa(roll, pitch, yaw)
        
        cartesian_state_aa = dict( 
            position = np.array([
                x, y, z
            ], dtype = np.float32),
            orientation = np.array([
                oritentation_aa[0], oritentation_aa[1], oritentation_aa[2]
            ], dtype=np.float32),
            timestamp = raw_cartesian_state.header.stamp.secs + (raw_cartesian_state.header.stamp.nsecs * 1e-9)
        )
        return cartesian_state_aa


    
    # ── Request arm joint state (Servo Angles) ──────────────────────────────────────────────────────── 
    def get_arm_position(self):
        if self.uf850_joint_state is None:
            return None
        return np.array(self.uf850_joint_state.position, dtype = np.float32)
    
    def get_arm_velocity(self):
        if self.uf850_joint_state is None:
            return None
        return np.array(self.uf850_joint_state.velocity, dtype = np.float32)
    
    def get_arm_torque(self):
        if self.uf850_joint_state is None:
            return None
        return np.array(self.uf850_joint_state.effort, dtype = np.float32)
    
    #  Formatted with time stamp
    def get_arm_joint_state(self):
        if self.uf850_joint_state is None:
            return None
        
        raw_joint_state = copy(self.uf850_joint_state)

        joint_state = dict(
            position = np.array(raw_joint_state.position[:6], dtype = np.float32),
            velocity = np.array(raw_joint_state.velocity[:6], dtype = np.float32),
            effort = np.array(raw_joint_state.effort[:6], dtype = np.float32),
            timestamp = raw_joint_state.header.stamp.secs + (raw_joint_state.header.stamp.nsecs * 1e-9)
        )
        return joint_state
    
    # def move_arm(self, uf850_angles):
        
    
    # ── Command Timer Ticks ────────────────────────────────────────────────────────
 
    def _publish_twist_tick(self):
        if not self._twist_active:
            return
        now = self._node.get_clock().now()
        if self._twist_deadline is not None and now >= self._twist_deadline:
            self._twist_active = False
            return

        x, y, z, p, q, r = self._twist_cmd
        msg = TwistStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self._command_frame
        # msg.twist.linear.x = x
        # msg.twist.linear.y = y
        # msg.twist.linear.z = z
        msg.twist.linear.x = 0.0
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = p
        msg.twist.angular.y = q
        msg.twist.angular.z = r
        self._twist_pub.publish(msg)
        # self._node.get_logger().info(
        #     f"Published Twist Command Translation: {x}, {y}, {z}, {np.rad2deg(p)}, {np.rad2deg(q)}, {np.rad2deg(r)} in {msg.header.frame_id}"
        # )
        # self._node.get_logger().info(
        #     f"Published Twist Command Orientation: {np.rad2deg(p)}, {np.rad2deg(q)}, {np.rad2deg(r)} in {msg.header.frame_id}"
        # )

    def _publish_jog_tick(self):
        if not self._jog_active or self._jog_session is None:
            return

        s = self._jog_session
        now = self._node.get_clock().now()

        if now >= s["deadline"]:
            self._node.get_logger().warn("move_arm_joint(): timeout; stopping jog stream.")
            self._jog_active = False
            return

        js = self.uf850_joint_state
        if js is None or len(js.position) != s["goal"].size:
            self._node.get_logger().warn("move_arm_joint(): missing/invalid /joint_states during stream.")
            self._jog_active = False
            return

        curr = np.array(js.position, dtype=np.float32)
        delta = s["goal"] - curr
        if np.all(np.abs(delta) <= s["tol"]):
            self._node.get_logger().info("move_arm_joint(): goal reached; stopping jog stream.")
            self._jog_active = False
            return

        step = np.clip(delta, -s["max_step"], s["max_step"])

        msg = JointJog()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = s["frame_id"]
        msg.joint_names = s["names"]
        msg.displacements = step.tolist()
        msg.velocities = [0.0] * len(step)
        self._joint_pub.publish(msg)


    # ── Move Arm Joint Angles (Joint Jog) ──────────────────────────────────────────────────────── 
    def move_arm_joint(self, joint_angles):
        if self.uf850_joint_state is None:
            self._node.get_logger().warn("move_arm_joint(): no ufactory/joint_states yet; cannot jog.")
            return False

        current_names = list(self.uf850_joint_state.name or [])
        if not current_names:
            self._node.get_logger().warn("move_arm_joint(): ufactory/joint_states has no names.")
            return False

        goal = np.array(joint_angles, dtype=float).reshape(-1)

        self._jog_session = {
            "names": current_names,
            "goal": goal,
            "rate_hz": float(JJ_RATE_HZ),
            "max_step": float(JJ_MAX_DELTA_PER_STEP),
            "tol": float(JJ_TOLERANCE),
            "deadline": self._node.get_clock().now() + rclpy.time.Duration(seconds=float(JJ_TIMEOUT_SEC)),
            "frame_id": JJ_FRAME_ID
        }
        self._jog_active = True   # <-- was _job_active typo; fix to _jog_active
        return True

    
    def move_arm_cartesian_velocity(self, cartesian_velocity_values, duration):
        if self._twist_pub is None:
            self._node.get_logger().error("Twist publisher not initialized.")
            return False

        self._twist_cmd = cartesian_velocity_values.astype(float).reshape(6)
        self._twist_deadline = self._node.get_clock().now() + rclpy.time.Duration(seconds=float(duration))
        self._twist_active = True
        return True

    # ── Helper Functions ────────────────────────────────────────────────────────
    def uf850_pose_rpy_to_aa(self, roll, pitch, yaw):
        rot = R.from_euler('XYZ', [roll, pitch, yaw], degrees=False)
        rotvec = rot.as_rotvec() # axis-angle as a vector; norm = angle(rad)
        return rotvec

    def uf850_pose_aa_to_affine(self,pose_aa: np.ndarray) -> np.ndarray:
        """Converts a robot pose in axis-angle format to an affine matrix.
        Args:
            pose_aa (list): [x, y, z, ax, ay, az] where (x, y, z) is the position and (ax, ay, az) is the axis-angle rotation.
            x, y, z are in mm and ax, ay, az are in radians.
        Returns:
            np.ndarray: 4x4 affine matrix [[R, t],[0, 1]]
        """

        rotation = R.from_rotvec(pose_aa[3:]).as_matrix()
        translation = np.array(pose_aa[:3]) / SCALE_FACTOR

        return np.block([[rotation, translation[:, np.newaxis]],
                        [0, 0, 0, 1]])
    
    # TODO: 
    def home_arm(self):
        # Possibly can enable cartesian position control with xarm_planner running at the same time as moveit servo
        # Could conflict with controllers though
        pass

    def reset_arm(self):
        self.home_arm()

    # ── Hand/Gripper Functions ────────────────────────────────────────────────────────
    def get_gripper_state(self):
        pass
        

