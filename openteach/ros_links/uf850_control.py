import numpy as np
import time
from types import SimpleNamespace

from copy import deepcopy as copy

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from moveit_msgs.srv import ServoCommandType


class DexArmControl():
    def __init__(self, record_type=None, robot_type='both'):
        # Initialize rclpy and node
        if not rclpy.ok():
            rclpy.init(args=None)
        self._node = Node('dex_arm', automatically_declare_parameters_from_overrides=True)
        self._node.get_logger().info('dex_arm (ROS 2 Jazzy) init start')

        # ── Parameters (align with MoveIt Servo conventions) ───────────────────
        def param(name, default):
            if not self._node.has_parameter(name):
                self._node.declare_parameter(name, default)
                return default
            return self._node.get_parameter(name).value

        ros_queue_size_ = int(param('ros_queue_size', 10))
        self._joint_states_topic = str(param('joint_states_topic', '/joint_states'))
        cartesian_command_in_topic_ = str(param('moveit_servo.cartesian_command_in_topic', '/servo_server/delta_twist_cmds'))
        joint_command_in_topic_ = str(param('moveit_servo.joint_command_in_topic', '/servo_server/delta_joint_cmds'))
        pose_command_in_topic_ = str(param('moveit_servo.pose_command_in_topic',
                                   '/servo_server/pose_command'))
        self._command_frame = str(param('moveit_servo.planning_frame', 'link_base'))

        # QoS compatible with MoveIt Servo
        qos = QoSProfile(depth=ros_queue_size_)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.history = HistoryPolicy.KEEP_LAST

        # ── Publishers ────────────────────────────────────────────────────────
        self._twist_pub = self._node.create_publisher(TwistStamped, cartesian_command_in_topic_, qos)
        self._joint_pub = self._node.create_publisher(JointJog, joint_command_in_topic_, qos)
        self._pose_pub = self._node.create_publisher(PoseStamped, pose_command_in_topic_, qos)

        # ── Services ─────────────────────────────────────────────────────────
        self._servo_start_cli = self._node.create_client(Trigger, '/servo_server/start_servo')
        self._servo_stop_cli = self._node.create_client(Trigger, '/servo_server/stop_servo')
        self._switch_cmd_cli = self._node.create_client(ServoCommandType, '/servo_server/switch_command_type')


        # ── Subscriptions ─────────────────────────────────────────────────────
        self._robot_joint_state_sub = self._node.create_subscription(
            JointState, self._joint_states_topic, self._callback_robot_joint_state, qos
        )
        # Subscribe to what we publish for commanded-joint echo (JointJog)
        self._robot_cmd_joint_sub = self._node.create_subscription(
            JointJog, joint_command_in_topic_, self._callback_robot_commanded_joint_state_from_jointjog, qos
        )

        # Internal state storage
        self.robot_joint_state = None  # latest sensor_msgs/JointState
        self.robot_commanded_joint_state = None  # synthesized JointState-like from JointJog
        self._current_mode = None

        # Start Servo server if available (non-blocking)
        if self._servo_start_cli.wait_for_service(timeout_sec=0.5):
            self._servo_start_cli.call_async(Trigger.Request())
        # Set Servo Command Type
        self._ensure_mode(ServoCommandType.Request.JOINT_JOG)

    def _ensure_mode(self, mode_const):
        if self._current_mode == mode_const:
            return
        if self._switch_cmd_cli.wait_for_service(timeout_sec=0.2):
            req = ServoCommandType.Request()
            req.command_type = mode_const
            self._switch_cmd_cli.call_async(req)
            self._current_mode = mode_const

    # Controller initializers
    def _init_robot_control(self):
        # Already initialized in __init__; present to satisfy the template.
        pass

    # Rostopic callback functions
    def _callback_robot_joint_state(self, joint_state: JointState):
        self.robot_joint_state = joint_state

    # Commanded joint state is basically the joint state being sent as an input to the controller
    def _callback_robot_commanded_joint_state(self, joint_state):
        # Template method kept for compatibility; we set from JointJog callback below.
        self.robot_commanded_joint_state = joint_state

    # Convert JointJog messages into a JointState-like object for get_commanded_robot_state()
    def _callback_robot_commanded_joint_state_from_jointjog(self, msg: JointJog):
        now = self._node.get_clock().now().to_msg()
        # Treat displacements as the primary field for incremental jog; velocities if provided
        position = list(msg.displacements) if msg.displacements else []
        velocity = list(msg.velocities) if msg.velocities else []
        # Build a minimal JointState-like object using SimpleNamespace
        js_like = SimpleNamespace(
            name=list(msg.joint_names),
            position=position,
            velocity=velocity,
            effort=[],
            header=SimpleNamespace(stamp=SimpleNamespace(sec=now.sec, nanosec=now.nanosec))
        )
        self._callback_robot_commanded_joint_state(js_like)

    # State information function
    def get_robot_state(self):
        # Get the robot joint state
        raw_joint_state = self.robot_joint_state
        if raw_joint_state is None:
            return None
        # Handle possible empty fields gracefully
        pos = np.array(raw_joint_state.position, dtype=np.float32) if raw_joint_state.position else np.array([], dtype=np.float32)
        vel = np.array(raw_joint_state.velocity, dtype=np.float32) if raw_joint_state.velocity else np.array([], dtype=np.float32)
        eff = np.array(raw_joint_state.effort, dtype=np.float32) if raw_joint_state.effort else np.array([], dtype=np.float32)
        # ROS 2 builtin_interfaces/Time fields are sec/nanosec
        stamp = getattr(raw_joint_state, 'header', None)
        if stamp and hasattr(raw_joint_state.header, 'stamp'):
            ts = raw_joint_state.header.stamp.sec + raw_joint_state.header.stamp.nanosec * 1e-9
        else:
            ts = time.time()
        joint_state = dict(
            position=pos,
            velocity=vel,
            effort=eff,
            timestamp=ts,
        )
        return joint_state

    # Commanded joint state is the joint state being sent as an input to the controller
    def get_commanded_robot_state(self):
        raw_joint_state = copy(self.robot_commanded_joint_state)
        if raw_joint_state is None:
            return None
        pos = np.array(getattr(raw_joint_state, 'position', []), dtype=np.float32)
        vel = np.array(getattr(raw_joint_state, 'velocity', []), dtype=np.float32)
        eff = np.array(getattr(raw_joint_state, 'effort', []), dtype=np.float32)
        # synthesized timestamp
        if hasattr(raw_joint_state, 'header') and hasattr(raw_joint_state.header, 'stamp'):
            ts = raw_joint_state.header.stamp.sec + raw_joint_state.header.stamp.nanosec * 1e-9
        else:
            ts = time.time()
        joint_state = dict(
            position=pos,
            velocity=vel,
            effort=eff,
            timestamp=ts,
        )
        return joint_state

    # Get the robot joint/cartesian position
    def get_robot_position(self):
        # Return latest joint positions (radians) from /joint_states
        state = self.get_robot_state()
        if state is None:
            return None
        return state['position']

    # Get the robot joint velocity
    def get_robot_velocity(self):
        state = self.get_robot_state()
        if state is None:
            return None
        return state['velocity']

    # Get the robot joint torque
    def get_robot_torque(self):
        state = self.get_robot_state()
        if state is None:
            return None
        return state['effort']

    # Get the commanded robot joint position
    def get_commanded_robot_joint_position(self):
        cmd = self.get_commanded_robot_state()
        if cmd is None:
            return None
        return cmd['position']

    # Movement functions
    def move_robot(self, joint_angles):
        """
        Publish incremental joint jog using JointJog.displacements.
        `joint_angles` is interpreted as Δθ per message (radians).
        """
        self._ensure_mode(ServoCommandType.Request.JOINT_JOG)
        msg = JointJog()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = self._command_frame
        # If we have a live joint state, use its joint order for names; else leave empty
        if self.robot_joint_state and getattr(self.robot_joint_state, 'name', None):
            msg.joint_names = list(self.robot_joint_state.name)
        # Displacements mode (incremental)
        msg.displacements = [float(x) for x in np.ravel(joint_angles).tolist()]
        self._joint_pub.publish(msg)

    # Home Robot
    def home_robot(self):
        # No absolute homing via Servo; this is a placeholder.
        self._node.get_logger().warn('home_robot(): not implemented for Servo-only control')

    # Reset the Robot
    def reset_robot(self):
        # Try stopping and starting Servo to clear state.
        if self._servo_stop_cli.wait_for_service(timeout_sec=0.5):
            self._servo_stop_cli.call_async(Trigger.Request())
        if self._servo_start_cli.wait_for_service(timeout_sec=0.5):
            self._servo_start_cli.call_async(Trigger.Request())

    # Full robot commands
    def move_robot(self, joint_angles, arm_angles):
        # Keep signature for compatibility; use joint jog only.
        self.move_robot(joint_angles)

    def arm_control(self, arm_pose):
        """
        Publish a geometry_msgs/PoseStamped to Servo's pose_command_in_topic.
        Accepts either:
        - dict/object with x,y,z and roll,pitch,yaw (radians), or
        - dict/object with x,y,z and qx,qy,qz,qw (unit quaternion).
        """
        self._ensure_mode(ServoCommandType.Request.POSE)
        def get(src, name, default=0.0):
            if isinstance(src, dict):
                return src.get(name, default)
            return getattr(src, name, default)

        # Position
        x = float(get(arm_pose, "x", 0.0))
        y = float(get(arm_pose, "y", 0.0))
        z = float(get(arm_pose, "z", 0.0))

        # Orientation: prefer quaternion if present, else RPY→quat
        has_quat = all(k in (arm_pose.keys() if isinstance(arm_pose, dict) else arm_pose.__dict__)
                    for k in ("qx", "qy", "qz", "qw"))
        if has_quat:
            qx = float(get(arm_pose, "qx"))
            qy = float(get(arm_pose, "qy"))
            qz = float(get(arm_pose, "qz"))
            qw = float(get(arm_pose, "qw"))
        else:
            import math
            roll  = float(get(arm_pose, "roll",  get(arm_pose, "rx", 0.0)))
            pitch = float(get(arm_pose, "pitch", get(arm_pose, "ry", 0.0)))
            yaw   = float(get(arm_pose, "yaw",   get(arm_pose, "rz", 0.0)))
            # RPY -> quaternion
            cy, sy = math.cos(yaw * 0.5),   math.sin(yaw * 0.5)
            cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
            cr, sr = math.cos(roll * 0.5),  math.sin(roll * 0.5)
            qw = cr*cp*cy + sr*sp*sy
            qx = sr*cp*cy - cr*sp*sy
            qy = cr*sp*cy + sr*cp*sy
            qz = cr*cp*sy - sr*sp*cy

        msg = PoseStamped()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        # Must be set: Servo expects header.frame_id for Twist/Pose commands.
        # Use your planning/command frame (often "link_base" or whatever you set via params).
        msg.header.frame_id = self._command_frame
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        self._pose_pub.publish(msg)

    #Home the Robot
    def home_robot(self):
        # Duplicate in template; keep and warn.
        self._node.get_logger().warn('home_robot(): duplicate template method; not implemented')
        # For now we're using cartesian values
