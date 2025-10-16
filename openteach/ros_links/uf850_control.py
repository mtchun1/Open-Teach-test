import numpy as np
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from copy import deepcopy as copy
from sensor_msgs.msg import JointState

from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from moveit_msgs.msg import PlanningScene
from std_srvs.srv import Trigger
from builtin_interfaces.msg import Time

# Topics
UF850_JOINT_STATE_TOPIC = '/joint_states'
UF850__COMMANDED_JOINT_STATE_TOPIC = '/servo_server/delta_twist_cmds'


class DexArmControl():
    def __init__(self, record_type=None, robot_type='both'):
    # Initialize Controller Specific Information
        if not rclpy.ok():
            rclpy.init(args=None)
        self._node = Node('dex_arm',
                          automatically_declare_parameters_from_overrides=True)
        self._node.get_logger().info("dex_arm init start")

        def param(name, default):
            if not self._node.has_parameter(name):
                self._node.declare_parameter(name, default)
                return default
            return self._node.get_parameter(name).value


        dof_ = int(param('dof', 7))
        ros_queue_size_ = int(param('ros_queue_size', 10))

        cartesian_command_in_topic_ = str(param('moveit_servo.cartesian_command_in_topic',
                                                '/servo_server/delta_twist_cmds'))
        joint_command_in_topic_ = str(param('moveit_servo.joint_command_in_topic',
                                            '/servo_server/delta_joint_cmds'))
        robot_link_command_frame_ = str(param('moveit_servo.robot_link_command_frame',
                                              'base_link'))
        ee_frame_name_ = str(param('moveit_servo.ee_frame_name', 'link_eef'))
        planning_frame_ = str(param('moveit_servo.planning_frame', 'link_base'))

        if cartesian_command_in_topic_.startswith("~/"):
            cartesian_command_in_topic_ = "/servo_server/" + cartesian_command_in_topic_[2:]

        if joint_command_in_topic_.startswith("~/"):
            joint_command_in_topic_ = "/servo_server/" + joint_command_in_topic_[2:]

        # QoS compatible with MoveIt Servo
        qos = QoSProfile(depth=ros_queue_size_)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.history = HistoryPolicy.KEEP_LAST

        # Publishers (Python equivalents of your C++ shared_ptrs)
        self.twist_pub_ = self._node.create_publisher(
            TwistStamped, cartesian_command_in_topic_, qos
        )
        self.joint_pub_ = self._node.create_publisher(
            JointJog, joint_command_in_topic_, qos
        )
        self.collision_pub_ = self._node.create_publisher(
            PlanningScene, '/planning_scene', qos
        )
        self.servo_start_client_ = self._node.create_client(
            Trigger, '/servo_server/start_servo'
        )
        self.servo_start_client_.wait_for_service(timeout_sec=1.0)

        req = Trigger.Request()
        future = self.servo_start_client_.call_async(req)
        
        self.uf850_joint_state = self._node.create_subscription(
            JointState,
            UF850_JOINT_STATE_TOPIC,
            self._callback_uf850_joint_state,
            qos
        )

        self.uf850_commanded_joint_state = self._node.create_subscription(
            JointState,
            UF850__COMMANDED_JOINT_STATE_TOPIC,
            self._callback_uf850_commanded_joint_state,
            qos
        )

        self.uf850_joint_state = None
        self.uf850_commanded_joint_state = None

    # Rostopic callback functions
    def _callback_uf850_joint_state(self, joint_state):
        self.uf850_joint_state = joint_state

    #Commanded joint state is basically the joint state being sent as an input to the controller
    def _callback_uf850_commanded_joint_state(self, joint_state):
        self.uf850_commanded_joint_state = joint_state

    # State information function
    def get_uf850_state(self):
        # Get the robot joint state 
        raw_joint_state =self.robot_joint_state
        joint_state = dict(
            position = np.array(raw_joint_state.position, dtype = np.float32),
            velocity = np.array(raw_joint_state.velocity, dtype = np.float32),
            effort = np.array(raw_joint_state.effort, dtype = np.float32),
            timestamp = raw_joint_state.header.stamp.secs + (raw_joint_state.header.stamp.nsecs * 1e-9)
        )
        return joint_state

    # Commanded joint state is the joint state being sent as an input to the controller
    def get_commanded_uf850_state(self):
        raw_joint_state = copy(self.uf850_commanded_joint_state)

        joint_state = dict(
            position = np.array(raw_joint_state.position, dtype = np.float32),
            velocity = np.array(raw_joint_state.velocity, dtype = np.float32),
            effort = np.array(raw_joint_state.effort, dtype = np.float32),
            timestamp = raw_joint_state.header.stamp.secs + (raw_joint_state.header.stamp.nsecs * 1e-9)
        )
        return joint_state
        
    # Get the robot joint/cartesian position
    def get_robot_position(self):
        #Get Robot Position
        joint_state =np.array(self.robot.get_servo_angle()[1]) # TODO: FIND SERVICE TO GET ROBOT JOINT POSITION 
        return joint_state

    # Get the robot joint velocity
    def get_robot_velocity(self):
        #Get Robot Velocity
        raise ValueError('get_arm_velocity() is being called - Arm Velocity cannot be collected in xArm arms, this method should not be called')

    # Get the robot joint torque
    def get_robot_torque(self):
        # Get torque applied by the robot.
        raise ValueError('get_arm_torque() is being called - Arm Torques cannot be collected in xArm arms, this method should not be called')

    # Get the commanded robot joint position
    def get_commanded_robot_joint_position(self):
        pass

    # Movement functions
    def move_robot(self, joint_angles):
        joint_msg = JointJog()

        joint_msg.header.stamp = self._node._get_clock().now().to_msg()
        joint_msg.header.frame_id = "joint"
        joint_msg.velocities = joint_angles
        self.joint_pub_.publish(joint_msg)

    # Home Robot
    def home_robot(self):
        pass

    # Reset the Robot
    def reset_robot(self):
        self.home_arm()
        pass

    # Full robot commands
    def move_robot(self, joint_angles, arm_angles):
        pass

    def arm_control(self, arm_pose):
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self._node.get_clock().now().to_msg()
        twist_msg.header.frame_id = "link_base"   # or "tcp_link", "world", etc.

        # Linear velocity (m/s)
        twist_msg.twist.linear.x = arm_pose.x
        twist_msg.twist.linear.y = arm_pose.y
        twist_msg.twist.linear.z = arm_pose.z

        # Angular velocity (rad/s)
        twist_msg.twist.angular.x = 0.0
        twist_msg.twist.angular.y = 0.0
        twist_msg.twist.angular.z = 0.2

        twist_pub_.publish(twist_msg)
        pass

    #Home the Robot
    def home_robot(self):
        self.home_arm()
        pass
        # For now we're using cartesian values
