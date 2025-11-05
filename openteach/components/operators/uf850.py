import numpy as np
import matplotlib.pyplot as plt
import zmq

from mpl_toolkits.mplot3d import Axes3D
from tqdm import tqdm

from copy import deepcopy as copy
from asyncio import threads
from openteach.constants import *
from openteach.utils.timer import FrequencyTimer
from openteach.utils.network import ZMQKeypointSubscriber, ZMQKeypointPublisher
from openteach.utils.vectorops import *
from openteach.utils.files import *

from openteach.robot.uf850 import UF850
from scipy.spatial.transform import Rotation, Slerp
from .operator import Operator
from scipy.spatial.transform import Rotation as R
from numpy.linalg import pinv



np.set_printoptions(precision=3, suppress=True)

# Filter for removing noise in the teleoperation
class Filter:
    def __init__(self, state, comp_ratio=0.6):
        self.pos_state = state[:3]
        self.ori_state = state[3:6]
        self.comp_ratio = comp_ratio

    def __call__(self, next_state):
        self.pos_state = self.pos_state[:3] * self.comp_ratio + next_state[:3] * (1 - self.comp_ratio)
        ori_interp = Slerp([0, 1], Rotation.from_rotvec(
            np.stack([self.ori_state, next_state[3:6]], axis=0)),)
        self.ori_state = ori_interp([1 - self.comp_ratio])[0].as_rotvec()
        return np.concatenate([self.pos_state, self.ori_state])

# UF850 Arm Teleoperation
class UF850ArmOperator(Operator):
    def __init__(
        self,
        host,
        transformed_keypoints_port,
        moving_average_limit,
        use_filter=True,
        arm_resolution_port = None,
        teleoperation_reset_port = None,):
        # cartesian_publisher_port = None,
        #joint_publisher_port = None):

        self.notify_component_start('UF850 arm operator')
        # Transformed Arm Keypoint Subscriber
        self._transformed_arm_keypoint_subscriber = ZMQKeypointSubscriber(
            host=host,
            port=transformed_keypoints_port,
            topic='transformed_hand_frame'
        )
        # Transformed Hand Keypoint Subscriber
        self._transformed_hand_keypoint_subscriber = ZMQKeypointSubscriber(
            host=host,
            port=transformed_keypoints_port,
            topic='transformed_hand_coords'
        )
        # # Cartesian Publisher
        # self.cartesian_publisher = ZMQKeypointPublisher(
        #     host=host,
        #     port=cartesian_publisher_port
        # )
        # # Joint Publisher
        # self.joint_publisher = ZMQKeypointPublisher(
        #     host=host,
        #     port=joint_publisher_port
        # )
        # Arm Resolution Subscriber
        self._arm_resolution_subscriber = ZMQKeypointSubscriber(
            host= host,
            port= arm_resolution_port,
            topic = 'button'
        )

        self._arm_teleop_state_subscriber = ZMQKeypointSubscriber(
            host = host,
            port = teleoperation_reset_port,
            topic = 'pause'
        )

        # Define Robot object
        self._robot = UF850()
        self.robot.reset()

        # Get the initial pose of the robot
        robot_coords = self.robot.get_cartesian_position_aa()
        print("HOME POSE", robot_coords)
        self.robot_init_H = self.robot_pose_aa_to_affine(robot_coords)
        self.is_first_frame = True

        # Frequency timer
        self._timer = FrequencyTimer(VR_FREQ)

        # Use the filter
        self.use_filter = use_filter
        if use_filter:
            robot_init_cart = self._homo2cart(self.robot_init_H)
            self.comp_filter = Filter(robot_init_cart, comp_ratio=0.8)

        # Class variables
        self.pause_flag=1
        self.prev_pause_flag=0
        self.is_first_frame= True
        self.pause_cnt=0
        self.resolution_scale =1
        self.arm_teleop_state = ARM_TELEOP_STOP

        # Getting the bounds to perform linear transformation
        bounds_file = get_path_in_package(
            'components/operators/configs/uf850.yaml')
        bounds_data = get_yaml_data(bounds_file)
        self.velocity_threshold = bounds_data['velocity_threshold']
        self.joint_velocity_threshold = bounds_data['joint_velocity_threshold']

        # Moving average queues
        self.moving_Average_queue = []
        self.moving_average_limit = moving_average_limit


    @property
    def timer(self):
        return self._timer

    @property
    def robot(self):
        return self._robot

    @property
    def transformed_hand_keypoint_subscriber(self):
        return self._transformed_hand_keypoint_subscriber

    @property
    def transformed_arm_keypoint_subscriber(self):
        return self._transformed_arm_keypoint_subscriber


    def robot_pose_rpy_to_affine(self, pose_rpy: np.ndarray) -> np.ndarray:
        """
        Convert a robot pose [x_mm, y_mm, z_mm, roll_rad, pitch_rad, yaw_rad]
        into a 4x4 homogeneous transform in meters.
        """
        rotation = R.from_euler('XYZ', pose_rpy[3:], degrees=False).as_matrix()
        translation = np.array(pose_rpy[:3]) / SCALE_FACTOR  # mm → m
        return np.block([[rotation, translation[:, np.newaxis]], [0, 0, 0, 1]])

    def robot_pose_aa_to_affine(self,pose_aa: np.ndarray) -> np.ndarray:
        """
        Converts a robot pose in axis-angle format to an affine matrix.
        Args:
            pose_aa (list): [x, y, z, ax, ay, az] where (x, y, z) is the position and (ax, ay, az) is the axis-angle rotation.
            x, y, z are in mm and ax, ay, az are in radians.
        Returns:
            np.ndarray: 4x4 affine matrix [[R, t],[0, 1]]
        """

        rotation = R.from_rotvec(pose_aa[3:]).as_matrix()
        translation = np.array(pose_aa[:3]) / SCALE_FACTOR

        return np.block([[rotation, translation[:, np.newaxis]],
                 [np.array([0, 0, 0, 1])]])


    #Function to differentiate between real and simulated robot
    def return_real(self):
        return True

    # Function Gets the transformed hand frame
    def _get_hand_frame(self):
        data = None  # Initialize with a default value
        for i in range(10):
            data = self.transformed_arm_keypoint_subscriber.recv_keypoints(flags=zmq.NOBLOCK)
            if data is not None:
                break
        if data is None:
            return None
        return np.asanyarray(data).reshape(4, 3)

    # Function to get the resolution scale mode
    def _get_resolution_scale_mode(self):
        data = self._arm_resolution_subscriber.recv_keypoints()
        res_scale = np.asanyarray(data).reshape(1)[0] # Make sure this data is one dimensional
        return res_scale

    # Function to get the arm teleop state from the hand keypoints
    # def _get_arm_teleop_state_from_hand_keypoints(self):
    #     pause_state ,pause_status,pause_right =self.get_pause_state_from_hand_keypoints()
    #     pause_status =np.asanyarray(pause_status).reshape(1)[0]

    #     return pause_state,pause_status,pause_right

    def _get_arm_teleop_state(self):
        reset_stat = self._arm_teleop_state_subscriber.recv_keypoints()
        reset_stat = np.asanyarray(reset_stat).reshape(1)[0] # Make sure this data is one dimensional
        return reset_stat

    # get the translation vector
    def _get_translation_vector(self, commanded_robot_position, current_robot_position):
        return commanded_robot_position - current_robot_position

    # Get the rotation angular displacement
    def _get_rotation_angles(self, robot_target_orientation, current_robot_rotation_values):
        # Calculating the angular displacement between the target hand frame and the current robot frame#
        target_rotation_state = Rotation.from_rotvec(robot_target_orientation)
        robot_rotation_state = Rotation.from_rotvec(current_robot_rotation_values)

        # Calculating the angular displacement between the target hand frame and the current robot frame
        angular_displacement = Rotation.from_matrix(
            np.matmul(robot_rotation_state.inv().as_matrix(),target_rotation_state.as_matrix())
        ).as_rotvec()

        return angular_displacement

    # Get the displacement vector
    def _get_displacement_vector(self, commanded_robot_position, current_robot_position):
        commanded_robot_pose = np.zeros(6)
        # Transformation from translation
        commanded_robot_pose[:3] = self._get_translation_vector(
            commanded_robot_position = commanded_robot_position[:3],
            current_robot_position = current_robot_position[:3]
        ) #* UF850_VELOCITY_SCALING_FACTOR

        # Transformation from rotation
        commanded_robot_pose[3:] = self._get_rotation_angles(
            robot_target_orientation=commanded_robot_position[3:],
            current_robot_rotation_values = current_robot_position[3:]
        )
        return commanded_robot_pose

    # Function to turn a frame to a homogeneous matrix
    def _turn_frame_to_homo_mat(self, frame):
        t = frame[0]
        R = frame[1:]

        homo_mat = np.zeros((4, 4))
        homo_mat[:3, :3] = np.transpose(R)
        homo_mat[:3, 3] = t
        homo_mat[3, 3] = 1

        return homo_mat

    # Function to turn homogenous matrix to cartesian vector
    def _homo2cart(self, homo_mat):
        # Here we will use the resolution scale to set the translation resolution
        t = homo_mat[:3, 3]
        rotvec = Rotation.from_matrix(homo_mat[:3, :3]).as_rotvec(degrees=False)

        cart = np.concatenate(
            [t, rotvec], axis=0
        )
        return cart


    # Get the scaled cartesian pose
    def _get_scaled_cart_pose(self, moving_robot_homo_mat):
        # Get the cart pose without the scaling
        unscaled_cart_pose = self._homo2cart(moving_robot_homo_mat) # m + rotvec

        # Get the current cart pose
        robot_coords = self.robot.get_cartesian_position_aa()
        current_homo_mat =  copy(self.robot_pose_aa_to_affine(robot_coords))
        current_cart_pose = self._homo2cart(current_homo_mat)#home_pose_array # m + rotvec

        # Get the difference in translation between these two cart poses
        diff_in_translation = unscaled_cart_pose[:3] - current_cart_pose[:3]
        scaled_diff_in_translation = diff_in_translation * self.resolution_scale

        scaled_cart_pose = np.zeros(6)
        scaled_cart_pose[3:] = unscaled_cart_pose[3:] # rotvec (rad)
        scaled_cart_pose[:3] = current_cart_pose[:3] + scaled_diff_in_translation # meters

        return scaled_cart_pose

    # Reset Teleoperation and make the current frame as initial frame
    def _reset_teleop(self):
        print('****** RESETTING TELEOP ****** ')
        robot_coords = self.robot.get_cartesian_position_aa()
        self.robot_init_H =  self.robot_pose_aa_to_affine(robot_coords)

        first_hand_frame = self._get_hand_frame()
        while first_hand_frame is None:
            first_hand_frame = self._get_hand_frame()
        self.hand_init_H = self._turn_frame_to_homo_mat(first_hand_frame)
        self.hand_init_t = copy(self.hand_init_H[:3, 3])
        self.is_first_frame = False
        print("Resetting complete")
        return first_hand_frame

    # Function to get gripper state from hand keypoints
    # def get_gripper_state_from_hand_keypoints(self):
    #     transformed_hand_coords= self._transformed_hand_keypoint_subscriber.recv_keypoints()
    #     distance = np.linalg.norm(transformed_hand_coords[OCULUS_JOINTS['pinky'][-1]]- transformed_hand_coords[OCULUS_JOINTS['thumb'][-1]])
    #     thresh = 0.03
    #     gripper_fl =False
    #     if distance < thresh:
    #         self.gripper_cnt+=1
    #         if self.gripper_cnt==1:
    #             self.prev_gripper_flag = self.gripper_flag
    #             self.gripper_flag = not self.gripper_flag
    #             gripper_fl=True
    #     else:
    #         self.gripper_cnt=0
    #     gripper_state = np.asanyarray(self.gripper_flag).reshape(1)[0]
    #     status= False
    #     if gripper_state!= self.prev_gripper_flag:
    #         status= True
    #     return gripper_state , status , gripper_fl

    # Toggle the robot to pause/resume using ring/middle finger pinch, both finger modes are supported to avoid any hand pose noise issue
    # def get_pause_state_from_hand_keypoints(self):
    #     transformed_hand_coords= self._transformed_hand_keypoint_subscriber.recv_keypoints()
    #     ring_distance = np.linalg.norm(transformed_hand_coords[OCULUS_JOINTS['ring'][-1]]- transformed_hand_coords[OCULUS_JOINTS['thumb'][-1]])
    #     middle_distance = np.linalg.norm(transformed_hand_coords[OCULUS_JOINTS['middle'][-1]]- transformed_hand_coords[OCULUS_JOINTS['thumb'][-1]])
    #     thresh = 0.03
    #     pause_right= True
    #     if ring_distance < thresh  or middle_distance < thresh:
    #         self.pause_cnt+=1
    #         if self.pause_cnt==1:
    #             self.prev_pause_flag=self.pause_flag
    #             self.pause_flag = not self.pause_flag
    #     else:
    #         self.pause_cnt=0
    #     pause_state = np.asanyarray(self.pause_flag).reshape(1)[0]
    #     pause_status= False
    #     if pause_state!= self.prev_pause_flag:
    #         pause_status= True
    #     return pause_state , pause_status , pause_right

    # Function to apply retargeted angles
    def _apply_retargeted_angles(self, log=False):

        # See if there is a reset in the teleop
        new_arm_teleop_state = self._get_arm_teleop_state()
        if self.is_first_frame or (self.arm_teleop_state == ARM_TELEOP_STOP and new_arm_teleop_state == ARM_TELEOP_CONT):
            moving_hand_frame = self._reset_teleop() # Should get the moving hand frame only once
        else:
            moving_hand_frame = self._get_hand_frame()
        if moving_hand_frame is None:
            if new_arm_teleop_state == ARM_TELEOP_STOP:
                self.arm_teleop_state = new_arm_teleop_state
            return # It means we are not on the arm mode yet instead of blocking it is directly returning
        raw_current_robot_position = self.robot.get_cartesian_position_aa()
        current_robot_position = np.concatenate([
            np.array(raw_current_robot_position[:3]) / SCALE_FACTOR,  # scale xyz
            np.array(raw_current_robot_position[3:])                  # keep rotvec as is
        ])

        if current_robot_position is None:
            if new_arm_teleop_state == ARM_TELEOP_STOP:
                self.arm_teleop_state = new_arm_teleop_state
            return

        self.arm_teleop_state = new_arm_teleop_state

        arm_teleoperation_scale_mode = self._get_resolution_scale_mode()

        if arm_teleoperation_scale_mode == ARM_HIGH_RESOLUTION:
            self.resolution_scale = 1
        elif arm_teleoperation_scale_mode == ARM_LOW_RESOLUTION:
            self.resolution_scale = 0.6

        # Find the moving hand frame
        self.hand_moving_H = self._turn_frame_to_homo_mat(moving_hand_frame)

        # (x is right, y is up, z is forward)
        # Transformation code
        H_HI_HH = copy(self.hand_init_H) # Homo matrix that takes P_HI to P_HH - Point in Inital Hand Frame to Point in Home Hand Frame
        H_HT_HH = copy(self.hand_moving_H) # Homo matrix that takes P_HT to P_HH
        H_RI_RH = copy(self.robot_init_H) # Homo matrix that takes P_RI to P_RH

        # ___ OLD TRANSFORMATION ___

        # Find the relative transformation in human hand space.
        H_HT_HI = np.linalg.pinv(H_HI_HH) @ H_HT_HH # Homo matrix that takes P_HT to P_HI

        # Transformation matrix
        # H_R_V=  [[0, -1, 0, 0],
        #         [ 1, 0, 0, 0],
        #         [ 0, 0, 1, 0],
        #         [ 0, 0, 0, 1]]

        # (+y, -z .-x)
        # H_R_V=  [[0, -1, 0, 0],
        #         [ 0, 0, -1, 0],
        #         [ 1, 0, 0, 0],
        #         [ 0, 0, 0, 1]]

        # Here there are two matrices because the rotation is asymmetric and we imagine we are holding the endeffector and moving the robot.
        # H_R_V= np.array([[0 , 1,  0, 0],
        #                 [ 0 , 0,  -1, 0],
        #                 [ -1,  0,  0, 0],
        #                 [ 0,  0,  0 , 1]])
        H_R_V =  np.array([[0, -1, 0, 0],
                           [ 1, 0, 0, 0],
                           [ 0, 0, 1, 0],
                           [ 0, 0, 0, 1]])
        # The translation is completely symmetric and mimics your hand movement and we imagine we are holding the endeffector and moving the robot.
        H_T_V = np.array([[0, -1 , 0, 0],
                         [1 , 0,  0, 0],
                         [0,  0,  1, 0],
                         [0,  0,  0, 1]])

        # Find the relative transform and apply it to robot initial position
        H_R_R= (np.linalg.pinv(H_R_V)@H_HT_HI@H_R_V)[:3,:3]
        H_R_T= (np.linalg.pinv(H_T_V)@H_HT_HI@H_R_V)[:3,3]
        H_F_H=np.block([[H_R_R,H_R_T.reshape(3,1)],[np.array([0,0,0]),1]])
        H_RT_RH = H_RI_RH  @ H_F_H # Homo matrix that takes P_RT to P_RH

        # ___ NEW TRANSFORMATION ___
        # Find the relative transformation in human hand space.
        # H_HT_HI = np.linalg.inv(H_HI_HH) @ H_HT_HH  # rigid inverse

        # H_R_V= np.array([[0 , -1,   0,  0],
        #                 [ 0 ,  0,   1,  0],
        #                 [ 1,   0,   0,  0],
        #                 [ 0,   0,   0 , 1]], dtype=float)

        # H_R_V_inv = np.linalg.inv(H_R_V)

        # # Express the hand relative motion in robot base coordinates
        # H_rel_robot = H_R_V @ H_HT_HI @ H_R_V_inv

        # # Apply relative motion to robot's initial pose
        # H_RT_RH = H_RI_RH @ H_rel_robot

        self.robot_moving_H = copy(H_RT_RH)

        final_pose = self._get_scaled_cart_pose(self.robot_moving_H)
        # filter the pose in case needed
        if self.use_filter:
            final_pose = self.comp_filter(final_pose)

        # print(final_pose, current_robot_position)
        # Calculated velocity
        calculated_velocity = self._get_displacement_vector(final_pose, current_robot_position)
        averaged_velocity = moving_average(
            calculated_velocity,
            self.moving_Average_queue,
            self.moving_average_limit
        )
        # print(final_pose[3:], current_robot_position[3:], calculated_velocity[3:])
        # print(calculated_velocity)

        vel_cmd = np.empty(6, dtype=float)
        vel_cmd[:3] = (UF850_LINEAR_VELOCITY_SCALING_FACTOR * averaged_velocity[:3]) / VR_FREQ
        vel_cmd[3:] = (UF850_ANGULAR_VELOCITY_SCALING_FACTOR * averaged_velocity[3:]) / VR_FREQ
        # print(vel_cmd)
        # Deadbands on the commanded RATES (optional)
        for i in range(3):
            if abs(vel_cmd[i]) < self.velocity_threshold:
                vel_cmd[i] = 0.0
        for i in range(3, 6):
            if abs(vel_cmd[i]) < self.joint_velocity_threshold:
                vel_cmd[i] = 0.0

        # Safety clamps (optional but smart)
        # vel_cmd[:3] = np.clip(vel_cmd[:3], -self.max_lin, self.max_lin)
        # vel_cmd[3:] = np.clip(vel_cmd[3:], -self.max_ang, self.max_ang)

        # print(averaged_velocity)
        # R_rel = R.from_rotvec(averaged_velocity[3:])
        # print(averaged_velocity[:3], R_rel.as_euler('xyz', degrees=True))
        # print(vel_cmd)
        self.robot.move_velocity(vel_cmd, 1 / VR_FREQ)


    # NOTE: This is for debugging should remove this when needed
    def stream(self):
        self.notify_component_start('{} control'.format(self.robot.name))
        print("Start controlling the robot hand using the Oculus Headset.\n")

        # Assume that the initial position is considered initial after 3 seconds of the start
        while True:
            try:
                if self.robot.get_joint_position() is not None:
                    self.timer.start_loop()

                    # Retargeting function
                    self._apply_retargeted_angles(log=False)

                    self.timer.end_loop()
            except KeyboardInterrupt:
                break

        self.transformed_arm_keypoint_subscriber.stop()
        print('Stopping the teleoperator!')





