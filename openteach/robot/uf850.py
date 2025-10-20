from openteach.ros_links.uf850_control import DexArmControl 
from .robot import RobotWrapper
import numpy as np
import time

class UF850(RobotWrapper):
    def __init__(self):
        self._controller = DexArmControl(robot_type='both')
        self._data_frequency = 100

    @property
    def recorder_functions(self):
        return{
            'joint_states': self.get_joint_state,
            'cartesian_states': self.get_cartesian_state
        }
    
    @property
    def name(self):
        return 'uf850'

    @property
    def data_frequency(self):
        return self._data_frequency

    def get_joint_state(self):
        return self._controller.get_arm_joint_state()

    def get_joint_position(self):
        pass

    def get_cartesian_state(self):
        cartesian_state=self.get_cartesian_position()
        cartesian_dict = dict(
            position = np.array(cartesian_state[0:3], dtype=np.float32).flatten(),
            orientation = np.array(cartesian_state[3:], dtype=np.float32).flatten(),
            timestamp = time.time()
        )
        return cartesian_dict

    def get_cartesian_position(self):
        return self._controller.get_arm_cartesian_coords()

    # @abstractmethod
    def get_joint_velocity(self):
        pass

    # @abstractmethod
    def get_joint_torque(self):
        pass

    def home(self):
        pass

    def move(self, input_angles):
        self._controller.move_robot(input_angles)

    def move_coords(self, input_coords):
        self._controller.arm_control(input_coords)

    # @abstractmethod
    # def reset(self):
    #     pass

    # @abstractmethod
    # def arm_control(self):
    #     pass

    # @abstractmethod
    # def set_gripper_state(self):
    #     pass