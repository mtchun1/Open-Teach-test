from openteach.ros_links.uf850_control import DexArmControl
from .robot import RobotWrapper

class UF850(RobotWrapper):
    def __init__(self,record_type=None):
        self._controller = DexArmControl(record_type=record_type, robot_type='uf850')
        self._data_frequency = 150

    @property
    def recorder_functions(self):
        return {
            'joint_states': self.get_joint_state,
            'cartesian_states': self.get_cartesian_state
        }

    @property
    def name(self):
        return 'uf850'

    @property
    def data_frequency(self):
        return self._data_frequency

    # State information functions
    def get_joint_state(self):
        return self._controller.get_arm_joint_state()

    def get_cartesian_state(self):
        return self._controller.get_arm_cartesian_state()
    
    def get_cartesian_state_aa(self):
        return self._controller.get_arm_cartesian_state_aa()

    def get_joint_position(self):
        return self._controller.get_arm_position()

    def get_joint_velocity(self):
        return self._controller.get_arm_velocity()

    def get_joint_torque(self):
        return self._controller.get_arm_torque()
    
    def get_cartesian_position(self):
        return self._controller.get_arm_cartesian_coords()
    
    def get_cartesian_position_aa(self):
        return self._controller.get_arm_cartesian_coords_aa()

    # Movement functions
    def home(self):
        return self._controller.home_arm()

    def move(self, input_angles):
        # self._controller.move_arm(input_angles)
        pass

    def move_coords(self, cartesian_coords):
        # self._controller.move_arm_cartesian(cartesian_coords)
        pass

    def reset(self):
        pass

    def move_velocity(self, input_velocity_values, duration):
        self._controller.move_arm_cartesian_velocity(input_velocity_values, duration) 
