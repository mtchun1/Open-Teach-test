import threading
import time
from contextlib import suppress
from typing import Optional

import numpy as np
from rclpy.executors import SingleThreadedExecutor

from openteach.ros_links.uf850_control import DexArmControl
from .robot import RobotWrapper

class UF850(RobotWrapper):
    def __init__(self):
        self._controller = DexArmControl(robot_type="both")
        self._data_frequency = 100

        # Spin the ROS node in the background so subscribers/services remain responsive
        self._executor: Optional[SingleThreadedExecutor] = None
        self._spin_thread: Optional[threading.Thread] = None
        self._start_background_executor()

    @property
    def recorder_functions(self):
        return {
            "joint_states": self.get_joint_state,
            "cartesian_states": self.get_cartesian_state,
        }
    
    @property
    def name(self):
        return "uf850"

    @property
    def data_frequency(self):
        return self._data_frequency

    def get_joint_state(self):
        return self._controller.get_arm_joint_state()

    def get_joint_position(self):
        joint_state = self._controller.get_robot_state()
        if joint_state is None or joint_state["position"].size == 0:
            return None
        return dict(
            position=np.array(joint_state["position"], dtype=np.float32),
            timestamp=joint_state["timestamp"],
        )

    def get_cartesian_state(self):
        cartesian_state = self.get_cartesian_position()
        if cartesian_state is None:
            return None
        return dict(
            position=np.array(cartesian_state[0:3], dtype=np.float32).flatten(),
            orientation=np.array(cartesian_state[3:], dtype=np.float32).flatten(),
            timestamp=time.time(),
        )

    def get_cartesian_position(self):
        return self._controller.get_arm_cartesian_coords()

    def get_joint_velocity(self):
        joint_state = self._controller.get_robot_state()
        if joint_state is None or joint_state["velocity"].size == 0:
            return None
        return dict(
            velocity=np.array(joint_state["velocity"], dtype=np.float32),
            timestamp=joint_state["timestamp"],
        )

    # @abstractmethod
    def get_joint_torque(self):
        joint_state = self._controller.get_robot_state()
        if joint_state is None or joint_state["effort"].size == 0:
            return None
        return dict(
            torque=np.array(joint_state["effort"], dtype=np.float32),
            timestamp=joint_state["timestamp"],
        )

    def home(self):
        self._controller.home_robot()

    def move(self, input_angles):
        self._controller.move_robot(input_angles)

    def move_coords(self, input_coords):
        self._controller.arm_control(input_coords)

    # @abstractmethod
    # def reset(self):
    #     pass

    def reset(self):
        self._controller.reset_robot()
        # Refresh cached pose so downstream components have immediate feedback
        self._controller.get_robot_position_aa()

    def shutdown(self):
        if self._executor is not None:
            with suppress(Exception):
                self._executor.remove_node(self._controller._node)
            with suppress(Exception):
                self._executor.shutdown()
            self._executor = None
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=0.1)
            self._spin_thread = None
        with suppress(Exception):
            self._controller._node.destroy_node()

    def _start_background_executor(self):
        if self._executor is not None:
            return
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._controller._node)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

    # @abstractmethod
    # def arm_control(self):
    #     pass

    # @abstractmethod
    # def set_gripper_state(self):
    #     pass
