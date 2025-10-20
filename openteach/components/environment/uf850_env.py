import threading
import time
from contextlib import suppress

import numpy as np

try:  # pragma: no cover - optional dependency for physical camera streaming
    import pyrealsense2 as rs  # type: ignore[attr-defined]
except ImportError:  # pragma: no cover - RealSense not available in all setups
    rs = None
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.time import Time

from tf2_ros import (
    Buffer,
    ConnectivityException,
    ExtrapolationException,
    LookupException,
    TransformListener,
)

from openteach.components.environment.arm_env import Arm_Env
from openteach.constants import DEPTH_PORT_OFFSET, VR_FREQ, VIZ_PORT_OFFSET
from openteach.ros_links.uf850_control import DexArmControl
from openteach.utils.images import rescale_image, rotate_image
from openteach.utils.network import (
    ZMQCameraPublisher,
    ZMQCompressedImageTransmitter,
    ZMQKeypointPublisher,
    ZMQKeypointSubscriber,
)
from openteach.utils.timer import FrequencyTimer


class UF850Env(Arm_Env):
    """Environment wrapper that bridges Open-Teach with the UF850 arm.

    The class is responsible for

    * streaming the RGB/depth feed coming from the robot mounted RealSense,
    * publishing end-effector state for VR visualisation, and
    * applying retargeted commands received from the teleoperation pipeline.

    The implementation mirrors :class:`LiberoEnv`, but swaps in the ROS 2 based
    :class:`~openteach.ros_links.uf850_control.DexArmControl` interface used by
    the physical UF850 platform. RealSense streaming is optional and disabled by
    default so the environment can run without the cameras attached.
    """

    def __init__(
        self,
        *,
        host,
        camport,
        timestamppublisherport,
        endeff_publish_port,
        endeffpossubscribeport,
        stream_oculus,
        camera_serial_number,
        camera_configs,
        base_frame="link_base",
        end_effector_frame="tool0",
        command_mode="cartesian_delta",
        translation_scale=1.0,
        rotation_scale=1.0,
        tf_buffer_cache_s=2.0,
        enable_realsense=False,
    ):
        # Timers / state -----------------------------------------------------------------
        self._timer = FrequencyTimer(VR_FREQ)
        self._stream_oculus = stream_oculus
        self.name = "UF850"

        self._command_mode = command_mode
        self._translation_scale = float(translation_scale)
        self._rotation_scale = float(rotation_scale)

        self._current_target_pose = None
        self._last_eef_pose = np.zeros(7, dtype=np.float32)

        # Camera configuration -----------------------------------------------------------
        self._camera_serial_number = camera_serial_number
        self._camera_configs = camera_configs
        self._use_realsense = bool(enable_realsense and rs is not None)
        if enable_realsense and not self._use_realsense:
            print("RealSense streaming disabled: pyrealsense2 module not available.")

        self.rgb_publisher = ZMQCameraPublisher(host=host, port=camport)
        if self._stream_oculus:
            self.rgb_viz_publisher = ZMQCompressedImageTransmitter(
                host=host,
                port=camport + VIZ_PORT_OFFSET,
            )
        self.depth_publisher = ZMQCameraPublisher(
            host=host,
            port=camport + DEPTH_PORT_OFFSET,
        )

        self.timestamp_publisher = ZMQKeypointPublisher(
            host=host,
            port=timestamppublisherport,
        )
        self.endeff_publisher = ZMQKeypointPublisher(
            host=host,
            port=endeff_publish_port,
        )
        self.endeff_pos_subscriber = ZMQKeypointSubscriber(
            host=host,
            port=endeffpossubscribeport,
            topic="endeff_coords",
        )

        # RealSense setup -----------------------------------------------------------------
        self._pipeline = None
        self._intrinsics_matrix = None
        self._align = None
        self._intrinsics_published = False
        if self._use_realsense:
            self._init_realsense_pipeline()

        # ROS interfaces -----------------------------------------------------------------
        self._controller = DexArmControl(robot_type="uf850")
        self._ros_executor = SingleThreadedExecutor()
        self._ros_executor.add_node(self._controller._node)
        self._ros_spin_thread = threading.Thread(
            target=self._ros_executor.spin,
            daemon=True,
        )
        self._ros_spin_thread.start()

        self._base_frame = base_frame
        self._end_effector_frame = end_effector_frame
        self._tf_buffer = Buffer(cache_time=Duration(seconds=float(tf_buffer_cache_s)))
        self._tf_listener = TransformListener(self._tf_buffer, self._controller._node)

    # ----------------------------------------------------------------------------------
    @property
    def timer(self):
        return self._timer

    # ----------------------------------------------------------------------------------
    def _init_realsense_pipeline(self):
        if rs is None:
            raise RuntimeError(
                "pyrealsense2 is unavailable; RealSense streaming cannot be enabled."
            )

        width = self._get_cam_config("width", 1280)
        height = self._get_cam_config("height", 720)
        fps = self._get_cam_config("fps", 30)
        processing_preset = self._get_cam_config("processing_preset", 1)

        self._rotation_angle = self._get_cam_config("rotation_angle", 0)

        config = rs.config()
        self._pipeline = rs.pipeline()
        config.enable_device(self._camera_serial_number)
        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)

        profile = self._pipeline.start(config)
        device = profile.get_device()
        depth_sensor = device.first_depth_sensor()
        depth_sensor.set_option(rs.option.visual_preset, processing_preset)

        active_profile = self._pipeline.get_active_profile()
        color_profile = rs.video_stream_profile(active_profile.get_stream(rs.stream.color))
        intrinsics = color_profile.get_intrinsics()
        self._intrinsics_matrix = np.array(
            [
                [intrinsics.fx, 0.0, intrinsics.ppx],
                [0.0, intrinsics.fy, intrinsics.ppy],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float32,
        )

        self._align = rs.align(rs.stream.color)

    # ----------------------------------------------------------------------------------
    def _get_cam_config(self, key, default):
        if self._camera_configs is None:
            return default
        if hasattr(self._camera_configs, key):
            return getattr(self._camera_configs, key)
        if isinstance(self._camera_configs, dict):
            return self._camera_configs.get(key, default)
        try:
            return self._camera_configs[key]
        except Exception:
            return default

    # ----------------------------------------------------------------------------------
    def get_rgb_depth_images(self):
        if not self._use_realsense:
            raise RuntimeError("RealSense streaming disabled for UF850Env instance.")

        frames = None
        while frames is None:
            frames = self._pipeline.wait_for_frames()
            frames = self._align.process(frames)
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                frames = None

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        if self._rotation_angle:
            color_image = rotate_image(color_image, self._rotation_angle)
            depth_image = rotate_image(depth_image, self._rotation_angle)

        if not self._intrinsics_published:
            self.depth_publisher.pub_intrinsics(self._intrinsics_matrix)
            self._intrinsics_published = True

        timestamp = frames.get_timestamp()
        return color_image, depth_image, timestamp

    # ----------------------------------------------------------------------------------
    def get_endeff_position(self):
        try:
            transform = self._tf_buffer.lookup_transform(
                self._base_frame,
                self._end_effector_frame,
                Time(),
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            # Fall back to the last known pose or zeros when TF is unavailable.
            return self._last_eef_pose

        translation = transform.transform.translation
        rotation = transform.transform.rotation

        pose = np.array(
            [
                translation.x,
                translation.y,
                translation.z,
                rotation.x,
                rotation.y,
                rotation.z,
                rotation.w,
            ],
            dtype=np.float32,
        )
        self._last_eef_pose = pose
        return pose

    # ----------------------------------------------------------------------------------
    def _ensure_current_target_pose(self):
        if self._current_target_pose is None:
            self._current_target_pose = np.array(self.get_endeff_position(), copy=True)
        return self._current_target_pose

    # ----------------------------------------------------------------------------------
    def take_action(self):
        command = self.endeff_pos_subscriber.recv_keypoints()
        if command is None:
            return

        command = np.asarray(command, dtype=np.float32).flatten()
        if not command.size:
            return

        if self._command_mode == "joint_delta":
            self._controller.move_robot(command)
            return

        if self._command_mode == "cartesian_pose":
            pose = self._vector_to_pose(command)
            self._controller.arm_control(pose)
            self._current_target_pose = np.array(
                [pose["x"], pose["y"], pose["z"], pose["qx"], pose["qy"], pose["qz"], pose["qw"]],
                dtype=np.float32,
            )
            return

        # Default: treat as delta cartesian command (translation + axis-angle)
        target_pose = self._ensure_current_target_pose().copy()
        if command.size >= 3:
            target_pose[:3] += command[:3] * self._translation_scale

        if command.size >= 6:
            delta_rot = Rotation.from_rotvec(command[3:6] * self._rotation_scale)
            current_rot = Rotation.from_quat(target_pose[3:7])
            target_pose[3:7] = (delta_rot * current_rot).as_quat()

        self._controller.arm_control(
            {
                "x": float(target_pose[0]),
                "y": float(target_pose[1]),
                "z": float(target_pose[2]),
                "qx": float(target_pose[3]),
                "qy": float(target_pose[4]),
                "qz": float(target_pose[5]),
                "qw": float(target_pose[6]),
            }
        )
        self._current_target_pose = target_pose

    # ----------------------------------------------------------------------------------
    def _vector_to_pose(self, vector):
        vector = np.asarray(vector, dtype=np.float32).flatten()
        if vector.size < 7:
            raise ValueError(
                "Cartesian pose command must contain at least 7 elements (x, y, z, qx, qy, qz, qw)."
            )
        return {
            "x": float(vector[0]),
            "y": float(vector[1]),
            "z": float(vector[2]),
            "qx": float(vector[3]),
            "qy": float(vector[4]),
            "qz": float(vector[5]),
            "qw": float(vector[6]),
        }

    # ----------------------------------------------------------------------------------
    def stream(self):
        self.notify_component_start(f"{self.name} environment")
        print("Start controlling the UF850 arm using the Oculus Headset.\n")

        try:
            while True:
                self.timer.start_loop()

                if self._use_realsense:
                    color_image, depth_image, timestamp = self.get_rgb_depth_images()
                    self.rgb_publisher.pub_rgb_image(color_image, timestamp)
                    self.timestamp_publisher.pub_keypoints(timestamp, "timestamps")

                    if self._stream_oculus:
                        self.rgb_viz_publisher.send_image(rescale_image(color_image, 2))

                    self.depth_publisher.pub_depth_image(depth_image, timestamp)
                else:
                    timestamp = time.time() * 1e3
                    self.timestamp_publisher.pub_keypoints(timestamp, "timestamps")

                endeff_pose = self.get_endeff_position()
                if endeff_pose is not None:
                    self.endeff_publisher.pub_keypoints(endeff_pose, "endeff_coords")

                self.take_action()
                self.timer.end_loop()
        except KeyboardInterrupt:
            pass
        finally:
            self._shutdown()

    # ----------------------------------------------------------------------------------
    def _shutdown(self):
        print("Stopping the UF850 environment!")
        with suppress(Exception):
            self.rgb_publisher.stop()
        if self._stream_oculus:
            with suppress(Exception):
                self.rgb_viz_publisher.stop()
        with suppress(Exception):
            self.depth_publisher.stop()
        with suppress(Exception):
            self.timestamp_publisher.stop()
        with suppress(Exception):
            self.endeff_publisher.stop()
        with suppress(Exception):
            self.endeff_pos_subscriber.stop()

        if hasattr(self, "_pipeline") and self._pipeline is not None:
            with suppress(Exception):
                self._pipeline.stop()

        if hasattr(self, "_ros_executor") and self._ros_executor is not None:
            with suppress(Exception):
                self._ros_executor.shutdown()

        if hasattr(self, "_controller") and self._controller is not None:
            node = getattr(self._controller, "_node", None)
            if node is not None and rclpy.ok():
                with suppress(Exception):
                    node.destroy_node()

        if rclpy.ok():
            with suppress(Exception):
                rclpy.shutdown()
