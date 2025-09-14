import threading

import mujoco
import mujoco.viewer

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState


class MujocoEnv(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)

        self._mj_lock = threading.Lock()

        self.declare_parameter('mjcf_path', '')
        self._mjcf_path = self.get_parameter('mjcf_path').get_parameter_value().string_value

        self._mj_model = mujoco.MjModel.from_xml_path(self._mjcf_path)
        self._mj_data = mujoco.MjData(self._mj_model)
        self._mj_viewer = mujoco.viewer.launch_passive(self._mj_model, self._mj_data, key_callback=self._key_callback)

        self._mj_ctrl = Float64MultiArray()
        self._mj_ctrl.data = [0.0 for _ in range(self._mj_model.nu)]

        self._joint_state = JointState()
        self._joint_state.position = [0.0 for _ in range(self._mj_model.nq)]
        self._joint_state.velocity = [0.0 for _ in range(self._mj_model.nv)]

        self._ctrl_subscription = self.create_subscription(Float64MultiArray, "mujoco_ctrl", self._ctrl_cb, 1)
        self._joint_state_publisher = self.create_publisher(JointState, "mujoco_joint_state", 1)

        self._mj_step_timer = self.create_timer(self._mj_model.opt.timestep, self._mj_step_cb)
        self._mj_viewer_timer = self.create_timer(0.01, self._mj_viewer_cb)

    def _mj_step_cb(self):
        self._mj_lock.acquire()
        self._update_ctrl()
        mujoco.mj_step(self._mj_model, self._mj_data)
        self._update_sensor()
        self._mj_lock.release()
        self._publish_sensor()

    def _mj_viewer_cb(self):
        self._mj_lock.acquire()
        self._mj_viewer.sync()
        self._mj_lock.release()

    def _update_ctrl(self):
        self._mj_data.ctrl[:] = self._mj_ctrl.data

    def _update_sensor(self):
        mujoco.mj_forward(self._mj_model, self._mj_data)
        self._joint_state.position = self._mj_data.qpos.tolist()
        self._joint_state.velocity = self._mj_data.qvel.tolist()

    def _publish_sensor(self):
        self._joint_state_publisher.publish(self._joint_state)

    def _ctrl_cb(self, msg: Float64MultiArray):
        self._mj_ctrl.data = msg.data

    def _key_callback(self, keycode):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = MujocoEnv("mujoco_env")
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
    rclpy.shutdown()
