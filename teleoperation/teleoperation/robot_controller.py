import queue

from scipy.spatial.transform import Rotation
import scipy.interpolate
import numpy as np
import placo

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Transform


class RobotController(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)

        parameters = [
            ('base_frame', ''),
            ('end_frame', ''),
            ('urdf_path', ''),
            ('ready_position', [])
        ]
        self.declare_parameters(namespace='', parameters=parameters)

        self._base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self._end_frame = self.get_parameter('end_frame').get_parameter_value().string_value
        self._urdf_path = self.get_parameter('urdf_path').get_parameter_value().string_value
        self._ready_position = np.deg2rad(
            self.get_parameter('ready_position').get_parameter_value().double_array_value).tolist()

        self._dt = 0.001

        self._env_init = False

        self._robot_wrapper = placo.RobotWrapper(self._urdf_path, placo.Flags.ignore_collisions)
        self._solver = placo.KinematicsSolver(self._robot_wrapper)
        self._solver.mask_fbase(True)
        self._effector_task = self._solver.add_frame_task(self._end_frame, np.eye(4))
        self._effector_task.configure("link_ee", "soft", 1.0, 0.1)
        self._manipulbility_task = self._solver.add_manipulability_task(self._end_frame, "both", 1.0)
        self._manipulbility_task.configure("manipulability", "soft", 1e-1)
        self._solver.enable_joint_limits(True)
        self._solver.enable_velocity_limits(True)
        self._solver.dt = self._dt
        self._robot_dof = len(self._robot_wrapper.joint_names())

        self._last_position = [0.0 for _ in range(len(self._robot_wrapper.joint_names()))]
        self._ctrl_position = Float64MultiArray()
        self._queue = queue.Queue(round(1 / self._dt))

        self._tf_broadcaster = TransformBroadcaster(self)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._joint_state = JointState()
        self._joint_state.name = list(self._robot_wrapper.joint_names())

        self._joint_state_subscription = self.create_subscription(JointState, "/mujoco_joint_state",
                                                                  self._joint_state_cb, 1)
        self._ctrl_publisher = self.create_publisher(Float64MultiArray, "/mujoco_ctrl", 1)
        self._joint_state_publisher = self.create_publisher(JointState, "/joint_states", qos_profile=1)

        self._ready_motion_service = self.create_service(SetBool, "/ready_motion", self._ready_motion_cb)

        self._controller_init_client_callback_group = MutuallyExclusiveCallbackGroup()
        self._controller_init_client = self.create_client(SetBool, "/controller_init",
                                                          callback_group=self._controller_init_client_callback_group)

        while not self._controller_init_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('waiting ...')

        self._timer = self.create_timer(10 * self._dt, self._timer_cb)

        self._ctrl_timer_callback_group = MutuallyExclusiveCallbackGroup()
        self._ctrl_timer = self.create_timer(self._dt, self._ctrl_timer_cb,
                                             callback_group=self._ctrl_timer_callback_group)

    def _timer_cb(self):
        if not self._tf_buffer.can_transform(self._base_frame, 'end_desired_frame', rclpy.time.Time()):
            return

        transform = self._tf_buffer.lookup_transform(self._base_frame, 'end_desired_frame', rclpy.time.Time())
        T_desired_ee = self._transform_to_matrix(transform.transform)

        self._effector_task.T_world_frame = T_desired_ee

        try:
            self._solver.solve(True)
            self._robot_wrapper.update_kinematics()
        except:
            pass
        desired_position = [self._robot_wrapper.get_joint(joint_name) for joint_name in
                            self._robot_wrapper.joint_names()]
        self._queue.put(desired_position)
        self._last_position[:] = desired_position

    def _ctrl_timer_cb(self):
        if not self._env_init:
            return

        position = self._last_position.copy()
        if not self._queue.empty():
            position[:] = self._queue.get()

        self._joint_state.header.stamp = self.get_clock().now().to_msg()
        self._joint_state.position = position
        self._joint_state_publisher.publish(self._joint_state)

        self._ctrl_position.data = position
        self._ctrl_publisher.publish(self._ctrl_position)

    def _ready_motion_cb(self, request: SetBool.Request, response: SetBool.Response):
        if not request.data:
            return response
        self._ready_motion()
        response.success = True
        return response

    def _joint_state_cb(self, msg: JointState):
        if self._env_init:
            return
        self._last_position[:] = msg.position
        self._env_init = True
        request = SetBool.Request()
        request.data = True
        self._controller_init_client.call(request)

    def _ready_motion(self):
        self._tf_buffer.clear()

        x = np.array([0, self._dt, 1 - self._dt, 1])
        y = np.zeros((4, self._robot_dof))
        y[[0, 1], :] = self._last_position
        y[[-2, -1], :] = self._ready_position
        funs = []
        for i in range(self._robot_dof):
            funs.append(scipy.interpolate.splrep(x, y[:, i]))
        x_new = np.arange(0, 1 + self._dt, self._dt)
        y_new = np.zeros((x_new.shape[0], self._robot_dof))
        for i in range(self._robot_dof):
            y_new[:, i] = scipy.interpolate.splev(x_new, funs[i])
        for i in range(x_new.shape[0]):
            last_position = y_new[i, :].tolist()
            self._queue.put(last_position)
            self._last_position[:] = last_position

        for i, joint_name in enumerate(self._robot_wrapper.joint_names()):
            self._robot_wrapper.set_joint(joint_name, self._ready_position[i])

    def _transform_to_matrix(self, transform: Transform):
        matrix = np.eye(4)
        matrix[0, 3] = transform.translation.x
        matrix[1, 3] = transform.translation.y
        matrix[2, 3] = transform.translation.z

        quat = [
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w
        ]

        matrix[:3, :3] = Rotation.from_quat(quat).as_matrix()
        return matrix


def main(args=None):
    rclpy.init(args=args)
    node = RobotController("robot_controller")
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
    rclpy.shutdown()
