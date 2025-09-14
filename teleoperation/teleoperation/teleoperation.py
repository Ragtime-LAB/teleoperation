import time
import numpy as np
from pynput import keyboard

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from system_ctrl.msg import AlgoCtrl


class Teleoperation(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)

        parameters = [
            ('base_frame', ''),
            ('end_frame', ''),
        ]
        self.declare_parameters(namespace='', parameters=parameters)

        self._base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self._end_frame = self.get_parameter('end_frame').get_parameter_value().string_value

        self._tf_init = False
        self._teleoperation_init = False
        self._odometry_init = False
        self._start_teleoperation = False

        self._dt = 0.01

        self._init_tf_stamped = TransformStamped()
        self._init_ee_tf_stamped = TransformStamped()

        self._pressed_keys = set()

        self._tf_broadcaster = TransformBroadcaster(self)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._odometry_subscription = self.create_subscription(Odometry, "/baton_mini/stereo3/odometry",
                                                               self._odometry_cb, qos_profile=1)

        self._algo_ctrl_publisher = self.create_publisher(AlgoCtrl, "/baton_mini/stereo3_ctrl", qos_profile=1)

        self._ready_motion_client = self.create_client(SetBool, "/ready_motion")
        self._controller_init_service = self.create_service(SetBool, "/controller_init", self._controller_init_cb)

        self._timer = self.create_timer(self._dt, self._timer_cb)

        self._listener = keyboard.Listener(on_press=self._on_press, on_release=self._on_release)
        self._listener.start()

        self._stop_teleoperate()

    def _odometry_cb(self, msg: Odometry):
        if not self._tf_init:
            return

        if not self._start_teleoperation:
            return

        if not self._odometry_init:
            self._init_tf_stamped.header.stamp = self.get_clock().now().to_msg()
            self._init_tf_stamped.header.frame_id = self._base_frame
            self._init_tf_stamped.child_frame_id = 'odom_base_frame'

            if np.linalg.norm([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]) > 1e-3:
                return

            self._init_tf_stamped.transform.translation.x = msg.pose.pose.position.x
            self._init_tf_stamped.transform.translation.y = msg.pose.pose.position.y
            self._init_tf_stamped.transform.translation.z = msg.pose.pose.position.z
            self._init_tf_stamped.transform.rotation.x = msg.pose.pose.orientation.x
            self._init_tf_stamped.transform.rotation.y = msg.pose.pose.orientation.y
            self._init_tf_stamped.transform.rotation.z = msg.pose.pose.orientation.z
            self._init_tf_stamped.transform.rotation.w = msg.pose.pose.orientation.w

            ee_tf_stamped = self._tf_buffer.lookup_transform(self._base_frame, self._end_frame, rclpy.time.Time())
            self._init_ee_tf_stamped.header.stamp = self.get_clock().now().to_msg()
            self._init_ee_tf_stamped.header.frame_id = self._base_frame
            self._init_ee_tf_stamped.child_frame_id = 'end_init_frame'
            self._init_ee_tf_stamped.transform = ee_tf_stamped.transform

            self._tf_broadcaster.sendTransform([self._init_tf_stamped, self._init_ee_tf_stamped])

            self._odometry_init = True

        tf_stamped = TransformStamped()
        tf_stamped.header.stamp = self.get_clock().now().to_msg()
        tf_stamped.header.frame_id = self._base_frame
        tf_stamped.child_frame_id = 'odom_current_frame'

        tf_stamped.transform.translation.x = msg.pose.pose.position.x
        tf_stamped.transform.translation.y = msg.pose.pose.position.y
        tf_stamped.transform.translation.z = msg.pose.pose.position.z
        tf_stamped.transform.rotation.x = msg.pose.pose.orientation.x
        tf_stamped.transform.rotation.y = msg.pose.pose.orientation.y
        tf_stamped.transform.rotation.z = msg.pose.pose.orientation.z
        tf_stamped.transform.rotation.w = msg.pose.pose.orientation.w
        self._tf_broadcaster.sendTransform([self._init_tf_stamped, tf_stamped])
        self._teleoperation_init = True

    def _timer_cb(self):
        if not self._teleoperation_init:
            return

        self._init_tf_stamped.header.stamp = self.get_clock().now().to_msg()
        self._init_ee_tf_stamped.header.stamp = self.get_clock().now().to_msg()
        self._tf_broadcaster.sendTransform([self._init_tf_stamped, self._init_ee_tf_stamped])

        if not self._tf_buffer.can_transform('odom_base_frame', 'odom_current_frame', rclpy.time.Time()):
            return

        transform = self._tf_buffer.lookup_transform('odom_base_frame', 'odom_current_frame', rclpy.time.Time())
        tf_stamped = TransformStamped()
        tf_stamped.header.stamp = self.get_clock().now().to_msg()
        tf_stamped.header.frame_id = 'end_init_frame'
        tf_stamped.child_frame_id = 'end_desired_frame'
        tf_stamped.transform = transform.transform
        self._tf_broadcaster.sendTransform([tf_stamped])

    def _controller_init_cb(self, request: SetBool.Request, response: SetBool.Response):
        if not request.data:
            return response
        self._tf_init = True
        response.success = True
        return response

    def _start_teleoperate(self):
        algo_ctrl = AlgoCtrl()
        algo_ctrl.algo_enable = True
        algo_ctrl.algo_reboot = False
        algo_ctrl.algo_reset = True
        self._algo_ctrl_publisher.publish(algo_ctrl)
        self._start_teleoperation = True

    def _stop_teleoperate(self):
        self._start_teleoperation = False
        time.sleep(5 * self._dt)
        self._odometry_init = False
        self._teleoperation_init = False

        algo_ctrl = AlgoCtrl()
        algo_ctrl.algo_enable = False
        algo_ctrl.algo_reboot = False
        algo_ctrl.algo_reset = False
        self._algo_ctrl_publisher.publish(algo_ctrl)

    def _on_press(self, key):
        try:
            key_char = key.char
        except AttributeError:
            key_char = str(key)

        if key_char not in self._pressed_keys:
            self._pressed_keys.add(key_char)
            if key_char == 'Key.ctrl':
                self._stop_teleoperate()

                request = SetBool.Request()
                request.data = True
                self._ready_motion_client.call(request)
            elif key_char == 'Key.ctrl_r':
                if self._start_teleoperation:
                    self._stop_teleoperate()
                else:
                    self._start_teleoperate()

    def _on_release(self, key):
        try:
            key_char = key.char
        except AttributeError:
            key_char = str(key)

        if key_char in self._pressed_keys:
            self._pressed_keys.remove(key_char)


def main(args=None):
    rclpy.init(args=args)
    node = Teleoperation("teleoperation")
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
    rclpy.shutdown()
