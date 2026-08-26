#!/usr/bin/env python3
"""ROS-only controller for the PX4-style three-axis Gazebo payload gimbal."""

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from usv_interfaces.msg import GimbalControl, GimbalState


class GimbalController(Node):
    MODE_NONE = 0
    MODE_SPEED = 1
    MODE_ANGLE = 2
    MODE_MIXED = 3

    def __init__(self):
        super().__init__('gimbal_controller')
        self.declare_parameter('robot_name', 'usv_1')
        self.declare_parameter('gimbal_name', 'payload_gimbal')
        self.declare_parameter('control_topic', '')
        self.declare_parameter('command_timeout_sec', 0.5)
        self.declare_parameter('roll_min_rad', -0.785398)
        self.declare_parameter('roll_max_rad', 0.785398)
        self.declare_parameter('pitch_min_rad', -2.35619)
        self.declare_parameter('pitch_max_rad', 0.7854)

        self.robot_name = self.get_parameter('robot_name').value
        self.gimbal_name = self.get_parameter('gimbal_name').value
        configured_topic = self.get_parameter('control_topic').value
        self.control_topic = self._robot_topic(
            configured_topic or f'/gimbal/{self.gimbal_name}/control'
        )
        self.command_timeout = float(
            self.get_parameter('command_timeout_sec').value
        )
        self.limits = {
            'roll': (
                float(self.get_parameter('roll_min_rad').value),
                float(self.get_parameter('roll_max_rad').value),
            ),
            'pitch': (
                float(self.get_parameter('pitch_min_rad').value),
                float(self.get_parameter('pitch_max_rad').value),
            ),
            'yaw': (None, None),
        }
        self.target = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        self.goal = dict(self.target)
        self.feedback = dict(self.target)
        self.speed = dict(self.target)
        self.mode = self.MODE_NONE
        self.last_command_wall_time = None
        self.last_update_wall_time = time.monotonic()

        self.command_publishers = {
            axis: self.create_publisher(
                Float64,
                self._robot_topic(
                    f'/gimbal/{self.gimbal_name}/{axis}/cmd_pos'
                ),
                10,
            )
            for axis in self.target
        }
        self.state_publisher = self.create_publisher(
            GimbalState,
            self._robot_topic(f'/gimbal/{self.gimbal_name}/state'),
            10,
        )
        self.create_subscription(
            GimbalControl, self.control_topic, self._control_callback, 10
        )
        self.create_subscription(
            JointState,
            self._robot_topic('/joint_states'),
            self._joint_state_callback,
            10,
        )
        self.create_timer(0.05, self._update)

    def _robot_topic(self, topic):
        if topic.startswith(f'/{self.robot_name}/'):
            return topic
        return f'/{self.robot_name}/' + topic.lstrip('/')

    def _control_callback(self, message):
        if message.mode not in (
            self.MODE_SPEED,
            self.MODE_ANGLE,
            self.MODE_MIXED,
        ):
            self.mode = self.MODE_NONE
            return

        angles = dict(zip(('roll', 'pitch', 'yaw'), message.angle))
        speeds = dict(zip(('roll', 'pitch', 'yaw'), message.speed))
        if message.mode in (self.MODE_ANGLE, self.MODE_MIXED):
            for axis, value in angles.items():
                if math.isfinite(value):
                    self.goal[axis] = self._clamp(axis, math.radians(value))
            if message.mode == self.MODE_ANGLE:
                self.target = dict(self.goal)
        if message.mode in (self.MODE_SPEED, self.MODE_MIXED):
            for axis, value in speeds.items():
                self.speed[axis] = (
                    math.radians(value) if math.isfinite(value) else 0.0
                )

        self.mode = message.mode
        self.last_command_wall_time = time.monotonic()
        if self.mode == self.MODE_ANGLE:
            self._publish_targets()

    def _joint_state_callback(self, message):
        for name, position in zip(message.name, message.position):
            for axis in self.feedback:
                expected = f'{self.gimbal_name}_{axis}_joint'
                if name == expected or name.endswith('/' + expected):
                    self.feedback[axis] = position

    def _update(self):
        now = time.monotonic()
        dt = min(now - self.last_update_wall_time, 0.1)
        self.last_update_wall_time = now
        active = (
            self.last_command_wall_time is not None
            and now - self.last_command_wall_time <= self.command_timeout
        )
        if active and self.mode in (self.MODE_SPEED, self.MODE_MIXED):
            for axis in self.target:
                candidate = self.target[axis] + self.speed[axis] * dt
                if self.mode == self.MODE_MIXED:
                    # The requested angle bounds rate motion without overshoot.
                    requested = self.goal[axis]
                    if self.speed[axis] > 0.0:
                        candidate = min(candidate, requested)
                    elif self.speed[axis] < 0.0:
                        candidate = max(candidate, requested)
                self.target[axis] = self._clamp(axis, candidate)
            self._publish_targets()
        self._publish_state()

    def _clamp(self, axis, value):
        lower, upper = self.limits[axis]
        if lower is None:
            return value
        return max(lower, min(upper, value))

    def _publish_targets(self):
        for axis, publisher in self.command_publishers.items():
            publisher.publish(Float64(data=self.target[axis]))

    def _publish_state(self):
        message = GimbalState()
        message.header.stamp = self.get_clock().now().to_msg()
        axes = ('roll', 'pitch', 'yaw')
        message.angle_rt = [math.degrees(self.feedback[axis]) for axis in axes]
        message.angle_rt_rate = [
            math.degrees(self.speed[axis]) for axis in axes
        ]
        # ROS stores fixed-size float32 fields as numpy.float32 values, while
        # this generated setter accepts a Python-float sequence or ndarray.
        message.angle_rate_set = [
            float(rate) for rate in message.angle_rt_rate
        ]
        self.state_publisher.publish(message)


def main():
    rclpy.init()
    node = GimbalController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
