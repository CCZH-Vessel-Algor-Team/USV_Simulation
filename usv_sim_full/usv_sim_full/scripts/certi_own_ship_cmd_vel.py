#!/usr/bin/env python3
"""认证场景：按设定航向与航速发布 cmd_vel；检测到其它发布者时退让停发。"""

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class CertiOwnShipCmdVel(Node):
    def __init__(self):
        super().__init__('certi_own_ship_cmd_vel')

        self.declare_parameter('namespace', 'usv_1')
        self.declare_parameter('speed_mps', 4.0)
        self.declare_parameter('course_deg', 0.0)
        self.declare_parameter('heading_kp', 1.2)
        self.declare_parameter('max_yaw_rate', 0.35)
        self.declare_parameter('publish_hz', 10.0)
        self.declare_parameter('yield_check_hz', 2.0)

        ns = self.get_parameter('namespace').get_parameter_value().string_value.strip('/')
        self.namespace = ns if ns else 'usv_1'
        self.speed_mps = float(
            self.get_parameter('speed_mps').get_parameter_value().double_value
        )
        self._course_rad = math.radians(
            float(self.get_parameter('course_deg').get_parameter_value().double_value)
        )
        self._heading_kp = float(
            self.get_parameter('heading_kp').get_parameter_value().double_value
        )
        self._max_yaw_rate = abs(
            float(self.get_parameter('max_yaw_rate').get_parameter_value().double_value)
        )
        self.publish_hz = float(
            self.get_parameter('publish_hz').get_parameter_value().double_value
        )
        yield_hz = float(
            self.get_parameter('yield_check_hz').get_parameter_value().double_value
        )

        self.cmd_vel_topic = f'/{self.namespace}/cmd_vel'
        self._current_yaw: Optional[float] = None
        self._publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self._publishing_enabled = True
        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self._pub_timer = self.create_timer(
            1.0 / max(self.publish_hz, 1.0),
            self._publish_cmd_vel,
        )
        self._yield_timer = self.create_timer(
            1.0 / max(yield_hz, 0.5),
            self._check_yield,
        )

        self.get_logger().info(
            f'Publishing {self.speed_mps:.2f} m/s @ course '
            f'{math.degrees(self._course_rad):.1f}° on {self.cmd_vel_topic} '
            f'(heading hold kp={self._heading_kp:.2f})'
        )

    def _on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        self._current_yaw = _yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def _publish_cmd_vel(self) -> None:
        if not self._publishing_enabled:
            return
        if self._current_yaw is None:
            return

        heading_err = _normalize_angle(self._course_rad - self._current_yaw)
        yaw_rate = max(
            -self._max_yaw_rate,
            min(self._max_yaw_rate, self._heading_kp * heading_err),
        )

        msg = Twist()
        msg.linear.x = self.speed_mps
        msg.angular.z = yaw_rate
        self._publisher.publish(msg)

    def _check_yield(self) -> None:
        try:
            count = self.count_publishers(self.cmd_vel_topic)
        except Exception:
            return
        should_yield = count > 1
        if should_yield and self._publishing_enabled:
            self._publishing_enabled = False
            stop = Twist()
            self._publisher.publish(stop)
            self.get_logger().info(
                f'Yielding {self.cmd_vel_topic}: {count} publisher(s) detected'
            )
        elif not should_yield and not self._publishing_enabled:
            self._publishing_enabled = True
            self.get_logger().info(
                f'Resuming publish on {self.cmd_vel_topic}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = CertiOwnShipCmdVel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
