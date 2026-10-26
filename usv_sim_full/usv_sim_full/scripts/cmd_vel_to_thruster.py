#!/usr/bin/env python3
import math
from time import monotonic

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64


class PID:
    def __init__(self, kp, ki, kd, out_min, out_max):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        self.integral = 0.0
        self.prev_error = 0.0
        self._first = True

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self._first = True

    def update(self, error, dt):
        if dt <= 0.0:
            return 0.0
        p = self.kp * error
        i_raw = self.integral + error * dt
        d = 0.0 if self._first else self.kd * (error - self.prev_error) / dt
        output = p + self.ki * i_raw + d
        if output > self.out_max:
            output = self.out_max
        elif output < self.out_min:
            output = self.out_min
        else:
            self.integral = i_raw
        self.prev_error = error
        self._first = False
        return output


class CmdVelToThruster(Node):
    def __init__(self, **kwargs):
        super().__init__('cmd_vel_to_thruster', **kwargs)

        self.declare_parameter('namespace', 'usv_1')
        ns = self.get_parameter('namespace').get_parameter_value().string_value.strip('/')
        self.namespace = ns if ns else 'usv_1'
        self._require_navigation_ready = self.declare_parameter(
            'require_navigation_ready', False,
            descriptor=ParameterDescriptor(read_only=True),
        ).value
        self._navigation_ready = False
        self._ready_stamp = None

        cmd_vel_topic = (
            f'/{self.namespace}/control/cmd_vel'
            if self._require_navigation_ready else f'/{self.namespace}/cmd_vel'
        )
        cmd_vel_fallback_topic = '/cmd_vel'
        odom_topic = f'/{self.namespace}/odom'
        left_thrust_topic = f'/{self.namespace}/thrusters/left/thrust'
        right_thrust_topic = f'/{self.namespace}/thrusters/right/thrust'
        left_pos_topic = f'/{self.namespace}/thrusters/left/pos'
        right_pos_topic = f'/{self.namespace}/thrusters/right/pos'

        self.cmd_sub = self.create_subscription(Twist, cmd_vel_topic, self.cmd_callback, 10)
        self.cmd_sub_fallback = None
        if not self._require_navigation_ready:
            self.cmd_sub_fallback = self.create_subscription(
                Twist, cmd_vel_fallback_topic, self.cmd_callback, 10,
            )
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)

        self.left_pub = self.create_publisher(Float64, left_thrust_topic, 10)
        self.right_pub = self.create_publisher(Float64, right_thrust_topic, 10)
        self.left_pos_pub = self.create_publisher(Float64, left_pos_topic, 10)
        self.right_pos_pub = self.create_publisher(Float64, right_pos_topic, 10)

        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('max_thrust', 1000.0)
        self.declare_parameter('v_kp', 2000.0)
        self.declare_parameter('v_ki', 60.0)
        self.declare_parameter('v_kd', 0.0)
        self.declare_parameter('w_kp', 5000.0)
        self.declare_parameter('w_ki', 40.0)
        self.declare_parameter('w_kd', 0.0)
        self.declare_parameter('odom_timeout', 0.5)
        self.declare_parameter('cmd_timeout', 1.0)
        self.declare_parameter('thrust_rate_limit', 0.0)

        self.desired_v = 0.0
        self.desired_w = 0.0
        self.actual_v = 0.0
        self.actual_w = 0.0
        self._odom_stamp = None
        self._cmd_stamp = None
        self._last_control = self.get_clock().now()
        self._cmd_active = False

        rate = self.get_parameter('control_rate').value
        timer_clock = (
            Clock(clock_type=ClockType.STEADY_TIME)
            if self._require_navigation_ready else self.get_clock()
        )
        self.timer = self.create_timer(1.0 / rate, self.control_loop, clock=timer_clock)

        self._last_left = 0.0
        self._last_right = 0.0
        self.ready_sub = None
        if self._require_navigation_ready:
            # No retained Bool can prove a live heartbeat; the topic is fixed
            # by the vessel namespace and cannot be changed by a ROS parameter.
            self.ready_sub = self.create_subscription(
                Bool, f'/{self.namespace}/navigation/ready', self.ready_callback,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.VOLATILE),
            )

        self.get_logger().info(
            f'PID cmd_vel->thruster started for {self.namespace}: '
            f'cmd={cmd_vel_topic}, odom={odom_topic}, rate={rate:.0f}Hz'
        )

    def cmd_callback(self, msg):
        if (
            not self._navigation_ready_ok()
            or not all(math.isfinite(value) for value in (
                msg.linear.x, msg.linear.y, msg.linear.z,
                msg.angular.x, msg.angular.y, msg.angular.z,
            ))
            or (self._require_navigation_ready
                and msg.linear.x == 0.0 and msg.angular.z == 0.0)
        ):
            self._publish_zero_thrust()
            return
        self.desired_v = msg.linear.x
        self.desired_w = msg.angular.z
        self._cmd_stamp = monotonic()
        self._cmd_active = True

    def _navigation_ready_ok(self):
        return not self._require_navigation_ready or (
            self._navigation_ready and self._ready_stamp is not None
            and 0.0 <= monotonic() - self._ready_stamp <= 1.5
        )

    def ready_callback(self, msg):
        if not msg.data or not self._navigation_ready_ok():
            self._publish_zero_thrust()
        self._navigation_ready = msg.data
        self._ready_stamp = monotonic()

    def _publish_zero_thrust(self):
        self.desired_v = self.desired_w = 0.0
        self._cmd_active = False
        self._cmd_stamp = None
        self._last_left = self._last_right = 0.0
        if hasattr(self, '_v_pid'):
            self._v_pid.reset()
        if hasattr(self, '_w_pid'):
            self._w_pid.reset()
        z = Float64(data=0.0)
        self.left_pub.publish(z)
        self.right_pub.publish(z)
        self.left_pos_pub.publish(z)
        self.right_pos_pub.publish(z)

    def odom_callback(self, msg):
        twist = msg.twist.twist
        if not all(math.isfinite(value) for value in (
            twist.linear.x, twist.linear.y, twist.linear.z,
            twist.angular.x, twist.angular.y, twist.angular.z,
        )):
            self._odom_stamp = None
            self._publish_zero_thrust()
            return
        self.actual_v = msg.twist.twist.linear.x
        self.actual_w = msg.twist.twist.angular.z
        self._odom_stamp = monotonic()

    def control_loop(self):
        now = self.get_clock().now()
        dt = (now - self._last_control).nanoseconds * 1e-9
        self._last_control = now
        dt = max(dt, 0.001)

        max_t = self.get_parameter('max_thrust').value

        v_kp = self.get_parameter('v_kp').value
        v_ki = self.get_parameter('v_ki').value
        v_kd = self.get_parameter('v_kd').value
        w_kp = self.get_parameter('w_kp').value
        w_ki = self.get_parameter('w_ki').value
        w_kd = self.get_parameter('w_kd').value

        cmd_timeout = self.get_parameter('cmd_timeout').value
        odom_timeout = self.get_parameter('odom_timeout').value

        steady_now = monotonic()
        if (
            not self._navigation_ready_ok()
            or not self._cmd_active
            or self._cmd_stamp is None or self._odom_stamp is None
            or not math.isfinite(cmd_timeout) or not math.isfinite(odom_timeout)
            or not 0.0 <= steady_now - self._cmd_stamp <= cmd_timeout
            or not 0.0 <= steady_now - self._odom_stamp <= odom_timeout
        ):
            self._publish_zero_thrust()
            return

        if not hasattr(self, '_v_pid') or v_kp != self._last_v_kp or v_ki != self._last_v_ki or v_kd != self._last_v_kd:
            self._v_pid = PID(v_kp, v_ki, v_kd, -max_t, max_t)
            self._last_v_kp, self._last_v_ki, self._last_v_kd = v_kp, v_ki, v_kd
        if not hasattr(self, '_w_pid') or w_kp != self._last_w_kp or w_ki != self._last_w_ki or w_kd != self._last_w_kd:
            self._w_pid = PID(w_kp, w_ki, w_kd, -max_t, max_t)
            self._last_w_kp, self._last_w_ki, self._last_w_kd = w_kp, w_ki, w_kd

        base = self._v_pid.update(self.desired_v - self.actual_v, dt)
        diff = self._w_pid.update(self.desired_w - self.actual_w, dt)
        if not math.isfinite(base) or not math.isfinite(diff):
            self._publish_zero_thrust()
            return

        left = base - diff
        right = base + diff

        if left > max_t:
            right -= (left - max_t)
            left = max_t
        elif left < -max_t:
            right -= (left + max_t)
            left = -max_t
        if right > max_t:
            left -= (right - max_t)
            right = max_t
        elif right < -max_t:
            left -= (right + max_t)
            right = -max_t
        left = max(-max_t, min(max_t, left))
        right = max(-max_t, min(max_t, right))

        rate_limit = self.get_parameter('thrust_rate_limit').value
        if rate_limit > 0.0:
            left = max(self._last_left - rate_limit, min(self._last_left + rate_limit, left))
            right = max(self._last_right - rate_limit, min(self._last_right + rate_limit, right))
        self._last_left = left
        self._last_right = right

        lt_msg = Float64(data=float(left))
        rt_msg = Float64(data=float(right))
        self.left_pub.publish(lt_msg)
        self.right_pub.publish(rt_msg)

        pos_msg = Float64(data=0.0)
        self.left_pos_pub.publish(pos_msg)
        self.right_pos_pub.publish(pos_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToThruster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node._publish_zero_thrust()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
