#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64


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
    def __init__(self):
        super().__init__('cmd_vel_to_thruster')

        self.declare_parameter('namespace', 'usv_1')
        ns = self.get_parameter('namespace').get_parameter_value().string_value.strip('/')
        self.namespace = ns if ns else 'usv_1'

        cmd_vel_topic = f'/{self.namespace}/cmd_vel'
        cmd_vel_fallback_topic = '/cmd_vel'
        odom_topic = f'/{self.namespace}/odom'
        left_thrust_topic = f'/{self.namespace}/thrusters/left/thrust'
        right_thrust_topic = f'/{self.namespace}/thrusters/right/thrust'
        left_pos_topic = f'/{self.namespace}/thrusters/left/pos'
        right_pos_topic = f'/{self.namespace}/thrusters/right/pos'

        self.cmd_sub = self.create_subscription(Twist, cmd_vel_topic, self.cmd_callback, 10)
        self.cmd_sub_fallback = self.create_subscription(Twist, cmd_vel_fallback_topic, self.cmd_callback, 10)
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
        self._odom_stamp = self.get_clock().now()
        self._cmd_stamp = self.get_clock().now()
        self._last_control = self.get_clock().now()
        self._cmd_active = False

        rate = self.get_parameter('control_rate').value
        self.timer = self.create_timer(1.0 / rate, self.control_loop)

        self._last_left = 0.0
        self._last_right = 0.0

        self.get_logger().info(
            f'PID cmd_vel->thruster started for {self.namespace}: '
            f'cmd={cmd_vel_topic}, odom={odom_topic}, rate={rate:.0f}Hz'
        )

    def cmd_callback(self, msg):
        self.desired_v = msg.linear.x
        self.desired_w = msg.angular.z
        self._cmd_stamp = self.get_clock().now()
        self._cmd_active = True

    def _publish_zero_thrust(self):
        z = Float64(data=0.0)
        self.left_pub.publish(z)
        self.right_pub.publish(z)
        self.left_pos_pub.publish(z)
        self.right_pos_pub.publish(z)

    def odom_callback(self, msg):
        self.actual_v = msg.twist.twist.linear.x
        self.actual_w = msg.twist.twist.angular.z
        self._odom_stamp = self.get_clock().now()

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

        cmd_age = (now - self._cmd_stamp).nanoseconds * 1e-9
        if cmd_age > cmd_timeout:
            self.desired_v = 0.0
            self.desired_w = 0.0
            self._publish_zero_thrust()
            if hasattr(self, '_v_pid'):
                self._v_pid.reset()
                self._w_pid.reset()
            self._cmd_active = False
            return

        if (now - self._odom_stamp).nanoseconds * 1e-9 > odom_timeout:
            self.actual_v = self.desired_v
            self.actual_w = self.desired_w

        if not hasattr(self, '_v_pid') or v_kp != self._last_v_kp or v_ki != self._last_v_ki or v_kd != self._last_v_kd:
            self._v_pid = PID(v_kp, v_ki, v_kd, -max_t, max_t)
            self._last_v_kp, self._last_v_ki, self._last_v_kd = v_kp, v_ki, v_kd
        if not hasattr(self, '_w_pid') or w_kp != self._last_w_kp or w_ki != self._last_w_ki or w_kd != self._last_w_kd:
            self._w_pid = PID(w_kp, w_ki, w_kd, -max_t, max_t)
            self._last_w_kp, self._last_w_ki, self._last_w_kd = w_kp, w_ki, w_kd

        base = self._v_pid.update(self.desired_v - self.actual_v, dt)
        diff = self._w_pid.update(self.desired_w - self.actual_w, dt)

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
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
