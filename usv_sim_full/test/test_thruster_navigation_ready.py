"""Exercise the real PID/control loop with mocked clocks and actuator publishers."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.clock import ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool


SOURCE = Path(__file__).parents[1] / 'usv_sim_full/scripts/cmd_vel_to_thruster.py'
SPEC = importlib.util.spec_from_file_location('thruster_under_test', SOURCE)
thruster = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(thruster)


class FakeTime:
    def __init__(self, seconds):
        self.nanoseconds = round(seconds * 1e9)

    def __sub__(self, other):
        return SimpleNamespace(nanoseconds=self.nanoseconds - other.nanoseconds)


def command(v=1.0, w=0.2):
    msg = Twist()
    msg.linear.x, msg.angular.z = v, w
    return msg


@pytest.fixture
def controller(monkeypatch):
    # Do not initialize rclpy or create a control node/real ROS publisher.
    node = thruster.CmdVelToThruster.__new__(thruster.CmdVelToThruster)
    clock = SimpleNamespace(steady=100.0, ros=0.0)
    monkeypatch.setattr(thruster, 'monotonic', lambda: clock.steady)
    node.get_clock = lambda: SimpleNamespace(now=lambda: FakeTime(clock.ros))
    params = dict(max_thrust=1000.0, v_kp=0.0, v_ki=200.0, v_kd=50.0,
                  w_kp=0.0, w_ki=100.0, w_kd=20.0, cmd_timeout=0.5,
                  odom_timeout=0.5, thrust_rate_limit=2.0)
    node.get_parameter = lambda name: SimpleNamespace(value=params[name])
    node._require_navigation_ready = True
    node._navigation_ready = False
    node._ready_stamp = None
    node._cmd_stamp = node._odom_stamp = None
    node._cmd_active = False
    node.desired_v = node.desired_w = node.actual_v = node.actual_w = 0.0
    node._last_left = node._last_right = 0.0
    node._last_control = FakeTime(clock.ros)
    node.left_pub = Mock()
    node.right_pub = Mock()
    node.left_pos_pub = Mock()
    node.right_pos_pub = Mock()
    node.ready_callback(Bool(data=True))
    return node, clock


def accumulate_pid_memory(node, clock):
    for _ in range(10):
        clock.steady += 0.05
        clock.ros += 0.05
        node.ready_callback(Bool(data=True))
        node.odom_callback(Odometry())
        node.cmd_callback(command())
        node.control_loop()
    assert node._v_pid.integral > 0.0 and node._w_pid.integral > 0.0
    assert not node._v_pid._first and not node._w_pid._first
    assert node._last_left != 0.0 and node._last_right != 0.0


def assert_hard_zero(node):
    for publisher in (node.left_pub, node.right_pub, node.left_pos_pub, node.right_pos_pub):
        assert publisher.publish.call_args.args[0].data == 0.0
    assert node._last_left == node._last_right == 0.0
    assert node.desired_v == node.desired_w == 0.0
    assert not node._cmd_active and node._cmd_stamp is None
    for pid in (node._v_pid, node._w_pid):
        assert pid.integral == pid.prev_error == 0.0
        assert pid._first


def test_pid_reproduces_nonzero_output_even_with_zero_tracking_error():
    pid = thruster.PID(0.0, 200.0, 0.0, -1000.0, 1000.0)
    for _ in range(10):
        pid.update(1.0, 0.05)
    assert pid.update(0.0, 0.05) == pytest.approx(100.0)
    pid.reset()
    assert pid.update(0.0, 0.05) == 0.0


@pytest.mark.parametrize('failure', ['zero_command', 'false_ready', 'stale_ready',
                                    'stale_odom', 'stale_command', 'invalid_odom'])
def test_faults_hard_zero_real_pid_and_slew_memory(controller, failure):
    node, clock = controller
    accumulate_pid_memory(node, clock)
    if failure == 'zero_command':
        node.cmd_callback(Twist())
    elif failure == 'false_ready':
        node.ready_callback(Bool(data=False))
    elif failure == 'stale_ready':
        clock.steady += 1.500001
        node.odom_callback(Odometry())
        node.cmd_callback(command())
    elif failure == 'stale_odom':
        clock.steady += 0.500001
        node.ready_callback(Bool(data=True))
        node.actual_v = 0.25
        node.cmd_callback(command())
    elif failure == 'stale_command':
        clock.steady += 0.500001
        node.ready_callback(Bool(data=True))
        node.odom_callback(Odometry())
    else:
        msg = Odometry()
        msg.twist.twist.angular.z = float('nan')
        node.odom_callback(msg)
    node.control_loop()
    assert_hard_zero(node)
    if failure == 'stale_odom':
        assert node.actual_v == 0.25  # Never substitute actual = desired.
    node.ready_callback(Bool(data=True))
    node.odom_callback(Odometry())
    node.control_loop()
    assert_hard_zero(node)  # Readiness recovery alone cannot replay the old command.
    node.cmd_callback(command())
    node.control_loop()
    assert node._cmd_active and node._v_pid.integral > 0.0


@pytest.mark.parametrize('age,allowed', [(1.5, True), (1.500001, False), (-0.1, False)])
def test_ready_age_uses_steady_time_with_frozen_ros_clock(controller, age, allowed):
    node, clock = controller
    clock.steady += age
    node.odom_callback(Odometry())
    node.cmd_callback(command())
    node.control_loop()
    assert node._cmd_active == allowed
    assert node._ready_stamp == 100.0
    assert clock.ros == 0.0


@pytest.mark.parametrize('stream', ['cmd', 'odom'])
@pytest.mark.parametrize('value', [float('nan'), float('inf'), -float('inf')])
def test_nonfinite_data_fails_closed(controller, stream, value):
    node, clock = controller
    accumulate_pid_memory(node, clock)
    if stream == 'cmd':
        msg = command()
        msg.linear.y = value
        node.cmd_callback(msg)
    else:
        msg = Odometry()
        msg.twist.twist.angular.x = value
        node.odom_callback(msg)
    node.control_loop()
    assert_hard_zero(node)


def test_true_after_heartbeat_gap_clears_old_command_without_a_control_tick(controller):
    node, clock = controller
    accumulate_pid_memory(node, clock)
    clock.steady += 1.6
    node.ready_callback(Bool(data=True))
    assert_hard_zero(node)


def test_legacy_mode_still_hard_stops_on_expired_odometry(controller):
    node, clock = controller
    node._require_navigation_ready = False
    accumulate_pid_memory(node, clock)
    clock.steady += 0.6
    node.cmd_callback(command())
    node.control_loop()
    assert_hard_zero(node)


@pytest.mark.parametrize('enabled', [False, True])
def test_constructor_has_fixed_ready_topic_readonly_guard_and_no_fallback_bypass(monkeypatch, enabled):
    params = {}

    def declare(name, default, **kwargs):
        value = enabled if name == 'require_navigation_ready' else default
        if name == 'namespace':
            value = 'boat'
        params[name] = SimpleNamespace(
            value=value, get_parameter_value=lambda: SimpleNamespace(string_value=value),
        )
        return params[name]

    monkeypatch.setattr(Node, '__init__', lambda *args, **kwargs: None)
    monkeypatch.setattr(Node, 'declare_parameter', Mock(side_effect=declare))
    monkeypatch.setattr(Node, 'get_parameter', lambda self, name: params[name])
    subscriptions = Mock(side_effect=lambda msg, topic, callback, qos:
                         SimpleNamespace(topic=topic, qos=qos, msg=msg))
    timer = Mock()
    monkeypatch.setattr(Node, 'create_subscription', subscriptions)
    monkeypatch.setattr(Node, 'create_publisher', Mock(return_value=Mock()))
    monkeypatch.setattr(Node, 'create_timer', timer)
    monkeypatch.setattr(Node, 'get_clock', lambda self: SimpleNamespace(now=lambda: FakeTime(0)))
    monkeypatch.setattr(Node, 'get_logger', Mock(return_value=Mock()))
    node = thruster.CmdVelToThruster()
    declaration = next(call for call in Node.declare_parameter.call_args_list
                       if call.args[0] == 'require_navigation_ready')
    assert declaration.args[1] is False and declaration.kwargs['descriptor'].read_only
    topics = [call.args[1] for call in subscriptions.call_args_list if call.args[0] == Twist]
    assert topics == (['/boat/control/cmd_vel'] if enabled else ['/boat/cmd_vel', '/cmd_vel'])
    if enabled:
        assert node.cmd_sub_fallback is None
        assert node.ready_sub.topic == '/boat/navigation/ready'
        assert node.ready_sub.qos.reliability == ReliabilityPolicy.RELIABLE
        assert node.ready_sub.qos.durability == DurabilityPolicy.VOLATILE
        assert timer.call_args.kwargs['clock'].clock_type == ClockType.STEADY_TIME
    else:
        assert node.ready_sub is None
