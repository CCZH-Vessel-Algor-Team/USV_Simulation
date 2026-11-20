"""Test trail lifetime/reset behavior without starting ROS processes."""
from pathlib import Path as FilePath
from collections import deque
from types import SimpleNamespace
from unittest.mock import Mock
import sys

from geometry_msgs.msg import PoseStamped, TransformStamped
from action_msgs.msg import GoalStatus, GoalStatusArray
from nav_msgs.msg import Odometry, Path
from rclpy.parameter import Parameter
from rclpy.time import Time
from usv_interfaces.msg import WaypointList

sys.path.insert(0, str(FilePath(__file__).parents[1]))
from usv_sim_full.scripts.history_path import HistoryPath, Trail


def pose(x):
    message = PoseStamped()
    message.pose.position.x = float(x)
    message.pose.orientation.w = 1.0
    return message


def test_window_and_capacity_keep_close_successive_points():
    trail = Trail(3)
    trail.append(1_000_000_000, pose(0))
    trail.append(1_100_000_000, pose(0.1))
    assert len(trail.points) == 2
    for second in (2, 3, 4):
        trail.append(second * 1_000_000_000, pose(second))
    assert [t for t, _ in trail.points] == [2e9, 3e9, 4e9]
    trail.prune(5_000_000_000, 2.0)
    assert [t for t, _ in trail.points] == [3e9, 4e9]
    trail.prune(7_000_000_000, 2.0)
    assert not trail.points


def node_fixture():
    node = HistoryPath.__new__(HistoryPath)
    clock = SimpleNamespace(seconds=10)
    settings = {'history_duration_sec': 5.0}
    node.get_clock = lambda: SimpleNamespace(now=lambda: Time(seconds=clock.seconds))
    node.get_parameter = lambda name: SimpleNamespace(value=settings[name])
    node._last_now = None
    node._frame = 'map'
    node._trail = Trail(100)
    node._pending = deque(maxlen=100)
    node._action_goal_ids = set()
    node._publisher = Mock()
    node._buffer = Mock()
    return node, clock, settings


def odom(second, x=0, frame='map'):
    message = Odometry()
    message.header.stamp = Time(seconds=second).to_msg()
    message.header.frame_id = frame
    message.pose.pose = pose(x).pose
    return message


def test_tf_is_applied_at_source_time():
    node, _, _ = node_fixture()
    transform = TransformStamped()
    transform.transform.rotation.w = 1.0
    transform.transform.translation.x = 20.0
    node._buffer.lookup_transform.return_value = transform
    node._on_odom(odom(10, 2, 'usv_1/odom'))
    node._publish()
    output = node._publisher.publish.call_args.args[0]
    assert output.poses[0].pose.position.x == 22.0
    assert output.poses[0].header.frame_id == 'map'
    assert output.poses[0].header.stamp.sec == 10
    assert node._buffer.lookup_transform.call_args.args[2].nanoseconds == 10e9


def test_every_odom_publishes_immediately_even_with_small_motion():
    node, clock, _ = node_fixture()
    for index in range(3):
        clock.seconds = 10 + index * 0.02
        node._on_odom(odom(clock.seconds, index * 0.01))
        assert node._publisher.publish.call_count == index + 1
        output = node._publisher.publish.call_args.args[0]
        assert len(output.poses) == index + 1
        assert output.poses[-1].pose.position.x == index * 0.01


def test_route_reset_publishes_empty_and_rejects_queued_old_odom():
    node, clock, _ = node_fixture()
    node._on_odom(odom(10))
    route = Path()
    route.poses = [pose(5), pose(10)]
    clock.seconds = 11
    node._on_route(route)
    assert node._publisher.publish.call_args.args[0].poses == []
    node._on_odom(odom(10, 1))
    assert not node._trail.points
    clock.seconds = 12
    node._on_odom(odom(12, 2))
    assert len(node._trail.points) == 1
    node._reset(PoseStamped())  # RViz new goal uses the same reset behavior.
    assert node._publisher.publish.call_args.args[0].poses == []


def test_tf_arriving_after_odom_keeps_both_frames():
    node, clock, _ = node_fixture()
    node._buffer.can_transform.return_value = False
    node._on_odom(odom(10, 1, 'usv_1/odom'))
    node._publisher.publish.assert_not_called()
    assert len(node._pending) == 1
    transform = TransformStamped()
    transform.transform.rotation.w = 1.0
    node._buffer.can_transform.return_value = True
    node._buffer.lookup_transform.return_value = transform
    clock.seconds = 10.02
    node._on_odom(odom(10.02, 2, 'usv_1/odom'))
    assert [p.pose.position.x for p in node._publisher.publish.call_args.args[0].poses] == [1, 2]
    assert not node._pending


def test_odom_slightly_ahead_of_clock_is_not_dropped():
    node, _, _ = node_fixture()
    node._on_odom(odom(10.02, 1))
    assert len(node._publisher.publish.call_args.args[0].poses) == 1


def test_reset_discards_positions_still_waiting_for_tf():
    node, clock, _ = node_fixture()
    node._buffer.can_transform.return_value = False
    node._on_odom(odom(10, 1, 'usv_1/odom'))
    clock.seconds = 11
    node._reset(WaypointList())
    assert not node._pending
    assert node._publisher.publish.call_args.args[0].poses == []


def test_action_new_goal_resets_once_and_repeated_status_keeps_new_trail():
    node, clock, _ = node_fixture()
    node._on_odom(odom(10, 1))
    clock.seconds = 11
    status = GoalStatus()
    status.goal_info.goal_id.uuid[0] = 1
    status.status = GoalStatus.STATUS_EXECUTING
    message = GoalStatusArray(status_list=[status])
    node._on_action_status(message)
    assert node._publisher.publish.call_args.args[0].poses == []
    clock.seconds = 12
    node._on_odom(odom(12, 2))
    node._on_action_status(message)
    assert len(node._trail.points) == 1
    status.goal_info.goal_id.uuid[0] = 2
    node._on_action_status(GoalStatusArray(status_list=[status]))
    assert node._publisher.publish.call_args.args[0].poses == []


def test_new_planning_request_clears_trail_before_route_selection():
    node, clock, _ = node_fixture()
    node._on_odom(odom(10, 100))
    clock.seconds = 11
    node._reset(WaypointList())  # /mission/waypoints callback, before a Path exists.
    assert node._publisher.publish.call_args.args[0].poses == []
    node._on_odom(odom(10.5, 101))  # Queued positions from the previous plan stay out.
    assert not node._trail.points
    clock.seconds = 11.02
    node._on_odom(odom(11.02, 102))
    output = node._publisher.publish.call_args.args[0]
    assert [p.pose.position.x for p in output.poses] == [102.0]


def test_timer_expires_points_without_odom_and_duration_can_change():
    node, clock, settings = node_fixture()
    node._on_odom(odom(10))
    clock.seconds = 14
    node._expire()
    assert len(node._publisher.publish.call_args.args[0].poses) == 1
    settings['history_duration_sec'] = 2.0
    node._expire()
    assert node._publisher.publish.call_args.args[0].poses == []
    assert not node._on_parameters([Parameter('history_duration_sec', value=0.0)]).successful
    assert node._on_parameters([Parameter('history_duration_sec', value=60.0)]).successful


def test_clock_rollback_starts_new_trail():
    node, clock, _ = node_fixture()
    node._on_odom(odom(10))
    clock.seconds = 1
    node._publish()
    assert node._publisher.publish.call_args.args[0].poses == []
    node._on_odom(odom(1))
    assert len(node._trail.points) == 1


def test_launch_reads_yaml_duration_unless_explicitly_overridden(monkeypatch, tmp_path):
    import importlib.util
    import yaml
    from launch import LaunchContext
    from launch_ros.utilities import evaluate_parameters, normalize_parameters

    launch_file = FilePath(__file__).parents[1] / 'launch/main.launch.py'
    spec = importlib.util.spec_from_file_location('history_simulation_launch', launch_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    config = tmp_path / 'config/history_path.yaml'
    config.parent.mkdir(parents=True)
    config.write_text('history_path:\n  ros__parameters:\n    history_duration_sec: 75.0\n')
    monkeypatch.setattr(module, 'get_package_share_directory', lambda _: str(tmp_path))
    monkeypatch.setattr(module, 'Node', lambda **kwargs: SimpleNamespace(**kwargs))
    context = LaunchContext()
    context.launch_configurations.update({
        'enable_history_path': 'true', 'history_duration_sec': '',
        'use_sim_time': 'true', 'odom_topic': '/usv_1/odom',
    })
    for override, expected in [('', 75.0), ('120.0', 120.0)]:
        context.launch_configurations['history_duration_sec'] = override
        action, = module._history_path_action(context)
        filename, parameters = evaluate_parameters(context, normalize_parameters(action.parameters))
        effective = yaml.safe_load(filename.read_text())['history_path']['ros__parameters']
        effective.update(parameters)
        assert effective['history_duration_sec'] == expected
        assert effective['use_sim_time'] is True
    context.launch_configurations['enable_history_path'] = 'false'
    assert module._history_path_action(context) == []
