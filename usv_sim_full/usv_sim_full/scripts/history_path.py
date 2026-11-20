"""Publish a bounded, time-windowed ownship trail; new route dispatches reset it."""

from collections import deque
from copy import deepcopy
import math

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformException, TransformListener
from usv_interfaces.msg import WaypointList


class Trail:
    """Keep source timestamps, never extending a point's lifetime on publication."""

    def __init__(self, max_points):
        self.points = deque(maxlen=max_points)
        self.reset_time = -1

    def clear(self, stamp):
        self.points.clear()
        self.reset_time = stamp

    def prune(self, now, duration):
        cutoff = now - int(duration * 1e9)
        while self.points and self.points[0][0] < cutoff:
            self.points.popleft()

    def append(self, stamp, pose):
        if stamp <= self.reset_time:
            return
        if self.points:
            previous_stamp, _ = self.points[-1]
            if stamp <= previous_stamp:
                return
        self.points.append((stamp, pose))


class HistoryPath(Node):
    def __init__(self, **kwargs):
        super().__init__('history_path', **kwargs)
        defaults = {
            'odom_topic': '/usv_1/odom',
            'path_topic': '/usv_1/history_path',
            'route_topic': '/route_planner/selected_path',
            'mission_topic': '/mission/waypoints',
            'goal_topic': '/usv_1/goal_pose',
            'navigation_actions': [
                '/usv_1/navigate_to_pose', '/usv_1/navigate_through_poses',
                '/usv_1/follow_waypoints',
            ],
            'fixed_frame': 'map',
            'max_points': 10000,
            'cleanup_rate': 5.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
        self.declare_parameter('history_duration_sec', 300.0)
        get = lambda name: self.get_parameter(name).value
        if not 0 < get('history_duration_sec') < math.inf:
            raise ValueError('history_duration_sec must be finite and > 0')
        self._frame = get('fixed_frame')
        self._trail = Trail(get('max_points'))
        self._pending = deque(maxlen=100)  # TF often arrives one frame after odometry.
        self._last_now = None
        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._publisher = self.create_publisher(Path, get('path_topic'), latched)
        self._odom_sub = self.create_subscription(
            Odometry, get('odom_topic'), self._on_odom,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self._route_sub = self.create_subscription(
            Path, get('route_topic'), self._on_route, latched)
        self._mission_sub = self.create_subscription(
            WaypointList, get('mission_topic'), self._reset, 10)
        self._goal_sub = self.create_subscription(
            PoseStamped, get('goal_topic'), self._reset, 10)
        self._action_goal_ids = set()
        self._action_subs = [
            self.create_subscription(
                GoalStatusArray, action + '/_action/status', self._on_action_status, latched)
            for action in get('navigation_actions')
        ]
        self.add_on_set_parameters_callback(self._on_parameters)
        self._timer = self.create_timer(1.0 / get('cleanup_rate'), self._expire)

    def _on_parameters(self, parameters):
        for parameter in parameters:
            if (parameter.name == 'history_duration_sec'
                    and not 0 < parameter.value < math.inf):
                return SetParametersResult(
                    successful=False, reason='history_duration_sec must be finite and > 0')
        # Read the committed parameter on the next timer/odometry callback.
        return SetParametersResult(successful=True)

    def _now(self):
        now = self.get_clock().now()
        if self._last_now is not None and now.nanoseconds < self._last_now:
            self._trail.clear(-1)
            self._pending.clear()
            self._action_goal_ids.clear()
        self._last_now = now.nanoseconds
        return now

    def _on_route(self, message):
        if message.poses:
            self._reset(message)

    def _on_action_status(self, message):
        active = {
            bytes(status.goal_info.goal_id.uuid) for status in message.status_list
            if status.status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING)
        }
        if active - self._action_goal_ids:
            self._action_goal_ids.update(active)
            self._reset(message)

    def _reset(self, _message):
        self._trail.clear(self._now().nanoseconds)
        self._pending.clear()
        self._publish()  # An empty Path immediately removes the previous RViz line.

    def _on_odom(self, message):
        now = self._now()
        stamp = Time.from_msg(message.header.stamp)
        duration = self.get_parameter('history_duration_sec').value
        self._trail.prune(now.nanoseconds, duration)
        if (stamp.nanoseconds < now.nanoseconds - int(duration * 1e9)
                or stamp.nanoseconds <= self._trail.reset_time):
            return
        self._pending.append(message)
        self._flush_pending()

    def _flush_pending(self):
        updated = False
        while self._pending:
            message = self._pending[0]
            stamp = Time.from_msg(message.header.stamp)
            frame = message.header.frame_id
            if not frame:
                self._pending.popleft()
                continue
            if frame != self._frame and not self._buffer.can_transform(self._frame, frame, stamp):
                break
            pose = PoseStamped()
            pose.header = deepcopy(message.header)
            pose.pose = deepcopy(message.pose.pose)
            if frame != self._frame:
                try:
                    transform = self._buffer.lookup_transform(self._frame, frame, stamp)
                    pose.pose = do_transform_pose(pose.pose, transform)
                except TransformException:
                    break
            self._pending.popleft()
            pose.header.frame_id = self._frame
            self._trail.append(stamp.nanoseconds, pose)
            updated = True
        if updated:
            self._publish()

    def _expire(self):
        self._flush_pending()
        count = len(self._trail.points)
        now = self._now()
        self._trail.prune(now.nanoseconds, self.get_parameter('history_duration_sec').value)
        if len(self._trail.points) != count:
            self._publish()

    def _publish(self):
        now = self._now()
        self._trail.prune(now.nanoseconds, self.get_parameter('history_duration_sec').value)
        message = Path()
        message.header.frame_id = self._frame
        message.header.stamp = now.to_msg()
        message.poses = [pose for _, pose in self._trail.points]
        self._publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = HistoryPath()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
