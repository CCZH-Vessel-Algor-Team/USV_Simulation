#!/usr/bin/env python3

import json
import math
import os
import signal
import subprocess
import tempfile
import threading
import time
import uuid
import yaml

import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PointStamped, Pose, Twist
from nav2_colregs_msgs.msg import TrackedShip, TrackedShipList
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.clock import JumpThreshold
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from usv_interfaces.srv import (
    SpawnDynamicShip, DeleteDynamicShip, ClearDynamicShips,
    SetDynamicShipConfig,
)
from usv_sim_full.pose_feedback import (
    PoseTracker, fresh_sample, is_feedback_fresh_ns, yaw_from_quaternion,
)


def _fmt_pose_xyzrpy(xyz, rpy):
    return (
        f"{float(xyz[0]):.6f} {float(xyz[1]):.6f} {float(xyz[2]):.6f} "
        f"{float(rpy[0]):.6f} {float(rpy[1]):.6f} {float(rpy[2]):.6f}"
    )


def _as_vec(raw, length, default=0.0):
    if isinstance(raw, (list, tuple)):
        vals = [float(v) for v in raw[:length]]
    else:
        vals = []
    if len(vals) < length:
        vals.extend([float(default)] * (length - len(vals)))
    return vals


def _resolve_profile_path(path_text, config_base_dir):
    p = str(path_text or '').strip()
    if not p:
        return ''
    if os.path.isabs(p):
        return p
    return os.path.normpath(os.path.join(config_base_dir, p))


class GazeboPoseCache:
    """Gazebo 动态实体位姿缓存：/world/<world>/dynamic_pose/info。

    One subscription supplies both accumulated control feedback and complete
    observation frames. Missing bindings leave tracked publication unavailable.
    """

    def __init__(self, world_name, logger=None, now_ns=None, pose_feedback_timeout=1.0):
        self._lock = threading.Lock()
        self._samples = {}
        self._frame = None
        self._pending_frame = None
        self._watermark_ns = None
        self._max_observed_ns = None
        self._not_before_ns = 0
        self._awaiting_absence = set()
        self._now_ns = now_ns
        self._future_window_ns = int(pose_feedback_timeout * 1e9)
        self._node = None
        self._topic = f'/world/{world_name}/dynamic_pose/info'
        try:
            import gz.transport13 as gz_transport
            from gz.msgs10.pose_v_pb2 import Pose_V
        except ImportError:
            if logger is not None:
                logger.warning(
                    'gz.transport Python 绑定不可用，停止发布动态船跟踪快照')
            return
        self._node = gz_transport.Node()
        self._node.subscribe(Pose_V, self._topic, self._on_pose_v)
        if logger is not None:
            logger.info(f'GazeboPoseCache 订阅 {self._topic}')

    def _on_pose_v(self, msg):
        if self._now_ns is None:
            return
        with self._lock:
            now_ns = self._now_ns()
            # Promote BEFORE considering a newer future frame. Retain the earliest
            # pending frame so a slower /clock cannot starve this subscription.
            self._promote_pending(now_ns)
            sec, nsec = msg.header.stamp.sec, msg.header.stamp.nsec
            if sec < 0 or not 0 <= nsec < 1_000_000_000:
                return
            stamp_ns = sec * 1_000_000_000 + nsec
            # A delayed old-epoch packet must not poison the next membership
            # fence. Only bounded clock/pose skew can enter pending/observed state.
            if stamp_ns - now_ns > self._future_window_ns:
                return
            # Separate from the accepted watermark: even a future frame not kept
            # in the single pending slot must fence a later membership change.
            self._max_observed_ns = max(
                stamp_ns, self._max_observed_ns if self._max_observed_ns is not None else stamp_ns)
            roots = {}
            for entry in msg.pose:
                name = entry.name
                if not name or '::' in name or '/' in name:
                    continue
                q = entry.orientation
                values = (entry.position.x, entry.position.y, entry.position.z,
                          q.x, q.y, q.z, q.w)
                if (name in roots or not all(map(math.isfinite, values))
                        or not any((q.x, q.y, q.z, q.w))):
                    roots[name] = None  # Ambiguous/malformed roots cannot disappear silently.
                    continue
                roots[name] = (float(entry.position.x), float(entry.position.y),
                               yaw_from_quaternion(q.x, q.y, q.z, q.w), stamp_ns)
            frame = (stamp_ns, roots)
            if stamp_ns <= now_ns:
                self._accept(frame)
            elif self._pending_frame is None or stamp_ns < self._pending_frame[0]:
                self._pending_frame = frame

    def _accept(self, frame):
        """Accept an advancing frame under the cache lock.

        :param frame: Exact timestamp and root-pose mapping.
        """
        stamp_ns, roots = frame
        if (stamp_ns < self._not_before_ns or
                (self._watermark_ns is not None and stamp_ns <= self._watermark_ns)):
            return
        self._watermark_ns = stamp_ns
        self._frame = frame
        self._samples.update(roots)
        if self._awaiting_absence.isdisjoint(roots):
            self._awaiting_absence.clear()

    def _promote_pending(self, now_ns):
        """Promote a waiting observation under the cache lock.

        :param now_ns: Current simulation time in nanoseconds.
        """
        if self._pending_frame is not None and self._pending_frame[0] <= now_ns:
            frame, self._pending_frame = self._pending_frame, None
            self._accept(frame)

    def snapshot(self):
        """Return accumulated control samples and the latest whole frame.

        :return: A control-sample copy and an immutable-by-convention frame tuple.
        """
        with self._lock:
            if self._now_ns is not None:
                self._promote_pending(self._now_ns())
            return dict(self._samples), self._frame

    def invalidate_frame(self, removed_name=None):
        """Fence membership and any deletion witness atomically under the cache lock.

        :param removed_name: Confirmed removed root requiring a subsequent absence witness.
        """
        with self._lock:
            if removed_name is not None:
                self._samples.pop(removed_name, None)
                self._awaiting_absence.add(removed_name)
            self._not_before_ns = max(
                self._not_before_ns,
                self._now_ns() if self._now_ns is not None else 0,
                self._max_observed_ns + 1 if self._max_observed_ns is not None else 0)
            self._frame = None
            self._pending_frame = None

    def reset(self):
        """Clear both cache views and ordering state after a ROS clock rewind."""
        with self._lock:
            self._samples.clear()
            self._frame = None
            self._pending_frame = None
            self._watermark_ns = None
            self._max_observed_ns = None
            self._not_before_ns = 0
            # A clock rewind does not discharge a physical deletion witness.

    def model_names(self, world_name, timeout_ms=1000):
        """Query scene inventory in a process isolated from the pose callback.

        :param world_name: Gazebo world whose model inventory is requested.
        :param timeout_ms: Total query budget, including process overhead, in milliseconds.
        :return: Root model names, or None for unavailable, invalid or late replies.
        """
        if self._node is None or timeout_ms <= 0:
            return None
        timeout_sec = timeout_ms / 1000.0
        deadline = time.monotonic() + timeout_sec
        try:
            from google.protobuf import text_format
            from gz.msgs10.scene_pb2 import Scene
        except ImportError:
            return None
        if time.monotonic() >= deadline:
            return None

        cmd = [
            'gz', 'service', '-s', f'/world/{world_name}/scene/info',
            '--reqtype', 'gz.msgs.Empty', '--reptype', 'gz.msgs.Scene',
            '--timeout', str(timeout_ms), '--req', '',
        ]
        try:
            # Isolate the synchronous transport wait from high-rate pose callback
            # processing; the CLI owns its separate request connection.
            result = subprocess.run(
                cmd, capture_output=True, text=True, timeout=timeout_sec)
            if (time.monotonic() > deadline or result.returncode != 0
                    or not result.stdout.strip()):
                return None
            scene = text_format.Parse(result.stdout, Scene())
        except (OSError, subprocess.TimeoutExpired, text_format.ParseError):
            return None
        if not scene.ListFields():
            return None
        names = {model.name for model in scene.model}
        # Popen startup and protobuf parsing are not covered by run's wait timeout.
        return names if time.monotonic() <= deadline else None

    def forget(self, name):
        """Discard previous feedback before reserving an absent model name.

        :param name: Model name whose absence was confirmed by the spawn preflight.
        """
        with self._lock:
            self._samples.pop(name, None)
            # The existing authoritative scene absence preflight allows the new
            # incarnation to appear even if no intervening pose frame was read.
            self._awaiting_absence.discard(name)

    def close(self):
        """Stop the existing pose subscription at shutdown."""
        if self._node is not None:
            self._node.unsubscribe(self._topic)


class DynamicShip:
    def __init__(self, model_name, target_id, pose, half_distance, shape, speed, color,
                 mesh_profile, node, config_base_dir, world_name):
        self.model_name = model_name
        self.target_id = target_id
        self.shape = shape
        self.speed = speed
        self.half_distance = half_distance
        self.color = color
        self.mesh_profile = mesh_profile
        self.node = node
        self.config_base_dir = config_base_dir
        self.world_name = world_name

        x = pose.position.x
        y = pose.position.y

        def quat_to_yaw(q):
            return math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )

        spawn_yaw = quat_to_yaw(pose.orientation)
        self.spawn_yaw = spawn_yaw
        self.actual_yaw = spawn_yaw
        self.pose_tracker = PoseTracker()
        self.feedback_fresh = False
        self.feedback_stamp_sec = None

        total_dist = 2.0 * half_distance
        self.waypoint_a = (x, y)
        self.waypoint_b = (
            x + total_dist * math.cos(spawn_yaw),
            y + total_dist * math.sin(spawn_yaw)
        )

        self.current_x = x
        self.current_y = y
        self.direction = 1
        self._turning_remaining = 0.0
        self.bridge_process = None
        self._bridge_lock = threading.Lock()
        self._publisher_lock = threading.Lock()
        self._closed = False

        self.cmd_vel_pub = node.create_publisher(
            Twist, f'/model/{model_name}/cmd_vel', 10)

    def start_bridge(self):
        """Start the actual bridge executable and own its child PID."""
        with self._bridge_lock, self._publisher_lock:
            if self._closed or self.bridge_process is not None:
                raise RuntimeError('Bridge already started or ship already retired')
            executable = os.path.join(
                get_package_prefix('ros_gz_bridge'), 'lib', 'ros_gz_bridge',
                'parameter_bridge')
            cmd = [
                executable,
                f'/model/{self.model_name}/cmd_vel'
                f'@geometry_msgs/msg/Twist]gz.msgs.Twist',
            ]
            self.bridge_process = subprocess.Popen(
                cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                start_new_session=True)

    def retire(self):
        """Reject future commands, synchronized with any in-flight publication."""
        with self._publisher_lock:
            self._closed = True

    def cleanup(self):
        """Retire resources, retaining failed handles for a later bounded retry."""
        with self._bridge_lock:
            try:
                with self._publisher_lock:
                    self._closed = True
                    if self.cmd_vel_pub is not None:
                        if not self.node.destroy_publisher(self.cmd_vel_pub):
                            raise RuntimeError('Publisher destruction was not confirmed')
                        self.cmd_vel_pub = None
            finally:
                process = self.bridge_process
                if process is not None:
                    # Reap even if a signal failed (including exit-before-signal).
                    # Never hold the publisher/registry locks during child waits.
                    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
                        try:
                            process.send_signal(sig)
                        except OSError:
                            pass
                        try:
                            process.wait(timeout=2)
                        except (subprocess.TimeoutExpired, OSError):
                            if sig == signal.SIGKILL:
                                raise
                        else:
                            self.bridge_process = None
                            break

    def publish_cmd_vel(self, twist):
        """Publish only while the ship owns a live publisher.

        :param twist: Body-frame command to publish.
        :return: True if published, False if the ship has retired.
        """
        with self._publisher_lock:
            if self._closed:
                return False
            self.cmd_vel_pub.publish(twist)
            return True

    def sync_from_feedback(self, sample, now_sec, timeout_sec):
        """用 Gazebo 实测位姿覆盖内部积分状态，返回反馈是否有效。"""
        sample = fresh_sample(sample, now_sec, timeout_sec)
        if sample is None:
            self.feedback_fresh = False
            return False
        self.current_x = sample[0]
        self.current_y = sample[1]
        self.actual_yaw = sample[2]
        self.feedback_stamp_sec = sample[3]
        self.pose_tracker.update(sample[0], sample[1], sample[3])
        self.feedback_fresh = True
        return True

    def reset_feedback(self):
        """Reset observation history while preserving physical motion and ownership."""
        self.pose_tracker.reset()
        self.feedback_fresh = False
        self.feedback_stamp_sec = None

    def sync_from_feedback_ns(self, sample, now_ns, timeout_sec):
        """Use exact-time measured feedback for control and tracked publication.

        :param sample: World X/Y/yaw and integer observation nanoseconds, or None.
        :param now_ns: Current simulation time in nanoseconds.
        :param timeout_sec: Maximum age and velocity-history gap in seconds.
        :return: Whether measured pose feedback is usable (velocity may be bootstrapping).
        """
        if (sample is None or not all(map(math.isfinite, sample[:3])) or
                not is_feedback_fresh_ns(sample[3], now_ns, timeout_sec)):
            self.reset_feedback()
            return False
        previous = self.pose_tracker.stamp_ns
        if previous is not None and sample[3] <= previous:
            return self.feedback_fresh
        self.pose_tracker.update_ns(sample[0], sample[1], sample[3], timeout_sec)
        self.current_x, self.current_y, self.actual_yaw = sample[:3]
        self.feedback_stamp_sec = sample[3] * 1e-9
        self.feedback_fresh = True
        return True

    def world_twist_to_body(self, vx_world, vy_world, yaw=None):
        if yaw is None:
            yaw = self.actual_yaw
        c = math.cos(yaw)
        s = math.sin(yaw)
        twist = Twist()
        twist.linear.x = c * vx_world + s * vy_world
        twist.linear.y = -s * vx_world + c * vy_world
        twist.angular.x = 1.0
        twist.angular.y = self.spawn_yaw
        return twist

    def compute_cmd_vel(self, dt, predict=True):
        if self._turning_remaining > 0.0:
            self._turning_remaining -= dt
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.x = 1.0
            twist.angular.y = self.spawn_yaw
            return twist

        target = self.waypoint_b if self.direction > 0 else self.waypoint_a
        dx = target[0] - self.current_x
        dy = target[1] - self.current_y
        dist = math.hypot(dx, dy)

        if dist < 0.5:
            self.direction *= -1
            target = self.waypoint_b if self.direction > 0 else self.waypoint_a
            dx = target[0] - self.current_x
            dy = target[1] - self.current_y
            dist = math.hypot(dx, dy)
            if dist > 0.0:
                new_heading = math.atan2(dy, dx)
                heading_error = new_heading - self.actual_yaw
                heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))
                self.spawn_yaw = new_heading
                turn_rate = 0.8
                self._turning_remaining = max(
                    1.0, abs(heading_error) / turn_rate)

        if dist > 0:
            vx = (dx / dist) * self.speed
            vy = (dy / dist) * self.speed
        else:
            vx = 0.0
            vy = 0.0

        if predict:
            self.current_x += vx * dt
            self.current_y += vy * dt
        return self.world_twist_to_body(vx, vy)

    def get_current_pose(self):
        pose = Pose()
        pose.position.x = self.current_x
        pose.position.y = self.current_y
        pose.position.z = 0.0
        yaw = self.actual_yaw if self.feedback_fresh else self.spawn_yaw
        pose.orientation.w = math.cos(yaw / 2.0)
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = math.sin(yaw / 2.0)
        return pose

    def get_current_twist(self):
        if self.feedback_fresh:
            vx, vy = self.pose_tracker.velocity
            twist = Twist()
            twist.linear.x = vx
            twist.linear.y = vy
            return twist
        target = self.waypoint_b if self.direction > 0 else self.waypoint_a
        dx = target[0] - self.current_x
        dy = target[1] - self.current_y
        dist = math.hypot(dx, dy)
        if dist > 0:
            vx = (dx / dist) * self.speed
            vy = (dy / dist) * self.speed
            twist = Twist()
            twist.linear.x = vx
            twist.linear.y = vy
            return twist
        return Twist()


class DynamicShipManager(Node):
    def __init__(self):
        super().__init__('dynamic_ship_manager')
        self._retained_spawn_sdf_paths = []

        self.declare_parameter('world_name', 'sydney_regatta')
        self.declare_parameter('default_mesh_profile', '')
        self.declare_parameter('config_base_dir', '')

        self.world_name = self.get_parameter('world_name').get_parameter_value().string_value
        self.default_mesh_profile = self.get_parameter('default_mesh_profile').get_parameter_value().string_value
        self.config_base_dir = self.get_parameter('config_base_dir').get_parameter_value().string_value

        if not self.config_base_dir:
            self.config_base_dir = os.path.dirname(os.path.abspath(__file__))

        self.declare_parameter('heading_deg', 0.0)
        self.declare_parameter('speed', 3.0)
        self.declare_parameter('shape', 'mesh_profile')
        self.declare_parameter('half_distance', 50.0)
        self.declare_parameter('pose_feedback_timeout', 1.0)

        self.pose_feedback_timeout = (
            self.get_parameter('pose_feedback_timeout').get_parameter_value().double_value)
        if not math.isfinite(self.pose_feedback_timeout) or self.pose_feedback_timeout <= 0:
            raise ValueError('pose_feedback_timeout must be finite and positive')
        self._clock_reset = threading.Event()
        self._clock_jump_handle = self.get_clock().create_jump_callback(
            JumpThreshold(min_forward=None, min_backward=Duration(nanoseconds=-1), on_clock_change=True),
            post_callback=lambda jump: self._clock_reset.set())
        self.gz_pose_cache = GazeboPoseCache(
            self.world_name, self.get_logger(), lambda: self.get_clock().now().nanoseconds,
            pose_feedback_timeout=self.pose_feedback_timeout)

        self.ships = {}
        self._ships_lock = threading.Lock()
        self._retired_ships = set()
        # Failed/dispatched creates: name -> whether creation was ever observed.
        self._pending_spawns = {}
        self._missing_model_counts = {}
        self._registry_revision = 0
        self._last_published_stamp_ns = None
        self.dt = 0.1

        # TimeSource places /clock in the default group. Keep blocking lifecycle
        # work serialized elsewhere so it cannot freeze this node's ROS clock.
        self.lifecycle_group = MutuallyExclusiveCallbackGroup()
        self.spawn_srv = self.create_service(
            SpawnDynamicShip, '/dynamic_ship/spawn', self.on_spawn,
            callback_group=self.lifecycle_group)
        self.delete_srv = self.create_service(
            DeleteDynamicShip, '/dynamic_ship/delete', self.on_delete,
            callback_group=self.lifecycle_group)
        self.clear_srv = self.create_service(
            ClearDynamicShips, '/dynamic_ship/clear', self.on_clear,
            callback_group=self.lifecycle_group)

        self.tracked_pub = self.create_publisher(
            TrackedShipList, '/dynamic_ship/tracked_ships', 10)

        # With two executor threads, control and /clock can share the available
        # worker while the other waits in the lifecycle group.
        self.control_group = MutuallyExclusiveCallbackGroup()
        self.timer = self.create_timer(
            self.dt, self.control_loop, callback_group=self.control_group)
        self.scene_timer = self.create_timer(
            1.0, self.reconcile_gazebo_models, callback_group=self.lifecycle_group)

        self._ship_counter = 0

        self.click_sub = self.create_subscription(
            PointStamped, '/clicked_point', self.on_clicked_point, 10,
            callback_group=self.lifecycle_group)

        self.config_srv = self.create_service(
            SetDynamicShipConfig, '/dynamic_ship/set_config',
            self.on_set_config, callback_group=self.lifecycle_group)

        self.names_pub = self.create_publisher(
            String, '/dynamic_ship/names', 10)

    def _read_config(self):
        return (
            self.get_parameter('heading_deg').get_parameter_value().double_value,
            self.get_parameter('speed').get_parameter_value().double_value,
            self.get_parameter('shape').get_parameter_value().string_value,
            self.get_parameter('half_distance').get_parameter_value().double_value,
        )

    def on_set_config(self, request, response):
        try:
            self.set_parameters([
                rclpy.parameter.Parameter(
                    'heading_deg', value=request.heading_deg),
                rclpy.parameter.Parameter(
                    'speed', value=request.speed),
                rclpy.parameter.Parameter(
                    'shape', value=request.shape),
                rclpy.parameter.Parameter(
                    'half_distance', value=request.half_distance),
            ])
            response.success = True
            response.message = 'config updated'
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def on_clicked_point(self, msg):
        heading_deg, speed, shape, half_dist = self._read_config()
        heading = math.pi / 2.0 - math.radians(heading_deg)

        self._ship_counter += 1
        name = f'dyn_target_{self._ship_counter}'

        self._spawn_ship_at(name, msg.point.x, msg.point.y, heading,
                            half_dist, shape, speed)

    def _spawn_ship_at(self, name, x, y, yaw, half_dist, shape, speed):
        with self._ships_lock:
            if name in self.ships or any(
                    ship.model_name == name for ship in self._retired_ships):
                self.get_logger().error(f"Ship '{name}' already exists or has cleanup pending")
                return False
        target_id = str(uuid.uuid5(uuid.NAMESPACE_DNS, name))
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = 0.0
        pose.orientation.w = math.cos(yaw / 2.0)
        pose.orientation.z = math.sin(yaw / 2.0)

        color = (1.0, 0.2, 0.2)

        ship = DynamicShip(
            model_name=name, target_id=target_id, pose=pose,
            half_distance=half_dist, shape=shape, speed=speed,
            color=color,
            mesh_profile=self.default_mesh_profile,
            node=self, config_base_dir=self.config_base_dir,
            world_name=self.world_name,
        )

        try:
            sdf_str = self._generate_sdf(ship)
            # A synchronous bridge failure must not create a Gazebo entity.
            ship.start_bridge()
            self._spawn_gazebo(ship, sdf_str)
        except Exception as e:
            self._cleanup_ship(ship)
            self.get_logger().error(f'spawn failed: {e}')
            return False

        with self._ships_lock:
            self._pending_spawns.pop(name, None)
            self._membership_changed()
        self.get_logger().info(
            f'clicked at ({x:.2f},{y:.2f}) heading={math.degrees(yaw):.0f}deg '
            f'-> spawned {name} speed={speed}m/s')
        return True

    def on_spawn(self, request, response):
        try:
            if not request.name.strip():
                self._ship_counter += 1
                name = f'dyn_target_{self._ship_counter}'
            else:
                name = request.name.strip()

            yaw = math.atan2(
                2.0 * (request.pose.orientation.w * request.pose.orientation.z
                       + request.pose.orientation.x * request.pose.orientation.y),
                1.0 - 2.0 * (request.pose.orientation.y * request.pose.orientation.y
                             + request.pose.orientation.z * request.pose.orientation.z))

            half_dist = request.half_distance if request.half_distance > 0 else 50.0
            speed = request.speed if request.speed > 0.0 else 3.0
            shape = request.shape.strip() or 'mesh_profile'

            if not self._spawn_ship_at(name, request.pose.position.x,
                                       request.pose.position.y, yaw,
                                       half_dist, shape, speed):
                raise RuntimeError('Spawn failed or name still owned; see manager log')
            response.success = True
            response.model_name = name
            response.message = f"Ship '{name}' spawned successfully"
        except Exception as e:
            response.success = False
            response.message = f"Spawn failed: {e}"
            self.get_logger().error(f"Spawn failed: {e}")
        return response

    def on_delete(self, request, response):
        name = request.model_name.strip()
        with self._ships_lock:
            registered = name in self.ships
        if not registered:
            response.success = False
            response.message = f"Ship '{name}' not found"
            return response
        try:
            self._remove_gazebo(name)
            self._forget_ship(name)
            response.success = True
            response.message = f"Ship '{name}' deleted"
            self.get_logger().info(f"Deleted dynamic ship '{name}'")
        except Exception as e:
            response.success = False
            response.message = f"Delete failed: {e}"
        return response

    def on_clear(self, request, response):
        with self._ships_lock:
            names = list(self.ships)
        removed = 0
        for name in names:
            try:
                self._remove_gazebo(name)
                self._forget_ship(name)
                removed += 1
            except Exception as e:
                self.get_logger().warn(f"Failed to remove '{name}' during clear: {e}")
        self.get_logger().info(f"Cleared {removed} of {len(names)} dynamic ships")
        response.success = removed == len(names)
        response.message = f"Cleared {removed} of {len(names)} ships"
        return response

    def _generate_sdf(self, ship):
        if ship.shape == 'mesh_profile':
            return self._generate_mesh_profile_sdf(ship)
        elif ship.shape == 'box':
            return self._generate_box_sdf(ship)
        else:
            return self._generate_cylinder_sdf(ship)

    @staticmethod
    def _environment_plugins(ship):
        return f"""
                <plugin filename="libSurface.so" name="vrx::Surface">
                    <link_name>base_link</link_name>
                    <hull_length>10.0</hull_length>
                    <hull_radius>0.55</hull_radius>
                    <fluid_level>0.0</fluid_level>
                    <points>
                        <point>2.5 1.2 0</point>
                        <point>-2.5 1.2 0</point>
                        <point>2.5 -1.2 0</point>
                        <point>-2.5 -1.2 0</point>
                    </points>
                    <wavefield>
                        <topic>/vrx/wavefield/parameters</topic>
                        <wave>
                            <model>PMS</model>
                            <period>5.0</period>
                            <direction>0.0</direction>
                            <gain>0.3</gain>
                            <steepness>0.0</steepness>
                        </wave>
                    </wavefield>
                </plugin>
                <plugin filename="gz-sim-hydrodynamics-system"
                        name="gz::sim::systems::Hydrodynamics">
                    <link_name>base_link</link_name>
                    <xU>-100</xU>
                    <xUabsU>-150</xUabsU>
                    <yV>-1200</yV>
                    <yVabsV>-1800</yVabsV>
                    <zW>-1800</zW>
                    <kP>-2500</kP>
                    <mQ>-3500</mQ>
                    <nR>-3000</nR>
                    <nRabsR>-4500</nRabsR>
                    <disable_added_mass>true</disable_added_mass>
                    <disable_coriolis>true</disable_coriolis>
                    <default_current>0 0 0</default_current>
                </plugin>
                <plugin filename="libUSVWind.so" name="vrx::USVWind">
                    <wind_obj>
                        <name>{ship.model_name}</name>
                        <link_name>base_link</link_name>
                        <coeff_vector>4.0 8.0 6.0</coeff_vector>
                    </wind_obj>
                    <wind_direction>0</wind_direction>
                    <wind_mean_velocity>0.0</wind_mean_velocity>
                    <var_wind_gain_constants>0</var_wind_gain_constants>
                    <var_wind_time_constants>2</var_wind_time_constants>
                    <update_rate>10</update_rate>
                    <topic_wind_velocity_cmd>/vrx/wind/velocity_cmd</topic_wind_velocity_cmd>
                    <topic_wind_speed>/model/{ship.model_name}/debug/wind/speed</topic_wind_speed>
                    <topic_wind_direction>/model/{ship.model_name}/debug/wind/direction</topic_wind_direction>
                </plugin>
                <plugin filename="libTargetShipController.so" name="vrx::TargetShipController">
                    <link_name>base_link</link_name>
                    <topic>/model/{ship.model_name}/cmd_vel</topic>
                    <kp_linear>5000</kp_linear>
                    <kp_heading>60000</kp_heading>
                    <kd_yaw>25000</kd_yaw>
                    <max_force>15000</max_force>
                    <max_torque>120000</max_torque>
                </plugin>"""

    def _generate_mesh_profile_sdf(self, ship):
        profile_path = _resolve_profile_path(ship.mesh_profile, self.config_base_dir)
        if not profile_path or not os.path.isfile(profile_path):
            raise FileNotFoundError(f"mesh_profile not found: {ship.mesh_profile}")

        with open(profile_path, 'r') as f:
            data = yaml.safe_load(f) or {}

        mesh = data.get('mesh') or {}
        mesh_uri = str(mesh.get('uri', '')).strip()
        mesh_scale = _as_vec(mesh.get('scale'), 3, 1.0)
        mesh_rgba = _as_vec(mesh.get('material_rgba'), 4, 1.0)
        mesh_pose = mesh.get('pose_offset') or {}
        mesh_xyz = _as_vec(mesh_pose.get('xyz'), 3, 0.0)
        mesh_rpy = _as_vec(mesh_pose.get('rpy'), 3, 0.0)

        collision_group = data.get('collision_group') or {}
        group_pose = collision_group.get('pose_offset') or {}
        group_xyz = _as_vec(group_pose.get('xyz'), 3, 0.0)
        group_rpy = _as_vec(group_pose.get('rpy'), 3, 0.0)

        collision_xml = []
        for idx, box in enumerate(data.get('boxes') or []):
            box_name = str(box.get('name') or f'collision_{idx}').strip() or f'collision_{idx}'
            size = _as_vec(box.get('size_lwh_m'), 3, 0.0)
            pose_xyz = _as_vec(box.get('pose_xyz_m'), 3, 0.0)
            pose_rpy = _as_vec(box.get('pose_rpy'), 3, 0.0)
            world_xyz = [
                group_xyz[0] + pose_xyz[0],
                group_xyz[1] + pose_xyz[1],
                group_xyz[2] + pose_xyz[2],
            ]
            world_rpy = [
                group_rpy[0] + pose_rpy[0],
                group_rpy[1] + pose_rpy[1],
                group_rpy[2] + pose_rpy[2],
            ]
            collision_xml.append(
                f"""
                    <collision name="{box_name}">
                        <pose>{_fmt_pose_xyzrpy(world_xyz, world_rpy)}</pose>
                        <geometry>
                            <box><size>{size[0]:.6f} {size[1]:.6f} {size[2]:.6f}</size></box>
                        </geometry>
                    </collision>"""
            )

        spawn_z = float(data.get('spawn_z', 0.0))
        environment_plugins = self._environment_plugins(ship)

        r, g, b = ship.color
        a = 1.0

        sdf = f"""<?xml version="1.0" ?>
        <sdf version="1.6">
            <model name="{ship.model_name}">
                <static>false</static>
                <link name="base_link">
                    <gravity>true</gravity>
                    <visual name="visual">
                        <pose>{_fmt_pose_xyzrpy(mesh_xyz, mesh_rpy)}</pose>
                        <geometry>
                            <mesh>
                                <uri>{mesh_uri}</uri>
                                <scale>{mesh_scale[0]:.6f} {mesh_scale[1]:.6f} {mesh_scale[2]:.6f}</scale>
                            </mesh>
                        </geometry>
                        <material>
                            <ambient>{r:.6f} {g:.6f} {b:.6f} {a:.6f}</ambient>
                            <diffuse>{r:.6f} {g:.6f} {b:.6f} {a:.6f}</diffuse>
                            <specular>0.15 0.15 0.10 1.0</specular>
                        </material>
                    </visual>
                    {''.join(collision_xml)}
                    <inertial>
                        <mass>3500.0</mass>
                        <inertia>
                            <ixx>5000.0</ixx>
                            <iyy>30000.0</iyy>
                            <izz>30000.0</izz>
                        </inertia>
                    </inertial>
                </link>
                {environment_plugins}
            </model>
        </sdf>
        """
        return sdf

    def _generate_box_sdf(self, ship):
        name = ship.model_name
        r, g, b = ship.color
        geom = "<box><size>3.6 10.0 2.0</size></box>"
        environment_plugins = self._environment_plugins(ship)
        sdf = f"""<?xml version="1.0" ?>
        <sdf version="1.6">
            <model name="{name}">
                <static>false</static>
                <link name="base_link">
                    <gravity>true</gravity>
                    <visual name="visual">
                        <geometry>{geom}</geometry>
                        <material>
                            <ambient>{r:.6f} {g:.6f} {b:.6f} 1.0</ambient>
                            <diffuse>{r:.6f} {g:.6f} {b:.6f} 1.0</diffuse>
                            <emissive>{r:.6f} {g:.6f} {b:.6f} 1.0</emissive>
                        </material>
                    </visual>
                    <collision name="collision">
                        <geometry>{geom}</geometry>
                    </collision>
                    <inertial>
                        <mass>3500.0</mass>
                        <inertia>
                            <ixx>5000.0</ixx>
                            <iyy>30000.0</iyy>
                            <izz>30000.0</izz>
                        </inertia>
                    </inertial>
                </link>
                {environment_plugins}
            </model>
        </sdf>
        """
        return sdf

    def _generate_cylinder_sdf(self, ship):
        name = ship.model_name
        r, g, b = ship.color
        geom = "<cylinder><radius>4.0</radius><length>5.0</length></cylinder>"
        environment_plugins = self._environment_plugins(ship)
        sdf = f"""<?xml version="1.0" ?>
        <sdf version="1.6">
            <model name="{name}">
                <static>false</static>
                <link name="base_link">
                    <gravity>true</gravity>
                    <visual name="visual">
                        <geometry>{geom}</geometry>
                        <material>
                            <ambient>{r:.6f} {g:.6f} {b:.6f} 1.0</ambient>
                            <diffuse>{r:.6f} {g:.6f} {b:.6f} 1.0</diffuse>
                            <emissive>{r:.6f} {g:.6f} {b:.6f} 1.0</emissive>
                        </material>
                    </visual>
                    <collision name="collision">
                        <geometry>{geom}</geometry>
                    </collision>
                    <inertial>
                        <mass>3500.0</mass>
                        <inertia>
                            <ixx>5000.0</ixx>
                            <iyy>30000.0</iyy>
                            <izz>30000.0</izz>
                        </inertia>
                    </inertial>
                </link>
                {environment_plugins}
            </model>
        </sdf>
        """
        return sdf

    def _spawn_gazebo(self, ship, sdf_str):
        """Dispatch only for an absent name and verify creation in the scene.

        :param ship: Ship whose exact model name must be created.
        :param sdf_str: Prepared model SDF.
        :raises RuntimeError: Preflight or creation cannot be confirmed.
        """
        z = 0.5
        if ship.shape == 'mesh_profile':
            profile_path = _resolve_profile_path(ship.mesh_profile, self.config_base_dir)
            if profile_path and os.path.isfile(profile_path):
                with open(profile_path, 'r') as f:
                    data = yaml.safe_load(f) or {}
                profile_spawn_z = float(data.get('spawn_z', 0.0))
                if profile_spawn_z != 0.0:
                    z = profile_spawn_z

        tmp_sdf_path = None
        try:
            with tempfile.NamedTemporaryFile(mode='w', suffix='.sdf', delete=False) as tmpf:
                tmpf.write(sdf_str)
                tmp_sdf_path = tmpf.name

            cmd = [
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-world', self.world_name,
                '-file', tmp_sdf_path,
                '-name', ship.model_name,
                '-x', str(ship.current_x),
                '-y', str(ship.current_y),
                '-z', str(z),
                '-Y', str(ship.spawn_yaw),
            ]
            present = self._gazebo_model_names()
            if present is None or ship.model_name in present:
                raise RuntimeError(
                    f"Cannot confirm '{ship.model_name}' is absent; create not dispatched")
            # Only dispatched attempts acquire an entity reservation. A late
            # create can outlive its CLI, so absence alone cannot release it.
            with self._ships_lock:
                self.ships[ship.model_name] = ship
                self._pending_spawns[ship.model_name] = False
                self.gz_pose_cache.forget(ship.model_name)
                self._membership_changed()
            try:
                result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
                command_ok = result.returncode == 0
                detail = f"code={result.returncode}: {(result.stderr or result.stdout or '').strip()}"
            except (subprocess.TimeoutExpired, OSError) as error:
                command_ok = False
                detail = str(error)
            created = self._wait_for_gazebo_model(ship.model_name, True)
            if created:
                with self._ships_lock:
                    self._pending_spawns[ship.model_name] = True
            # ros_gz_sim create can exit zero on timeout/rejection. Its exit
            # status or output alone is never proof that the model exists.
            if command_ok and created:
                self.get_logger().info(
                    f"Gazebo scene confirms creation of '{ship.model_name}'")
            else:
                self.get_logger().error(
                    f"Gazebo spawn failed for '{ship.model_name}': {detail}")
                raise RuntimeError(
                    f"Gazebo creation unconfirmed or command failed; name quarantined: {detail}")
        except Exception:
            raise
        finally:
            if tmp_sdf_path and os.path.exists(tmp_sdf_path):
                self._retained_spawn_sdf_paths.append(tmp_sdf_path)

    def _gazebo_model_names(self, timeout_ms=1000):
        """Read authoritative inventory without adding a pose pipeline.

        :param timeout_ms: Total scene query budget in milliseconds.
        :return: Model names, or None when inventory cannot be read.
        """
        try:
            return self.gz_pose_cache.model_names(self.world_name, timeout_ms=timeout_ms)
        except Exception as error:
            self.get_logger().warn(
                f'Cannot read Gazebo model inventory: {error}',
                throttle_duration_sec=5.0)
            return None

    def _wait_for_gazebo_model(self, model_name, expected_present):
        """Poll scene state for up to two wall-clock seconds without holding locks.

        :param model_name: Exact model name whose state must be confirmed.
        :param expected_present: True for creation, False for removal.
        :return: True only if a successful inventory confirms the requested state.
        """
        deadline = time.monotonic() + 2.0
        while True:
            timeout_ms = min(1000, int((deadline - time.monotonic()) * 1000))
            if timeout_ms <= 0:
                return False
            names = self._gazebo_model_names(timeout_ms=timeout_ms)
            remaining = deadline - time.monotonic()
            if remaining < 0.0:
                return False
            if names is not None and (model_name in names) == expected_present:
                return True
            if remaining > 0.0:
                time.sleep(min(0.05, remaining))

    def _cleanup_ship(self, ship):
        """Keep failed resource cleanup owned and isolate failures per ship.

        :param ship: Ship whose publisher and child must be released.
        """
        with self._ships_lock:
            self._retired_ships.add(ship)
        try:
            ship.cleanup()
        except Exception as error:
            self.get_logger().warn(f"Cleanup pending for '{ship.model_name}': {error}")
        else:
            with self._ships_lock:
                self._retired_ships.discard(ship)

    def _forget_ship(self, name):
        """Retire a registration only after Gazebo removal is confirmed.

        :param name: Removed model name.
        """
        with self._ships_lock:
            ship = self.ships.get(name)
            if ship is None:
                return
            ship.retire()
            del self.ships[name]
            self._retired_ships.add(ship)
            self._pending_spawns.pop(name, None)
            self._missing_model_counts.pop(name, None)
            self._membership_changed(removed_name=name)
        self._cleanup_ship(ship)

    def _membership_changed(self, removed_name=None):
        """Invalidate publication after a change; caller holds the registry lock.

        :param removed_name: Confirmed deletion requiring a root-absence witness.
        """
        self._registry_revision += 1
        self.gz_pose_cache.invalidate_frame(removed_name=removed_name)

    def reconcile_gazebo_models(self):
        """Retry cleanup even with zero active ships and reconcile scene deletions.

        Two successful missing inventory reads confirm deletion only after a
        model was observed. Unresolved creates remain quarantined despite absence.
        Missing pose samples or failed inventory reads never imply removal.
        """
        with self._ships_lock:
            retired = list(self._retired_ships)
        for ship in retired:
            self._cleanup_ship(ship)
        with self._ships_lock:
            names = list(self.ships)
            pending = dict(self._pending_spawns)
        if not names:
            return
        present = self._gazebo_model_names()
        for name in names:
            if name in pending:
                if present is not None and name in present:
                    with self._ships_lock:
                        self._pending_spawns[name] = True
                    self._missing_model_counts.pop(name, None)
                    try:
                        self._remove_gazebo(name)
                    except Exception as error:
                        self.get_logger().warn(f"Spawn rollback pending for '{name}': {error}")
                    else:
                        self._forget_ship(name)
                    continue
                if not pending[name]:
                    # Repeated absence cannot exclude a still-queued create.
                    continue
            if present is None:
                continue
            if name in present:
                self._missing_model_counts.pop(name, None)
                continue
            missing = self._missing_model_counts.get(name, 0) + 1
            self._missing_model_counts[name] = missing
            if missing >= 2:
                self._forget_ship(name)
                self.get_logger().info(f"Removed absent Gazebo model registration '{name}'")

    def cleanup_ships(self):
        """After executor shutdown, clean all resources and retry failed handles.

        Entity registrations remain owned because shutdown does not remove models.
        Each child gets bounded attempts even when another child's cleanup fails.
        """
        with self._ships_lock:
            ships = set(self.ships.values()) | self._retired_ships
        for ship in ships:
            self._cleanup_ship(ship)
        with self._ships_lock:
            retired = list(self._retired_ships)
        for ship in retired:
            self._cleanup_ship(ship)

    def _remove_gazebo(self, model_name):
        with self._ships_lock:
            unresolved_create = self._pending_spawns.get(model_name) is False
        if unresolved_create:
            present = self._gazebo_model_names()
            if present is None or model_name not in present:
                raise RuntimeError(
                    f"Create for '{model_name}' is unresolved; name remains quarantined")
            with self._ships_lock:
                self._pending_spawns[model_name] = True
        cmd = [
            'gz', 'service', '-s', f'/world/{self.world_name}/remove',
            '--reqtype', 'gz.msgs.Entity',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '5000',
            '--req', f'name: {json.dumps(model_name)} type: MODEL'
        ]
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
            detail = f'{result.stdout} {result.stderr}'
        except (subprocess.TimeoutExpired, OSError) as error:
            detail = str(error)
        # Boolean true only acknowledges a queued remove. A timeout can also
        # follow actual removal; only a successful scene read proves absence.
        if self._wait_for_gazebo_model(model_name, False):
            return
        raise RuntimeError(f"Gazebo did not confirm removal of '{model_name}': {detail}")

    def control_loop(self):
        if self._clock_reset.is_set():
            # The clock callback only sets an Event: no cache/registry locks in
            # that callback, and only this control thread mutates tracker history.
            self._clock_reset.clear()
            with self._ships_lock:
                self.gz_pose_cache.reset()
                for ship in self.ships.values():
                    ship.reset_feedback()
                self._last_published_stamp_ns = None
        stale_ships = []
        names = []
        with self._ships_lock:
            revision = self._registry_revision
            pending = bool(self._pending_spawns)
            ships = [ship for name, ship in self.ships.items()
                     if name not in self._pending_spawns]
            gz_samples, frame = self.gz_pose_cache.snapshot()
        now = self.get_clock().now()
        complete = (not pending and frame is not None and
                    is_feedback_fresh_ns(frame[0], now.nanoseconds, self.pose_feedback_timeout))
        msg = TrackedShipList()
        msg.header.frame_id = 'map'
        for ship in ships:
            sample = gz_samples.get(ship.model_name)
            fresh = ship.sync_from_feedback_ns(
                sample, now.nanoseconds, self.pose_feedback_timeout)
            if not fresh:
                stale_ships.append(ship.model_name)

            twist = ship.compute_cmd_vel(self.dt, predict=not fresh)
            if ship.publish_cmd_vel(twist):
                names.append(ship.model_name)

            if (not complete or not fresh or not ship.pose_tracker.ready or
                    frame[1].get(ship.model_name) is None or
                    ship.pose_tracker.stamp_ns != frame[0] or
                    not all(map(math.isfinite, ship.pose_tracker.velocity))):
                complete = False
                continue

            ts = TrackedShip()
            ts.target_id.uuid = list(bytes.fromhex(ship.target_id.replace('-', '')))
            ts.pose = ship.get_current_pose()
            ts.twist = ship.get_current_twist()
            ts.radius = 5.0
            msg.ships.append(ts)

        if stale_ships:
            self.get_logger().warn(
                'dynamic ship feedback unavailable; tracked snapshot withheld: %s'
                % ', '.join(stale_ships),
                throttle_duration_sec=5.0)

        if complete:
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(frame[0], 1_000_000_000)
            cache = self.gz_pose_cache
            # Lock order is registry -> cache. Only the final validation/publish
            # runs here, never scene queries or process waits. Frame identity is
            # its generation token: accepted/replaced/reset frames cannot pass as
            # the candidate selected earlier, even with an identical timestamp.
            with self._ships_lock, cache._lock:
                final_now_ns = self.get_clock().now().nanoseconds
                cache._promote_pending(final_now_ns)
                if (revision == self._registry_revision and not self._pending_spawns and
                        cache._frame is frame and not cache._awaiting_absence and
                        is_feedback_fresh_ns(frame[0], final_now_ns, self.pose_feedback_timeout) and
                        (self._last_published_stamp_ns is None or
                         frame[0] > self._last_published_stamp_ns) and
                        not self._clock_reset.is_set()):
                    # This post-clock-read Event check is the commit point. The
                    # ROS jump callback only sets the Event, never takes either
                    # lock (Clock.now may hold the ROS clock mutex during it).
                    self.tracked_pub.publish(msg)
                    self._last_published_stamp_ns = frame[0]
        self.names_pub.publish(String(data=json.dumps(names)))


def main(args=None):
    rclpy.init(args=args)
    node = DynamicShipManager()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.cleanup_ships()
        try:
            node.gz_pose_cache.close()
        except Exception as error:
            node.get_logger().warn(f'Cannot close Gazebo pose subscription: {error}')
        for p in getattr(node, '_retained_spawn_sdf_paths', []):
            if p and os.path.isfile(p):
                try:
                    os.remove(p)
                except OSError:
                    pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
