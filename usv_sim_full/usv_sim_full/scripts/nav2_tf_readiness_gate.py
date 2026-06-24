#!/usr/bin/env python3
"""
等待仿真 TF / 地图就绪后退出，供 nav2_sim_full_bringup 通过 OnProcessExit 拉起 Nav2。

检测项（可配置）：
  - /clock（use_sim_time 场景）
  - map -> {namespace}/odom、{namespace}/odom -> {namespace}/base_link
  - map -> {namespace}/base_link（完整链）
  - 可选 /map（transient_local）

默认在 /{namespace}/tf、/{namespace}/tf_static 上检测（与 Nav2 命名空间 TF 一致；
需 main.launch 中 tf_namespace_relay 已随仿真启动）。
"""

from __future__ import annotations

import sys
import threading
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException, TransformListener


def _sim_time_sec(msg: Clock) -> float:
    return float(msg.clock.sec) + float(msg.clock.nanosec) * 1e-9


class Nav2TfReadinessGate(Node):
    """Poll readiness conditions, then exit 0 (ready) or 1 (timeout)."""

    def __init__(self, **node_kwargs) -> None:
        super().__init__('nav2_tf_readiness_gate', **node_kwargs)
        self.declare_parameter('namespace', 'usv_1')
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        self.declare_parameter('min_wait_sec', 5.0)
        self.declare_parameter('readiness_timeout_sec', 120.0)
        self.declare_parameter('poll_period_sec', 0.5)
        self.declare_parameter('tf_stable_checks', 3)
        self.declare_parameter('tf_lookup_timeout_sec', 0.25)
        self.declare_parameter('require_map', True)
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('min_clock_messages', 2)
        self.declare_parameter('min_sim_time_sec', 0.01)
        self.declare_parameter('use_namespaced_tf_topics', True)

        ns = str(self.get_parameter('namespace').value).strip().strip('/')
        self._namespace = ns if ns else 'usv_1'
        self._use_sim_time = bool(self.get_parameter('use_sim_time').value)
        self._min_wait = max(0.0, float(self.get_parameter('min_wait_sec').value))
        self._timeout = max(1.0, float(self.get_parameter('readiness_timeout_sec').value))
        self._poll_period = max(0.1, float(self.get_parameter('poll_period_sec').value))
        self._stable_required = max(1, int(self.get_parameter('tf_stable_checks').value))
        self._tf_lookup_timeout = max(
            0.05, float(self.get_parameter('tf_lookup_timeout_sec').value)
        )
        self._require_map = bool(self.get_parameter('require_map').value)
        self._map_topic = str(self.get_parameter('map_topic').value).strip() or '/map'
        self._min_clock_msgs = max(1, int(self.get_parameter('min_clock_messages').value))
        self._min_sim_t = max(0.0, float(self.get_parameter('min_sim_time_sec').value))
        self._use_namespaced_tf = bool(
            self.get_parameter('use_namespaced_tf_topics').value
        )

        self._odom_frame = f'{self._namespace}/odom'
        self._base_frame = f'{self._namespace}/base_link'
        self._map_frame = 'map'

        self._exit_code = 1
        self._done = False
        self._clock_count = 0
        self._max_sim_t = 0.0
        self._map_received = not self._require_map
        self._stable_count = 0
        self._t0 = time.monotonic()
        self._min_wait_logged = False

        self._tf_buffer = Buffer()
        if self._use_namespaced_tf:
            self._tf_topic = f'/{self._namespace}/tf'
            self._tf_static_topic = f'/{self._namespace}/tf_static'
            tf_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=100,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            )
            tf_static_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.create_subscription(
                TFMessage, self._tf_topic, self._on_tf, tf_qos
            )
            self.create_subscription(
                TFMessage, self._tf_static_topic, self._on_tf_static, tf_static_qos
            )
        else:
            self._tf_topic = '/tf'
            self._tf_static_topic = '/tf_static'
            TransformListener(self._tf_buffer, self)

        if self._require_map:
            map_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.create_subscription(
                OccupancyGrid, self._map_topic, self._on_map, map_qos
            )

        self.create_subscription(Clock, '/clock', self._on_clock, 10)
        self._poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._poll_thread.start()

    def _poll_loop(self) -> None:
        while rclpy.ok() and not self._done:
            self._poll()
            time.sleep(self._poll_period)

        self.get_logger().info(
            f'Nav2 TF readiness gate: namespace={self._namespace}, '
            f'tf_topics={self._tf_topic},{self._tf_static_topic}, '
            f'min_wait={self._min_wait:.1f}s, timeout={self._timeout:.0f}s, '
            f'require_map={self._require_map}, map_topic={self._map_topic}, '
            f'stable_checks={self._stable_required}'
        )

    @property
    def exit_code(self) -> int:
        return self._exit_code

    def _on_tf(self, msg: TFMessage) -> None:
        for transform in msg.transforms:
            self._tf_buffer.set_transform(transform, 'default_authority')

    def _on_tf_static(self, msg: TFMessage) -> None:
        for transform in msg.transforms:
            self._tf_buffer.set_transform_static(transform, 'default_authority')

    def _on_clock(self, msg: Clock) -> None:
        self._clock_count += 1
        self._max_sim_t = max(self._max_sim_t, _sim_time_sec(msg))

    def _on_map(self, msg: OccupancyGrid) -> None:
        if msg.header.frame_id:
            self._map_received = True

    def _clock_ready(self) -> bool:
        if not self._use_sim_time:
            return True
        if self._clock_count < self._min_clock_msgs:
            return False
        return self._max_sim_t >= self._min_sim_t

    def _lookup_ok(self, target: str, source: str) -> bool:
        try:
            self._tf_buffer.lookup_transform(
                target,
                source,
                Time(),
                timeout=Duration(seconds=self._tf_lookup_timeout),
            )
            return True
        except TransformException:
            return False

    def _tf_chain_ready(self) -> bool:
        if not self._lookup_ok(self._map_frame, self._odom_frame):
            return False
        if not self._lookup_ok(self._odom_frame, self._base_frame):
            return False
        if not self._lookup_ok(self._map_frame, self._base_frame):
            return False
        return True

    def _finish(self, exit_code: int, message: str, level: str = 'info') -> None:
        if self._done:
            return
        self._done = True
        self._exit_code = exit_code
        log_fn = self.get_logger().error if level == 'error' else self.get_logger().info
        log_fn(message)
        rclpy.shutdown()

    def _poll(self) -> None:
        if self._done:
            return

        elapsed = time.monotonic() - self._t0
        if elapsed > self._timeout:
            self._finish(
                1,
                (
                    f'Nav2 TF readiness gate 超时 ({self._timeout:.0f}s): '
                    f'clock_msgs={self._clock_count}, map={self._map_received}, '
                    f'stable={self._stable_count}/{self._stable_required}, '
                    f'frames=map,{self._odom_frame},{self._base_frame}, '
                    f'tf_topics={self._tf_topic},{self._tf_static_topic}'
                ),
                level='error',
            )
            return

        if elapsed < self._min_wait:
            if not self._min_wait_logged:
                self._min_wait_logged = True
                self.get_logger().info(
                    f'最短等待 {self._min_wait:.1f}s 后再检测 TF/地图…'
                )
            return

        if not self._clock_ready():
            self._stable_count = 0
            return

        if self._require_map and not self._map_received:
            self._stable_count = 0
            return

        if not self._tf_chain_ready():
            self._stable_count = 0
            return

        self._stable_count += 1
        if self._stable_count < self._stable_required:
            return

        self._finish(
            0,
            (
                f'Nav2 TF readiness gate 通过 ({elapsed:.1f}s): '
                f'map->{self._odom_frame}, {self._odom_frame}->{self._base_frame}, '
                f'map->{self._base_frame}, tf={self._tf_topic}'
                + (f', {self._map_topic} OK' if self._require_map else '')
            ),
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Nav2TfReadinessGate()
    exit_code = 1
    try:
        rclpy.spin(node)
        exit_code = node.exit_code
    except KeyboardInterrupt:
        exit_code = 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
