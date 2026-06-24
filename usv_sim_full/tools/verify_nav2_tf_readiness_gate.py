#!/usr/bin/env python3
"""本地验证 nav2_tf_readiness_gate：模拟 /clock、TF、/map 后应 exit 0。"""

from __future__ import annotations

import sys
import threading
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rosgraph_msgs.msg import Clock
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from usv_sim_full.scripts.nav2_tf_readiness_gate import Nav2TfReadinessGate
from usv_sim_full.scripts.tf_namespace_relay import TfNamespaceRelay


class FakeSimPublisher(Node):
    def __init__(self) -> None:
        super().__init__(
            'fake_sim_for_nav2_gate_test',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )
        self._clock_pub = self.create_publisher(Clock, '/clock', 10)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._static_broadcaster = StaticTransformBroadcaster(self)
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._map_pub = self.create_publisher(OccupancyGrid, '/map', map_qos)
        self._sim_t = 0.0
        self._tick_thread = threading.Thread(target=self._tick_loop, daemon=True)
        self._tick_thread.start()

    def _tick_loop(self) -> None:
        while rclpy.ok():
            self._tick()
            time.sleep(0.05)

    def _tick(self) -> None:
        self._sim_t += 0.05
        sec = int(self._sim_t)
        nsec = int((self._sim_t - sec) * 1e9)
        clock = Clock()
        clock.clock.sec = sec
        clock.clock.nanosec = nsec
        self._clock_pub.publish(clock)

        stamp = self.get_clock().now().to_msg()
        static_tf = TransformStamped()
        static_tf.header.stamp = stamp
        static_tf.header.frame_id = 'map'
        static_tf.child_frame_id = 'usv_1/odom'
        self._static_broadcaster.sendTransform(static_tf)

        dyn_tf = TransformStamped()
        dyn_tf.header.stamp = stamp
        dyn_tf.header.frame_id = 'usv_1/odom'
        dyn_tf.child_frame_id = 'usv_1/base_link'
        self._tf_broadcaster.sendTransform(dyn_tf)

        grid = OccupancyGrid()
        grid.header.stamp = stamp
        grid.header.frame_id = 'map'
        grid.info.resolution = 0.3
        grid.info.width = 10
        grid.info.height = 10
        grid.data = [0] * 100
        self._map_pub.publish(grid)


def main() -> int:
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])

    pub = FakeSimPublisher()
    relay = TfNamespaceRelay(
        parameter_overrides=[
            Parameter('namespace', Parameter.Type.STRING, 'usv_1'),
            Parameter('use_sim_time', Parameter.Type.BOOL, True),
        ],
    )
    gate = Nav2TfReadinessGate(
        parameter_overrides=[
            Parameter('namespace', Parameter.Type.STRING, 'usv_1'),
            Parameter('min_wait_sec', Parameter.Type.DOUBLE, 0.5),
            Parameter('readiness_timeout_sec', Parameter.Type.DOUBLE, 15.0),
            Parameter('poll_period_sec', Parameter.Type.DOUBLE, 0.2),
            Parameter('tf_stable_checks', Parameter.Type.INTEGER, 2),
            Parameter('require_map', Parameter.Type.BOOL, True),
        ],
    )

    executor = MultiThreadedExecutor()
    executor.add_node(pub)
    executor.add_node(relay)
    executor.add_node(gate)

    exit_code = 1
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    deadline = time.monotonic() + 20.0
    while time.monotonic() < deadline:
        if gate._done:
            exit_code = gate.exit_code
            break
        time.sleep(0.1)

    executor.shutdown()
    pub.destroy_node()
    relay.destroy_node()
    gate.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    spin_thread.join(timeout=2.0)

    if exit_code != 0:
        print(f'FAIL: gate exit code {exit_code}', file=sys.stderr)
        return 1
    print('PASS: nav2_tf_readiness_gate exited 0 with fake sim data')
    return 0


if __name__ == '__main__':
    sys.exit(main())
