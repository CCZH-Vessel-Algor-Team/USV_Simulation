#!/usr/bin/env python3
"""Relay authoritative snapshots; merge only exactly aligned complete inputs."""

from __future__ import annotations

import threading

import rclpy
from nav2_colregs_msgs.msg import TrackedShipList
from rclpy.clock import JumpThreshold
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile


class TrackedShipListMerger(Node):
    """Preserve observation headers and reject partial or ambiguous unions."""

    def __init__(self) -> None:
        super().__init__('tracked_ship_list_merger')

        self.declare_parameter('input_topics', [
            '/dynamic_ship/tracked_ships/_internal',
        ])
        self.declare_parameter('output_topic', '/dynamic_ship/tracked_ships')
        self.declare_parameter('frame_id', 'map')

        self._output_topic = self._get_string('output_topic')
        self._frame_id = self._get_string('frame_id')
        self._cache: dict[str, TrackedShipList] = {}
        self._clock_reset = threading.Event()
        self._clock_jump_handle = self.get_clock().create_jump_callback(
            JumpThreshold(min_forward=None, min_backward=Duration(nanoseconds=-1), on_clock_change=True),
            post_callback=lambda jump: self._clock_reset.set())

        input_topics = self.get_parameter('input_topics').value
        if isinstance(input_topics, str):
            input_topics = [t.strip() for t in input_topics.split(',') if t.strip()]
        if not input_topics or not self._output_topic.strip() or any(
                not isinstance(topic, str) or not topic.strip() for topic in input_topics):
            raise ValueError('Input and output topics must be nonempty')
        self._input_topics = [self.resolve_topic_name(topic) for topic in input_topics]
        if len(set(self._input_topics)) != len(self._input_topics):
            raise ValueError('Duplicate tracked-list input topics')
        if self.resolve_topic_name(self._output_topic) in self._input_topics:
            raise ValueError('Tracked-list input/output self-loop')

        qos = QoSProfile(depth=10)
        self._pub = self.create_publisher(TrackedShipList, self._output_topic, qos)

        for topic in self._input_topics:
            self.create_subscription(
                TrackedShipList,
                topic,
                lambda msg, t=topic: self._on_input(t, msg),
                qos,
            )
            self.get_logger().info('TrackedShipListMerger listening on %s' % topic)

        self.get_logger().info(
            'TrackedShipListMerger publishing merged list on %s' % self._output_topic)

    def _on_input(self, topic: str, msg: TrackedShipList) -> None:
        """Accept a complete input without fabricating its frame or observation time.

        :param topic: Resolved configured input topic.
        :param msg: Complete tracked snapshot from that source.
        """
        if self._clock_reset.is_set():
            self._clock_reset.clear()
            self._cache.clear()
        if topic not in self._input_topics:
            return
        stamp = msg.header.stamp
        ids = [tuple(ship.target_id.uuid) for ship in msg.ships]
        if (not msg.header.frame_id.strip() or
                (self._frame_id and msg.header.frame_id != self._frame_id) or
                stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000 or
                any(len(key) != 16 for key in ids) or len(set(ids)) != len(ids)):
            self._cache.pop(topic, None)
            self.get_logger().warn('Rejected malformed/duplicate tracked snapshot',
                                   throttle_duration_sec=5.0)
            return
        if len(self._input_topics) == 1:
            if not self._clock_reset.is_set():
                self._pub.publish(msg)
            return
        self._cache[topic] = msg
        self._publish_merged()

    def _publish_merged(self) -> None:
        """Publish only when every configured source has the same exact header."""
        if any(topic not in self._cache for topic in self._input_topics):
            return
        messages = [self._cache[topic] for topic in self._input_topics]
        header = messages[0].header
        if any(msg.header != header for msg in messages[1:]):
            return

        out = TrackedShipList()
        out.header = header
        seen: set[tuple[int, ...]] = set()
        for msg in messages:
            for ship in msg.ships:
                key = tuple(ship.target_id.uuid)
                if key in seen:
                    self.get_logger().warn('Rejected overlapping tracked-source UUIDs',
                                           throttle_duration_sec=5.0)
                    return
                seen.add(key)
                out.ships.append(ship)

        if not self._clock_reset.is_set():
            self._pub.publish(out)

    def _get_string(self, name: str) -> str:
        value = self.get_parameter(name).value
        return str(value) if value is not None else ''


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrackedShipListMerger()
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
