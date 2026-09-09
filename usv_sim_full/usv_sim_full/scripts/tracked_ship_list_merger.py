#!/usr/bin/env python3
"""Merge multiple TrackedShipList sources into one COLREGS input topic."""

from __future__ import annotations

import rclpy
from nav2_colregs_msgs.msg import TrackedShip, TrackedShipList
from rclpy.node import Node
from rclpy.qos import QoSProfile


class TrackedShipListMerger(Node):
    """Combine ship and buoy tracked lists without feedback loops."""

    def __init__(self) -> None:
        super().__init__('tracked_ship_list_merger')

        self.declare_parameter('input_topics', [
            '/dynamic_ship/tracked_ships/_internal',
            '/dynamic_buoy/tracked_ships',
        ])
        self.declare_parameter('output_topic', '/dynamic_ship/tracked_ships')
        self.declare_parameter('frame_id', 'map')

        self._output_topic = self._get_string('output_topic')
        self._frame_id = self._get_string('frame_id')
        self._cache: dict[str, TrackedShipList] = {}

        qos = QoSProfile(depth=10)
        self._pub = self.create_publisher(TrackedShipList, self._output_topic, qos)

        input_topics = self.get_parameter('input_topics').value
        if isinstance(input_topics, str):
            input_topics = [t.strip() for t in input_topics.split(',') if t.strip()]

        for topic in input_topics:
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
        self._cache[topic] = msg
        self._publish_merged()

    def _publish_merged(self) -> None:
        out = TrackedShipList()
        if self._cache:
            latest = max(self._cache.values(), key=lambda m: m.header.stamp.sec)
            out.header = latest.header
        if not out.header.frame_id:
            out.header.frame_id = self._frame_id
        out.header.stamp = self.get_clock().now().to_msg()

        seen: set[tuple[int, ...]] = set()
        for msg in self._cache.values():
            for ship in msg.ships:
                key = tuple(ship.target_id.uuid)
                if key in seen:
                    continue
                seen.add(key)
                out.ships.append(ship)

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
