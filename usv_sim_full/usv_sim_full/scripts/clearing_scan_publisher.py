#!/usr/bin/env python3
"""Publish a ring of max-range clearing points for costmap raytracing."""

import math
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField


class ClearingScanPublisher(Node):
    def __init__(self):
        super().__init__('clearing_scan_publisher')

        self.declare_parameter('topic', '/clearing_scan')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('max_range', 200.0)
        self.declare_parameter('num_rays', 360)
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('z_height', 0.0)

        topic = self.get_parameter('topic').value
        self._frame_id = self.get_parameter('frame_id').value
        max_r = self.get_parameter('max_range').value
        n = self.get_parameter('num_rays').value
        rate = self.get_parameter('publish_rate').value
        z = self.get_parameter('z_height').value

        self._cloud = self._make_ring(z, max_r, n)
        self._pub = self.create_publisher(PointCloud2, topic, 10)
        self._timer = self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info(
            f'Clearing scan on {topic}: {n} rays at r={max_r:.0f}m, {rate:.0f}Hz'
        )

    def _make_ring(self, z, r, n):
        cloud = PointCloud2()
        cloud.height = 1
        cloud.width = n
        cloud.is_dense = True
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.point_step = 12
        cloud.row_step = 12 * n
        cloud.data = bytearray(cloud.row_step)
        for i in range(n):
            a = 2.0 * math.pi * i / n
            struct.pack_into('fff', cloud.data, i * 12, r * math.cos(a), r * math.sin(a), z)
        return cloud

    def _publish(self):
        self._cloud.header.frame_id = self._frame_id
        self._cloud.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(self._cloud)


def main(args=None):
    rclpy.init(args=args)
    node = ClearingScanPublisher()
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
