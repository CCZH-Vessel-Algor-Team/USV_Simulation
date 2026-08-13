"""Publish a test Nav2 plan crossing the default shoal for closed-loop testing."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node


class TestPlanPublisher(Node):
    def __init__(self):
        super().__init__("test_plan_publisher")
        self.declare_parameter("start_x", 200.0)
        self.declare_parameter("start_y", 200.0)
        self.declare_parameter("end_x", 400.0)
        self.declare_parameter("end_y", 300.0)
        self.declare_parameter("publish_hz", 1.0)
        self.pub = self.create_publisher(Path, "plan", 10)
        hz = float(self.get_parameter("publish_hz").value)
        self.timer = self.create_timer(1.0 / hz, self.timer_cb)

    def timer_cb(self):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        for x, y in [
            (float(self.get_parameter("start_x").value),
             float(self.get_parameter("start_y").value)),
            (float(self.get_parameter("end_x").value),
             float(self.get_parameter("end_y").value)),
        ]:
            p = PoseStamped()
            p.header = msg.header
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.position.z = 0.0
            p.pose.orientation.w = 1.0
            msg.poses.append(p)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TestPlanPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
