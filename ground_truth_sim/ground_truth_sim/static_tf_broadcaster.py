#!/usr/bin/env python3
"""Static TF broadcaster to anchor base_link and sensor frames for RViz visualization."""

from __future__ import annotations

import math
from typing import List

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


_SENSOR_TRANSFORMS = [
    ("nav_radar_link", 0.0, 0.0, 2.0, 0.0),
    ("camera_front_link", 0.25, 0.0, 1.15, 0.0),
    ("camera_left_link", 0.0, 0.25, 1.15, math.pi / 2.0),
    ("camera_right_link", 0.0, -0.25, 1.15, -math.pi / 2.0),
    ("camera_back_link", -0.25, 0.0, 1.15, math.pi),
    ("mmwave_front_link", 0.25, 0.0, 1.0, 0.0),
    ("mmwave_left_link", 0.0, 0.25, 1.0, math.pi / 2.0),
    ("mmwave_right_link", 0.0, -0.25, 1.0, -math.pi / 2.0),
    ("mmwave_back_link", -0.25, 0.0, 1.0, math.pi),
]


class StaticFramePublisher(Node):
    """Publishes static transforms for base_link anchor and onboard sensors."""

    def __init__(self) -> None:
        super().__init__("static_frame_publisher")
        self.declare_parameter("parent_frame", "map")
        self.declare_parameter("child_frame", "base_link")
        self.declare_parameter("xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("rpy", [0.0, 0.0, 0.0])
        self.declare_parameter("sensor_parent_frame", "base_link")

        parent_frame = self.get_parameter("parent_frame").get_parameter_value().string_value
        child_frame = self.get_parameter("child_frame").get_parameter_value().string_value
        xyz_param = self._get_float_list("xyz", 3)
        rpy_param = self._get_float_list("rpy", 3)
        sensor_parent = (
            self.get_parameter("sensor_parent_frame").get_parameter_value().string_value
        )

        stamp = self.get_clock().now().to_msg()
        self._broadcaster = StaticTransformBroadcaster(self)

        transforms: List[TransformStamped] = []
        transforms.append(
            self._make_transform(stamp, parent_frame, child_frame, xyz_param, rpy_param)
        )

        for child, x, y, z, yaw in _SENSOR_TRANSFORMS:
            transforms.append(
                self._make_transform(
                    stamp,
                    sensor_parent,
                    child,
                    [x, y, z],
                    [0.0, 0.0, yaw],
                )
            )

        self._broadcaster.sendTransform(transforms)
        self.get_logger().info(
            "Published %d static transforms (anchor + sensors)." % len(transforms)
        )

    def _get_float_list(self, name: str, expected_len: int) -> List[float]:
        param = self.get_parameter(name).get_parameter_value().double_array_value
        values = list(param)
        if len(values) != expected_len:
            self.get_logger().warn(
                "Parameter %s has %d elements, expected %d. Using zeros." % (
                    name,
                    len(values),
                    expected_len,
                )
            )
            return [0.0] * expected_len
        return values

    def _make_transform(
        self,
        stamp,
        parent_frame: str,
        child_frame: str,
        xyz: List[float],
        rpy: List[float],
    ) -> TransformStamped:
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame
        transform.transform.translation.x = xyz[0]
        transform.transform.translation.y = xyz[1]
        transform.transform.translation.z = xyz[2]
        qx, qy, qz, qw = euler_to_quaternion(rpy[0], rpy[1], rpy[2])
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        return transform


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StaticFramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
