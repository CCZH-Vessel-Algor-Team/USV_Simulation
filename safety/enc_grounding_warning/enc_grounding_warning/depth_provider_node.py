"""Publish a near-field depth grid around the own ship."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener
from usv_interfaces.msg import VesselState

from enc_grounding_warning_msgs.msg import DepthGrid

from .bathymetry import SimGridProvider
from .common import default_config_path, lookup_map_pose
from .ukc_model import yaw_from_quaternion


class DepthProviderNode(Node):
    def __init__(self):
        super().__init__("depth_provider_node")

        self.declare_parameter("depth_grid_file", "")
        self.declare_parameter("publish_hz", 1.0)
        self.declare_parameter("near_extent_m", 500.0)
        self.declare_parameter("near_resolution_m", 2.0)
        self.declare_parameter("robot_base_frame", "usv_1/base_link")

        depth_grid_file = self.get_parameter("depth_grid_file").value
        if not depth_grid_file:
            depth_grid_file = default_config_path("sim_depth_grid.yaml")
        self.get_logger().info(f"Loading depth grid from {depth_grid_file}")
        self.provider = SimGridProvider(depth_grid_file)

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.pub = self.create_publisher(DepthGrid, "safety/depth_grid", qos)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.vessel_sub = self.create_subscription(
            VesselState, "state/vessel", self.vessel_cb, qos
        )
        self.latest_vessel = None

        hz = float(self.get_parameter("publish_hz").value)
        self.timer = self.create_timer(1.0 / hz, self.timer_cb)

    def vessel_cb(self, msg: VesselState):
        self.latest_vessel = msg

    def _fallback_pose(self):
        if self.latest_vessel is None:
            return None
        pose = self.latest_vessel.pose
        return (
            float(pose.position.x),
            float(pose.position.y),
            yaw_from_quaternion(pose.orientation),
        )

    def timer_cb(self):
        try:
            x, y, _yaw = lookup_map_pose(
                self.tf_buffer,
                self.get_parameter("robot_base_frame").value,
                fallback=self._fallback_pose(),
            )
        except Exception as exc:  # noqa: BLE001 - TF may not be ready
            self.get_logger().warn(f"TF lookup failed: {exc}", throttle_duration_sec=5.0)
            return

        data = self.provider.near_grid(
            x,
            y,
            float(self.get_parameter("near_extent_m").value),
            float(self.get_parameter("near_resolution_m").value),
        )
        msg = DepthGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.provider.frame_id
        msg.origin_x = float(data["origin_x"])
        msg.origin_y = float(data["origin_y"])
        msg.resolution = float(data["resolution"])
        msg.width = int(data["width"])
        msg.height = int(data["height"])
        msg.depth_m = data["depth"].reshape(-1).tolist()
        msg.uncertainty_m = data["uncertainty"].reshape(-1).tolist()
        msg.quality = data["quality"].reshape(-1).tolist()
        msg.flags = data["flags"].reshape(-1).tolist()
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthProviderNode()
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
