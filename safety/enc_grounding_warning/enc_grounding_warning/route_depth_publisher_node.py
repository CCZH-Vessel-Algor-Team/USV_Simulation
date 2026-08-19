"""Publish a route-corridor depth grid and colored RViz markers.

Only cells within a configurable corridor around the active Nav2 plan are
published, so RViz stays light and the visualization matches the grounding
warning corridor logic.
"""

from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from enc_grounding_warning_msgs.msg import DepthGrid

from .bathymetry import SimGridProvider, UNKNOWN_DEPTH
from .common import default_config_path, load_ukc_params, path_to_points


_DEPTH_COLOR_STOPS = [
    (0.00, (0.85, 0.05, 0.05)),   # 极浅：暗红
    (0.25, (1.00, 0.40, 0.00)),   # 浅：橙
    (0.50, (1.00, 0.95, 0.10)),   # 中：黄
    (0.75, (0.00, 0.80, 0.90)),   # 较深：青
    (1.00, (0.05, 0.10, 0.90)),   # 深：蓝
]


def depth_to_color(depth: float, min_d: float = 0.0, max_d: float = 5.0):
    """Warm (shallow) -> cold (deep) multi-stop gradient."""
    t = (depth - min_d) / max(1e-6, max_d - min_d)
    t = min(1.0, max(0.0, t))

    for i in range(len(_DEPTH_COLOR_STOPS) - 1):
        t0, (r0, g0, b0) = _DEPTH_COLOR_STOPS[i]
        t1, (r1, g1, b1) = _DEPTH_COLOR_STOPS[i + 1]
        if t <= t1:
            f = (t - t0) / max(1e-6, t1 - t0)
            return (
                r0 + (r1 - r0) * f,
                g0 + (g1 - g0) * f,
                b0 + (b1 - b0) * f,
            )
    return _DEPTH_COLOR_STOPS[-1][1]


class RouteDepthPublisherNode(Node):
    def __init__(self):
        super().__init__("route_depth_publisher_node")

        self.declare_parameter("params_file", "")
        self.declare_parameter("depth_grid_file", "")
        self.declare_parameter("publish_hz", 1.0)

        params_file = self.get_parameter("params_file").value
        if not params_file:
            params_file = default_config_path("grounding_warning_params.yaml")
        self.params = load_ukc_params(params_file)

        depth_grid_file = self.get_parameter("depth_grid_file").value
        if not depth_grid_file:
            depth_grid_file = default_config_path("sim_depth_grid.yaml")
        self.provider = SimGridProvider(depth_grid_file)

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.plan_sub = self.create_subscription(Path, "plan", self.plan_cb, qos)
        self.grid_pub = self.create_publisher(
            DepthGrid, "safety/route_depth_grid", qos
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, "safety/route_depth_markers", qos
        )

        self.latest_plan = None
        self._last_immediate = 0.0
        hz = float(self.get_parameter("publish_hz").value)
        self.timer = self.create_timer(1.0 / hz, self.timer_cb)

    def plan_cb(self, msg: Path):
        self.latest_plan = msg
        if self._should_immediate():
            self.run_once()

    def _should_immediate(self) -> bool:
        now = time.monotonic()
        min_interval = float(self.params.get("plan_immediate_min_interval_s", 0.2))
        if now - self._last_immediate < min_interval:
            return False
        self._last_immediate = now
        return True

    def _corridor_half_width(self) -> float:
        return float(
            self.params.get(
                "route_corridor_half_width_m",
                float(self.params.get("beam_m", 2.0)) / 2.0
                + float(self.params.get("position_margin_m", 1.0))
                + 5.0,
            )
        )

    def timer_cb(self):
        self.run_once()

    def run_once(self):
        if self.latest_plan is None or len(self.latest_plan.poses) < 2:
            return

        points = path_to_points(self.latest_plan)
        resolution = float(self.params.get("route_grid_resolution_m", 1.0))
        data = self.provider.corridor_grid(
            points,
            self._corridor_half_width(),
            resolution,
        )
        if data is None:
            return

        self._publish_grid(data)
        self._publish_markers(data)

    def _publish_grid(self, data: dict):
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
        self.grid_pub.publish(msg)

    def _publish_markers(self, data: dict):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.provider.frame_id
        marker.ns = "route_depth_grid"
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.scale.x = float(data["resolution"])
        marker.scale.y = float(data["resolution"])
        marker.scale.z = 0.05
        marker.pose.orientation.w = 1.0

        depth = data["depth"]
        known_depths = [
            float(d)
            for d in depth.reshape(-1)
            if d > UNKNOWN_DEPTH / 2.0
        ]
        min_d = min(known_depths) if known_depths else 0.0
        max_d = max(known_depths) if known_depths else 5.0
        if max_d - min_d < 1e-3:
            max_d = min_d + 1.0

        for iy in range(data["height"]):
            for ix in range(data["width"]):
                d = float(depth[iy, ix])
                if d <= UNKNOWN_DEPTH / 2.0:
                    continue
                p = Point()
                p.x = float(data["origin_x"]) + (ix + 0.5) * float(data["resolution"])
                p.y = float(data["origin_y"]) + (iy + 0.5) * float(data["resolution"])
                p.z = 0.0
                marker.points.append(p)

                r, g, b = depth_to_color(d, min_d, max_d)
                color = ColorRGBA()
                color.r = r
                color.g = g
                color.b = b
                color.a = 1.0
                marker.colors.append(color)

        self.marker_pub.publish(MarkerArray(markers=[marker]))


def main(args=None):
    rclpy.init(args=args)
    node = RouteDepthPublisherNode()
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
