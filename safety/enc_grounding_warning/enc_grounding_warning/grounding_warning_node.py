"""Automatic look-ahead grounding warning + route check service."""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from tf2_ros import Buffer, TransformListener
from usv_interfaces.msg import VesselState
from visualization_msgs.msg import Marker, MarkerArray

from enc_grounding_warning_msgs.msg import (
    GroundingAlert,
    RouteDepthProfile,
)
from enc_grounding_warning_msgs.srv import RouteCheck

from .bathymetry import SimGridProvider, UNKNOWN_DEPTH
from .common import default_config_path, load_ukc_params, lookup_map_pose, path_to_points
from .markers import risk_rgba
from .ukc_model import RISK_UNKNOWN, RISK_WARNING, compute_ukc, yaw_from_quaternion


class GroundingWarningNode(Node):
    def __init__(self):
        super().__init__("grounding_warning_node")

        self.declare_parameter("params_file", "")
        self.declare_parameter("depth_grid_file", "")
        self.declare_parameter("robot_base_frame", "usv_1/base_link")
        self.declare_parameter("publish_hz", 1.0)

        params_file = self.get_parameter("params_file").value
        if not params_file:
            params_file = default_config_path("grounding_warning_params.yaml")
        self.params = load_ukc_params(params_file)

        depth_grid_file = self.get_parameter("depth_grid_file").value
        if not depth_grid_file:
            depth_grid_file = default_config_path("sim_depth_grid.yaml")
        self.provider = SimGridProvider(depth_grid_file)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.plan_sub = self.create_subscription(Path, "plan", self.plan_cb, qos)
        self.vessel_sub = self.create_subscription(
            VesselState, "state/vessel", self.vessel_cb, qos
        )
        self.alert_pub = self.create_publisher(
            GroundingAlert, "safety/grounding_alerts", qos
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, "safety/grounding_markers", 1
        )
        self.route_srv = self.create_service(
            RouteCheck, "safety/route_check", self.route_check_cb
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_plan = None
        self.latest_vessel = None
        self._last_alert_key = None
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

    def vessel_cb(self, msg: VesselState):
        self.latest_vessel = msg

    def _sog(self) -> float:
        if self.latest_vessel is None:
            return 0.0
        v = self.latest_vessel.velocity.linear
        return math.hypot(v.x, v.y)

    def _fallback_pose(self):
        if self.latest_vessel is None:
            return None
        pose = self.latest_vessel.pose
        return (
            float(pose.position.x),
            float(pose.position.y),
            yaw_from_quaternion(pose.orientation),
        )

    def _profile_from_plan(self, points):
        sample_ds = float(self.params.get("sample_ds_m", 2.0))
        half_length = float(self.params.get("length_m", 4.9))
        half_width = float(self.params.get("beam_m", 2.0))
        margin = float(self.params.get("position_margin_m", 1.0))
        return self.provider.route_profile(
            points, sample_ds, half_length, half_width, margin
        )

    def timer_cb(self):
        self.run_once()

    def run_once(self):
        try:
            x, y, yaw = lookup_map_pose(
                self.tf_buffer,
                self.get_parameter("robot_base_frame").value,
                fallback=self._fallback_pose(),
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"TF lookup failed: {exc}", throttle_duration_sec=5.0)
            return

        points = None
        if self.latest_plan is not None and len(self.latest_plan.poses) >= 2:
            points = path_to_points(self.latest_plan)
        else:
            sog = self._sog()
            if sog < 0.05:
                return
            horizon = min(
                float(self.params.get("lookahead_time_s", 300.0)) * sog,
                float(self.params.get("lookahead_distance_m", 1000.0)),
            )
            points = [
                (x, y),
                (x + horizon * math.cos(yaw), y + horizon * math.sin(yaw)),
            ]

        samples = self._profile_from_plan(points)
        if not samples:
            return

        water_level = float(self.params.get("water_level_m", 0.0))
        sog = self._sog()
        min_ukc = None
        first_danger = None
        for sample in samples:
            result = compute_ukc(sample["depth"], water_level, self.params, sog)
            sample["ukc"] = result["ukc"]
            sample["risk"] = result["risk"]
            if sample["ukc"] is not None:
                min_ukc = sample["ukc"] if min_ukc is None else min(min_ukc, sample["ukc"])
            if first_danger is None and result["risk"] >= RISK_WARNING:
                first_danger = sample

        if first_danger is None:
            if self._last_alert_key is not None:
                self.get_logger().info("Grounding risk cleared")
            self._last_alert_key = None
            self.publish_route_markers(samples, None)
            return

        distance = float(first_danger["distance"])
        time_to_danger = distance / max(sog, 0.1)
        if time_to_danger > 600.0:
            level = 1  # INFO
        elif time_to_danger > 120.0:
            level = 2  # WARNING
        else:
            level = 3  # CRITICAL

        key = (level, first_danger["route_index"], round(distance, 1))
        if key == self._last_alert_key:
            return
        self._last_alert_key = key

        alert = GroundingAlert()
        alert.header.stamp = self.get_clock().now().to_msg()
        alert.header.frame_id = "map"
        alert.level = level
        alert.type = 2  # LOOKAHEAD
        alert.min_ukc_m = float(first_danger.get("ukc") or 0.0)
        alert.ukc_required_m = 0.0
        alert.distance_to_danger_m = float(distance)
        alert.time_to_danger_s = float(time_to_danger)
        alert.danger_x = float(first_danger["x"])
        alert.danger_y = float(first_danger["y"])
        alert.route_index = int(first_danger["route_index"])
        alert.description = (
            f"Grounding risk ahead at {distance:.1f} m / {time_to_danger:.0f} s, "
            f"min UKC {alert.min_ukc_m:.2f} m"
        )
        self.alert_pub.publish(alert)
        self.get_logger().warn(alert.description)
        self.publish_route_markers(samples, first_danger)

    def publish_route_markers(self, samples, first_danger):
        arr = MarkerArray()

        points = Marker()
        points.header.stamp = self.get_clock().now().to_msg()
        points.header.frame_id = "map"
        points.ns = "grounding_route"
        points.id = 0
        points.type = Marker.POINTS
        points.action = Marker.ADD
        points.scale.x = 1.0
        points.scale.y = 1.0
        points.pose.orientation.w = 1.0
        for s in samples:
            p = Point()
            p.x = float(s["x"])
            p.y = float(s["y"])
            p.z = 0.0
            points.points.append(p)
            c = ColorRGBA()
            c.r, c.g, c.b, c.a = risk_rgba(int(s.get("risk", RISK_UNKNOWN)))
            points.colors.append(c)
        arr.markers.append(points)

        if first_danger is not None:
            danger = Marker()
            danger.header.stamp = points.header.stamp
            danger.header.frame_id = "map"
            danger.ns = "grounding_danger"
            danger.id = 1
            danger.type = Marker.SPHERE
            danger.action = Marker.ADD
            danger.pose.position.x = float(first_danger["x"])
            danger.pose.position.y = float(first_danger["y"])
            danger.pose.position.z = 0.0
            danger.pose.orientation.w = 1.0
            danger.scale.x = 6.0
            danger.scale.y = 6.0
            danger.scale.z = 6.0
            danger.color.r, danger.color.g, danger.color.b, danger.color.a = (
                1.0,
                0.0,
                0.0,
                0.7,
            )
            arr.markers.append(danger)

        self.marker_pub.publish(arr)

    def route_check_cb(self, request: RouteCheck.Request, response: RouteCheck.Response):
        points = path_to_points(request.route)
        if len(points) < 2:
            response.success = False
            response.first_unsafe_index = -1
            response.message = "route must contain at least 2 poses"
            return response

        samples = self._profile_from_plan(points)
        if not samples:
            response.success = False
            response.first_unsafe_index = -1
            response.message = "empty route profile"
            return response

        water_level = float(self.params.get("water_level_m", 0.0))
        sog = self._sog()
        profile = RouteDepthProfile()
        profile.header.stamp = self.get_clock().now().to_msg()
        profile.header.frame_id = "map"
        profile.route_id = request.route.header.frame_id
        profile.corridor_half_width_m = float(
            self.params.get("beam_m", 2.0) / 2.0
            + self.params.get("position_margin_m", 1.0)
        )

        first_unsafe = -1
        min_ukc = None
        danger_x = 0.0
        danger_y = 0.0
        for sample in samples:
            result = compute_ukc(sample["depth"], water_level, self.params, sog)
            ukc = result["ukc"]
            risk = result["risk"]
            if ukc is not None:
                min_ukc = ukc if min_ukc is None else min(min_ukc, ukc)
            profile.distance_m.append(float(sample["distance"]))
            profile.depth_min_m.append(float(sample["depth"] or UNKNOWN_DEPTH))
            profile.ukc_m.append(float(ukc or 0.0))
            profile.uncertainty_m.append(float(sample.get("uncertainty") or 0.0))
            profile.risk.append(int(risk))
            profile.route_index.append(int(sample["route_index"]))
            if first_unsafe < 0 and risk >= RISK_WARNING:
                first_unsafe = int(sample["route_index"])
                danger_x = float(sample["x"])
                danger_y = float(sample["y"])

        response.success = True
        response.profile = profile
        response.first_unsafe_index = first_unsafe
        response.danger_x = danger_x
        response.danger_y = danger_y
        response.min_ukc_m = float(min_ukc or 0.0)
        if first_unsafe >= 0:
            response.time_to_danger_s = float(
                _time_at_first_unsafe(profile, first_unsafe, sog)
            )
            response.message = f"first unsafe at route index {first_unsafe}"
        else:
            response.time_to_danger_s = -1.0
            response.message = "route is safe"
        depths = [s["depth"] for s in samples if s["depth"] is not None]
        self.get_logger().info(
            f"route_check done: samples={len(samples)} min_ukc={min_ukc} "
            f"first_unsafe={first_unsafe} depth_min={min(depths) if depths else None}"
        )
        self.get_logger().info(
            f"route_check response first_unsafe={response.first_unsafe_index}"
        )
        return response


def _time_at_first_unsafe(profile: RouteDepthProfile, route_index: int, sog: float):
    """Distance at the first sample whose route_index matches."""
    for i, idx in enumerate(profile.route_index):
        if int(idx) >= route_index and int(profile.risk[i]) >= RISK_WARNING:
            return float(profile.distance_m[i]) / max(sog, 0.1)
    return -1.0


def main(args=None):
    rclpy.init(args=args)
    node = GroundingWarningNode()
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
