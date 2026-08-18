"""Estimate current UKC / risk at the own ship position."""

from __future__ import annotations

import math
import time

import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener
from usv_interfaces.msg import VesselState
from visualization_msgs.msg import Marker, MarkerArray

from enc_grounding_warning_msgs.msg import GroundingRiskGrid, UKCState

from .bathymetry import SimGridProvider
from .common import default_config_path, load_ukc_params, lookup_map_pose
from .markers import risk_rgba
from .ukc_model import (
    RISK_WARNING,
    compute_ukc,
    yaw_from_quaternion,
)


class UkcEstimatorNode(Node):
    def __init__(self):
        super().__init__("ukc_estimator_node")

        self.declare_parameter("params_file", "")
        self.declare_parameter("depth_grid_file", "")
        self.declare_parameter("robot_base_frame", "usv_1/base_link")
        self.declare_parameter("publish_hz", 2.0)
        self.declare_parameter("near_extent_m", 500.0)
        self.declare_parameter("near_resolution_m", 2.0)

        params_file = self.get_parameter("params_file").value
        if not params_file:
            params_file = default_config_path("grounding_warning_params.yaml")
        self.params = load_ukc_params(params_file)

        depth_grid_file = self.get_parameter("depth_grid_file").value
        if not depth_grid_file:
            depth_grid_file = default_config_path("sim_depth_grid.yaml")
        self.provider = SimGridProvider(depth_grid_file)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.vessel_sub = self.create_subscription(
            VesselState, "state/vessel", self.vessel_cb, qos
        )
        self.plan_sub = self.create_subscription(Path, "plan", self.plan_cb, qos)
        self.ukc_pub = self.create_publisher(UKCState, "safety/ukc_state", qos)
        self.risk_pub = self.create_publisher(
            GroundingRiskGrid, "safety/grounding_risk_grid", qos
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, "safety/current_risk_marker", 1
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_vessel = None
        self._risk_counter = 0
        self._active_risk = None
        self._last_immediate = 0.0

        hz = float(self.get_parameter("publish_hz").value)
        self.timer = self.create_timer(1.0 / hz, self.timer_cb)
        self.risk_timer = self.create_timer(1.0, self.publish_risk_grid)

    def vessel_cb(self, msg: VesselState):
        self.latest_vessel = msg

    def plan_cb(self, msg: Path):
        if self._should_immediate():
            self.timer_cb()
            self.publish_risk_grid()

    def _should_immediate(self) -> bool:
        now = time.monotonic()
        min_interval = float(self.params.get("plan_immediate_min_interval_s", 0.2))
        if now - self._last_immediate < min_interval:
            return False
        self._last_immediate = now
        return True

    def _current_sog(self) -> float:
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

    def timer_cb(self):
        if self.latest_vessel is None:
            return
        try:
            x, y, yaw = lookup_map_pose(
                self.tf_buffer,
                self.get_parameter("robot_base_frame").value,
                fallback=self._fallback_pose(),
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"TF lookup failed: {exc}", throttle_duration_sec=5.0)
            return

        half_length = float(self.params.get("length_m", 4.9))
        half_width = float(self.params.get("beam_m", 2.0))
        margin = float(self.params.get("position_margin_m", 1.0))
        hit = self.provider.query_footprint(
            x, y, yaw, half_length, half_width, margin
        )
        depth = hit["depth"] if hit else None
        water_level = float(self.params.get("water_level_m", 0.0))
        result = compute_ukc(depth, water_level, self.params, self._current_sog())

        risk = self._debounced_risk(
            result["risk"],
            result["ukc"],
            result["ukc_required"],
            result["uncertainty"],
        )
        result["risk"] = risk

        msg = UKCState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.chart_depth_m = float(result["chart_depth"] or 0.0)
        msg.water_level_m = float(result["water_level"])
        msg.available_depth_m = float(result["available_depth"] or 0.0)
        msg.static_draft_m = float(result["static_draft"])
        msg.squat_m = float(result["squat"])
        msg.heel_allowance_m = float(result["heel_allowance"])
        msg.trim_allowance_m = float(result["trim_allowance"])
        msg.wave_allowance_m = float(result["wave_allowance"])
        msg.dynamic_draft_m = float(result["dynamic_draft"])
        msg.ukc_m = float(result["ukc"] or 0.0)
        msg.ukc_required_m = float(result["ukc_required"])
        msg.uncertainty_m = float(result["uncertainty"])
        msg.safety_depth_m = float(result["safety_depth"])
        msg.risk_level = int(result["risk"])
        self.ukc_pub.publish(msg)
        self.publish_current_marker(x, y, result)

    def publish_current_marker(self, x, y, result):
        arr = MarkerArray()

        sphere = Marker()
        sphere.header.stamp = self.get_clock().now().to_msg()
        sphere.header.frame_id = "map"
        sphere.ns = "ukc_current"
        sphere.id = 0
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position.x = float(x)
        sphere.pose.position.y = float(y)
        sphere.pose.position.z = 0.0
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = 2.0
        sphere.scale.y = 2.0
        sphere.scale.z = 2.0
        sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = risk_rgba(
            int(result["risk"])
        )
        arr.markers.append(sphere)

        text = Marker()
        text.header.stamp = sphere.header.stamp
        text.header.frame_id = "map"
        text.ns = "ukc_current"
        text.id = 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = float(x)
        text.pose.position.y = float(y)
        text.pose.position.z = 3.0
        text.pose.orientation.w = 1.0
        text.scale.z = 2.0
        text.color.r, text.color.g, text.color.b, text.color.a = (1.0, 1.0, 1.0, 1.0)
        text.text = f"UKC {result['ukc'] or 0.0:.2f} m"
        arr.markers.append(text)

        self.marker_pub.publish(arr)

    def _debounced_risk(self, risk, ukc, ukc_req, u_total):
        params = self.params
        debounce = int(params.get("debounce_count", 3))
        hysteresis = float(params.get("hysteresis_m", 0.1))
        margin_safe = float(params.get("margin_safe_m", 0.1))

        if risk >= RISK_WARNING:
            self._risk_counter += 1
        else:
            self._risk_counter = max(0, self._risk_counter - 1)

        if self._risk_counter >= debounce:
            self._active_risk = risk

        if (
            self._active_risk is not None
            and risk < RISK_WARNING
            and ukc is not None
            and ukc >= ukc_req + u_total + margin_safe + hysteresis
        ):
            self._active_risk = None
            self._risk_counter = 0

        return self._active_risk if self._active_risk is not None else risk

    def publish_risk_grid(self):
        try:
            x, y, _yaw = lookup_map_pose(
                self.tf_buffer,
                self.get_parameter("robot_base_frame").value,
                fallback=self._fallback_pose(),
            )
        except Exception:  # noqa: BLE001
            return
        data = self.provider.near_grid(
            x,
            y,
            float(self.get_parameter("near_extent_m").value),
            float(self.get_parameter("near_resolution_m").value),
        )
        depth = data["depth"]
        water_level = float(self.params.get("water_level_m", 0.0))
        sog = self._current_sog()
        risk = _build_risk_grid(self.provider, depth, water_level, self.params, sog)

        msg = GroundingRiskGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.origin_x = float(data["origin_x"])
        msg.origin_y = float(data["origin_y"])
        msg.resolution = float(data["resolution"])
        msg.width = int(data["width"])
        msg.height = int(data["height"])
        msg.risk = risk.reshape(-1).tolist()
        self.risk_pub.publish(msg)


def _build_risk_grid(provider, depth, water_level, params, sog):
    import numpy as np

    risk = np.full(depth.shape, 255, dtype=np.int8)
    h, w = depth.shape
    for iy in range(h):
        for ix in range(w):
            d = float(depth[iy, ix])
            if d <= -4999.0:
                continue
            result = compute_ukc(d, water_level, params, sog)
            risk[iy, ix] = int(result["risk"])
    return risk


def main(args=None):
    rclpy.init(args=args)
    node = UkcEstimatorNode()
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
