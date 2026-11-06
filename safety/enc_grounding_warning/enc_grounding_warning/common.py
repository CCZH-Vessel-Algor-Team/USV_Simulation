"""Shared helpers for grounding warning nodes."""

from __future__ import annotations

import math
import os

import yaml
from rclpy.duration import Duration
from rclpy.time import Time


# 与历史行为一致：未配置时全部默认开启发布
_DEFAULT_GRID_PUBLISH = {
    "depth_grid": True,
    "grounding_risk_grid": True,
    "route_depth_grid": True,
    "route_depth_markers": True,
    "current_risk_marker": True,
    "grounding_markers": True,
}


def load_full_params(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def load_ukc_params(path: str) -> dict:
    data = load_full_params(path)
    return data.get("ukc", data)


def load_grid_publish_flags(path: str) -> dict:
    """Load ``grid_publish`` switches from the same params yaml.

    Prefer a top-level ``grid_publish:`` block (recommended). Nested
    ``ukc.grid_publish`` is also accepted. Missing keys keep historical
    always-on defaults.
    """
    data = load_full_params(path)
    section = data.get("grid_publish")
    if section is None and isinstance(data.get("ukc"), dict):
        section = data["ukc"].get("grid_publish")
    out = dict(_DEFAULT_GRID_PUBLISH)
    if isinstance(section, dict):
        for key in _DEFAULT_GRID_PUBLISH:
            if key in section:
                out[key] = bool(section[key])
    return out


def default_config_path(filename: str) -> str:
    from ament_index_python.packages import get_package_share_directory

    return os.path.join(
        get_package_share_directory("enc_grounding_warning"), "config", filename
    )


def lookup_map_pose(
    tf_buffer,
    robot_base_frame: str,
    timeout_s: float = 1.0,
    fallback=None,
):
    """Return (x, y, yaw) of robot in the map frame.

    ``fallback`` is used when the map TF is unavailable; in the default
    simulation the map->odom transform is an identity, so VesselState pose
    (odom frame) can be used directly.
    """
    try:
        trans = tf_buffer.lookup_transform(
            "map",
            robot_base_frame,
            Time(),
            timeout=Duration(seconds=timeout_s),
        )
        t = trans.transform.translation
        q = trans.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        return float(t.x), float(t.y), yaw
    except Exception:
        if fallback is not None:
            return fallback
        raise


def path_to_points(path_msg):
    return [
        (float(p.pose.position.x), float(p.pose.position.y))
        for p in path_msg.poses
    ]
