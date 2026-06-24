"""Fixed-route waypoint motion for ground_truth_node."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from typing import Any, List, Optional, Sequence, Tuple

import numpy as np
from geometry_msgs.msg import Point

from ground_truth_sim.arc_follow import plan_arc_follow
from ground_truth_sim.ctrv import TargetState, ctrv_step, wrap_angle


@dataclass
class WaypointTargetState(TargetState):
    """TargetState with ping-pong waypoint routing metadata."""

    waypoints: List[Tuple[float, float]] = field(default_factory=list)
    current_wp_idx: int = 1
    direction: int = 1
    loop: bool = True
    waypoint_active: bool = True


def _parse_waypoints(raw: Any, label: str) -> List[Tuple[float, float]]:
    if not isinstance(raw, (list, tuple)) or len(raw) < 2:
        raise ValueError(f"{label}: waypoints 至少需要 2 个点")
    out: List[Tuple[float, float]] = []
    for i, wp in enumerate(raw):
        if not isinstance(wp, (list, tuple)) or len(wp) < 2:
            raise ValueError(f"{label}: waypoints[{i}] 必须为 [x, y]")
        out.append((float(wp[0]), float(wp[1])))
    return out


def _as_bool(value: Any, default: bool) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    return str(value).lower() in ("true", "1", "yes")


def _resolve_fixed_targets_raw(raw_param: Any, json_param: str) -> list:
    if isinstance(raw_param, list) and raw_param:
        return raw_param
    text = str(json_param or "").strip()
    if text:
        parsed = json.loads(text)
        if not isinstance(parsed, list):
            raise ValueError("fixed_targets_json 必须为 JSON 数组")
        return parsed
    if isinstance(raw_param, list):
        return raw_param
    return []


def parse_fixed_targets(
    raw_param: Any,
    json_param: str,
    defaults: dict,
    rng: np.random.Generator,
) -> List[WaypointTargetState]:
    """Parse fixed_targets parameter list into WaypointTargetState instances."""
    entries = _resolve_fixed_targets_raw(raw_param, json_param)
    if not entries:
        raise ValueError("motion_mode=waypoint 时 fixed_targets 不能为空")

    seen_ids: set[int] = set()
    targets: List[WaypointTargetState] = []

    for idx, entry in enumerate(entries):
        label = f"fixed_targets[{idx}]"
        if not isinstance(entry, dict):
            raise ValueError(f"{label}: 必须为对象")

        if "track_id" not in entry:
            raise ValueError(f"{label}: 缺少 track_id")
        track_id = int(entry["track_id"])
        if track_id in seen_ids:
            raise ValueError(f"{label}: track_id={track_id} 重复")
        seen_ids.add(track_id)

        waypoints = _parse_waypoints(entry.get("waypoints"), label)
        speed = float(entry.get("speed", defaults.get("speed_min", 2.0)))
        if speed <= 0.0:
            raise ValueError(f"{label}: speed 必须 > 0")

        loop = _as_bool(entry.get("loop"), True)
        x0, y0 = waypoints[0]
        theta = _heading_toward(x0, y0, waypoints[1][0], waypoints[1][1])

        size_w = float(entry.get("size_width", defaults.get("size_width_min", 2.0)))
        size_l = float(entry.get("size_length", defaults.get("size_length_min", 5.0)))
        size_h = float(entry.get("size_height", defaults.get("size_height_min", 2.0)))

        if "is_ais_matched" in entry:
            is_ais_matched = _as_bool(entry["is_ais_matched"], False)
        else:
            prob = float(defaults.get("ais_match_probability", 0.4))
            is_ais_matched = bool(rng.random() < prob)

        if "matched_mmsi" in entry:
            matched_mmsi = int(entry["matched_mmsi"])
        else:
            matched_mmsi = (
                int(rng.integers(100_000_000, 999_999_999)) if is_ais_matched else 0
            )

        target = WaypointTargetState(
            track_id=track_id,
            x=x0,
            y=y0,
            speed=speed,
            theta=theta,
            omega=0.0,
            size_w=size_w,
            size_l=size_l,
            size_h=size_h,
            is_dark_target=not is_ais_matched,
            is_ais_matched=is_ais_matched,
            matched_mmsi=matched_mmsi,
            waypoints=waypoints,
            current_wp_idx=1 if len(waypoints) > 1 else 0,
            direction=1,
            loop=loop,
            waypoint_active=len(waypoints) > 1,
        )
        target.history.append(Point(x=float(x0), y=float(y0), z=0.0))
        targets.append(target)

    return targets


def _advance_waypoint_index(target: WaypointTargetState) -> None:
    """Ping-pong waypoint index update (same logic as scenario_manager_node)."""
    wps = target.waypoints
    if len(wps) <= 1:
        return

    if target.loop:
        target.current_wp_idx += target.direction
        if target.current_wp_idx >= len(wps):
            target.direction = -1
            target.current_wp_idx = len(wps) - 2
            if target.current_wp_idx < 0:
                target.current_wp_idx = 0
        elif target.current_wp_idx < 0:
            target.direction = 1
            target.current_wp_idx = 1
            if target.current_wp_idx >= len(wps):
                target.current_wp_idx = 0
    else:
        target.current_wp_idx += 1
        if target.current_wp_idx >= len(wps):
            target.waypoint_active = False


def _heading_toward(x0: float, y0: float, x1: float, y1: float) -> float:
    dx = x1 - x0
    dy = y1 - y0
    if abs(dx) + abs(dy) < 1e-9:
        return 0.0
    return math.atan2(dy, dx)


def _segment_prev_index(target: WaypointTargetState) -> Optional[int]:
    prev = target.current_wp_idx - target.direction
    if 0 <= prev < len(target.waypoints):
        return prev
    return None


def _passed_waypoint_along_segment(
    x: float,
    y: float,
    from_wp: Tuple[float, float],
    to_wp: Tuple[float, float],
    *,
    margin_m: float,
) -> bool:
    """沿 from→to 线段方向，判断 (x,y) 是否已越过 to 航点（含 margin 容差）。"""
    seg_dx = to_wp[0] - from_wp[0]
    seg_dy = to_wp[1] - from_wp[1]
    seg_len = math.hypot(seg_dx, seg_dy)
    if seg_len < 1e-6:
        return False
    along = ((x - from_wp[0]) * seg_dx + (y - from_wp[1]) * seg_dy) / seg_len
    return along >= max(0.0, seg_len - max(0.0, margin_m))


def should_advance_waypoint(
    target: WaypointTargetState,
    x: float,
    y: float,
    arrival_threshold: float,
) -> bool:
    """到达阈值内，或沿当前航段 overshoot 过点，均应切换下一航点。"""
    if not target.waypoint_active or len(target.waypoints) <= 1:
        return False
    wps = target.waypoints
    idx = target.current_wp_idx
    tx, ty = wps[idx]
    if math.hypot(tx - x, ty - y) < arrival_threshold:
        return True
    prev_idx = _segment_prev_index(target)
    if prev_idx is not None:
        return _passed_waypoint_along_segment(
            x,
            y,
            wps[prev_idx],
            wps[idx],
            margin_m=arrival_threshold,
        )
    return False


def entity_waypoint_desired_velocity(
    target: WaypointTargetState,
    x: float,
    y: float,
    arrival_threshold: float,
) -> Tuple[float, float]:
    """Gazebo 实体闭环：按实测 pose 推进航点索引，返回指向下一个航点的期望 map 速度。"""
    if not target.waypoint_active or len(target.waypoints) <= 1:
        return 0.0, 0.0

    if should_advance_waypoint(target, x, y, arrival_threshold):
        _advance_waypoint_index(target)

    if not target.waypoint_active:
        return 0.0, 0.0

    tx, ty = target.waypoints[target.current_wp_idx]
    dx, dy = tx - x, ty - y
    dist = math.hypot(dx, dy)
    if dist < 1e-6:
        return 0.0, 0.0
    return target.speed * dx / dist, target.speed * dy / dist


def propagate_waypoint_target(
    target: WaypointTargetState,
    dt: float,
    arrival_threshold: float,
) -> None:
    if not target.waypoint_active or len(target.waypoints) <= 1:
        target.omega = 0.0
        return

    tx, ty = target.waypoints[target.current_wp_idx]
    dx = tx - target.x
    dy = ty - target.y
    dist = math.hypot(dx, dy)

    if dist < arrival_threshold:
        _advance_waypoint_index(target)
        if not target.waypoint_active:
            target.omega = 0.0
            return
        tx, ty = target.waypoints[target.current_wp_idx]
        dx = tx - target.x
        dy = ty - target.y
        dist = math.hypot(dx, dy)

    if dist > 0.0:
        vx = (dx / dist) * target.speed
        vy = (dy / dist) * target.speed
        target.theta = wrap_angle(math.atan2(vy, vx))
    else:
        vx = 0.0
        vy = 0.0

    target.omega = 0.0
    target.x += vx * dt
    target.y += vy * dt


def propagate_waypoint_target_arc(
    target: WaypointTargetState,
    dt: float,
    arrival_threshold: float,
    omega_limit: float,
    turn_radius_min_m: float,
    align_threshold_deg: float,
) -> None:
    """Physical waypoint follow: turn in place / small arc at endpoints (matches Gazebo cmd_vel)."""
    if not target.waypoint_active or len(target.waypoints) <= 1:
        target.omega = 0.0
        return

    tx, ty = target.waypoints[target.current_wp_idx]
    dx = tx - target.x
    dy = ty - target.y
    dist = math.hypot(dx, dy)

    if dist < arrival_threshold:
        _advance_waypoint_index(target)
        if not target.waypoint_active:
            target.omega = 0.0
            return
        tx, ty = target.waypoints[target.current_wp_idx]
        dx = tx - target.x
        dy = ty - target.y
        dist = math.hypot(dx, dy)

    if dist > 0.0:
        desired_yaw = math.atan2(dy, dx)
    else:
        desired_yaw = target.theta

    forward_speed, omega = plan_arc_follow(
        target.speed,
        target.theta,
        desired_yaw,
        omega_limit=omega_limit,
        turn_radius_min_m=turn_radius_min_m,
        align_threshold_deg=align_threshold_deg,
    )
    align_rad = math.radians(max(1.0, align_threshold_deg))
    heading_err = wrap_angle(desired_yaw - target.theta)
    if abs(heading_err) <= align_rad:
        target.theta = desired_yaw
        target.omega = 0.0
        forward_speed = target.speed
        omega = 0.0
    else:
        target.omega = omega
    x, y, theta = ctrv_step(
        target.x, target.y, forward_speed, target.theta, omega, dt
    )
    target.x = x
    target.y = y
    target.theta = theta


def _copy_waypoint_target(target: WaypointTargetState) -> WaypointTargetState:
    return WaypointTargetState(
        track_id=target.track_id,
        x=target.x,
        y=target.y,
        speed=target.speed,
        theta=target.theta,
        omega=target.omega,
        size_w=target.size_w,
        size_l=target.size_l,
        size_h=target.size_h,
        is_dark_target=target.is_dark_target,
        is_ais_matched=target.is_ais_matched,
        matched_mmsi=target.matched_mmsi,
        waypoints=list(target.waypoints),
        current_wp_idx=target.current_wp_idx,
        direction=target.direction,
        loop=target.loop,
        waypoint_active=target.waypoint_active,
    )


def predict_waypoint_path_arc(
    target: WaypointTargetState,
    prediction_horizon: float,
    prediction_dt: float,
    arrival_threshold: float,
    omega_limit: float,
    turn_radius_min_m: float,
    align_threshold_deg: float,
) -> List[Point]:
    """Preview path using the same arc kinematics as propagate_waypoint_target_arc."""
    if prediction_dt <= 0.0 or prediction_horizon <= 0.0 or target.speed <= 0.0:
        return []

    sim = _copy_waypoint_target(target)
    points: List[Point] = []
    elapsed = 0.0
    max_steps = int(prediction_horizon / prediction_dt) + 1
    for _ in range(max_steps):
        if elapsed >= prediction_horizon:
            break
        propagate_waypoint_target_arc(
            sim,
            prediction_dt,
            arrival_threshold,
            omega_limit,
            turn_radius_min_m,
            align_threshold_deg,
        )
        points.append(Point(x=float(sim.x), y=float(sim.y), z=0.0))
        elapsed += prediction_dt
    return points


def _collect_route_segments(
    target: WaypointTargetState,
) -> List[Tuple[float, float, float, float]]:
    """Segments from current position to remaining waypoints along the active route."""
    if not target.waypoints:
        return []

    segments: List[Tuple[float, float, float, float]] = []
    x, y = target.x, target.y

    idx = target.current_wp_idx
    direction = target.direction
    wps = target.waypoints
    loop = target.loop
    active = target.waypoint_active

    def append_segment(to_idx: int) -> None:
        nonlocal x, y
        tx, ty = wps[to_idx]
        segments.append((x, y, tx, ty))
        x, y = tx, ty

    if not active:
        return segments

    max_steps = len(wps) * 4 + 2
    steps = 0
    while steps < max_steps:
        append_segment(idx)
        steps += 1
        if not loop:
            idx += 1
            if idx >= len(wps):
                break
            continue

        idx += direction
        if idx >= len(wps):
            direction = -1
            idx = len(wps) - 2
            if idx < 0:
                break
        elif idx < 0:
            direction = 1
            idx = 1
            if idx >= len(wps):
                break

    return segments


def predict_waypoint_path(
    target: WaypointTargetState,
    prediction_horizon: float,
    prediction_dt: float,
) -> List[Point]:
    """Preview path along remaining waypoint polyline."""
    if prediction_dt <= 0.0 or prediction_horizon <= 0.0 or target.speed <= 0.0:
        return []

    points: List[Point] = []
    cx, cy = target.x, target.y
    step_dist = target.speed * prediction_dt
    max_dist = target.speed * prediction_horizon
    walked = 0.0

    for _x0, _y0, x1, y1 in _collect_route_segments(target):
        while walked < max_dist:
            dx = x1 - cx
            dy = y1 - cy
            seg_left = math.hypot(dx, dy)
            if seg_left < 1e-9:
                cx, cy = x1, y1
                break
            step = min(step_dist, seg_left, max_dist - walked)
            cx += (dx / seg_left) * step
            cy += (dy / seg_left) * step
            walked += step
            points.append(Point(x=float(cx), y=float(cy), z=0.0))
            if math.hypot(x1 - cx, y1 - cy) < 1e-6:
                cx, cy = x1, y1
                break

    return points


def point_to_polyline_distance(
    x: float,
    y: float,
    waypoints: Sequence[Tuple[float, float]],
) -> float:
    """Minimum distance from (x,y) to the waypoint polyline."""
    if not waypoints:
        return float("inf")
    if len(waypoints) == 1:
        wx, wy = waypoints[0]
        return math.hypot(x - wx, y - wy)

    best = float("inf")
    for i in range(len(waypoints) - 1):
        x0, y0 = waypoints[i]
        x1, y1 = waypoints[i + 1]
        best = min(best, _point_to_segment_distance(x, y, x0, y0, x1, y1))
    return best


def _point_to_segment_distance(
    px: float, py: float, x0: float, y0: float, x1: float, y1: float
) -> float:
    dx = x1 - x0
    dy = y1 - y0
    if dx * dx + dy * dy < 1e-12:
        return math.hypot(px - x0, py - y0)
    t = ((px - x0) * dx + (py - y0) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    proj_x = x0 + t * dx
    proj_y = y0 + t * dy
    return math.hypot(px - proj_x, py - proj_y)
