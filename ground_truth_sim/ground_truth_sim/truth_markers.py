"""RViz MarkerArray helpers shared by ground truth nodes."""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Protocol, Set

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from ground_truth_sim.ctrv import TargetState, predict_future_path
from ground_truth_sim.waypoint import (
    WaypointTargetState,
    predict_waypoint_path,
    predict_waypoint_path_arc,
)


class _TrackLike(Protocol):
    track_id: int
    x: float
    y: float
    size_w: float
    size_l: float
    size_h: float
    is_ais_matched: bool
    history: List[Point]


@dataclass
class MarkerPublishConfig:
    frame_id: str
    motion_mode: str = "waypoint"
    waypoint_kinematics: str = "arc"
    prediction_horizon: float = 5.0
    prediction_dt: float = 0.25
    waypoint_arrival_threshold: float = 0.5
    waypoint_omega_limit: float = 0.22
    waypoint_turn_radius_min: float = 10.0
    waypoint_align_threshold_deg: float = 10.0


def build_track_array_from_states(
    frame_id: str,
    stamp,
    targets: List[TargetState],
    *,
    vx_override: Optional[dict[int, tuple[float, float]]] = None,
    source_model_names: Optional[dict[int, str]] = None,
) -> "GlobalTrackArray":
    from usv_interfaces.msg import GlobalTrack, GlobalTrackArray

    msg = GlobalTrackArray()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    vx_map = vx_override or {}
    model_map = source_model_names or {}
    for target in targets:
        track = GlobalTrack()
        track.track_id = int(target.track_id)
        track.x = float(target.x)
        track.y = float(target.y)
        if int(target.track_id) in vx_map:
            track.v_x, track.v_y = vx_map[int(target.track_id)]
        else:
            track.v_x = float(target.v_x)
            track.v_y = float(target.v_y)
        track.size_w = float(target.size_w)
        track.size_l = float(target.size_l)
        track.size_h = float(target.size_h)
        track.covariance = [0.0] * 16
        track.is_dark_target = bool(target.is_dark_target)
        track.is_ais_matched = bool(target.is_ais_matched)
        track.matched_mmsi = int(target.matched_mmsi)
        track.source_model_name = str(model_map.get(int(target.track_id), ""))
        msg.tracks.append(track)
    return msg


def make_delete_markers_for_track(frame_id: str, track_id: int, stamp) -> List[Marker]:
    out: List[Marker] = []
    for ns, mid in (
        ("target_pose", track_id),
        ("target_path", track_id + 1000),
        ("target_history", track_id + 2000),
    ):
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = stamp
        m.ns = ns
        m.id = mid
        m.action = Marker.DELETE
        out.append(m)
    return out


def make_position_marker(frame_id: str, target: _TrackLike, stamp) -> Marker:
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = "target_pose"
    marker.id = int(target.track_id)
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = float(target.x)
    marker.pose.position.y = float(target.y)
    marker.pose.position.z = float(target.size_h) * 0.5
    marker.pose.orientation.w = 1.0
    marker.scale.x = max(float(target.size_l), 2.0)
    marker.scale.y = max(float(target.size_w), 2.0)
    marker.scale.z = max(float(target.size_h), 1.5)
    if target.is_ais_matched:
        marker.color.r = 0.1
        marker.color.g = 0.8
        marker.color.b = 0.2
    else:
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.0
    marker.color.a = 0.9
    marker.lifetime.sec = 0
    marker.lifetime.nanosec = 0
    return marker


def make_path_marker(frame_id: str, target: TargetState, stamp, cfg: MarkerPublishConfig) -> Marker:
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = "target_path"
    marker.id = int(target.track_id) + 1000
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = 0.8
    marker.color.a = 0.85
    marker.color.r = 0.2
    marker.color.g = 0.6
    marker.color.b = 1.0 if target.is_ais_matched else 0.2
    if cfg.motion_mode == "waypoint" and isinstance(target, WaypointTargetState):
        if cfg.waypoint_kinematics == "arc":
            marker.points = predict_waypoint_path_arc(
                target,
                cfg.prediction_horizon,
                cfg.prediction_dt,
                cfg.waypoint_arrival_threshold,
                cfg.waypoint_omega_limit,
                cfg.waypoint_turn_radius_min,
                cfg.waypoint_align_threshold_deg,
            )
        else:
            marker.points = predict_waypoint_path(
                target, cfg.prediction_horizon, cfg.prediction_dt
            )
    else:
        marker.points = predict_future_path(
            target, cfg.prediction_horizon, cfg.prediction_dt
        )
    return marker


def make_history_marker(frame_id: str, target: _TrackLike, stamp) -> Marker:
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = "target_history"
    marker.id = int(target.track_id) + 2000
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = 0.4
    marker.color.a = 0.9
    marker.color.r = 0.7
    marker.color.g = 0.7
    marker.color.b = 0.7
    marker.points = list(target.history)
    return marker


def build_marker_array(
    targets: List[TargetState],
    stamp,
    cfg: MarkerPublishConfig,
    last_published_ids: Set[int],
) -> tuple[MarkerArray, Set[int]]:
    current_ids = {int(t.track_id) for t in targets}
    removed = last_published_ids - current_ids
    marker_list: List[Marker] = []
    for tid in removed:
        marker_list.extend(make_delete_markers_for_track(cfg.frame_id, tid, stamp))
    for target in targets:
        marker_list.append(make_position_marker(cfg.frame_id, target, stamp))
        marker_list.append(make_path_marker(cfg.frame_id, target, stamp, cfg))
        marker_list.append(make_history_marker(cfg.frame_id, target, stamp))
    markers = MarkerArray()
    markers.markers = marker_list
    return markers, current_ids
