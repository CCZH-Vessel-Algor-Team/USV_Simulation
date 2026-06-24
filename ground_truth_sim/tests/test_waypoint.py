"""Unit tests for waypoint motion helpers."""

import math

import numpy as np
import pytest

from ground_truth_sim.waypoint import (
    WaypointTargetState,
    entity_waypoint_desired_velocity,
    parse_fixed_targets,
    point_to_polyline_distance,
    predict_waypoint_path,
    propagate_waypoint_target,
    propagate_waypoint_target_arc,
    should_advance_waypoint,
)


def _make_target(
    track_id: int = 1,
    x: float = 0.0,
    y: float = 0.0,
    speed: float = 1.0,
    waypoints=None,
    loop: bool = True,
) -> WaypointTargetState:
    wps = waypoints or [(0.0, 0.0), (10.0, 0.0)]
    theta = 0.0 if len(wps) < 2 else math.atan2(wps[1][1] - wps[0][1], wps[1][0] - wps[0][0])
    return WaypointTargetState(
        track_id=track_id,
        x=x,
        y=y,
        speed=speed,
        theta=theta,
        omega=0.0,
        size_w=2.0,
        size_l=5.0,
        size_h=2.0,
        is_dark_target=False,
        is_ais_matched=True,
        matched_mmsi=123,
        waypoints=wps,
        current_wp_idx=1 if len(wps) > 1 else 0,
        direction=1,
        loop=loop,
        waypoint_active=len(wps) > 1,
    )


def test_kinematic_propagate_moves_along_x():
    t = _make_target(x=0.0, y=0.0, speed=2.0, waypoints=[(0.0, 0.0), (10.0, 0.0)])
    propagate_waypoint_target(t, 0.5, 0.5)
    assert abs(t.x - 1.0) < 1e-6
    assert abs(t.y) < 1e-6
    assert abs(t.theta) < 1e-6


def test_arc_propagate_turns_in_place_at_endpoint():
    t = _make_target(x=10.0, y=0.0, speed=2.0, waypoints=[(0.0, 0.0), (10.0, 0.0)])
    t.current_wp_idx = 1
    t.theta = 0.0
    # Simulate arrival: advance to return leg toward x=0
    propagate_waypoint_target_arc(t, 0.01, 0.5, 0.22, 10.0, 10.0)
    assert t.direction == -1
    x_before = t.x
    for _ in range(20):
        propagate_waypoint_target_arc(t, 0.02, 0.5, 0.22, 10.0, 10.0)
    assert t.theta < -0.05
    assert abs(t.x - x_before) < 1.0


def test_arc_propagate_moves_along_straight_when_aligned():
    t = _make_target(x=0.0, y=0.0, speed=2.0, waypoints=[(0.0, 0.0), (10.0, 0.0)])
    propagate_waypoint_target_arc(t, 0.5, 0.5, 0.22, 10.0, 10.0)
    assert abs(t.x - 1.0) < 1e-3
    assert abs(t.y) < 1e-3


def test_arrival_switches_waypoint():
    t = _make_target(
        x=10.0,
        y=0.0,
        speed=2.0,
        waypoints=[(0.0, 0.0), (10.0, 0.0), (10.0, 10.0)],
    )
    t.current_wp_idx = 1
    propagate_waypoint_target(t, 0.01, 0.5)
    assert t.current_wp_idx == 2
    assert abs(t.theta - math.pi / 2) < 1e-4


def test_ping_pong_reverses_at_end():
    t = _make_target(
        x=9.8,
        y=10.0,
        speed=2.0,
        waypoints=[(0.0, 0.0), (10.0, 0.0), (10.0, 10.0)],
        loop=True,
    )
    t.current_wp_idx = 2
    t.direction = 1
    propagate_waypoint_target(t, 0.1, 0.5)
    assert t.direction == -1
    assert t.current_wp_idx == 1


def test_no_loop_stops_at_last_waypoint():
    t = _make_target(
        x=9.8,
        y=0.0,
        speed=2.0,
        waypoints=[(0.0, 0.0), (10.0, 0.0)],
        loop=False,
    )
    propagate_waypoint_target(t, 0.5, 0.5)
    assert not t.waypoint_active
    assert abs(t.x - 9.8) < 1e-6


def test_parse_fixed_targets_ok():
    rng = np.random.default_rng(0)
    entries = [
        {
            "track_id": 1,
            "speed": 3.0,
            "loop": True,
            "waypoints": [[0.0, 0.0], [5.0, 0.0]],
            "is_ais_matched": True,
            "matched_mmsi": 999,
        },
        {
            "track_id": 2,
            "speed": 4.0,
            "waypoints": [[1.0, 1.0], [2.0, 2.0], [3.0, 1.0]],
        },
    ]
    targets = parse_fixed_targets(entries, "", {"speed_min": 2.0}, rng)
    assert len(targets) == 2
    assert targets[0].track_id == 1
    assert targets[0].matched_mmsi == 999
    assert targets[1].x == 1.0 and targets[1].y == 1.0


def test_parse_duplicate_track_id_raises():
    rng = np.random.default_rng(0)
    entries = [
        {"track_id": 1, "speed": 1.0, "waypoints": [[0, 0], [1, 0]]},
        {"track_id": 1, "speed": 1.0, "waypoints": [[2, 0], [3, 0]]},
    ]
    with pytest.raises(ValueError, match="重复"):
        parse_fixed_targets(entries, "", {}, rng)


def test_parse_insufficient_waypoints_raises():
    rng = np.random.default_rng(0)
    entries = [{"track_id": 1, "speed": 1.0, "waypoints": [[0, 0]]}]
    with pytest.raises(ValueError, match="至少需要 2"):
        parse_fixed_targets(entries, "", {}, rng)


def test_predict_waypoint_path_non_empty():
    t = _make_target(x=0.0, y=0.0, speed=2.0, waypoints=[(0.0, 0.0), (20.0, 0.0)])
    pts = predict_waypoint_path(t, 5.0, 0.25)
    assert len(pts) > 0
    assert pts[-1].x <= 20.0 + 1e-6


def test_point_to_polyline_distance():
    wps = [(0.0, 0.0), (10.0, 0.0)]
    assert point_to_polyline_distance(5.0, 3.0, wps) == pytest.approx(3.0)
    assert point_to_polyline_distance(0.0, 0.0, wps) == pytest.approx(0.0)


def test_should_advance_on_overshoot():
    t = _make_target(
        x=105.0,
        y=0.0,
        speed=3.0,
        waypoints=[(0.0, 0.0), (100.0, 0.0)],
        loop=True,
    )
    t.current_wp_idx = 1
    t.direction = 1
    assert should_advance_waypoint(t, 105.0, 0.0, 0.5)


def test_entity_waypoint_desired_velocity_reverses_after_overshoot():
    t = _make_target(
        x=105.0,
        y=0.0,
        speed=3.0,
        waypoints=[(0.0, 0.0), (100.0, 0.0)],
        loop=True,
    )
    t.current_wp_idx = 1
    t.direction = 1
    vx, vy = entity_waypoint_desired_velocity(t, 105.0, 0.0, 0.5)
    assert t.direction == -1
    assert t.current_wp_idx == 0
    assert vx < 0.0
    assert abs(vy) < 1e-6
