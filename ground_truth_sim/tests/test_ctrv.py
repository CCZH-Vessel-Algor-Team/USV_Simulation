"""Unit tests for CTRV helpers."""

import math

import numpy as np

from ground_truth_sim.ctrv import (
    TargetState,
    ctrv_step,
    propagate_target,
    sample_annulus_radius,
    wrap_angle,
)


def test_wrap_angle():
    w = wrap_angle(3.0)
    assert -math.pi - 1e-9 <= w <= math.pi + 1e-9
    assert abs(wrap_angle(0.0)) < 1e-9


def test_ctrv_straight_line():
    x, y, th = ctrv_step(0.0, 0.0, 1.0, 0.0, 0.0, 0.1)
    assert abs(x - 0.1) < 1e-6 and abs(y) < 1e-6


def test_sample_annulus_in_annulus():
    rng = np.random.default_rng(0)
    for _ in range(20):
        r = sample_annulus_radius(rng, 10.0, 20.0)
        assert 10.0 <= r <= 20.0


def test_propagate_changes_position():
    rng = np.random.default_rng(1)
    t = TargetState(
        track_id=1,
        x=0.0,
        y=0.0,
        speed=2.0,
        theta=0.0,
        omega=0.0,
        size_w=2.0,
        size_l=5.0,
        size_h=2.0,
        is_dark_target=False,
        is_ais_matched=True,
        matched_mmsi=123,
    )
    x0, y0 = t.x, t.y
    propagate_target(t, 0.02, 0.001, 0.99, 0.1, rng)
    assert (t.x != x0) or (t.y != y0) or (t.theta != 0.0)
