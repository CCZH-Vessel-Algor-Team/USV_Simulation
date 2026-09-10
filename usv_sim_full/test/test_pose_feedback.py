"""pose_feedback 纯函数与轨迹滤波测试。"""

import math

import pytest

from usv_sim_full.pose_feedback import (
    PoseTracker,
    fresh_sample,
    is_feedback_fresh,
    yaw_from_quaternion,
)


def _yaw_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def test_yaw_from_quaternion_identity_is_zero():
    assert yaw_from_quaternion(0.0, 0.0, 0.0, 1.0) == pytest.approx(0.0)


def test_yaw_from_quaternion_recovers_heading():
    x, y, z, w = _yaw_quaternion(math.pi / 2.0)
    assert yaw_from_quaternion(x, y, z, w) == pytest.approx(math.pi / 2.0)


def test_pose_tracker_starts_at_zero_velocity():
    tracker = PoseTracker()
    assert tracker.update(0.0, 0.0, 0.0) == (0.0, 0.0)


def test_pose_tracker_converges_to_constant_velocity():
    tracker = PoseTracker(tau_sec=0.3)
    stamp = 0.0
    vx, vy = 0.0, 0.0
    for i in range(60):
        vx, vy = tracker.update(0.3 * i, 0.0, stamp)
        stamp += 0.1
    assert vx == pytest.approx(3.0, abs=0.05)
    assert vy == pytest.approx(0.0, abs=1e-9)


def test_pose_tracker_ignores_non_advancing_stamp():
    tracker = PoseTracker()
    tracker.update(0.0, 0.0, 1.0)
    tracker.update(5.0, 5.0, 1.0)
    assert tracker.velocity == (0.0, 0.0)


def test_feedback_is_fresh_within_timeout():
    assert is_feedback_fresh(10.0, 10.5, timeout_sec=1.0)


def test_feedback_is_stale_after_timeout():
    assert not is_feedback_fresh(10.0, 12.0, timeout_sec=1.0)


def test_missing_feedback_is_not_fresh():
    assert not is_feedback_fresh(None, 10.0, timeout_sec=1.0)


def test_fresh_sample_returns_sample_when_fresh():
    sample = (1.0, 2.0, 0.5, 10.0)
    assert fresh_sample(sample, 10.5, timeout_sec=1.0) == sample


def test_fresh_sample_returns_none_when_stale():
    sample = (1.0, 2.0, 0.5, 10.0)
    assert fresh_sample(sample, 12.0, timeout_sec=1.0) is None


def test_fresh_sample_returns_none_when_missing():
    assert fresh_sample(None, 10.0, timeout_sec=1.0) is None
