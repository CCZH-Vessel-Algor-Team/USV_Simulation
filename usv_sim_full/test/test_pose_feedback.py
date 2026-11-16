"""Regress the nanosecond feedback path used by current dynamic ships."""

import math

import pytest

from usv_sim_full.pose_feedback import PoseTracker, is_feedback_fresh_ns, yaw_from_quaternion


@pytest.mark.parametrize('yaw', [0.0, math.pi / 2, -math.pi / 2, math.pi])
def test_yaw_matches_arrow(yaw):
    assert yaw_from_quaternion(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)) == pytest.approx(yaw)


@pytest.mark.parametrize('stamp, now, fresh', [
    (None, 10, False), (10, 9, False), (10, 1_000_000_010, True),
    (10, 1_000_000_011, False),
])
def test_feedback_freshness_bounds(stamp, now, fresh):
    assert is_feedback_fresh_ns(stamp, now, 1.0) is fresh


def test_velocity_uses_exact_observation_time_and_ignores_reordered_samples():
    tracker = PoseTracker(tau_sec=0.0)
    stamp = 1_700_000_000_000_000_000
    tracker.update_ns(0.0, 0.0, stamp)
    assert not tracker.ready
    assert tracker.update_ns(0.3, -0.2, stamp + 100_000_000) == pytest.approx((3.0, -2.0))
    assert tracker.ready
    for old_stamp in [stamp, stamp + 100_000_000]:
        assert tracker.update_ns(99.0, 99.0, old_stamp) == pytest.approx((3.0, -2.0))
    assert tracker.stamp_ns == stamp + 100_000_000


def test_timeout_and_clock_reset_require_new_velocity_history():
    tracker = PoseTracker(tau_sec=0.0)
    tracker.update_ns(0.0, 0.0, 0)
    tracker.update_ns(1.0, 0.0, 100_000_000)
    tracker.update_ns(99.0, 0.0, 2_000_000_000, timeout_sec=1.0)
    assert not tracker.ready and tracker.velocity == (0.0, 0.0)
    tracker.reset()
    assert tracker.stamp_ns is None and not tracker.ready
    tracker.update_ns(0.0, 0.0, 1)
    assert not tracker.ready
