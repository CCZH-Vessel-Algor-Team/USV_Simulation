"""Unit tests for entity pose velocity estimation."""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

_PKG_ROOT = Path(__file__).resolve().parents[1]
if str(_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT))

from ground_truth_sim.entity_velocity import (  # noqa: E402
    EntityVelocityEstimator,
    velocity_heading_deg,
)


class TestEntityVelocity(unittest.TestCase):
    def test_stationary_returns_zero(self) -> None:
        est = EntityVelocityEstimator(tau_sec=0.2, speed_eps=0.05)
        est.update(1.0, 2.0, 0)
        vx, vy = est.update(1.0, 2.0, 50_000_000)
        self.assertAlmostEqual(vx, 0.0, places=5)
        self.assertAlmostEqual(vy, 0.0, places=5)

    def test_constant_velocity_converges(self) -> None:
        est = EntityVelocityEstimator(tau_sec=0.1, speed_eps=0.01)
        t = 0
        for i in range(30):
            t += 20_000_000
            vx, vy = est.update(1.0 + 0.02 * (i + 1), 0.0, t)
        self.assertAlmostEqual(vx, 1.0, delta=0.15)
        self.assertAlmostEqual(vy, 0.0, delta=0.05)

    def test_velocity_heading(self) -> None:
        h = velocity_heading_deg(1.0, 1.0)
        assert h is not None
        self.assertAlmostEqual(h, 45.0, places=1)
        self.assertIsNone(velocity_heading_deg(0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
