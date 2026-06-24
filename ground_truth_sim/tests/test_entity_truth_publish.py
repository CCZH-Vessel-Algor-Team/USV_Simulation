"""Unit tests for truth message building from entity states."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

_PKG_ROOT = Path(__file__).resolve().parents[1]
if str(_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT))

from builtin_interfaces.msg import Time

from ground_truth_sim.ground_truth_gazebo_models_node import (  # noqa: E402
    _parse_model_poses_from_pose_info,
)
from ground_truth_sim.truth_markers import build_track_array_from_states  # noqa: E402
from ground_truth_sim.waypoint import WaypointTargetState  # noqa: E402


def _target(tid: int, x: float, y: float) -> WaypointTargetState:
    return WaypointTargetState(
        track_id=tid,
        x=x,
        y=y,
        speed=3.0,
        theta=0.0,
        omega=0.0,
        size_w=3.6,
        size_l=10.0,
        size_h=2.0,
        is_dark_target=False,
        is_ais_matched=True,
        matched_mmsi=123,
        waypoints=[(x, y), (x + 10.0, y)],
        current_wp_idx=1,
        direction=1,
        loop=True,
        waypoint_active=True,
    )


class TestEntityTruthPublish(unittest.TestCase):
    def test_build_track_array_velocity_override(self) -> None:
        stamp = Time(sec=1, nanosec=0)
        targets = [_target(1, 5.0, 6.0)]
        msg = build_track_array_from_states(
            "map",
            stamp,
            targets,
            vx_override={1: (1.5, -0.5)},
            source_model_names={1: "gt_ctrv_1"},
        )
        self.assertEqual(len(msg.tracks), 1)
        tr = msg.tracks[0]
        self.assertAlmostEqual(tr.x, 5.0)
        self.assertAlmostEqual(tr.y, 6.0)
        self.assertAlmostEqual(tr.v_x, 1.5)
        self.assertAlmostEqual(tr.v_y, -0.5)
        self.assertEqual(tr.source_model_name, "gt_ctrv_1")

    def test_parse_pose_info_omitted_zero_quat_components(self) -> None:
        text = """
pose {
  name: "gt_ctrv_1"
  position { x: 40 y: -20 z: 1 }
  orientation { z: 0.38268343236508984 w: 0.92387953251128674 }
}
"""
        poses = _parse_model_poses_from_pose_info(text)
        self.assertIn("gt_ctrv_1", poses)
        pose = poses["gt_ctrv_1"]
        self.assertAlmostEqual(pose.x, 40.0)
        self.assertAlmostEqual(pose.y, -20.0)
        self.assertAlmostEqual(pose.qx, 0.0)
        self.assertAlmostEqual(pose.qy, 0.0)


if __name__ == "__main__":
    unittest.main()
