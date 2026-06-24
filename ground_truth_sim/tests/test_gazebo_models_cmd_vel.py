"""Unit tests for cmd_vel helpers in ground_truth_gazebo_models_node."""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

# Allow importing the node module without ROS workspace install.
_PKG_ROOT = Path(__file__).resolve().parents[1]
if str(_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT))

from ground_truth_sim.arc_follow import (
    clamp_omega,
    compute_arc_follow_cmd_vel,
    plan_arc_follow,
)
from ground_truth_sim.ground_truth_gazebo_models_node import (  # noqa: E402
    ModelWorldPose,
    _build_box_sdf,
    _quat_to_roll_pitch,
    _velocity_control_plugin_xml,
    attitude_violation_reason,
    compute_omega,
    normalize_angle,
    world_velocity_to_body_twist,
    yaw_from_velocity,
)


class TestCmdVelHelpers(unittest.TestCase):
    def test_normalize_angle(self) -> None:
        self.assertAlmostEqual(normalize_angle(3.5 * math.pi), -0.5 * math.pi, places=5)

    def test_yaw_from_velocity_east(self) -> None:
        self.assertAlmostEqual(yaw_from_velocity(5.0, 0.0, 0.0), 0.0, places=5)

    def test_yaw_from_velocity_north(self) -> None:
        self.assertAlmostEqual(yaw_from_velocity(0.0, 3.0, 0.0), 0.5 * math.pi, places=5)

    def test_yaw_from_velocity_low_speed_keeps_last(self) -> None:
        self.assertAlmostEqual(yaw_from_velocity(0.0, 0.0, 1.2), 1.2, places=5)

    def test_world_velocity_to_body_east(self) -> None:
        twist = world_velocity_to_body_twist(5.0, 0.0, 0.0, 0.0)
        self.assertAlmostEqual(twist.linear.x, 5.0, places=5)
        self.assertAlmostEqual(twist.linear.y, 0.0, places=5)
        self.assertAlmostEqual(twist.angular.z, 0.0, places=5)

    def test_world_velocity_to_body_northeast(self) -> None:
        yaw = 0.25 * math.pi
        vx = 3.0 * math.cos(yaw)
        vy = 3.0 * math.sin(yaw)
        twist = world_velocity_to_body_twist(vx, vy, yaw, 0.1)
        self.assertAlmostEqual(twist.linear.x, 3.0, places=4)
        self.assertAlmostEqual(twist.linear.y, 0.0, places=4)
        self.assertAlmostEqual(twist.angular.z, 0.1, places=4)

    def test_compute_omega_quarter_turn(self) -> None:
        omega = compute_omega(0.5 * math.pi, 0.0, 0.02)
        self.assertAlmostEqual(omega, 0.5 * math.pi / 0.02, places=3)

    def test_clamp_omega(self) -> None:
        self.assertAlmostEqual(clamp_omega(5.0, 0.12), 0.12, places=5)
        self.assertAlmostEqual(clamp_omega(-5.0, 0.12), -0.12, places=5)
        self.assertAlmostEqual(clamp_omega(0.05, 0.12), 0.05, places=5)

    def test_arc_follow_aligned_east(self) -> None:
        twist = compute_arc_follow_cmd_vel(
            4.0, 0.0, 0.0, omega_limit=0.12, turn_radius_min_m=25.0, align_threshold_deg=12.0
        )
        self.assertAlmostEqual(twist.linear.x, 4.0, places=5)
        self.assertAlmostEqual(twist.angular.z, 0.0, places=5)

    def test_arc_follow_reversal_turns_in_place(self) -> None:
        """真值 vx 反向且船体仍朝东：原地转向，不扫出邻道。"""
        twist = compute_arc_follow_cmd_vel(
            -4.0, 0.0, 0.0, omega_limit=0.22, turn_radius_min_m=10.0, align_threshold_deg=10.0
        )
        self.assertAlmostEqual(twist.linear.x, 0.0, places=5)
        self.assertAlmostEqual(abs(twist.angular.z), 0.22, places=5)

    def test_arc_follow_moderate_error_uses_small_arc(self) -> None:
        twist = compute_arc_follow_cmd_vel(
            4.0, 0.0, math.radians(45.0),
            omega_limit=0.22, turn_radius_min_m=10.0, align_threshold_deg=10.0,
        )
        self.assertGreater(twist.linear.x, 0.0)
        self.assertLessEqual(twist.linear.x, 4.0)
        self.assertGreater(abs(twist.angular.z), 0.0)

    def test_arc_follow_westbound_when_aligned(self) -> None:
        twist = compute_arc_follow_cmd_vel(
            -4.0, 0.0, math.pi, omega_limit=0.12, turn_radius_min_m=25.0, align_threshold_deg=12.0
        )
        self.assertAlmostEqual(twist.linear.x, 4.0, places=5)
        self.assertAlmostEqual(twist.angular.z, 0.0, places=5)

    def test_velocity_control_plugin_in_box_sdf(self) -> None:
        sdf = _build_box_sdf("gt_ctrv_1", 10.0, 4.0, 2.0, "1 0 0 1", 0xFFFF, 50.0)
        self.assertIn("gz-sim-velocity-control-system", sdf)
        self.assertIn("/model/gt_ctrv_1/cmd_vel", sdf)
        self.assertIn("<mass>50.0</mass>", sdf)

    def test_velocity_control_plugin_snippet(self) -> None:
        xml = _velocity_control_plugin_xml("gt_ctrv_42")
        self.assertIn("gt_ctrv_42", xml)
        self.assertIn("VelocityControl", xml)

    def test_quat_to_roll_pitch_level(self) -> None:
        roll, pitch = _quat_to_roll_pitch(0.0, 0.0, 0.0, 1.0)
        self.assertAlmostEqual(roll, 0.0, places=5)
        self.assertAlmostEqual(pitch, 0.0, places=5)

    def test_attitude_violation_roll(self) -> None:
        half = 0.2
        qx = math.sin(half * 0.5)
        qw = math.cos(half * 0.5)
        pose = ModelWorldPose(0.0, 0.0, 1.0, qx, 0.0, 0.0, qw)
        reason = attitude_violation_reason(pose, 1.0, 10.0, 2.0)
        self.assertIsNotNone(reason)
        assert reason is not None
        self.assertIn("roll=", reason)

    def test_attitude_violation_z(self) -> None:
        pose = ModelWorldPose(0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 1.0)
        reason = attitude_violation_reason(pose, 1.0, 10.0, 2.0)
        self.assertIsNotNone(reason)
        assert reason is not None
        self.assertIn("z=", reason)


if __name__ == "__main__":
    unittest.main()
