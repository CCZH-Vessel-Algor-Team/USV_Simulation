"""Shared arc-following kinematics for waypoint ground truth and Gazebo cmd_vel."""

from __future__ import annotations

import math

from geometry_msgs.msg import Twist

from ground_truth_sim.ctrv import wrap_angle


def clamp_omega(omega: float, limit: float) -> float:
    lim = max(0.0, float(limit))
    if lim <= 0.0:
        return omega
    return max(-lim, min(lim, omega))


def plan_arc_follow(
    speed: float,
    body_yaw: float,
    desired_yaw: float,
    *,
    omega_limit: float,
    turn_radius_min_m: float,
    align_threshold_deg: float,
    heading_gain: float = 1.5,
    omega_deadband_deg: float = 4.0,
) -> tuple[float, float]:
    """Return body-frame forward speed and yaw rate for arc / in-place following."""
    if speed <= 0.0:
        return 0.0, 0.0

    heading_err = wrap_angle(desired_yaw - body_yaw)
    align_rad = math.radians(max(1.0, align_threshold_deg))
    deadband_rad = math.radians(max(0.0, omega_deadband_deg))
    omega_lim = max(0.0, float(omega_limit))
    r_min = max(1.0, float(turn_radius_min_m))
    abs_err = abs(heading_err)

    if abs_err <= align_rad:
        return speed, 0.0

    if abs_err <= deadband_rad:
        return speed * max(0.0, math.cos(heading_err)), 0.0

    omega = clamp_omega(heading_err * heading_gain, omega_lim)

    if abs_err >= math.radians(90.0):
        forward_speed = 0.0
    else:
        arc_speed = min(speed, omega_lim * r_min) if omega_lim > 1e-6 else 0.0
        blend = (abs_err - align_rad) / max(align_rad, math.radians(90.0) - align_rad)
        blend = max(0.0, min(1.0, blend))
        cruise = speed * max(0.0, math.cos(heading_err))
        forward_speed = (1.0 - blend) * cruise + blend * arc_speed

    return forward_speed, omega


def compute_arc_follow_cmd_vel(
    vx_world: float,
    vy_world: float,
    body_yaw: float,
    *,
    omega_limit: float,
    turn_radius_min_m: float,
    align_threshold_deg: float,
    speed_eps: float = 1e-3,
    heading_gain: float = 1.5,
    omega_deadband_deg: float = 4.0,
) -> Twist:
    """Map-frame desired velocity + actual body yaw → body Twist for VelocityControl."""
    speed = math.hypot(vx_world, vy_world)
    twist = Twist()
    if speed < speed_eps:
        return twist

    desired_yaw = math.atan2(vy_world, vx_world)
    forward_speed, omega = plan_arc_follow(
        speed,
        body_yaw,
        desired_yaw,
        omega_limit=omega_limit,
        turn_radius_min_m=turn_radius_min_m,
        align_threshold_deg=align_threshold_deg,
        heading_gain=heading_gain,
        omega_deadband_deg=omega_deadband_deg,
    )
    twist.linear.x = forward_speed
    twist.angular.z = omega
    return twist
