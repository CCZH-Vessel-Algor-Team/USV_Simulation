"""Shared CTRV motion model, annulus sampling, and target state for ground truth nodes."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List

import numpy as np
from geometry_msgs.msg import Point


@dataclass
class TargetState:
    track_id: int
    x: float
    y: float
    speed: float
    theta: float
    omega: float
    size_w: float
    size_l: float
    size_h: float
    is_dark_target: bool
    is_ais_matched: bool
    matched_mmsi: int
    history: List[Point] = field(default_factory=list)

    @property
    def v_x(self) -> float:
        return self.speed * math.cos(self.theta)

    @property
    def v_y(self) -> float:
        return self.speed * math.sin(self.theta)


def wrap_angle(angle: float) -> float:
    wrapped = math.fmod(angle + math.pi, 2.0 * math.pi)
    if wrapped < 0.0:
        wrapped += 2.0 * math.pi
    return wrapped - math.pi


def sample_annulus_radius(rng: np.random.Generator, r_min: float, r_max: float) -> float:
    u = rng.random()
    return math.sqrt((r_max**2 - r_min**2) * u + r_min**2)


def ctrv_step(
    x: float,
    y: float,
    speed: float,
    theta: float,
    omega: float,
    dt: float,
) -> tuple[float, float, float]:
    if abs(omega) < 1e-3:
        x_next = x + speed * math.cos(theta) * dt
        y_next = y + speed * math.sin(theta) * dt
        theta_next = theta
    else:
        x_next = x + (speed / omega) * (math.sin(theta + omega * dt) - math.sin(theta))
        y_next = y + (speed / omega) * (-math.cos(theta + omega * dt) + math.cos(theta))
        theta_next = theta + omega * dt
    return x_next, y_next, wrap_angle(theta_next)


def propagate_target(
    target: TargetState,
    dt: float,
    omega_noise_std: float,
    omega_decay: float,
    omega_limit: float,
    rng: np.random.Generator,
) -> None:
    omega_noise = rng.normal(0.0, omega_noise_std)
    omega = (target.omega + omega_noise) * omega_decay
    omega = float(np.clip(omega, -omega_limit, omega_limit))
    target.omega = omega

    if abs(omega) < 1e-3:
        dx = target.speed * math.cos(target.theta) * dt
        dy = target.speed * math.sin(target.theta) * dt
    else:
        theta = target.theta
        dx = (target.speed / omega) * (math.sin(theta + omega * dt) - math.sin(theta))
        dy = (target.speed / omega) * (-math.cos(theta + omega * dt) + math.cos(theta))
    target.x += dx
    target.y += dy
    target.theta = wrap_angle(target.theta + omega * dt)


def append_history(target: TargetState, history_max_points: int) -> None:
    target.history.append(Point(x=float(target.x), y=float(target.y), z=0.0))
    if len(target.history) > history_max_points:
        del target.history[0 : len(target.history) - history_max_points]


def predict_future_path(
    target: TargetState,
    prediction_horizon: float,
    prediction_dt: float,
) -> List[Point]:
    points: List[Point] = []
    x, y, theta = target.x, target.y, target.theta
    omega = target.omega
    steps = int(prediction_horizon / prediction_dt)
    for _ in range(steps):
        x, y, theta = ctrv_step(x, y, target.speed, theta, omega, prediction_dt)
        points.append(Point(x=float(x), y=float(y), z=0.0))
    return points
