"""Estimate planar velocity from Gazebo entity pose samples."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class EntityVelocityEstimator:
    """First-order low-pass filtered velocity from pose deltas."""

    tau_sec: float = 0.2
    speed_eps: float = 0.05
    last_x: Optional[float] = None
    last_y: Optional[float] = None
    last_t_ns: Optional[int] = None
    vx: float = 0.0
    vy: float = 0.0

    def reset(self) -> None:
        self.last_x = None
        self.last_y = None
        self.last_t_ns = None
        self.vx = 0.0
        self.vy = 0.0

    def update(self, x: float, y: float, t_ns: int) -> Tuple[float, float]:
        if self.last_t_ns is None or self.last_x is None or self.last_y is None:
            self.last_x = float(x)
            self.last_y = float(y)
            self.last_t_ns = int(t_ns)
            self.vx = 0.0
            self.vy = 0.0
            return 0.0, 0.0

        dt = (int(t_ns) - self.last_t_ns) * 1e-9
        if dt <= 1e-6:
            return self.vx, self.vy

        raw_vx = (float(x) - self.last_x) / dt
        raw_vy = (float(y) - self.last_y) / dt
        tau = max(1e-3, float(self.tau_sec))
        alpha = min(1.0, dt / tau)
        self.vx += alpha * (raw_vx - self.vx)
        self.vy += alpha * (raw_vy - self.vy)

        if math.hypot(self.vx, self.vy) < self.speed_eps:
            self.vx = 0.0
            self.vy = 0.0

        self.last_x = float(x)
        self.last_y = float(y)
        self.last_t_ns = int(t_ns)
        return self.vx, self.vy


def velocity_heading_deg(vx: float, vy: float) -> Optional[float]:
    speed = math.hypot(vx, vy)
    if speed < 1e-6:
        return None
    return math.degrees(math.atan2(vy, vx))
