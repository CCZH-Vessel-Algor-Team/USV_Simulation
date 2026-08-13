"""UKC estimation and risk classification logic (shared by nodes)."""

from __future__ import annotations

import math


RISK_UNKNOWN = 255
RISK_SAFE = 0
RISK_CAUTION = 1
RISK_WARNING = 2
RISK_DANGER = 3
RISK_GROUNDED = 4

KNOTS_PER_MPS = 1.9438444924406


def dynamic_draft(sog_mps: float, params: dict, h_avail: float | None = None) -> float:
    """Static draft + squat + heel/trim/wave allowances.

    Squat uses the simplified Barrass formula. It is only applied in
    shallow conditions (h/T < 1.2); below 1.1 the confined-water factor
    (denominator 50) is used.
    """
    static_draft = float(params.get("static_draft_m", 0.0))
    block_coefficient = float(params.get("block_coefficient", 0.55))
    sog_knots = max(0.0, float(sog_mps or 0.0)) * KNOTS_PER_MPS

    squat = 0.0
    if h_avail is not None and static_draft > 0.0:
        h_t = h_avail / static_draft
        if h_t < 1.2:
            if h_t < 1.1:
                denominator = float(params.get("squat_confined_denominator", 50.0))
            else:
                denominator = float(params.get("squat_open_denominator", 100.0))
            squat = block_coefficient * sog_knots * sog_knots / denominator

    return (
        static_draft
        + squat
        + float(params.get("heel_allowance_m", 0.0))
        + float(params.get("trim_allowance_m", 0.0))
        + float(params.get("wave_allowance_m", 0.0))
    )


def classify(
    ukc: float | None,
    ukc_req: float,
    u_total: float,
    params: dict,
    h_avail: float | None = None,
) -> int:
    if ukc is None:
        return RISK_UNKNOWN

    static_draft = float(params.get("static_draft_m", 0.0))
    if ukc <= 0.0:
        return RISK_GROUNDED
    if h_avail is not None and static_draft > 0.0 and h_avail / static_draft < 1.1:
        return RISK_DANGER

    margin_safe = float(params.get("margin_safe_m", 0.1))
    safe_line = ukc_req + u_total + margin_safe
    if ukc >= safe_line:
        return RISK_SAFE
    if ukc >= ukc_req + u_total:
        return RISK_CAUTION
    if ukc >= ukc_req:
        return RISK_WARNING
    return RISK_DANGER


def required_safety_depth(
    t_dyn: float, ukc_req: float, u_total: float, water_level: float
) -> float:
    return t_dyn + ukc_req + u_total - water_level


def compute_ukc(
    depth: float | None,
    water_level: float,
    params: dict,
    sog_mps: float,
):
    """Return a dict with all fields needed to fill UKCState."""
    h_avail = None if depth is None else depth + water_level
    t_dyn = dynamic_draft(sog_mps, params, h_avail)
    ukc = None if h_avail is None else h_avail - t_dyn

    u_depth = float(params.get("u_depth_m", 0.1))
    u_tide = float(params.get("u_tide_m", 0.0))
    u_draft = float(params.get("u_draft_m", 0.02))
    u_squat = float(params.get("u_squat_m", 0.02))
    u_wave = float(params.get("u_wave_m", 0.0))
    safety_factor = float(params.get("safety_factor", 1.0))
    u_total = safety_factor * math.sqrt(
        u_depth * u_depth
        + u_tide * u_tide
        + u_draft * u_draft
        + u_squat * u_squat
        + u_wave * u_wave
    )

    ukc_req = max(
        float(params.get("ukc_required_abs_m", 0.3)),
        float(params.get("ukc_required_pct", 0.10)) * t_dyn,
    )
    risk = classify(ukc, ukc_req, u_total, params, h_avail)
    safety_depth = required_safety_depth(t_dyn, ukc_req, u_total, water_level)

    return {
        "chart_depth": depth,
        "water_level": water_level,
        "available_depth": h_avail,
        "static_draft": float(params.get("static_draft_m", 0.0)),
        "squat": max(0.0, t_dyn - float(params.get("static_draft_m", 0.0))
                     - float(params.get("heel_allowance_m", 0.0))
                     - float(params.get("trim_allowance_m", 0.0))
                     - float(params.get("wave_allowance_m", 0.0))),
        "heel_allowance": float(params.get("heel_allowance_m", 0.0)),
        "trim_allowance": float(params.get("trim_allowance_m", 0.0)),
        "wave_allowance": float(params.get("wave_allowance_m", 0.0)),
        "dynamic_draft": t_dyn,
        "ukc": ukc,
        "ukc_required": ukc_req,
        "uncertainty": u_total,
        "safety_depth": safety_depth,
        "risk": risk,
    }


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )
