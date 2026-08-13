import math

from enc_grounding_warning.ukc_model import (
    RISK_CAUTION,
    RISK_DANGER,
    RISK_GROUNDED,
    RISK_SAFE,
    RISK_WARNING,
    classify,
    compute_ukc,
    dynamic_draft,
)


PARAMS = {
    "static_draft_m": 0.5,
    "length_m": 4.9,
    "beam_m": 2.0,
    "block_coefficient": 0.55,
    "squat_open_denominator": 100.0,
    "squat_confined_denominator": 50.0,
    "heel_allowance_m": 0.05,
    "trim_allowance_m": 0.03,
    "wave_allowance_m": 0.0,
    "ukc_required_abs_m": 0.3,
    "ukc_required_pct": 0.10,
    "u_depth_m": 0.1,
    "u_tide_m": 0.0,
    "u_draft_m": 0.02,
    "u_squat_m": 0.02,
    "u_wave_m": 0.0,
    "safety_factor": 1.0,
    "water_level_m": 0.0,
    "margin_safe_m": 0.1,
}


def test_dynamic_draft_no_squat_in_deep_water():
    draft = dynamic_draft(2.0, PARAMS, h_avail=5.0)
    assert abs(draft - (0.5 + 0.05 + 0.03)) < 1e-6


def test_dynamic_draft_squat_in_shallow_water():
    draft = dynamic_draft(3.0, PARAMS, h_avail=0.54)
    expected_squat = 0.55 * (3.0 * 1.9438444924406) ** 2 / 50.0
    assert abs(draft - (0.5 + expected_squat + 0.05 + 0.03)) < 1e-3


def test_classify_levels():
    ukc_req = 0.3
    u_total = 0.1
    assert classify(2.0, ukc_req, u_total, PARAMS, 3.0) == RISK_SAFE
    assert classify(0.45, ukc_req, u_total, PARAMS, 3.0) == RISK_CAUTION
    assert classify(0.35, ukc_req, u_total, PARAMS, 3.0) == RISK_WARNING
    assert classify(0.2, ukc_req, u_total, PARAMS, 3.0) == RISK_DANGER
    assert classify(0.0, ukc_req, u_total, PARAMS, 3.0) == RISK_GROUNDED


def test_compute_ukc_manual():
    result = compute_ukc(3.0, 0.0, PARAMS, 1.0)
    assert result["available_depth"] == 3.0
    assert abs(result["dynamic_draft"] - 0.58) < 1e-6
    assert abs(result["ukc"] - 2.42) < 1e-6
    assert result["risk"] == RISK_SAFE
    assert abs(result["safety_depth"] - (0.58 + 0.3 + result["uncertainty"])) < 1e-6
