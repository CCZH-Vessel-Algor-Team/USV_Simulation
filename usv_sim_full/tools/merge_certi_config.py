#!/usr/bin/env python3
"""合并 certi_senario 基底与 certificate_case，生成 full_config 兼容的 merged YAML。"""

from __future__ import annotations

import argparse
import math
import os
import sys
from copy import deepcopy
from typing import Any, Dict, List, Optional, Tuple

import yaml

KNOTS_TO_MPS = 0.514444

STBD_BEARING_MIN_DEG = -112.5
STBD_BEARING_MAX_DEG = -5.0
PORT_BEARING_MIN_DEG = 5.0
PORT_BEARING_MAX_DEG = 112.5

# 会遇前初始相距（沿航线/会遇轴），由 (v_own+v_tgt)*tcpa 计算并夹在 min~max
DEFAULT_ENCOUNTER_RANGE_MAX_M = 320.0
DEFAULT_ENCOUNTER_RANGE_MIN_M = 60.0
DEFAULT_PAST_CPA_M = 50.0
DEFAULT_LATERAL_SPAN_M = 100.0
DEFAULT_DCPA_SAFE_M = 50.0
DCPA_TOLERANCE_M = 2.0

MESH_PROFILE_REL = '../description/models/target_ship/10m_mesh_profile.yaml'

CASE_SKIP_KEYS = frozenset({
    'meta', 'encounter', 'scenario_id', 'description', 'own_ship', 'target_ships',
})


def resolve_mesh_profile_rel(base_path: str, out_path: str) -> str:
    base_dir = os.path.dirname(os.path.abspath(base_path))
    mesh_abs = os.path.normpath(os.path.join(base_dir, MESH_PROFILE_REL))
    if not os.path.isfile(mesh_abs):
        raise FileNotFoundError(f'mesh_profile not found: {mesh_abs}')
    out_dir = os.path.dirname(os.path.abspath(out_path))
    rel = os.path.relpath(mesh_abs, out_dir)
    return rel.replace('\\', '/')


def deep_merge(base: Dict[str, Any], overlay: Dict[str, Any]) -> Dict[str, Any]:
    out = deepcopy(base)
    for key, val in overlay.items():
        if key in CASE_SKIP_KEYS:
            continue
        if isinstance(val, dict) and isinstance(out.get(key), dict):
            out[key] = deep_merge(out[key], val)
        else:
            out[key] = deepcopy(val)
    return out


def _knots_to_mps(knots: float) -> float:
    return float(knots) * KNOTS_TO_MPS


def _course_rad(course_deg: float) -> float:
    return math.radians(course_deg)


def _unit_from_course(course_deg: float) -> Tuple[float, float]:
    r = _course_rad(course_deg)
    return math.cos(r), math.sin(r)


def _perp_left(u: Tuple[float, float]) -> Tuple[float, float]:
    return (-u[1], u[0])


def _starboard_unit(u: Tuple[float, float]) -> Tuple[float, float]:
    return (u[1], -u[0])


def _track_heading_deg(wp0: Tuple[float, float], wp1: Tuple[float, float]) -> float:
    dx = wp1[0] - wp0[0]
    dy = wp1[1] - wp0[1]
    if abs(dx) + abs(dy) < 1e-9:
        return 0.0
    return math.degrees(math.atan2(dy, dx))


def line_line_dcpa(
    p1: Tuple[float, float],
    u1: Tuple[float, float],
    p2: Tuple[float, float],
    u2: Tuple[float, float],
) -> float:
    cross_z = u1[0] * u2[1] - u1[1] * u2[0]
    if abs(cross_z) > 1e-9:
        return 0.0
    un1 = math.hypot(u1[0], u1[1])
    if un1 < 1e-12:
        return math.hypot(p2[0] - p1[0], p2[1] - p1[1])
    nx, ny = -u1[1] / un1, u1[0] / un1
    return abs((p2[0] - p1[0]) * nx + (p2[1] - p1[1]) * ny)


def compute_dcpa_tcpa(
    p_own: Tuple[float, float],
    v_own: Tuple[float, float],
    p_tgt: Tuple[float, float],
    v_tgt: Tuple[float, float],
) -> Tuple[float, float]:
    rx = p_tgt[0] - p_own[0]
    ry = p_tgt[1] - p_own[1]
    vx = v_tgt[0] - v_own[0]
    vy = v_tgt[1] - v_own[1]
    vv = vx * vx + vy * vy
    if vv < 1e-12:
        return math.hypot(rx, ry), float('inf')
    tcpa = -(rx * vx + ry * vy) / vv
    if tcpa < 0.0:
        return math.hypot(rx, ry), tcpa
    cpa_x = rx + vx * tcpa
    cpa_y = ry + vy * tcpa
    return math.hypot(cpa_x, cpa_y), tcpa


def _relative_bearing_deg(
    p_own: Tuple[float, float],
    p_tgt: Tuple[float, float],
    course_deg: float,
) -> float:
    dx = p_tgt[0] - p_own[0]
    dy = p_tgt[1] - p_own[1]
    bearing = math.degrees(math.atan2(dy, dx)) - course_deg
    while bearing > 180.0:
        bearing -= 360.0
    while bearing < -180.0:
        bearing += 360.0
    return bearing


def _in_starboard_sector(bearing_deg: float) -> bool:
    return STBD_BEARING_MIN_DEG <= bearing_deg <= STBD_BEARING_MAX_DEG


def _in_port_sector(bearing_deg: float) -> bool:
    return PORT_BEARING_MIN_DEG <= bearing_deg <= PORT_BEARING_MAX_DEG


def _placement_origin(
    p_own: Tuple[float, float],
    u: Tuple[float, float],
    target: Dict[str, Any],
) -> Tuple[float, float]:
    idx = int(target.get('sequence_index', 0))
    if idx <= 0 and target.get('placement_along_m') is None and target.get('placement_lateral_m') is None:
        return p_own
    enc = _normalize_type(target.get('type', 'head_on'))
    if enc == 'overtaken':
        along_m = float(target.get('placement_along_m', -55.0 * idx))
    elif enc == 'overtaking':
        along_m = float(target.get('placement_along_m', 35.0 * idx))
    else:
        along_m = float(target.get('placement_along_m', 70.0 * idx))
    if enc in ('head_on', 'overtaking', 'overtaken'):
        default_lat = 0.0
    else:
        default_lat = 45.0 * idx * (1.0 if idx % 2 else -1.0)
    lat_m = float(target.get('placement_lateral_m', default_lat))
    port = _perp_left(u)
    return (
        p_own[0] + u[0] * along_m + port[0] * lat_m,
        p_own[1] + u[1] * along_m + port[1] * lat_m,
    )


def _encounter_range_m(
    speed_own: float,
    speed_tgt: float,
    tcpa_s: float,
    target: Dict[str, Any],
) -> float:
    """目标船起点相对本船沿会遇方向的初始距离（米）。"""
    if target.get('encounter_range_m') is not None:
        return float(target['encounter_range_m'])
    raw = (speed_own + speed_tgt) * max(float(tcpa_s), 1.0)
    hi = float(target.get('encounter_range_max_m', DEFAULT_ENCOUNTER_RANGE_MAX_M))
    lo = float(target.get('encounter_range_min_m', DEFAULT_ENCOUNTER_RANGE_MIN_M))
    return max(lo, min(raw, hi))


def _normalize_type(ts_type: str) -> str:
    t = str(ts_type).strip().lower()
    if t in ('crossing_right', 'crossing_starboard', 'crossing_starboard_danger'):
        return 'crossing_right'
    if t in ('crossing_left', 'crossing_port', 'crossing_port_danger'):
        return 'crossing_left'
    if t in ('overtaking', 'overtake'):
        return 'overtaking'
    if t in ('overtaken', 'overtaken_by'):
        return 'overtaken'
    if t == 'head_on':
        return 'head_on'
    raise ValueError(f'unsupported target ship type: {ts_type}')


def parse_certificate_case(case: Dict[str, Any]) -> Dict[str, Any]:
    """统一为内部结构：case_id, description, own_ship, target_ships[]。"""
    if 'target_ships' in case:
        own = case.get('own_ship', {})
        return {
            'case_id': case.get('scenario_id', 'unknown'),
            'description': case.get('description', ''),
            'own_ship': {
                'course_deg': float(own.get('initial_heading_deg', 0.0)),
                'speed_mps': _knots_to_mps(own.get('initial_speed_knots', 0.0)),
            },
            'target_ships': list(case.get('target_ships', [])),
        }

    meta = case.get('meta', {})
    enc = case.get('encounter', {})
    own = enc.get('own_ship', {})
    tgt = enc.get('target', {})
    return {
        'case_id': meta.get('case_id', 'unknown'),
        'description': meta.get('title', ''),
        'own_ship': {
            'course_deg': float(own.get('course_deg', 0.0)),
            'speed_mps': float(own.get('speed_mps', 4.0)),
        },
        'target_ships': [{
            'id': 'TS1',
            'type': enc.get('encounter_type', 'head_on'),
            'is_dangerous': bool(enc.get('dangerous', True)),
            'target_dcpa_meters': float(tgt.get('dcpa_m', 0.0)),
            'target_tcpa_seconds': float(enc.get('target_tcpa_seconds', 300.0)),
            'speed_knots': float(tgt.get('speed_mps', 5.0)) / KNOTS_TO_MPS,
            'crossing_angle_deg': float(enc.get('crossing_angle_deg', 90.0)),
            'dcpa_safe_m': float(enc.get('dcpa_safe_m', DEFAULT_DCPA_SAFE_M)),
        }],
    }


def generate_target_track(
    own_ship: Dict[str, Any],
    target: Dict[str, Any],
) -> Tuple[List[List[float]], Dict[str, Any]]:
    """生成航路点 [起点, 终点]；船首朝向与起点→终点一致。"""
    course_deg = float(own_ship['course_deg'])
    speed_own = float(own_ship['speed_mps'])
    speed_tgt = _knots_to_mps(target.get('speed_knots', 5.0))
    dcpa_m = float(target.get('target_dcpa_meters', 0.0))
    tcpa_s = float(target.get('target_tcpa_seconds', 300.0))
    enc_type = _normalize_type(target.get('type', 'head_on'))
    dangerous = bool(target.get('is_dangerous', True))
    dcpa_safe = float(target.get('dcpa_safe_m', DEFAULT_DCPA_SAFE_M))
    crossing_angle = float(target.get('crossing_angle_deg', 90.0))

    u = _unit_from_course(course_deg)
    p_own = (0.0, 0.0)
    p_anchor = _placement_origin(p_own, u, target)
    v_own = (speed_own * u[0], speed_own * u[1])
    range_m = _encounter_range_m(speed_own, speed_tgt, tcpa_s, target)
    v_close = max(speed_own + speed_tgt, 1e-6)
    t_encounter_s = range_m / v_close
    past_m = float(target.get('past_cpa_m', DEFAULT_PAST_CPA_M))
    stbd = _starboard_unit(u)
    cross = _perp_left(u)
    port = cross

    if enc_type == 'head_on':
        idx = int(target.get('sequence_index', 0))
        if idx > 0:
            range_m += float(target.get('range_along_extra_m', 85.0 * idx))
        ut = (-u[0], -u[1])
        perp = _perp_left(u)
        offset = dcpa_m if dangerous else max(dcpa_safe, dcpa_m)
        off = (perp[0] * offset, perp[1] * offset)
        # 目标在本船船首方向 range_m 处出发，驶向本船（CPA 约在 range_m 处相遇）
        p_cpa = (
            p_anchor[0] + u[0] * speed_own * t_encounter_s,
            p_anchor[1] + u[1] * speed_own * t_encounter_s,
        )
        wp_start = (
            p_anchor[0] + u[0] * range_m + off[0],
            p_anchor[1] + u[1] * range_m + off[1],
        )
        wp_end = (
            p_anchor[0] - u[0] * past_m + off[0],
            p_anchor[1] - u[1] * past_m + off[1],
        )
        v_tgt = (speed_tgt * ut[0], speed_tgt * ut[1])
    elif enc_type in ('crossing_right', 'crossing_left'):
        lateral = float(target.get('lateral_span_m', DEFAULT_LATERAL_SPAN_M))
        parallel_off = 0.0 if dangerous else max(dcpa_safe, dcpa_m)
        along = min(range_m * 0.55, float(target.get('cpa_along_max_m', 200.0)))
        cx = p_anchor[0] + u[0] * along + u[0] * parallel_off
        cy = p_anchor[1] + u[1] * along + u[1] * parallel_off
        p_cpa = (cx, cy)

        if enc_type == 'crossing_right':
            side_start, side_end = stbd, cross
        else:
            side_start, side_end = port, stbd

        if abs(crossing_angle - 90.0) < 1.0:
            wp_start = (cx + side_start[0] * lateral, cy + side_start[1] * lateral)
            wp_end = (cx + side_end[0] * lateral, cy + side_end[1] * lateral)
        else:
            sign = -1.0 if enc_type == 'crossing_right' else 1.0
            tgt_course = course_deg + sign * crossing_angle
            ut = _unit_from_course(tgt_course)
            leg_back = min(speed_tgt * t_encounter_s * 0.6, lateral * 1.2)
            leg_fwd = max(lateral * 0.8, 50.0)
            wp_start = (
                cx + side_start[0] * lateral - ut[0] * leg_back,
                cy + side_start[1] * lateral - ut[1] * leg_back,
            )
            wp_end = (
                cx + side_start[0] * dcpa_m * 0.5 + ut[0] * leg_fwd,
                cy + side_start[1] * dcpa_m * 0.5 + ut[1] * leg_fwd,
            )

        dist = math.hypot(wp_end[0] - wp_start[0], wp_end[1] - wp_start[1])
        if dist < 1e-6:
            raise ValueError(f'{enc_type}: waypoints too close')
        v_tgt = (
            speed_tgt * (wp_end[0] - wp_start[0]) / dist,
            speed_tgt * (wp_end[1] - wp_start[1]) / dist,
        )
    elif enc_type == 'overtaking':
        lat = dcpa_m if dangerous else max(dcpa_safe, dcpa_m)
        if range_m < 80.0:
            lat *= max(range_m / 80.0, 0.15)
        track_len = float(target.get('track_length_m', max(range_m * 0.8, 80.0)))
        wp_start = (
            p_anchor[0] + u[0] * range_m + port[0] * lat,
            p_anchor[1] + u[1] * range_m + port[1] * lat,
        )
        wp_end = (
            p_anchor[0] + u[0] * (range_m + track_len) + port[0] * lat,
            p_anchor[1] + u[1] * (range_m + track_len) + port[1] * lat,
        )
        p_cpa = (wp_start[0], wp_start[1])
        dist = math.hypot(wp_end[0] - wp_start[0], wp_end[1] - wp_start[1])
        v_tgt = (
            speed_tgt * (wp_end[0] - wp_start[0]) / max(dist, 1e-6),
            speed_tgt * (wp_end[1] - wp_start[1]) / max(dist, 1e-6),
        )
    elif enc_type == 'overtaken':
        lat = dcpa_m if dangerous else max(dcpa_safe, dcpa_m)
        if range_m < 80.0:
            lat *= max(range_m / 80.0, 0.15)
        track_len = float(target.get('track_length_m', max(range_m + past_m, 80.0)))
        wp_start = (
            p_anchor[0] - u[0] * range_m + port[0] * lat,
            p_anchor[1] - u[1] * range_m + port[1] * lat,
        )
        wp_end = (
            p_anchor[0] + u[0] * track_len + port[0] * lat,
            p_anchor[1] + u[1] * track_len + port[1] * lat,
        )
        p_cpa = (p_anchor[0], p_anchor[1])
        dist = math.hypot(wp_end[0] - wp_start[0], wp_end[1] - wp_start[1])
        v_tgt = (
            speed_tgt * (wp_end[0] - wp_start[0]) / max(dist, 1e-6),
            speed_tgt * (wp_end[1] - wp_start[1]) / max(dist, 1e-6),
        )
    else:
        raise ValueError(f'unsupported encounter type: {enc_type}')

    waypoints = [[wp_start[0], wp_start[1]], [wp_end[0], wp_end[1]]]
    heading_deg = _track_heading_deg(wp_start, wp_end)
    info = {
        'p_own': p_own,
        'v_own': v_own,
        'p_tgt_start': wp_start,
        'v_tgt': v_tgt,
        'course_deg': course_deg,
        'encounter_type': enc_type,
        'spawn_heading_deg': heading_deg,
        'encounter_range_m': round(range_m, 2),
        'planned_encounter_time_s': round(t_encounter_s, 2),
        'p_cpa': p_cpa,
    }
    return waypoints, info


def validate_target_track(
    own_ship: Dict[str, Any],
    target: Dict[str, Any],
) -> Tuple[float, float]:
    waypoints, info = generate_target_track(own_ship, target)
    enc_type = info['encounter_type']
    dangerous = bool(target.get('is_dangerous', True))
    dcpa_safe = float(target.get('dcpa_safe_m', DEFAULT_DCPA_SAFE_M))
    dcpa_target = float(target.get('target_dcpa_meters', 0.0))

    wp_start = tuple(info['p_tgt_start'])
    wp_end = tuple(waypoints[1])
    u_tgt = (wp_end[0] - wp_start[0], wp_end[1] - wp_start[1])
    td = math.hypot(u_tgt[0], u_tgt[1])
    u_tgt_n = (u_tgt[0] / td, u_tgt[1] / td) if td > 1e-9 else (0.0, 0.0)
    u_own = info['v_own']
    un = math.hypot(u_own[0], u_own[1])
    u_own_n = (u_own[0] / un, u_own[1] / un) if un > 1e-9 else (1.0, 0.0)

    dcpa, tcpa = compute_dcpa_tcpa(info['p_own'], info['v_own'], wp_start, info['v_tgt'])

    if enc_type == 'head_on':
        dcpa = line_line_dcpa(info['p_own'], u_own_n, wp_start, u_tgt_n)
    elif enc_type == 'crossing_right':
        bearing = _relative_bearing_deg(info['p_own'], wp_start, info['course_deg'])
        if not _in_starboard_sector(bearing):
            raise ValueError(
                f'crossing_right: initial bearing {bearing:.1f}° '
                f'not in starboard sector [{STBD_BEARING_MIN_DEG}, {STBD_BEARING_MAX_DEG}]'
            )
        parallel_off = 0.0 if dangerous else max(dcpa_safe, dcpa_target)
        dcpa = parallel_off
    elif enc_type == 'crossing_left':
        bearing = _relative_bearing_deg(info['p_own'], wp_start, info['course_deg'])
        if not _in_port_sector(bearing):
            raise ValueError(
                f'crossing_left: initial bearing {bearing:.1f}° '
                f'not in port sector [{PORT_BEARING_MIN_DEG}, {PORT_BEARING_MAX_DEG}]'
            )
        parallel_off = 0.0 if dangerous else max(dcpa_safe, dcpa_target)
        dcpa = parallel_off
    elif enc_type in ('overtaking', 'overtaken'):
        bearing = _relative_bearing_deg(info['p_own'], wp_start, info['course_deg'])
        if enc_type == 'overtaking' and abs(bearing) > 40.0:
            raise ValueError(f'overtaking: target not ahead, bearing={bearing:.1f}°')
        if enc_type == 'overtaken' and abs(bearing) < 120.0:
            raise ValueError(f'overtaken: target not astern, bearing={bearing:.1f}°')
        lat_nom = dcpa_target if dangerous else max(dcpa_safe, dcpa_target)
        dcpa = lat_nom

    if dangerous:
        if enc_type in ('crossing_right', 'crossing_left'):
            if line_line_dcpa(info['p_own'], u_own_n, wp_start, u_tgt_n) > DCPA_TOLERANCE_M:
                raise ValueError('dangerous crossing: track lines do not intersect at CPA')
        elif enc_type == 'head_on' and abs(dcpa - dcpa_target) > max(DCPA_TOLERANCE_M, dcpa_target * 0.5 + 5.0):
            if dcpa > dcpa_target + DCPA_TOLERANCE_M:
                raise ValueError(
                    f'dangerous head_on: DCPA {dcpa:.2f} m vs target {dcpa_target:.2f} m'
                )
    else:
        if enc_type in ('crossing_right', 'crossing_left'):
            parallel_off = max(dcpa_safe, dcpa_target)
            if parallel_off < dcpa_safe - DCPA_TOLERANCE_M:
                raise ValueError('non-dangerous crossing: insufficient along-track offset')
        elif dcpa < dcpa_safe - DCPA_TOLERANCE_M:
            raise ValueError(f'non-dangerous: DCPA {dcpa:.2f} m < safe {dcpa_safe} m')

    if tcpa <= 0.0:
        raise ValueError(f'TCPA {tcpa:.1f} s <= 0')

    return dcpa, tcpa


def apply_certificate_case(
    merged: Dict[str, Any],
    parsed: Dict[str, Any],
    mesh_profile_rel: str,
) -> None:
    own_ship = parsed['own_ship']
    course_deg = float(own_ship['course_deg'])
    speed_own = float(own_ship['speed_mps'])

    robot = merged.setdefault('robot_1', {})
    spawn = list(robot.get('spawn_pose', [0.0, 0.0, 0.5, 0.0, 0.0, 0.0]))
    while len(spawn) < 6:
        spawn.append(0.0)
    spawn[5] = _course_rad(course_deg)
    robot['spawn_pose'] = spawn
    ns = str(robot.get('name', 'usv_1'))

    obstacles: List[Dict[str, Any]] = []
    validations: List[Dict[str, Any]] = []

    for ts in parsed['target_ships']:
        ts_id = str(ts.get('id', 'TS1')).strip() or 'TS1'
        dcpa, tcpa = validate_target_track(own_ship, ts)
        waypoints, info = generate_target_track(own_ship, ts)
        speed_tgt = _knots_to_mps(ts.get('speed_knots', 5.0))

        obs_entry: Dict[str, Any] = {
            'name': f'target_{ts_id}',
            'shape': 'mesh_profile',
            'mesh_profile': mesh_profile_rel,
            'color': 'Yellow',
            'speed': speed_tgt,
            'loop': False,
            'waypoints': waypoints,
            'spawn_heading_deg': round(info['spawn_heading_deg'], 4),
        }
        if ts.get('spawn_delay_sec') is not None:
            obs_entry['spawn_delay_sec'] = float(ts['spawn_delay_sec'])
        obstacles.append(obs_entry)
        validations.append({
            'id': ts_id,
            'type': ts.get('type'),
            'dcpa_m': round(dcpa, 3),
            'tcpa_s': round(tcpa, 3),
            'spawn_heading_deg': round(info['spawn_heading_deg'], 4),
            'encounter_range_m': info.get('encounter_range_m'),
            'planned_encounter_time_s': info.get('planned_encounter_time_s'),
        })

    scenario = merged.setdefault('scenario', {})
    scenario['dynamic_obstacles'] = obstacles
    scenario['ground_truth_sim'] = {'enabled': False}

    merged['certificate_runtime'] = {
        'case_id': parsed['case_id'],
        'description': parsed.get('description', ''),
        'target_validation': validations,
        'own_ship_velocity': {
            'enabled': True,
            'namespace': ns,
            'speed_mps': speed_own,
            'course_deg': course_deg,
            'speed_knots': round(speed_own / KNOTS_TO_MPS, 3),
        },
    }


def merge_configs(
    base_path: str,
    case_path: str,
    out_path: Optional[str] = None,
) -> str:
    with open(base_path, 'r', encoding='utf-8') as f:
        base = yaml.safe_load(f) or {}
    with open(case_path, 'r', encoding='utf-8') as f:
        case = yaml.safe_load(f) or {}

    parsed = parse_certificate_case(case)
    if not parsed['target_ships']:
        raise ValueError(f'{case_path}: no target_ships defined')

    merged = deep_merge(base, case)

    if not out_path:
        case_id = parsed['case_id']
        out_dir = os.path.join(os.path.dirname(base_path), 'generated')
        os.makedirs(out_dir, exist_ok=True)
        out_path = os.path.join(out_dir, f'{case_id}.merged.yaml')

    mesh_profile_rel = resolve_mesh_profile_rel(base_path, out_path)
    apply_certificate_case(merged, parsed, mesh_profile_rel)

    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    with open(out_path, 'w', encoding='utf-8') as f:
        yaml.safe_dump(merged, f, default_flow_style=False, allow_unicode=True, sort_keys=False)

    print(f'[merge_certi_config] OK: {out_path}')
    for v in merged['certificate_runtime'].get('target_validation', []):
        print(
            f"  {v['id']}: range={v.get('encounter_range_m')} m "
            f"~{v.get('planned_encounter_time_s')} s to CPA, "
            f"DCPA={v['dcpa_m']} m TCPA={v['tcpa_s']} s "
            f"heading={v['spawn_heading_deg']}°"
        )
    return out_path


def main() -> int:
    pkg_config = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'config',
    )
    default_base = os.path.join(pkg_config, 'certi_senario.yaml')

    parser = argparse.ArgumentParser(description='Merge certi base + certificate case YAML')
    parser.add_argument('--base', default=default_base, help='Base certi_senario.yaml path')
    parser.add_argument('--case', required=True, help='certificate_case/*.yaml path')
    parser.add_argument('--out', default=None, help='Output merged yaml path')
    args = parser.parse_args()

    try:
        merge_configs(args.base, args.case, args.out)
        return 0
    except (ValueError, KeyError, OSError) as exc:
        print(f'[merge_certi_config] ERROR: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
