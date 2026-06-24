#!/usr/bin/env python3
"""按 senario_cule.md 批量生成 config/certificate_case/*.yaml。"""

from __future__ import annotations

import os
from typing import Any, Dict, List, Optional, Tuple

import yaml

# (type, is_dangerous) 简写
H = ('head_on', True)
S = ('head_on', False)
CR = ('crossing_right', True)
SR = ('crossing_right', False)
CL = ('crossing_left', True)
SL = ('crossing_left', False)
OT = ('overtaking', True)
ST = ('overtaking', False)
OD = ('overtaken', True)
SD = ('overtaken', False)

DEFAULT_OWN = {'initial_speed_knots': 10.0, 'initial_heading_deg': 0.0}

# C1-001 .. C1-010
C1: List[Tuple[str, str, List[Tuple[str, Tuple[str, bool]]]]] = [
    ('C1-001', '危险对遇', [('', H)]),
    ('C1-002', '非危险对遇', [('', S)]),
    ('C1-003', '危险右舷交叉', [('', CR)]),
    ('C1-004', '非危险右舷交叉', [('', SR)]),
    ('C1-005', '危险左舷交叉', [('', CL)]),
    ('C1-006', '非危险左舷交叉', [('', SL)]),
    ('C1-007', '危险追越', [('', OT)]),
    ('C1-008', '非危险追越', [('', ST)]),
    ('C1-009', '危险被追越', [('', OD)]),
    ('C1-010', '非危险被追越', [('', SD)]),
]

# C2: 危险主目标 + 非危险远距船
C2: List[Tuple[str, str, Tuple[str, bool], Tuple[str, bool]]] = [
    ('C2-001', '危险对遇船+非危险会遇船', H, S),
    ('C2-002', '危险右舷交叉船+非危险会遇船', CR, SR),
    ('C2-003', '危险左舷交叉船+非危险会遇船', CL, SL),
    ('C2-004', '危险追越船+非危险会遇船', OT, ST),
    ('C2-005', '危险被追越船+非危险会遇船', OD, SD),
]

# C3 依次 / C4 同时：(id, pair of encounter types)
C3: List[Tuple[str, str, Tuple[Tuple[str, bool], Tuple[str, bool]]]] = [
    ('C3-001', '危险对遇+危险对遇', H, H),
    ('C3-002', '危险对遇+危险左交叉', H, CL),
    ('C3-003', '危险对遇+危险右交叉', H, CR),
    ('C3-004', '危险对遇+危险被追越', H, OD),
    ('C3-005', '危险对遇+危险追越', H, OT),
    ('C3-006', '危险左交叉+危险左交叉', CL, CL),
    ('C3-007', '危险左交叉+危险右交叉', CL, CR),
    ('C3-008', '危险左交叉+危险被追越', CL, OD),
    ('C3-009', '危险左交叉+危险追越', CL, OT),
    ('C3-010', '危险右交叉+危险右交叉', CR, CR),
    ('C3-011', '危险右交叉+危险追越', CR, OT),
    ('C3-012', '危险右交叉+危险被追越', CR, OD),
    ('C3-013', '危险追越+危险追越', OT, OT),
    ('C3-014', '危险追越+危险被追越', OT, OD),
    ('C3-015', '危险被追越+危险被追越', OD, OD),
]

C4: List[Tuple[str, str, Tuple[Tuple[str, bool], Tuple[str, bool]]]] = [
    ('C4-001', '危险对遇+危险左舷交叉', H, CL),
    ('C4-002', '危险对遇+危险右舷交叉', H, CR),
    ('C4-003', '危险对遇+危险被追越', H, OD),
    ('C4-004', '危险左舷交叉+危险左舷交叉', CL, CL),
    ('C4-005', '危险左舷交叉+危险右舷交叉', CL, CR),
    ('C4-006', '危险左舷交叉+危险追越', CL, OT),
    ('C4-007', '危险左舷交叉+危险被追越', CL, OD),
    ('C4-008', '危险右舷交叉+危险右舷交叉', CR, CR),
    ('C4-009', '危险右舷交叉+危险追越', CR, OT),
    ('C4-010', '危险右舷交叉+危险被追越', CR, OD),
    ('C4-011', '危险追越+危险被追越', OT, OD),
]


def _target_spec(
    ts_id: str,
    enc: Tuple[str, bool],
    *,
    role: str = 'primary',
    spawn_delay_sec: float = 0.0,
    sequence_index: int = 0,
) -> Dict[str, Any]:
    typ, dangerous = enc
    spec: Dict[str, Any] = {
        'id': ts_id,
        'type': typ,
        'is_dangerous': dangerous,
        'target_tcpa_seconds': 10.0,
        'speed_knots': 14.0 if dangerous else 12.0,
        'encounter_range_max_m': 50.0,
    }
    if dangerous:
        spec['target_dcpa_meters'] = 5.0 if typ == 'head_on' else 0.0
    else:
        spec['target_dcpa_meters'] = 55.0
        spec['dcpa_safe_m'] = 55.0
    if typ in ('crossing_right', 'crossing_left'):
        spec['crossing_angle_deg'] = 90.0
        spec['lateral_span_m'] = 90.0 if dangerous else 110.0
    if typ == 'overtaking':
        spec['speed_knots'] = 8.0 if dangerous else 9.0
        spec['encounter_range_m'] = 50.0
    if typ == 'overtaken':
        spec['speed_knots'] = 16.0 if dangerous else 14.0
        spec['encounter_range_m'] = 50.0
    if role == 'decoy':
        spec['is_dangerous'] = False
        spec['target_dcpa_meters'] = 60.0
        spec['encounter_range_m'] = 50.0
        spec['speed_knots'] = 10.0
        spec['type'] = 'head_on'
        spec['target_tcpa_seconds'] = 10.0
    if spawn_delay_sec > 0.0:
        spec['spawn_delay_sec'] = spawn_delay_sec
    if sequence_index > 0:
        spec['sequence_index'] = sequence_index
    return spec


def build_c1(case_id: str, title: str, enc: Tuple[str, bool]) -> Dict[str, Any]:
    return {
        'scenario_id': case_id,
        'description': title,
        'own_ship': dict(DEFAULT_OWN),
        'target_ships': [_target_spec('TS1', enc)],
    }


def build_c2(case_id: str, title: str, main_e: Tuple[str, bool], decoy_e: Tuple[str, bool]) -> Dict[str, Any]:
    return {
        'scenario_id': case_id,
        'description': title,
        'own_ship': dict(DEFAULT_OWN),
        'timing': 'simultaneous',
        'target_ships': [
            _target_spec('TS1', main_e, role='primary'),
            _target_spec('TS2', decoy_e, role='decoy'),
        ],
    }


def build_c3_c4(
    case_id: str,
    title: str,
    e1: Tuple[str, bool],
    e2: Tuple[str, bool],
    *,
    sequential: bool,
) -> Dict[str, Any]:
    delay = 0.0
    if sequential:
        delay = 95.0
    return {
        'scenario_id': case_id,
        'description': title,
        'own_ship': dict(DEFAULT_OWN),
        'timing': 'sequential' if sequential else 'simultaneous',
        'target_ships': [
            _target_spec('TS1', e1, role='primary', sequence_index=0),
            _target_spec('TS2', e2, role='primary', spawn_delay_sec=delay, sequence_index=1),
        ],
    }


def main() -> int:
    out_dir = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'config',
        'certificate_case',
    )
    os.makedirs(out_dir, exist_ok=True)

    count = 0
    for case_id, title, pairs in C1:
        data = build_c1(case_id, title, pairs[0][1])
        path = os.path.join(out_dir, f'{case_id}.yaml')
        with open(path, 'w', encoding='utf-8') as f:
            yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False, default_flow_style=False)
        count += 1

    for case_id, title, main_e, decoy_e in C2:
        data = build_c2(case_id, title, main_e, decoy_e)
        path = os.path.join(out_dir, f'{case_id}.yaml')
        with open(path, 'w', encoding='utf-8') as f:
            yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False, default_flow_style=False)
        count += 1

    for case_id, title, e1, e2 in C3:
        data = build_c3_c4(case_id, title, e1, e2, sequential=True)
        path = os.path.join(out_dir, f'{case_id}.yaml')
        with open(path, 'w', encoding='utf-8') as f:
            yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False, default_flow_style=False)
        count += 1

    for case_id, title, e1, e2 in C4:
        data = build_c3_c4(case_id, title, e1, e2, sequential=False)
        path = os.path.join(out_dir, f'{case_id}.yaml')
        with open(path, 'w', encoding='utf-8') as f:
            yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False, default_flow_style=False)
        count += 1

    print(f'[generate_certificate_cases] wrote {count} yaml files to {out_dir}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
