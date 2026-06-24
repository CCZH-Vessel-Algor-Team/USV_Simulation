#!/usr/bin/env python3
"""采样 /sim/ground_truth 与 Gazebo 中 gt_ctrv_* 模型数量，用于 ground_truth_test 回归。"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
import time
from typing import Dict, List, Tuple

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

try:
    from usv_interfaces.msg import GlobalTrackArray
except ImportError as exc:  # pragma: no cover
    print('请先 source install/setup.bash（需要 usv_interfaces）', file=sys.stderr)
    raise SystemExit(1) from exc

try:
    from ground_truth_sim.waypoint import point_to_polyline_distance
except ImportError:
    _PKG_ROOT = __import__('pathlib').Path(__file__).resolve().parents[2]
    if str(_PKG_ROOT / 'ground_truth_sim') not in sys.path:
        sys.path.insert(0, str(_PKG_ROOT / 'ground_truth_sim'))
    from ground_truth_sim.waypoint import point_to_polyline_distance


def _gz_models_once(prefix: str, world: str) -> list[str]:
    try:
        proc = subprocess.run(
            [
                'gz',
                'topic',
                '-e',
                '-n',
                '5',
                '-t',
                f'/world/{world}/pose/info',
            ],
            capture_output=True,
            text=True,
            timeout=15.0,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired) as ex:
        return [f'<gz list failed: {ex}>']

    if proc.returncode != 0:
        text = (proc.stdout or '') + (proc.stderr or '')
        return [f'<gz list rc={proc.returncode}: {text.strip()[:200]}>']

    names = []
    for match in re.finditer(r'name:\s*"([^"]+)"', proc.stdout or ''):
        base = match.group(1).split('::', 1)[0]
        if base.startswith(prefix):
            names.append(base)
    return sorted(set(names))


def _gz_models(prefix: str, world: str) -> list[str]:
    """多次采样取并集，降低 pose/info 单帧漏检导致的误报。"""
    merged: set[str] = set()
    error: list[str] = []
    for _ in range(5):
        names = _gz_models_once(prefix, world)
        if names and names[0].startswith('<'):
            error = names
            continue
        merged.update(names)
        time.sleep(0.1)
    if not merged and error:
        return error
    return sorted(merged)


def load_waypoint_routes(config_path: str) -> Dict[int, List[Tuple[float, float]]]:
    with open(config_path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    gt = (data.get('scenario') or {}).get('ground_truth_sim') or {}
    routes: Dict[int, List[Tuple[float, float]]] = {}
    for entry in gt.get('fixed_targets') or []:
        if not isinstance(entry, dict):
            continue
        tid = int(entry['track_id'])
        wps = []
        for wp in entry.get('waypoints') or []:
            wps.append((float(wp[0]), float(wp[1])))
        if wps:
            routes[tid] = wps
    return routes


class GroundTruthMonitor(Node):
    def __init__(
        self,
        topic: str,
        expected: int,
        prefix: str,
        world: str,
        routes: Dict[int, List[Tuple[float, float]]],
        route_tolerance_m: float,
    ) -> None:
        super().__init__('ground_truth_count_monitor')
        self._expected = expected
        self._prefix = prefix
        self._world = world
        self._routes = routes
        self._route_tolerance_m = route_tolerance_m
        self._last_msg: GlobalTrackArray | None = None
        self.create_subscription(GlobalTrackArray, topic, self._on_msg, qos_profile_sensor_data)

    def _on_msg(self, msg: GlobalTrackArray) -> None:
        self._last_msg = msg

    def _route_violations(self) -> List[str]:
        if not self._routes or self._last_msg is None:
            return []
        out: List[str] = []
        by_id = {int(t.track_id): t for t in self._last_msg.tracks}
        for tid, wps in self._routes.items():
            track = by_id.get(tid)
            if track is None:
                continue
            dist = point_to_polyline_distance(float(track.x), float(track.y), wps)
            if dist > self._route_tolerance_m:
                out.append('track_id=%d dist=%.2fm' % (tid, dist))
        return out

    def sample(self) -> dict:
        track_ids = []
        if self._last_msg is not None:
            track_ids = sorted({int(t.track_id) for t in self._last_msg.tracks})
        gz_names = _gz_models(self._prefix, self._world)
        gz_ok = all(not n.startswith('<') for n in gz_names)
        route_bad = self._route_violations()
        return {
            'track_count': len(track_ids),
            'track_ids': track_ids,
            'gz_count': len(gz_names) if gz_ok else -1,
            'gz_names': gz_names,
            'gz_ok': gz_ok,
            'route_bad': route_bad,
        }


def main() -> int:
    parser = argparse.ArgumentParser(description='监测 ground truth 与 Gazebo 实体数量')
    parser.add_argument('--topic', default='/sim/ground_truth')
    parser.add_argument('--expected', type=int, default=5)
    parser.add_argument('--prefix', default='gt_ctrv_')
    parser.add_argument('--world', default='sydney_regatta_open_water')
    parser.add_argument('--duration-sec', type=float, default=60.0)
    parser.add_argument('--interval-sec', type=float, default=5.0)
    parser.add_argument('--wait-topic-sec', type=float, default=30.0)
    parser.add_argument(
        '--config',
        default='',
        help='full_config YAML；与 --check-route 联用读取 fixed_targets 航路',
    )
    parser.add_argument(
        '--check-route',
        action='store_true',
        help='校验各 track 位置距航路折线不超过 --route-tolerance-m',
    )
    parser.add_argument(
        '--gz-violation-ratio-max',
        type=float,
        default=0.15,
        help='允许的 Gazebo 实体计数不一致采样占比（track/route 仍为零容忍）',
    )
    parser.add_argument('--route-tolerance-m', type=float, default=2.0)
    args = parser.parse_args()

    routes: Dict[int, List[Tuple[float, float]]] = {}
    if args.check_route:
        if not args.config:
            print('--check-route 需要 --config', file=sys.stderr)
            return 2
        routes = load_waypoint_routes(args.config)

    rclpy.init()
    node = GroundTruthMonitor(
        args.topic,
        args.expected,
        args.prefix,
        args.world,
        routes,
        args.route_tolerance_m,
    )
    t0 = time.monotonic()
    while node._last_msg is None and (time.monotonic() - t0) < args.wait_topic_sec:
        rclpy.spin_once(node, timeout_sec=0.2)

    if node._last_msg is None:
        print('超时：未收到', args.topic)
        node.destroy_node()
        rclpy.shutdown()
        return 2

    print(
        '监测开始 expected=%d topic=%s gz_prefix=%s world=%s duration=%.0fs check_route=%s'
        % (
            args.expected,
            args.topic,
            args.prefix,
            args.world,
            args.duration_sec,
            args.check_route,
        )
    )

    violations = 0
    gz_violations = 0
    samples = 0
    end = time.monotonic() + max(1.0, args.duration_sec)
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        s = node.sample()
        samples += 1
        bad_gt = s['track_count'] != args.expected
        bad_gz = s['gz_ok'] and s['gz_count'] != args.expected
        bad_route = bool(s['route_bad'])
        if bad_gz:
            gz_violations += 1
        flag = 'FAIL' if (bad_gt or bad_route or bad_gz) else 'OK'
        if bad_gt or bad_route:
            violations += 1
        route_txt = ''
        if bad_route:
            route_txt = ' route=' + ','.join(s['route_bad'])
        print(
            '[%s] tracks=%d ids=%s | gz=%s count=%d names=%s%s'
            % (
                flag,
                s['track_count'],
                s['track_ids'],
                'ok' if s['gz_ok'] else 'err',
                s['gz_count'],
                s['gz_names'][:12],
                route_txt,
            )
        )
        time.sleep(max(0.5, args.interval_sec))

    node.destroy_node()
    rclpy.shutdown()
    gz_ratio = (gz_violations / samples) if samples else 0.0
    if gz_ratio > args.gz_violation_ratio_max:
        violations += 1
        print(
            'Gazebo 实体计数不一致占比 %.1f%% 超过阈值 %.1f%%'
            % (100.0 * gz_ratio, 100.0 * args.gz_violation_ratio_max)
        )
    print(
        '监测结束 violations=%d gz_mismatch_ratio=%.1f%% samples=%d'
        % (violations, 100.0 * gz_ratio, samples)
    )
    return 1 if violations else 0


if __name__ == '__main__':
    raise SystemExit(main())
