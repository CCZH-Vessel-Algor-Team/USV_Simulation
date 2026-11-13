#!/usr/bin/env python3
"""Compare truth and fused TrackedShipList topics one-to-one.

Subscribes to ``/dynamic_ship/tracked_ships`` (truth) and
``/navigation/tracked_ships`` (fused), associates ships by nearest position
inside a gate, and reports speed, course and heading errors.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import rclpy
from nav2_colregs_msgs.msg import TrackedShip, TrackedShipList
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

DEFAULT_TRUTH_TOPIC = '/dynamic_ship/tracked_ships'
DEFAULT_FUSION_TOPIC = '/navigation/tracked_ships'
CSV_HEADER = [
    'stamp_sec',
    'stamp_dt_sec',
    'truth_frame',
    'fusion_frame',
    'truth_id',
    'fusion_id',
    'match_distance_m',
    'truth_x',
    'truth_y',
    'fusion_x',
    'fusion_y',
    'truth_vx',
    'truth_vy',
    'fusion_vx',
    'fusion_vy',
    'truth_speed',
    'fusion_speed',
    'speed_error',
    'truth_course_deg',
    'fusion_course_deg',
    'course_error_deg',
    'truth_heading_deg',
    'fusion_heading_deg',
    'heading_error_deg',
]


def _stamp_sec(stamp) -> float:
    """Convert a ROS stamp to seconds.

    :param stamp: ROS message stamp.
    :return: Stamp value in seconds.
    """
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def _wrap_angle(angle: float) -> float:
    """Wrap an angle to ``[-pi, pi]``.

    :param angle: Angle in radians.
    :return: Wrapped angle in radians.
    """
    return math.atan2(math.sin(angle), math.cos(angle))


def _quat_to_yaw(quaternion) -> float:
    """Convert a quaternion to yaw.

    :param quaternion: Geometry quaternion message.
    :return: Yaw in radians.
    """
    w = float(quaternion.w)
    x = float(quaternion.x)
    y = float(quaternion.y)
    z = float(quaternion.z)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _short_id(ship: TrackedShip) -> str:
    """Return a short printable target ID.

    :param ship: Tracked ship message.
    :return: First four UUID bytes in hex.
    """
    raw = bytes(ship.target_id.uuid)
    return raw[:4].hex() if len(raw) >= 4 else raw.hex()


def _sample_from_msg(ship: TrackedShip, frame_id: str) -> 'ShipSample':
    """Build a comparison sample from a tracked ship.

    :param ship: Tracked ship message.
    :param frame_id: Frame inherited from the list header.
    :return: Extracted sample.
    """
    vx = float(ship.twist.linear.x)
    vy = float(ship.twist.linear.y)
    speed = math.hypot(vx, vy)
    return ShipSample(
        target_id=_short_id(ship),
        x=float(ship.pose.position.x),
        y=float(ship.pose.position.y),
        vx=vx,
        vy=vy,
        speed=speed,
        course=math.atan2(vy, vx),
        heading=_quat_to_yaw(ship.pose.orientation),
        frame_id=frame_id,
    )


@dataclass(frozen=True)
class ShipSample:
    """One truth or fusion target in a common frame."""

    target_id: str
    x: float
    y: float
    vx: float
    vy: float
    speed: float
    course: float
    heading: float
    frame_id: str


@dataclass(frozen=True)
class PairComparison:
    """One truth-to-fusion association and its errors."""

    stamp_sec: float
    stamp_dt: float
    truth: ShipSample
    fusion: ShipSample
    match_distance: float
    speed_error: float
    course_error: Optional[float]
    heading_error: Optional[float]


@dataclass
class Stats:
    """Accumulated comparison statistics."""

    frames: int = 0
    matched: int = 0
    unmatched_truth: int = 0
    unmatched_fusion: int = 0
    stamp_skips: int = 0
    frame_skips: int = 0
    position_errors: List[float] = field(default_factory=list)
    speed_errors_abs: List[float] = field(default_factory=list)
    speed_errors_signed: List[float] = field(default_factory=list)
    course_errors_abs: List[float] = field(default_factory=list)
    heading_errors_abs: List[float] = field(default_factory=list)

    def add(self, comparison: PairComparison) -> None:
        """Accumulate one comparison.

        :param comparison: Matched truth/fusion pair.
        :return: None
        """
        self.matched += 1
        self.position_errors.append(comparison.match_distance)
        self.speed_errors_abs.append(abs(comparison.speed_error))
        self.speed_errors_signed.append(comparison.speed_error)
        if comparison.course_error is not None:
            self.course_errors_abs.append(abs(comparison.course_error))
        if comparison.heading_error is not None:
            self.heading_errors_abs.append(abs(comparison.heading_error))


def _associate(
    truths: Sequence[ShipSample],
    fusions: Sequence[ShipSample],
    gate_m: float,
) -> Tuple[List[Tuple[int, int, float]], List[int], List[int]]:
    """Associate truth and fusion samples one-to-one by nearest position.

    :param truths: Truth samples.
    :param fusions: Fusion samples.
    :param gate_m: Maximum association distance in meters.
    :return: Matched ``(truth_index, fusion_index, distance)`` tuples, plus
        unmatched truth and fusion indices.
    """
    candidates: List[Tuple[float, int, int]] = []
    for truth_index, truth in enumerate(truths):
        for fusion_index, fusion in enumerate(fusions):
            distance = math.hypot(truth.x - fusion.x, truth.y - fusion.y)
            if distance <= gate_m:
                candidates.append((distance, truth_index, fusion_index))
    candidates.sort(key=lambda item: item[0])

    used_truth: set[int] = set()
    used_fusion: set[int] = set()
    matched: List[Tuple[int, int, float]] = []
    for distance, truth_index, fusion_index in candidates:
        if truth_index in used_truth or fusion_index in used_fusion:
            continue
        used_truth.add(truth_index)
        used_fusion.add(fusion_index)
        matched.append((truth_index, fusion_index, distance))

    unmatched_truth = [
        index for index in range(len(truths)) if index not in used_truth
    ]
    unmatched_fusion = [
        index for index in range(len(fusions)) if index not in used_fusion
    ]
    return matched, unmatched_truth, unmatched_fusion


def _percentile(values: Sequence[float], fraction: float) -> float:
    """Return a nearest-rank percentile.

    :param values: Non-empty numeric values.
    :param fraction: Percentile fraction in ``[0, 1]``.
    :return: Percentile value.
    """
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, int(math.ceil(fraction * len(ordered))) - 1))
    return ordered[index]


def _format_signed_stat(name: str, values: Sequence[float], unit: str) -> str:
    """Format a signed metric summary line.

    :param name: Metric label.
    :param values: Signed metric samples.
    :param unit: Unit suffix.
    :return: Formatted line.
    """
    if not values:
        return f'{name}: n=0'
    mean = sum(values) / len(values)
    rmse = math.sqrt(sum(value * value for value in values) / len(values))
    return (
        f'{name}: n={len(values)} mean={mean:.3f} rmse={rmse:.3f} '
        f'min={min(values):.3f} max={max(values):.3f} {unit}'
    )


def _format_stat(
    name: str,
    values: Sequence[float],
    unit: str,
    scale: float = 1.0,
) -> str:
    """Format one summary statistic line.

    :param name: Metric label.
    :param values: Metric samples.
    :param unit: Unit suffix.
    :param scale: Multiplier applied before formatting.
    :return: Formatted line.
    """
    if not values:
        return f'{name}: n=0'
    scaled = [value * scale for value in values]
    mean = sum(scaled) / len(scaled)
    rmse = math.sqrt(sum(value * value for value in scaled) / len(scaled))
    return (
        f'{name}: n={len(scaled)} mean={mean:.3f} rmse={rmse:.3f} '
        f'p95={_percentile(scaled, 0.95):.3f} max={max(scaled):.3f} {unit}'
    )


class TrackedShipCompare(Node):
    """Live truth/fusion comparison node."""

    def __init__(self, args: argparse.Namespace) -> None:
        """Initialize subscriptions and output sinks.

        :param args: Parsed command-line arguments.
        :return: None
        """
        super().__init__('tracked_ship_compare')
        self.args = args
        self.stats = Stats()
        self.done = False
        self._truth_msg: Optional[TrackedShipList] = None
        self._fusion_msg: Optional[TrackedShipList] = None
        self._truth_recv = 0.0
        self._last_compared_stamp: Optional[float] = None
        self._next_print = 0.0
        self._last_warn: Dict[str, float] = {}

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            TrackedShipList, args.truth_topic, self._on_truth, qos
        )
        self.create_subscription(
            TrackedShipList, args.fusion_topic, self._on_fusion, qos
        )

        self._csv_file = None
        self._csv_writer = None
        if args.csv:
            self._csv_file = open(args.csv, 'w', newline='', encoding='utf-8')
            self._csv_writer = csv.writer(self._csv_file)
            self._csv_writer.writerow(CSV_HEADER)

    def _on_truth(self, msg: TrackedShipList) -> None:
        """Store the latest truth list.

        :param msg: Truth tracked ship list.
        :return: None
        """
        self._truth_msg = msg
        self._truth_recv = time.monotonic()

    def _on_fusion(self, msg: TrackedShipList) -> None:
        """Store the latest fusion list and run one comparison.

        :param msg: Fusion tracked ship list.
        :return: None
        """
        self._fusion_msg = msg
        self._compare()

    def _warn_throttled(self, key: str, text: str) -> None:
        """Emit a warning at most once every five seconds.

        :param key: Warning category.
        :param text: Warning text.
        :return: None
        """
        now = time.monotonic()
        if now - self._last_warn.get(key, 0.0) < 5.0:
            return
        self._last_warn[key] = now
        self.get_logger().warn(text)

    def _compare(self) -> None:
        """Compare the latest truth and fusion messages.

        :return: None
        """
        if self._truth_msg is None or self._fusion_msg is None:
            return

        now = time.monotonic()
        if now - self._truth_recv > self.args.max_truth_age:
            self.stats.stamp_skips += 1
            self._warn_throttled('truth_age', 'truth topic is stale; skipping frame')
            return

        truth_frame = self._truth_msg.header.frame_id
        fusion_frame = self._fusion_msg.header.frame_id
        expected = self.args.expected_frame
        if expected and (truth_frame != expected or fusion_frame != expected):
            self.stats.frame_skips += 1
            self._warn_throttled(
                'frame',
                f'frame mismatch truth={truth_frame or "<empty>"} '
                f'fusion={fusion_frame or "<empty>"} expected={expected}',
            )
            return

        stamp_sec = _stamp_sec(self._fusion_msg.header.stamp)
        stamp_dt = stamp_sec - _stamp_sec(self._truth_msg.header.stamp)
        if abs(stamp_dt) > self.args.max_stamp_diff:
            self.stats.stamp_skips += 1
            self._warn_throttled(
                'stamp',
                f'stamp diff {stamp_dt:+.3f}s exceeds '
                f'{self.args.max_stamp_diff:.3f}s',
            )
            return
        if stamp_sec == self._last_compared_stamp:
            return
        self._last_compared_stamp = stamp_sec

        truths = [
            _sample_from_msg(ship, truth_frame) for ship in self._truth_msg.ships
        ]
        fusions = [
            _sample_from_msg(ship, fusion_frame) for ship in self._fusion_msg.ships
        ]
        matched, unmatched_truth, unmatched_fusion = _associate(
            truths, fusions, self.args.gate_m
        )

        self.stats.frames += 1
        self.stats.unmatched_truth += len(unmatched_truth)
        self.stats.unmatched_fusion += len(unmatched_fusion)

        comparisons: List[PairComparison] = []
        for truth_index, fusion_index, distance in matched:
            truth = truths[truth_index]
            fusion = fusions[fusion_index]
            both_moving = (
                truth.speed >= self.args.min_speed
                and fusion.speed >= self.args.min_speed
            )
            comparison = PairComparison(
                stamp_sec=stamp_sec,
                stamp_dt=stamp_dt,
                truth=truth,
                fusion=fusion,
                match_distance=distance,
                speed_error=fusion.speed - truth.speed,
                course_error=(
                    _wrap_angle(fusion.course - truth.course)
                    if both_moving
                    else None
                ),
                heading_error=(
                    _wrap_angle(fusion.heading - truth.heading)
                    if both_moving
                    else None
                ),
            )
            comparisons.append(comparison)
            self.stats.add(comparison)
            if self._csv_writer is not None:
                self._write_csv(comparison)

        self._print_frame(
            comparisons,
            unmatched_truth,
            unmatched_fusion,
            truths,
            fusions,
            stamp_sec,
            stamp_dt,
        )

        if self.args.once:
            self.done = True

    def _write_csv(self, comparison: PairComparison) -> None:
        """Write one comparison row.

        :param comparison: Matched truth/fusion pair.
        :return: None
        """
        truth = comparison.truth
        fusion = comparison.fusion
        self._csv_writer.writerow([
            f'{comparison.stamp_sec:.9f}',
            f'{comparison.stamp_dt:.6f}',
            truth.frame_id,
            fusion.frame_id,
            truth.target_id,
            fusion.target_id,
            f'{comparison.match_distance:.6f}',
            f'{truth.x:.6f}',
            f'{truth.y:.6f}',
            f'{fusion.x:.6f}',
            f'{fusion.y:.6f}',
            f'{truth.vx:.6f}',
            f'{truth.vy:.6f}',
            f'{fusion.vx:.6f}',
            f'{fusion.vy:.6f}',
            f'{truth.speed:.6f}',
            f'{fusion.speed:.6f}',
            f'{comparison.speed_error:.6f}',
            f'{math.degrees(truth.course):.3f}',
            f'{math.degrees(fusion.course):.3f}',
            (
                f'{math.degrees(comparison.course_error):.3f}'
                if comparison.course_error is not None
                else ''
            ),
            f'{math.degrees(truth.heading):.3f}',
            f'{math.degrees(fusion.heading):.3f}',
            (
                f'{math.degrees(comparison.heading_error):.3f}'
                if comparison.heading_error is not None
                else ''
            ),
        ])
        self._csv_file.flush()

    def _print_frame(
        self,
        comparisons: Sequence[PairComparison],
        unmatched_truth: Sequence[int],
        unmatched_fusion: Sequence[int],
        truths: Sequence[ShipSample],
        fusions: Sequence[ShipSample],
        stamp_sec: float,
        stamp_dt: float,
    ) -> None:
        """Print one comparison frame when due.

        :param comparisons: Matched pairs with errors.
        :param unmatched_truth: Unmatched truth indices.
        :param unmatched_fusion: Unmatched fusion indices.
        :param truths: Truth samples for the current frame.
        :param fusions: Fusion samples for the current frame.
        :param stamp_sec: Fusion stamp in seconds.
        :param stamp_dt: Fusion stamp minus truth stamp in seconds.
        :return: None
        """
        now = time.monotonic()
        if not self.args.once and now < self._next_print:
            return
        self._next_print = now + max(0.0, self.args.print_period)

        truth_ids = [truths[index].target_id for index in unmatched_truth]
        fusion_ids = [fusions[index].target_id for index in unmatched_fusion]
        print(
            f'[stamp={stamp_sec:.3f}s dt={stamp_dt:+.3f}s] '
            f'matched={len(comparisons)} '
            f'unmatched_truth={truth_ids} unmatched_fusion={fusion_ids}'
        )
        for comparison in comparisons:
            if comparison.course_error is None:
                course_error = 'n/a'
            else:
                course_error = f'{math.degrees(comparison.course_error):+.3f}deg'
            if comparison.heading_error is None:
                heading_error = 'n/a'
            else:
                heading_error = f'{math.degrees(comparison.heading_error):+.3f}deg'
            print(
                f'  truth={comparison.truth.target_id} '
                f'fusion={comparison.fusion.target_id} '
                f'pos={comparison.match_distance:.3f}m | '
                f'speed T={comparison.truth.speed:.3f} '
                f'F={comparison.fusion.speed:.3f} '
                f'err={comparison.speed_error:+.3f}m/s | '
                f'course T={math.degrees(comparison.truth.course):.3f} '
                f'F={math.degrees(comparison.fusion.course):.3f} '
                f'err={course_error} | '
                f'heading T={math.degrees(comparison.truth.heading):.3f} '
                f'F={math.degrees(comparison.fusion.heading):.3f} '
                f'err={heading_error}'
            )

    def finish(self) -> None:
        """Print the summary and close output files.

        :return: None
        """
        if self._csv_file is not None:
            self._csv_file.close()

        print('\n=== tracked ship comparison summary ===')
        print(
            f'frames={self.stats.frames} matched={self.stats.matched} '
            f'unmatched_truth={self.stats.unmatched_truth} '
            f'unmatched_fusion={self.stats.unmatched_fusion} '
            f'stamp_skips={self.stats.stamp_skips} '
            f'frame_skips={self.stats.frame_skips}'
        )
        print(_format_stat('position error', self.stats.position_errors, 'm'))
        print(_format_stat('speed error (abs)', self.stats.speed_errors_abs, 'm/s'))
        print(
            _format_signed_stat(
                'speed error (signed)', self.stats.speed_errors_signed, 'm/s'
            )
        )
        print(
            _format_stat(
                'course error (abs)',
                self.stats.course_errors_abs,
                'deg',
                scale=180.0 / math.pi,
            )
        )
        print(
            _format_stat(
                'heading error (abs)',
                self.stats.heading_errors_abs,
                'deg',
                scale=180.0 / math.pi,
            )
        )


def _build_parser() -> argparse.ArgumentParser:
    """Build the command-line parser.

    :return: Configured parser.
    """
    parser = argparse.ArgumentParser(
        description=(
            'Compare truth and fused TrackedShipList topics one-to-one by '
            'position and report speed, course and heading errors.'
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument('--truth-topic', default=DEFAULT_TRUTH_TOPIC)
    parser.add_argument('--fusion-topic', default=DEFAULT_FUSION_TOPIC)
    parser.add_argument(
        '--gate-m',
        type=float,
        default=10.0,
        help='maximum truth/fusion association distance [m]',
    )
    parser.add_argument(
        '--max-stamp-diff',
        type=float,
        default=0.15,
        help='maximum allowed header stamp difference [s]',
    )
    parser.add_argument(
        '--max-truth-age',
        type=float,
        default=0.5,
        help='maximum wall age of the last truth message [s]',
    )
    parser.add_argument(
        '--min-speed',
        type=float,
        default=0.5,
        help='minimum speed for valid course/heading comparison [m/s]',
    )
    parser.add_argument(
        '--expected-frame',
        default='map',
        help='expected frame for both lists; empty disables the check',
    )
    parser.add_argument(
        '--print-period',
        type=float,
        default=1.0,
        help='seconds between printed frames; 0 prints every frame',
    )
    parser.add_argument(
        '--duration-sec',
        type=float,
        default=0.0,
        help='run time [s]; 0 runs until Ctrl+C',
    )
    parser.add_argument('--csv', default='', help='optional per-pair CSV output path')
    parser.add_argument(
        '--once',
        action='store_true',
        help='print one comparison frame and exit',
    )
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    """Run the comparison node.

    :param argv: Optional argument list for tests.
    :return: Process exit code.
    """
    parser = _build_parser()
    args = parser.parse_args(argv)
    if args.gate_m <= 0.0:
        parser.error('--gate-m must be positive')
    if args.max_stamp_diff < 0.0:
        parser.error('--max-stamp-diff must be non-negative')

    rclpy.init()
    node: Optional[TrackedShipCompare] = None
    try:
        node = TrackedShipCompare(args)
        print(
            f'truth={args.truth_topic} -> fusion={args.fusion_topic} '
            f'gate={args.gate_m:.1f}m frame={args.expected_frame or "any"}'
        )
        deadline = (
            time.monotonic() + args.duration_sec
            if args.duration_sec > 0.0
            else None
        )
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
            if deadline is not None and time.monotonic() >= deadline:
                break
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.finish()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())