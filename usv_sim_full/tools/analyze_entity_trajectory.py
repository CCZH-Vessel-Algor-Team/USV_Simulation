#!/usr/bin/env python3
"""采样 /sim/ground_truth：直线 / ping-pong 折返轨迹分析与非预期倒退检测。"""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

try:
    from usv_interfaces.msg import GlobalTrackArray
except ImportError as exc:  # pragma: no cover
    print("请先 source install/setup.bash（需要 usv_interfaces）", file=sys.stderr)
    raise SystemExit(1) from exc


@dataclass
class RouteMeta:
    track_id: int
    waypoints: List[Tuple[float, float]]
    loop: bool


@dataclass
class TrackSample:
    t_sec: float
    x: float
    y: float
    vx: float
    vy: float


@dataclass
class TrackStats:
    track_id: int
    samples: List[TrackSample] = field(default_factory=list)
    min_x: float = float("inf")
    max_x: float = float("-inf")
    min_y: float = float("inf")
    max_y: float = float("-inf")
    turnaround_events: int = 0
    unexpected_backward_events: int = 0
    max_backward_speed: float = 0.0
    max_position_jump_m: float = 0.0
    reached_far: bool = False
    returned: bool = False

    def add_sample(self, s: TrackSample) -> None:
        self.samples.append(s)
        self.min_x = min(self.min_x, s.x)
        self.max_x = max(self.max_x, s.x)
        self.min_y = min(self.min_y, s.y)
        self.max_y = max(self.max_y, s.y)


def load_routes(config_path: str) -> Dict[int, RouteMeta]:
    with open(config_path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f) or {}
    gt = (cfg.get("scenario") or {}).get("ground_truth_sim") or {}
    routes: Dict[int, RouteMeta] = {}
    for entry in gt.get("fixed_targets") or []:
        if not isinstance(entry, dict):
            continue
        tid = int(entry["track_id"])
        wps = [(float(wp[0]), float(wp[1])) for wp in (entry.get("waypoints") or [])]
        if len(wps) >= 2:
            loop = bool(entry.get("loop", True))
            routes[tid] = RouteMeta(track_id=tid, waypoints=wps, loop=loop)
    return routes


def _dist_to_nearest_waypoint(x: float, y: float, wps: List[Tuple[float, float]]) -> float:
    return min(math.hypot(x - wx, y - wy) for wx, wy in wps)


def analyze_straight(
    st: TrackStats,
    route: RouteMeta,
    *,
    backward_speed_threshold: float,
) -> None:
    wps = route.waypoints
    x0, y0 = wps[0]
    x1, y1 = wps[1]
    dx, dy = x1 - x0, y1 - y0
    norm = math.hypot(dx, dy)
    if norm < 1e-6:
        return
    sx, sy = dx / norm, dy / norm
    prev_x: Optional[float] = None
    prev_y: Optional[float] = None
    for s in st.samples:
        along_speed = s.vx * sx + s.vy * sy
        if along_speed < -backward_speed_threshold:
            st.unexpected_backward_events += 1
            st.max_backward_speed = min(st.max_backward_speed, along_speed)
        if prev_x is not None:
            jump = math.hypot(s.x - prev_x, s.y - prev_y)
            st.max_position_jump_m = max(st.max_position_jump_m, jump)
        prev_x, prev_y = s.x, s.y


def analyze_pingpong(
    st: TrackStats,
    route: RouteMeta,
    *,
    backward_speed_threshold: float,
    turn_zone_m: float,
    endpoint_reach_ratio: float,
) -> None:
    wps = route.waypoints
    if len(wps) < 2:
        return

    xs = [wp[0] for wp in wps]
    ys = [wp[1] for wp in wps]
    span_x = max(xs) - min(xs)
    span_y = max(ys) - min(ys)
    use_x = span_x >= span_y

    prev_x: Optional[float] = None
    prev_t: Optional[float] = None
    prev_vx_sign: Optional[int] = None
    reached_far = False
    returned = False

    for s in st.samples:
        coord = s.x if use_x else s.y
        vel = s.vx if use_x else s.vy
        near_end = min(xs) if use_x else min(ys)
        far_end = max(xs) if use_x else max(ys)
        span = far_end - near_end
        reach_coord = far_end - (1.0 - endpoint_reach_ratio) * span
        return_coord = near_end + (1.0 - endpoint_reach_ratio) * span

        if coord >= reach_coord:
            reached_far = True
        if reached_far and coord <= return_coord:
            returned = True

        in_turn = _dist_to_nearest_waypoint(s.x, s.y, wps) <= turn_zone_m

        if prev_x is not None and prev_t is not None:
            dt = s.t_sec - prev_t
            if dt > 1e-6:
                dx_dt = (s.x - prev_x) / dt
                dy_dt = (s.y - prev_y) / dt if prev_y is not None else 0.0
                motion = dx_dt if use_x else dy_dt
                if not in_turn and abs(motion) > 0.4:
                    if motion > 0.4 and vel < -backward_speed_threshold:
                        st.unexpected_backward_events += 1
                        st.max_backward_speed = min(st.max_backward_speed, vel)
                    elif motion < -0.4 and vel > backward_speed_threshold:
                        st.unexpected_backward_events += 1
                        st.max_backward_speed = max(st.max_backward_speed, vel)
                jump = math.hypot(s.x - prev_x, s.y - (prev_y if prev_y is not None else s.y))
                st.max_position_jump_m = max(st.max_position_jump_m, jump)

        if not in_turn and abs(vel) > 0.5:
            sign = 1 if vel > 0 else -1
            if prev_vx_sign is not None and sign != prev_vx_sign:
                st.turnaround_events += 1
            prev_vx_sign = sign

        prev_x, prev_y, prev_t = s.x, s.y, s.t_sec

    st.reached_far = reached_far
    st.returned = returned


class TrajectoryMonitor(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("analyze_entity_trajectory")
        self._stats: Dict[int, TrackStats] = {}
        self._start = time.monotonic()
        self.create_subscription(
            GlobalTrackArray,
            topic,
            self._on_tracks,
            qos_profile_sensor_data,
        )

    def _on_tracks(self, msg: GlobalTrackArray) -> None:
        now = time.monotonic() - self._start
        for tr in msg.tracks:
            tid = int(tr.track_id)
            st = self._stats.setdefault(tid, TrackStats(track_id=tid))
            st.add_sample(
                TrackSample(
                    t_sec=now,
                    x=float(tr.x),
                    y=float(tr.y),
                    vx=float(tr.v_x),
                    vy=float(tr.v_y),
                )
            )

    @property
    def stats(self) -> Dict[int, TrackStats]:
        return self._stats


def main() -> int:
    parser = argparse.ArgumentParser(description="分析 Gazebo 实体轨迹折返/回退")
    parser.add_argument("--config", required=True)
    parser.add_argument("--topic", default="/sim/ground_truth")
    parser.add_argument("--mode", choices=("auto", "straight", "pingpong"), default="auto")
    parser.add_argument("--duration-sec", type=float, default=120.0)
    parser.add_argument("--wait-topic-sec", type=float, default=30.0)
    parser.add_argument("--backward-speed-threshold", type=float, default=0.5)
    parser.add_argument("--jump-threshold-m", type=float, default=8.0)
    parser.add_argument("--min-samples", type=int, default=200)
    parser.add_argument("--turn-zone-m", type=float, default=20.0)
    parser.add_argument("--endpoint-reach-ratio", type=float, default=0.75)
    args = parser.parse_args()

    routes = load_routes(args.config)
    if not routes:
        print("FAIL: 配置中无 fixed_targets 航路", file=sys.stderr)
        return 1

    rclpy.init()
    topic = args.topic if args.topic.startswith("/") else "/" + args.topic
    node = TrajectoryMonitor(topic)

    deadline = time.monotonic() + args.wait_topic_sec
    while time.monotonic() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.stats:
            break
    if not node.stats:
        print("FAIL: 未收到 /sim/ground_truth", file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    end_at = time.monotonic() + args.duration_sec
    while time.monotonic() < end_at and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)

    failed = False
    print("== 轨迹分析 ==")
    for tid, route in sorted(routes.items()):
        st = node.stats.get(tid)
        if st is None or len(st.samples) < args.min_samples:
            print(
                "FAIL track %d: 样本不足 (got=%d, need>=%d)"
                % (tid, 0 if st is None else len(st.samples), args.min_samples)
            )
            failed = True
            continue

        mode = args.mode
        if mode == "auto":
            mode = "pingpong" if route.loop and len(route.waypoints) == 2 else "straight"

        if mode == "pingpong":
            analyze_pingpong(
                st,
                route,
                backward_speed_threshold=args.backward_speed_threshold,
                turn_zone_m=args.turn_zone_m,
                endpoint_reach_ratio=args.endpoint_reach_ratio,
            )
            print(
                "track %d [pingpong]: samples=%d x=[%.1f, %.1f] turnarounds=%d "
                "unexpected_backward=%d max_jump=%.2f m reached_far=%s returned=%s"
                % (
                    tid,
                    len(st.samples),
                    st.min_x,
                    st.max_x,
                    st.turnaround_events,
                    st.unexpected_backward_events,
                    st.max_position_jump_m,
                    st.reached_far,
                    st.returned,
                )
            )
            if not st.reached_far:
                print("  FAIL: 未到达远端航点（折返前）")
                failed = True
            if not st.returned:
                print("  FAIL: 折返后未回到近端区域")
                failed = True
            if st.turnaround_events < 1:
                print("  FAIL: 未检测到速度方向反转（折返）")
                failed = True
        else:
            analyze_straight(
                st,
                route,
                backward_speed_threshold=args.backward_speed_threshold,
            )
            progress = st.max_x - st.min_x if abs(route.waypoints[1][0] - route.waypoints[0][0]) >= abs(
                route.waypoints[1][1] - route.waypoints[0][1]
            ) else st.max_y - st.min_y
            print(
                "track %d [straight]: samples=%d span=%.1f unexpected_backward=%d max_jump=%.2f m"
                % (tid, len(st.samples), progress, st.unexpected_backward_events, st.max_position_jump_m)
            )
            if progress < 5.0:
                print("  FAIL: 前进距离过短")
                failed = True

        if st.unexpected_backward_events > 0:
            print("  FAIL: 检测到非预期倒退（航段中部与运动方向相反）")
            failed = True
        if st.max_position_jump_m > args.jump_threshold_m:
            print("  FAIL: 单步位置跳变过大")
            failed = True

    node.destroy_node()
    rclpy.shutdown()
    if failed:
        print("RESULT: FAIL")
        return 1
    print("RESULT: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
