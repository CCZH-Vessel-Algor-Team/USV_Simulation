#!/usr/bin/env python3
"""监听 /sim/ground_truth 与 /fusion/snapshot，统计目标数量差异。"""

from __future__ import annotations

import argparse
import signal
import sys
import time
from dataclasses import dataclass, field
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

try:
    from usv_interfaces.msg import FusedSceneSnapshot, GlobalTrackArray
except ImportError as exc:  # pragma: no cover
    print("请先 source install/setup.bash（需要 usv_interfaces）", file=sys.stderr)
    raise SystemExit(1) from exc


def _short_uuid(uuid_bytes: bytes) -> str:
    if len(uuid_bytes) >= 4:
        return uuid_bytes[:4].hex()
    return uuid_bytes.hex()


@dataclass
class SampleStats:
    samples: int = 0
    mismatch_samples: int = 0
    max_abs_delta: int = 0
    min_gt: Optional[int] = None
    min_fusion: Optional[int] = None
    max_gt: int = 0
    max_fusion: int = 0

    def update(self, gt_count: int, fusion_count: int) -> None:
        delta = gt_count - fusion_count
        self.samples += 1
        if delta != 0:
            self.mismatch_samples += 1
        self.max_abs_delta = max(self.max_abs_delta, abs(delta))
        self.min_gt = gt_count if self.min_gt is None else min(self.min_gt, gt_count)
        self.min_fusion = (
            fusion_count if self.min_fusion is None else min(self.min_fusion, fusion_count)
        )
        self.max_gt = max(self.max_gt, gt_count)
        self.max_fusion = max(self.max_fusion, fusion_count)


@dataclass
class TopicState:
    count: int = 0
    ids_text: str = "-"
    stamp_sec: float = 0.0
    recv_monotonic: float = 0.0
    msg_count: int = 0


class GtFusionCountMonitor(Node):
    def __init__(self, gt_topic: str, fusion_topic: str) -> None:
        super().__init__("gt_fusion_count_monitor")
        self._gt = TopicState()
        self._fusion = TopicState()
        self.create_subscription(GlobalTrackArray, gt_topic, self._on_gt, qos_profile_sensor_data)
        self.create_subscription(
            FusedSceneSnapshot, fusion_topic, self._on_fusion, qos_profile_sensor_data
        )

    def _on_gt(self, msg: GlobalTrackArray) -> None:
        track_ids = sorted({int(t.track_id) for t in msg.tracks})
        self._gt.count = len(track_ids)
        self._gt.ids_text = str(track_ids)
        self._gt.stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        self._gt.recv_monotonic = time.monotonic()
        self._gt.msg_count += 1

    def _on_fusion(self, msg: FusedSceneSnapshot) -> None:
        short_ids = sorted(
            _short_uuid(bytes(t.target_id.uuid))
            for t in msg.targets
        )
        self._fusion.count = len(short_ids)
        self._fusion.ids_text = str(short_ids)
        self._fusion.stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        self._fusion.recv_monotonic = time.monotonic()
        self._fusion.msg_count += 1

    def gt_ready(self) -> bool:
        return self._gt.msg_count > 0

    def fusion_ready(self) -> bool:
        return self._fusion.msg_count > 0

    def sample(self, stale_sec: float) -> dict:
        now = time.monotonic()
        gt_age = (now - self._gt.recv_monotonic) if self.gt_ready() else None
        fusion_age = (now - self._fusion.recv_monotonic) if self.fusion_ready() else None
        delta = self._gt.count - self._fusion.count
        gt_stale = gt_age is not None and gt_age > stale_sec
        fusion_stale = fusion_age is not None and fusion_age > stale_sec
        return {
            "gt_count": self._gt.count,
            "fusion_count": self._fusion.count,
            "delta": delta,
            "gt_ids": self._gt.ids_text,
            "fusion_ids": self._fusion.ids_text,
            "gt_age": gt_age,
            "fusion_age": fusion_age,
            "gt_stale": gt_stale,
            "fusion_stale": fusion_stale,
            "gt_msgs": self._gt.msg_count,
            "fusion_msgs": self._fusion.msg_count,
        }


def _format_age(age: Optional[float]) -> str:
    if age is None:
        return "n/a"
    return "%.1fs" % age


def _print_line(sample: dict, expected_gt: int) -> None:
    delta = sample["delta"]
    bad_delta = delta != 0
    bad_expected = expected_gt > 0 and sample["gt_count"] != expected_gt
    bad_stale = sample["gt_stale"] or sample["fusion_stale"]
    if bad_stale:
        status = "STALE"
    elif bad_delta or bad_expected:
        status = "DIFF"
    else:
        status = "OK"

    extra = ""
    if expected_gt > 0 and sample["gt_count"] != expected_gt:
        extra += " | gt!=expected(%d)" % expected_gt
    if sample["gt_stale"]:
        extra += " | gt_stale"
    if sample["fusion_stale"]:
        extra += " | fusion_stale"

    print(
        "[%s] gt=%d fusion=%d delta=%+d | gt_age=%s fusion_age=%s | gt_ids=%s | fusion_ids=%s%s"
        % (
            status,
            sample["gt_count"],
            sample["fusion_count"],
            delta,
            _format_age(sample["gt_age"]),
            _format_age(sample["fusion_age"]),
            sample["gt_ids"],
            sample["fusion_ids"],
            extra,
        )
    )


def _wait_topics(node: GtFusionCountMonitor, wait_sec: float) -> bool:
    t0 = time.monotonic()
    while time.monotonic() - t0 < wait_sec:
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.gt_ready() and node.fusion_ready():
            return True
    return node.gt_ready() or node.fusion_ready()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="监听 ground truth 与 fusion snapshot 的目标数量差异",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--gt-topic", default="/sim/ground_truth")
    parser.add_argument("--fusion-topic", default="/fusion/snapshot")
    parser.add_argument("--interval-sec", type=float, default=2.0, help="打印间隔 [s]")
    parser.add_argument(
        "--duration-sec",
        type=float,
        default=0.0,
        help="监测时长 [s]；0 表示持续运行直到 Ctrl+C",
    )
    parser.add_argument("--wait-topic-sec", type=float, default=30.0, help="启动时等待首条消息 [s]")
    parser.add_argument(
        "--stale-sec",
        type=float,
        default=5.0,
        help="超过该时间未收到新消息则标记 STALE",
    )
    parser.add_argument(
        "--expected-gt",
        type=int,
        default=0,
        help="期望 ground truth 目标数；0 表示不校验",
    )
    parser.add_argument("--once", action="store_true", help="收到两路数据后打印一次并退出")
    args = parser.parse_args()

    rclpy.init()
    node = GtFusionCountMonitor(args.gt_topic, args.fusion_topic)
    stats = SampleStats()
    stop = {"flag": False}

    def _handle_sigint(_signum, _frame) -> None:
        stop["flag"] = True

    signal.signal(signal.SIGINT, _handle_sigint)
    signal.signal(signal.SIGTERM, _handle_sigint)

    print(
        "开始监听 gt=%s fusion=%s interval=%.1fs duration=%s expected_gt=%s"
        % (
            args.gt_topic,
            args.fusion_topic,
            args.interval_sec,
            "∞" if args.duration_sec <= 0 else "%.0fs" % args.duration_sec,
            args.expected_gt if args.expected_gt > 0 else "off",
        )
    )
    print(
        "说明: delta = gt - fusion；fusion 仅发布已确认航迹，启动阶段或 tentative 航迹会导致 delta>0"
    )

    if not _wait_topics(node, args.wait_topic_sec):
        print(
            "警告: 等待超时，gt_ready=%s fusion_ready=%s"
            % (node.gt_ready(), node.fusion_ready()),
            file=sys.stderr,
        )
        if not node.gt_ready() and not node.fusion_ready():
            node.destroy_node()
            rclpy.shutdown()
            return 2

    end_time = None
    if args.duration_sec > 0:
        end_time = time.monotonic() + args.duration_sec
    if args.once:
        end_time = time.monotonic()

    next_print = 0.0
    exit_code = 0
    try:
        while not stop["flag"]:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if end_time is not None and now >= end_time:
                if args.once and next_print == 0.0:
                    pass
                else:
                    break

            if now < next_print:
                continue

            sample = node.sample(args.stale_sec)
            stats.update(sample["gt_count"], sample["fusion_count"])
            _print_line(sample, args.expected_gt)
            next_print = now + max(0.5, args.interval_sec)

            if args.once:
                break
    finally:
        print(
            "统计: samples=%d mismatch=%d mismatch_ratio=%.1f%% max_abs_delta=%d "
            "gt=[%s..%s] fusion=[%s..%s]"
            % (
                stats.samples,
                stats.mismatch_samples,
                (100.0 * stats.mismatch_samples / stats.samples) if stats.samples else 0.0,
                stats.max_abs_delta,
                stats.min_gt if stats.min_gt is not None else "-",
                stats.max_gt,
                stats.min_fusion if stats.min_fusion is not None else "-",
                stats.max_fusion,
            )
        )
        if stats.mismatch_samples > 0:
            exit_code = 1
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
