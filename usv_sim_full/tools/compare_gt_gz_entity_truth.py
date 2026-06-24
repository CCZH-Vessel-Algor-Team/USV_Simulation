#!/usr/bin/env python3
"""Compare published /sim/ground_truth XY with Gazebo gt_ctrv_* poses (entity-authoritative mode)."""

from __future__ import annotations

import argparse
import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy

try:
    from usv_interfaces.msg import GlobalTrackArray
except ImportError as exc:  # pragma: no cover
    print("请先 source install/setup.bash", file=sys.stderr)
    raise SystemExit(1) from exc

try:
    import gz.transport13 as gz_transport
    from gz.msgs10.pose_v_pb2 import Pose_V
except ImportError:  # pragma: no cover
    gz_transport = None  # type: ignore
    Pose_V = None  # type: ignore


def _base_model_name(entity_name: str) -> str:
    return entity_name.split("::", 1)[0] if entity_name else ""


class GzPoseCache:
    """Thread-safe latest Gazebo model XY from pose/info."""

    def __init__(self, world: str, prefix: str) -> None:
        self._prefix = prefix
        self._lock = threading.Lock()
        self._poses: dict[str, tuple[float, float]] = {}
        self._node = gz_transport.Node() if gz_transport is not None else None
        if self._node is not None and Pose_V is not None:
            topic = f"/world/{world}/pose/info"
            self._node.subscribe(Pose_V, topic, self._on_pose_v)

    def _on_pose_v(self, msg: Pose_V) -> None:
        fresh: dict[str, tuple[float, float]] = {}
        for entry in msg.pose:
            base = _base_model_name(entry.name)
            if base.startswith(self._prefix):
                fresh[base] = (float(entry.position.x), float(entry.position.y))
        if not fresh:
            return
        with self._lock:
            self._poses.update(fresh)

    def snapshot(self) -> dict[str, tuple[float, float]]:
        with self._lock:
            return dict(self._poses)

    def wait_until(self, names: set[str], timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            with self._lock:
                if names.issubset(self._poses.keys()):
                    return True
            time.sleep(0.05)
        return False


class TruthCompareNode(Node):
    def __init__(self, topic: str, gz_cache: GzPoseCache) -> None:
        super().__init__("compare_gt_gz_entity_truth")
        self._gz_cache = gz_cache
        self._latest: GlobalTrackArray | None = None
        self._gz_snapshot: dict[str, tuple[float, float]] = {}
        self.create_subscription(
            GlobalTrackArray,
            topic,
            self._on_msg,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
        )

    def _on_msg(self, msg: GlobalTrackArray) -> None:
        self._latest = msg
        self._gz_snapshot = self._gz_cache.snapshot()


def main() -> int:
    parser = argparse.ArgumentParser(description="对比发布的 truth 与 Gazebo pose")
    parser.add_argument("--topic", default="/sim/ground_truth")
    parser.add_argument("--prefix", default="gt_ctrv_")
    parser.add_argument("--world", default="sydney_regatta_open_water")
    parser.add_argument("--samples", type=int, default=5)
    parser.add_argument("--interval-sec", type=float, default=2.0)
    parser.add_argument("--max-dxy-m", type=float, default=0.5)
    parser.add_argument("--wait-topic-sec", type=float, default=45.0)
    args = parser.parse_args()

    if gz_transport is None or Pose_V is None:
        print("gz.transport 不可用，无法做同步 pose 对比", file=sys.stderr)
        return 2

    gz_cache = GzPoseCache(args.world, args.prefix)

    rclpy.init()
    node = TruthCompareNode(args.topic, gz_cache)
    node.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])

    t0 = time.monotonic()
    while node._latest is None and (time.monotonic() - t0) < args.wait_topic_sec:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node._latest is None:
        print("超时：未收到", args.topic)
        node.destroy_node()
        rclpy.shutdown()
        return 2

    max_errors: list[float] = []
    for i in range(args.samples):
        for _ in range(30):
            rclpy.spin_once(node, timeout_sec=0.01)
        msg = node._latest
        assert msg is not None
        want = {"%s%d" % (args.prefix, int(tr.track_id)) for tr in msg.tracks}
        gz_cache.wait_until(want, timeout_sec=1.0)
        gz = node._gz_snapshot if node._gz_snapshot else gz_cache.snapshot()
        sample_max = 0.0
        print("\n--- sample %d ---" % i)
        for tr in msg.tracks:
            tid = int(tr.track_id)
            name = "%s%d" % (args.prefix, tid)
            tx, ty = float(tr.x), float(tr.y)
            if name not in gz:
                print("  T%d: truth=(%.2f,%.2f) GZ=MISSING" % (tid, tx, ty))
                sample_max = max(sample_max, 999.0)
                continue
            gx, gy = gz[name]
            d = math.hypot(tx - gx, ty - gy)
            sample_max = max(sample_max, d)
            print(
                "  T%d: truth=(%.2f,%.2f) gz=(%.2f,%.2f) dXY=%.3fm"
                % (tid, tx, ty, gx, gy, d)
            )
        max_errors.append(sample_max)
        end = time.monotonic() + args.interval_sec
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)

    overall = max(max_errors) if max_errors else 999.0
    print("\n汇总 max dXY per sample:", [round(e, 3) for e in max_errors])
    print("overall max=%.3fm threshold=%.3fm" % (overall, args.max_dxy_m))

    node.destroy_node()
    rclpy.shutdown()
    # 避免 gz.transport 析构时段错误（compare 为短生命周期 CLI）
    gz_cache._node = None  # type: ignore[attr-defined]
    return 0 if overall <= args.max_dxy_m else 1


if __name__ == "__main__":
    raise SystemExit(main())
