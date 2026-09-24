#!/usr/bin/env python3
"""CCS / USV 仿真相关进程 CPU 占用采样（Linux /proc 差分）。

不依赖 ROS graph 发现；仅读取 /proc，适合 FastDDS 节点列表不全时做进程级画像。
100% = 1 个逻辑核（多线程进程可超过 100%）。
"""

from __future__ import annotations

import argparse
import os
import re
import sys
import time
from collections import defaultdict
from typing import Dict, List, Optional, Sequence, Tuple

# 进程 cmdline 命中任一关键词即纳入采样（大小写不敏感）。
DEFAULT_KEYWORDS: Tuple[str, ...] = (
    "gz sim",
    "gz-sim",
    "nav2_",
    "usv_",
    "late_fusion",
    "component_container",
    "robot_state_publisher",
    "parameter_bridge",
    "rviz",
    "ground_truth",
    "enc_ground",
    "radar_",
    "mmwave",
    "convert_to_track",
    "route_planner",
    "controller_server",
    "planner_server",
    "bt_navigator",
    "smoother_server",
    "behavior_server",
    "velocity_smoother",
    "waypoint_follower",
    "map_server",
    "lifecycle",
    "scenario_manager",
    "dynamic_ship",
    "dynamic_buoy",
    "storm_field",
    "tf_namespace",
    "odom_tf",
    "gz_spawn",
    "usv_sim_wrapper",
    "ros_gz",
    "colregs",
    "costmap",
    "maritime",
    "sim_vision",
    "sim_mmwave",
    "sim_ais",
    "tracked_ship",
    "depth_provider",
    "grounding",
    "ukc_",
    "hull_draft",
    "adaptive_radar",
    "vector_object",
    "target_snapshot",
    "route_depth",
    "static_transform",
    "clearing_scan",
    "cmd_vel_to_thruster",
    "ais_aggregator",
    "ais_perception",
)

DEFAULT_PATH_MARKERS: Tuple[str, ...] = (
    "/install/",
    "/opt/ros/humble/lib/",
)


def _clk_tck() -> float:
    return float(os.sysconf(os.sysconf_names["SC_CLK_TCK"]))


def _read_cmdline(pid: int) -> str:
    with open(f"/proc/{pid}/cmdline", "rb") as f:
        raw = f.read()
    return raw.replace(b"\x00", b" ").decode("utf-8", "ignore").strip()


def _read_cpu_jiffies(pid: int) -> int:
    with open(f"/proc/{pid}/stat", encoding="utf-8") as f:
        st = f.read().split()
    # utime(13) + stime(14), 0-based index after comm field handling:
    # /proc/pid/stat: pid (comm) state ... utime stime → fields[13], fields[14]
    return int(st[13]) + int(st[14])


def _interesting(
    cmd: str,
    keywords: Sequence[str],
    path_markers: Sequence[str],
    exclude_substrings: Sequence[str],
) -> bool:
    if not cmd:
        return False
    for ex in exclude_substrings:
        if ex and ex in cmd:
            return False
    cl = cmd.lower()
    for marker in path_markers:
        if marker and marker.lower() in cl:
            return True
    for key in keywords:
        if key and key.lower() in cl:
            return True
    return False


def short_name(cmd: str) -> str:
    node_m = re.search(r"__node:=(\S+)", cmd)
    ns_m = re.search(r"__ns:=(\S+)", cmd)
    node = node_m.group(1) if node_m else ""
    nsp = ns_m.group(1) if ns_m else ""
    if node:
        return f"{nsp}/{node}".replace("//", "/") if nsp else node
    if "gz sim gui" in cmd:
        return "gz-sim-gui"
    if "gz sim server" in cmd:
        return "gz-sim-server"
    if "gz" in cmd and "sim" in cmd:
        return "gz-sim"
    parts = cmd.split()
    exe = os.path.basename(parts[0]) if parts else "unknown"
    for p in parts:
        if "/lib/" in p or p.endswith(".py"):
            exe = os.path.basename(p)
            break
    return exe


def categorize(name: str, cmd: str) -> str:
    n = f"{name} {cmd}".lower()
    if "gz-sim" in n or "gz sim" in n:
        return "Gazebo"
    if "rviz" in n:
        return "RViz"
    if "late_fusion" in n:
        return "late_fusion"
    if "sim_vision" in n:
        return "sim_vision"
    if any(
        x in n
        for x in (
            "controller_server",
            "planner_server",
            "bt_navigator",
            "costmap",
            "behavior_server",
            "velocity_smoother",
            "waypoint_follower",
            "smoother_server",
            "lifecycle_manager",
            "map_server",
            "nav2_",
        )
    ):
        return "Nav2"
    if any(x in n for x in ("ground_truth", "dynamic_ship", "scenario", "storm_field", "dynamic_buoy")):
        return "GT/scenario/ships"
    if any(x in n for x in ("maritime", "colregs", "vector_object")):
        return "COLREGS/situation"
    if "mmwave" in n:
        return "mmwave"
    if "radar" in n:
        return "radar"
    if any(
        x in n
        for x in ("enc_ground", "depth_", "grounding", "ukc_", "route_depth", "hull_draft")
    ):
        return "ENC/grounding"
    if "parameter_bridge" in n or "ros_gz" in n:
        return "ros_gz_bridge"
    if any(x in n for x in ("tf_", "odom_tf", "robot_state", "static_transform")):
        return "TF/state"
    if any(x in n for x in ("convert_to_track", "target_snapshot", "tracked_ship", "ais_")):
        return "track/AIS pipeline"
    if "route_planner" in n:
        return "route_planner"
    return "other"


def snapshot_processes(
    keywords: Sequence[str],
    path_markers: Sequence[str],
    exclude_substrings: Sequence[str],
) -> Dict[int, Tuple[int, str, str]]:
    snaps: Dict[int, Tuple[int, str, str]] = {}
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        pid = int(entry)
        try:
            cmd = _read_cmdline(pid)
            if not _interesting(cmd, keywords, path_markers, exclude_substrings):
                continue
            jiffies = _read_cpu_jiffies(pid)
            snaps[pid] = (jiffies, short_name(cmd), cmd[:240])
        except (FileNotFoundError, ProcessLookupError, PermissionError, ValueError, OSError):
            continue
    return snaps


def profile(
    duration_sec: float,
    min_pct: float,
    keywords: Sequence[str],
    path_markers: Sequence[str],
    exclude_substrings: Sequence[str],
) -> Tuple[List[Tuple[float, int, str, str]], Dict[str, float], Dict[str, float]]:
    hz = _clk_tck()
    snaps = snapshot_processes(keywords, path_markers, exclude_substrings)
    if not snaps:
        return [], {}, {}

    time.sleep(duration_sec)

    rows: List[Tuple[float, int, str, str]] = []
    for pid, (u0, name, cmd) in snaps.items():
        try:
            u1 = _read_cpu_jiffies(pid)
        except (FileNotFoundError, ProcessLookupError, PermissionError, ValueError, OSError):
            continue
        pct = 100.0 * (u1 - u0) / hz / duration_sec
        if pct >= min_pct:
            rows.append((pct, pid, name, cmd))
    rows.sort(reverse=True)

    by_name: Dict[str, float] = defaultdict(float)
    by_cat: Dict[str, float] = defaultdict(float)
    for pct, _pid, name, cmd in rows:
        by_name[name] += pct
        by_cat[categorize(name, cmd)] += pct
    return rows, dict(by_name), dict(by_cat)


def _parse_csv_list(value: Optional[str]) -> List[str]:
    if not value:
        return []
    return [x.strip() for x in value.split(",") if x.strip()]


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Sample CPU usage of CCS/USV-related processes via Linux /proc.",
    )
    p.add_argument(
        "--duration",
        type=float,
        default=30.0,
        help="Sampling window in seconds (default: 30).",
    )
    p.add_argument(
        "--min-pct",
        type=float,
        default=0.4,
        help="Omit processes below this average CPU%% (default: 0.4).",
    )
    p.add_argument(
        "--top",
        type=int,
        default=55,
        help="Max rows in per-process table (default: 55; 0 = all).",
    )
    p.add_argument(
        "--path-marker",
        action="append",
        default=None,
        help=(
            "Extra cmdline path substring to include "
            "(repeatable). Defaults include /install/ and /opt/ros/humble/lib/."
        ),
    )
    p.add_argument(
        "--keyword",
        action="append",
        default=None,
        help="Extra cmdline keyword to include (repeatable).",
    )
    p.add_argument(
        "--exclude",
        default="ccs_cpu_profile",
        help="Comma-separated cmdline substrings to exclude (default: ccs_cpu_profile).",
    )
    p.add_argument(
        "--tsv",
        default="",
        help="Optional path to write TSV: cpu\\tpid\\tname\\tcmd",
    )
    p.add_argument(
        "--no-category",
        action="store_true",
        help="Skip category aggregation section.",
    )
    return p


def main(argv: Optional[Sequence[str]] = None) -> int:
    if not sys.platform.startswith("linux"):
        print("错误：本工具依赖 Linux /proc，当前平台不支持。", file=sys.stderr)
        return 2

    args = build_arg_parser().parse_args(argv)
    if args.duration <= 0:
        print("错误：--duration 必须 > 0", file=sys.stderr)
        return 2

    path_markers = list(DEFAULT_PATH_MARKERS)
    if args.path_marker:
        path_markers.extend(args.path_marker)

    keywords = list(DEFAULT_KEYWORDS)
    if args.keyword:
        keywords.extend(args.keyword)

    excludes = _parse_csv_list(args.exclude)
    # Always skip this tool itself if invoked as python .../ccs_cpu_profile.py
    if "ccs_cpu_profile" not in excludes:
        excludes.append("ccs_cpu_profile")

    print(
        f"sampling duration={args.duration:.1f}s min_pct={args.min_pct} "
        f"path_markers={path_markers[:3]}{'...' if len(path_markers) > 3 else ''}"
    )
    pre = snapshot_processes(keywords, path_markers, excludes)
    print(f"tracking {len(pre)} processes...")
    if not pre:
        print("未匹配到进程。请确认仿真已启动，或用 --path-marker / --keyword 放宽过滤。")
        return 1

    rows, by_name, by_cat = profile(
        duration_sec=args.duration,
        min_pct=args.min_pct,
        keywords=keywords,
        path_markers=path_markers,
        exclude_substrings=excludes,
    )

    shown = rows if args.top <= 0 else rows[: args.top]

    print(f"sample {args.duration:.0f}s; CPU>={args.min_pct}%")
    print(f"{'CPU%':>8} {'PID':>7} NAME")
    print("-" * 78)
    for pct, pid, name, _cmd in shown:
        print(f"{pct:8.1f} {pid:7d} {name}")

    print("\n=== aggregated by name ===")
    print(f"{'CPU%':>8} NAME")
    print("-" * 78)
    for name, pct in sorted(by_name.items(), key=lambda x: -x[1])[:35]:
        print(f"{pct:8.1f} {name}")
    total = sum(by_name.values())
    print(f"\nsum≈{total:.1f}% (100%=1 core)")

    if not args.no_category:
        print("\n=== by category ===")
        print(f"{'CPU%':>8} CATEGORY")
        print("-" * 78)
        for name, pct in sorted(by_cat.items(), key=lambda x: -x[1]):
            print(f"{pct:8.1f} {name}")

    if args.tsv:
        out_dir = os.path.dirname(os.path.abspath(args.tsv))
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)
        with open(args.tsv, "w", encoding="utf-8") as f:
            f.write("cpu\tpid\tname\tcmd\n")
            for pct, pid, name, cmd in rows:
                safe_cmd = cmd.replace("\t", " ").replace("\n", " ")
                f.write(f"{pct:.2f}\t{pid}\t{name}\t{safe_cmd}\n")
        print(f"\nTSV written: {args.tsv} ({len(rows)} rows)")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
