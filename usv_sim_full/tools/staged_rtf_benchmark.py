#!/usr/bin/env python3
import argparse
import csv
import datetime as dt
import json
import os
import signal
import statistics
import subprocess
import sys
import tempfile
import time
from dataclasses import dataclass
from typing import Dict, List, Optional

import yaml


def _default_usv_sim_full_config() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory

        share = get_package_share_directory('usv_sim_full')
        return os.path.join(share, 'config', 'full_config.yaml')
    except Exception:
        return os.path.normpath(
            os.path.join(os.path.dirname(__file__), '..', 'config', 'full_config.yaml')
        )


DEFAULT_LAUNCH = "ros2 launch usv_sim_full nav2_sim_full_bringup.launch.py"
DEFAULT_CONFIG = _default_usv_sim_full_config()
DEFAULT_WORLD_STATS_TOPIC = "/world/sydney_regatta/stats"


@dataclass
class Stage:
    name: str
    description: str
    enable_ground_truth: bool
    enable_maritime_radar: bool
    enable_mmwave: bool
    launch_rviz: bool = False


DEFAULT_STAGES = [
    Stage(
        name="stage1_core",
        description="核心仿真：关闭真值可视化、海事雷达、毫米波",
        enable_ground_truth=False,
        enable_maritime_radar=False,
        enable_mmwave=False,
    ),
    Stage(
        name="stage2_ground_truth",
        description="在核心基础上启用 ground_truth_sim",
        enable_ground_truth=True,
        enable_maritime_radar=False,
        enable_mmwave=False,
    ),
    Stage(
        name="stage3_full",
        description="完整链路：ground_truth + 海事雷达 + 毫米波",
        enable_ground_truth=True,
        enable_maritime_radar=True,
        enable_mmwave=True,
    ),
    Stage(
        name="stage4_maritime_radar",
        description="在 stage2 基础上启用海事雷达链路",
        enable_ground_truth=True,
        enable_maritime_radar=True,
        enable_mmwave=False,
    ),
]


def deep_copy_config(cfg: dict) -> dict:
    return json.loads(json.dumps(cfg))


def _toggle_sensor_types(cfg: dict, sensor_types: List[str], enabled: bool) -> None:
    for key, value in cfg.items():
        if not (isinstance(key, str) and key.startswith("robot_") and isinstance(value, dict)):
            continue
        sensors = value.get("sensors", [])
        if not isinstance(sensors, list):
            continue
        for sensor in sensors:
            if not isinstance(sensor, dict):
                continue
            stype = str(sensor.get("type", "")).strip().lower()
            if stype in sensor_types:
                sensor["enabled"] = bool(enabled)


def build_stage_config(base_cfg: dict, stage: Stage) -> dict:
    cfg = deep_copy_config(base_cfg)
    visualization = cfg.setdefault("visualization", {})
    if isinstance(visualization, dict):
        visualization["launch_rviz"] = bool(stage.launch_rviz)

    scenario = cfg.setdefault("scenario", {})
    if isinstance(scenario, dict):
        gt = scenario.setdefault("ground_truth_sim", {})
        if isinstance(gt, dict):
            gt["enabled"] = bool(stage.enable_ground_truth)
            if not stage.enable_ground_truth:
                gt["gazebo_visual"] = False

    _toggle_sensor_types(cfg, ["maritime_radar", "radar"], stage.enable_maritime_radar)
    _toggle_sensor_types(cfg, ["mmwave_radar", "mmwave"], stage.enable_mmwave)
    return cfg


def percentile(sorted_values: List[float], p: float) -> Optional[float]:
    if not sorted_values:
        return None
    idx = int(p * (len(sorted_values) - 1))
    return sorted_values[idx]


def run_cmd_capture(cmd: str) -> str:
    return subprocess.check_output(["bash", "-lc", cmd], text=True, stderr=subprocess.STDOUT)


def parse_rtf_from_chunk(text: str) -> List[float]:
    vals = []
    for line in text.splitlines():
        line = line.strip()
        if line.startswith("real_time_factor:"):
            try:
                vals.append(float(line.split(":", 1)[1].strip()))
            except ValueError:
                pass
    return vals


def get_resource_snapshot(process_patterns: List[str]) -> Dict[str, float]:
    ps_cmd = "ps -eo pid,comm,%cpu,%mem,rss,args --sort=-%cpu"
    out = run_cmd_capture(ps_cmd)
    cpu_total = 0.0
    mem_percent_total = 0.0
    rss_kb_total = 0.0
    matched = 0

    for line in out.splitlines()[1:]:
        parts = line.strip().split(None, 5)
        if len(parts) < 6:
            continue
        _, comm, cpu, memp, rss, args = parts
        if any(pat in args for pat in process_patterns):
            try:
                cpu_total += float(cpu)
                mem_percent_total += float(memp)
                rss_kb_total += float(rss)
                matched += 1
            except ValueError:
                continue

    meminfo = {}
    with open("/proc/meminfo", "r", encoding="utf-8") as f:
        for line in f:
            if ":" not in line:
                continue
            k, v = line.split(":", 1)
            meminfo[k.strip()] = v.strip()

    loadavg = open("/proc/loadavg", "r", encoding="utf-8").read().strip().split()
    return {
        "matched_process_count": float(matched),
        "proc_cpu_sum_percent": cpu_total,
        "proc_mem_sum_percent": mem_percent_total,
        "proc_rss_sum_gb": rss_kb_total / 1024.0 / 1024.0,
        "loadavg_1m": float(loadavg[0]),
        "loadavg_5m": float(loadavg[1]),
        "loadavg_15m": float(loadavg[2]),
        "mem_total_gb": float(meminfo.get("MemTotal", "0 kB").split()[0]) / 1024.0 / 1024.0,
        "mem_available_gb": float(meminfo.get("MemAvailable", "0 kB").split()[0]) / 1024.0 / 1024.0,
    }


def summarize(values: List[float]) -> Dict[str, Optional[float]]:
    if not values:
        return {
            "avg": None,
            "median": None,
            "min": None,
            "max": None,
            "p10": None,
            "p90": None,
        }
    sv = sorted(values)
    return {
        "avg": statistics.mean(values),
        "median": statistics.median(values),
        "min": min(values),
        "max": max(values),
        "p10": percentile(sv, 0.1),
        "p90": percentile(sv, 0.9),
    }


def wait_for_gz_stats(topic: str, timeout_sec: int) -> None:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        try:
            out = run_cmd_capture(f"gz topic -e -n 1 -t {topic}")
            if parse_rtf_from_chunk(out):
                return
        except subprocess.CalledProcessError:
            pass
        time.sleep(1.0)
    raise RuntimeError(f"等待 Gazebo stats 超时: {topic}")


def _ros_source_snippet() -> str:
    ros_setup = os.environ.get('ROS_SETUP', '/opt/ros/humble/setup.bash')
    ws = os.environ.get('USV_WS', os.getcwd())
    nav_setup = os.path.join(ws, 'src', 'usv_nav', 'install', 'setup.bash')
    parts = [f'source {ros_setup}']
    if os.path.isfile(nav_setup):
        parts.append(f'source {nav_setup}')
    parts.append('source install/setup.bash')
    return ' && '.join(parts) + ' && '


def launch_stage(launch_cmd: str, stage_cfg_path: str, world_stats_topic: str) -> subprocess.Popen:
    cmd = (
        _ros_source_snippet()
        + f"{launch_cmd} config_path:={stage_cfg_path} auto_cleanup:=true verbose_launch:=false"
    )
    proc = subprocess.Popen(
        ["bash", "-lc", cmd],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        preexec_fn=os.setsid,
    )
    wait_for_gz_stats(world_stats_topic, timeout_sec=120)
    return proc


def stop_stage(proc: subprocess.Popen) -> None:
    if proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=15)
    except Exception:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            proc.wait(timeout=10)
        except Exception:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:
                pass


def run_stage_measurement(
    stage: Stage,
    stage_cfg_path: str,
    launch_cmd: str,
    world_stats_topic: str,
    run_seconds: int,
    tail_seconds: int,
) -> Dict:
    proc = None
    samples = []
    rtf_samples: List[Dict[str, float]] = []
    proc_patterns = [
        "gz sim server",
        "gz sim gui",
        "ground_truth_gazebo_models_node",
        "ground_truth_node",
        "adaptive_radar_grid_map_node",
        "radar_converter_node",
        "parameter_bridge",
        "mmwave_4d_cloud_node",
        "usv_env_dynamics",
        "scenario_manager_node",
    ]
    try:
        proc = launch_stage(launch_cmd, stage_cfg_path, world_stats_topic)
        start = time.monotonic()
        while time.monotonic() - start < run_seconds:
            now = time.monotonic()
            try:
                chunk = run_cmd_capture(f"gz topic -e -n 1 -t {world_stats_topic}")
                vals = parse_rtf_from_chunk(chunk)
                if vals:
                    rtf_samples.append({"t": now, "rtf": vals[-1]})
            except subprocess.CalledProcessError:
                pass
            res = get_resource_snapshot(proc_patterns)
            res["t"] = now
            samples.append(res)
            time.sleep(1.0)

        tail_begin = start + max(0, run_seconds - tail_seconds)
        rtf_tail = [x["rtf"] for x in rtf_samples if x["t"] >= tail_begin]
        sample_tail = [x for x in samples if x["t"] >= tail_begin]

        cpu_tail = [x["proc_cpu_sum_percent"] for x in sample_tail]
        mem_tail = [x["proc_mem_sum_percent"] for x in sample_tail]
        rss_tail = [x["proc_rss_sum_gb"] for x in sample_tail]
        load_tail = [x["loadavg_1m"] for x in sample_tail]

        return {
            "stage": stage.name,
            "description": stage.description,
            "config_path": stage_cfg_path,
            "run_seconds": run_seconds,
            "tail_seconds": tail_seconds,
            "rtf": summarize(rtf_tail),
            "resource": {
                "proc_cpu_sum_percent": summarize(cpu_tail),
                "proc_mem_sum_percent": summarize(mem_tail),
                "proc_rss_sum_gb": summarize(rss_tail),
                "loadavg_1m": summarize(load_tail),
            },
            "samples_count": len(samples),
            "rtf_samples_count": len(rtf_samples),
            "tail_sample_count": len(sample_tail),
            "tail_rtf_count": len(rtf_tail),
        }
    finally:
        if proc is not None:
            stop_stage(proc)


def write_reports(results: List[Dict], out_dir: str) -> Dict[str, str]:
    os.makedirs(out_dir, exist_ok=True)
    ts = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    json_path = os.path.join(out_dir, f"staged_rtf_benchmark_{ts}.json")
    csv_path = os.path.join(out_dir, f"staged_rtf_benchmark_{ts}.csv")

    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(results, f, indent=2, ensure_ascii=False)

    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(
            [
                "stage",
                "rtf_avg",
                "rtf_median",
                "rtf_min",
                "rtf_max",
                "cpu_avg_percent",
                "cpu_median_percent",
                "mem_avg_percent",
                "mem_median_percent",
                "rss_avg_gb",
                "rss_median_gb",
                "load1_avg",
                "load1_median",
                "tail_rtf_count",
                "tail_sample_count",
            ]
        )
        for r in results:
            w.writerow(
                [
                    r["stage"],
                    r["rtf"]["avg"],
                    r["rtf"]["median"],
                    r["rtf"]["min"],
                    r["rtf"]["max"],
                    r["resource"]["proc_cpu_sum_percent"]["avg"],
                    r["resource"]["proc_cpu_sum_percent"]["median"],
                    r["resource"]["proc_mem_sum_percent"]["avg"],
                    r["resource"]["proc_mem_sum_percent"]["median"],
                    r["resource"]["proc_rss_sum_gb"]["avg"],
                    r["resource"]["proc_rss_sum_gb"]["median"],
                    r["resource"]["loadavg_1m"]["avg"],
                    r["resource"]["loadavg_1m"]["median"],
                    r["tail_rtf_count"],
                    r["tail_sample_count"],
                ]
            )

    return {"json": json_path, "csv": csv_path}


def main() -> int:
    parser = argparse.ArgumentParser(
        description="分阶段启动 usv_sim_full，并统计各阶段后30秒 RTF/资源占用。"
    )
    parser.add_argument(
        "--base-config",
        default=DEFAULT_CONFIG,
        help="基础 full_config.yaml 路径",
    )
    parser.add_argument(
        "--launch-cmd",
        default=DEFAULT_LAUNCH,
        help="启动命令（不含 source），例如 ros2 launch usv_sim_full nav2_sim_full_bringup.launch.py",
    )
    parser.add_argument(
        "--world-stats-topic",
        default=DEFAULT_WORLD_STATS_TOPIC,
        help="Gazebo stats 话题",
    )
    parser.add_argument(
        "--run-seconds",
        type=int,
        default=60,
        help="每阶段总运行时长（秒）",
    )
    parser.add_argument(
        "--tail-seconds",
        type=int,
        default=30,
        help="用于统计的尾部窗口（秒）",
    )
    parser.add_argument(
        "--out-dir",
        default=os.path.join(os.getcwd(), 'log', 'staged_benchmark'),
        help="结果输出目录",
    )
    args = parser.parse_args()

    if args.tail_seconds > args.run_seconds:
        print("tail-seconds 不能大于 run-seconds", file=sys.stderr)
        return 2
    if not os.path.isfile(args.base_config):
        print(f"基础配置不存在: {args.base_config}", file=sys.stderr)
        return 2

    with open(args.base_config, "r", encoding="utf-8") as f:
        base_cfg = yaml.safe_load(f) or {}

    temp_dir = tempfile.mkdtemp(prefix="usv_stage_cfg_")
    results = []
    try:
        for idx, stage in enumerate(DEFAULT_STAGES, start=1):
            cfg = build_stage_config(base_cfg, stage)
            cfg_path = os.path.join(temp_dir, f"{idx:02d}_{stage.name}.yaml")
            with open(cfg_path, "w", encoding="utf-8") as f:
                yaml.safe_dump(cfg, f, sort_keys=False, allow_unicode=True)

            print(f"\n=== [{idx}/{len(DEFAULT_STAGES)}] {stage.name} ===")
            print(stage.description)
            print(f"config: {cfg_path}")
            result = run_stage_measurement(
                stage=stage,
                stage_cfg_path=cfg_path,
                launch_cmd=args.launch_cmd,
                world_stats_topic=args.world_stats_topic,
                run_seconds=args.run_seconds,
                tail_seconds=args.tail_seconds,
            )
            results.append(result)
            print(
                "RTF(avg/median/min/max): "
                f"{result['rtf']['avg']:.3f}/{result['rtf']['median']:.3f}/"
                f"{result['rtf']['min']:.3f}/{result['rtf']['max']:.3f}"
            )
            print(
                "CPU(sum) avg/median: "
                f"{result['resource']['proc_cpu_sum_percent']['avg']:.1f}%/"
                f"{result['resource']['proc_cpu_sum_percent']['median']:.1f}%"
            )

        paths = write_reports(results, args.out_dir)
        print("\n=== 完成 ===")
        print(f"JSON: {paths['json']}")
        print(f"CSV : {paths['csv']}")
        return 0
    finally:
        try:
            for p in os.listdir(temp_dir):
                os.remove(os.path.join(temp_dir, p))
            os.rmdir(temp_dir)
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
