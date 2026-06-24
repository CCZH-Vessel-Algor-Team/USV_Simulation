#!/usr/bin/env python3
"""Gazebo entity-authoritative ground truth: spawn, cmd_vel, read pose, publish truth."""

from __future__ import annotations

import math
import os
import subprocess
import tempfile
import threading
import time
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Set

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Point, Twist
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.publisher import Publisher

from ground_truth_sim.arc_follow import compute_arc_follow_cmd_vel
from ground_truth_sim.entity_velocity import EntityVelocityEstimator
from ground_truth_sim.ground_truth_gazebo_models_node import (
    ModelWorldPose,
    TrackDriver,
    _as_vec,
    _base_model_name,
    _build_box_sdf,
    _build_cylinder_sdf,
    _collision_surface_xml,
    _cylinder_radius,
    _fmt_pose_xyzrpy,
    _gz_executable,
    _inertial_xml,
    _parse_model_poses_from_pose_info,
    _quat_to_yaw,
    _quat_to_roll_pitch,
    _velocity_control_plugin_xml,
    _yaw_to_quaternion,
    attitude_violation_reason,
    normalize_angle,
)
from ground_truth_sim.truth_markers import (
    MarkerPublishConfig,
    build_marker_array,
    build_track_array_from_states,
)
from ground_truth_sim.waypoint import (
    WaypointTargetState,
    entity_waypoint_desired_velocity,
    parse_fixed_targets,
)

try:
    from ros_gz_interfaces.msg import Contacts
except ImportError:  # pragma: no cover
    Contacts = None  # type: ignore

try:
    import gz.transport13 as gz_transport
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.msgs10.twist_pb2 import Twist as GzTwist
except ImportError:  # pragma: no cover
    gz_transport = None  # type: ignore
    Pose_V = None  # type: ignore
    GzTwist = None  # type: ignore

from std_msgs.msg import String
from usv_interfaces.msg import GlobalTrackArray

_COLLIDE_BITMASK_MAX = 0xFFFF


@dataclass
class EntityTrackRuntime:
    target: WaypointTargetState
    display: WaypointTargetState
    velocity: EntityVelocityEstimator = field(default_factory=EntityVelocityEstimator)


class GroundTruthGazeboEntityNode(Node):
    """Spawn Gazebo targets, drive cmd_vel from internal waypoint arc, publish truth from pose."""

    def __init__(self) -> None:
        super().__init__("ground_truth_gazebo_entity_node")
        self._cb_group = ReentrantCallbackGroup()
        self._timer_cb_group = MutuallyExclusiveCallbackGroup()
        self._declare_parameters()
        self._load_parameters()
        self._init_targets()
        self._init_publishers()
        self._init_collision_subscribers()
        self._init_pose_updates()
        self.create_timer(self._dt, self._tick, callback_group=self._timer_cb_group)
        self.get_logger().info(
            "ground_truth_gazebo_entity_node: world=%s tracks=%d prefix=%s geometry=%s "
            "truth_source=gazebo_entity update_dt=%.3fs"
            % (
                self._world,
                len(self._runtimes),
                self._prefix,
                self._geom_mode,
                self._dt,
            )
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("tracks_topic", "sim/ground_truth")
        self.declare_parameter("markers_topic", "sim/ground_truth_markers")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("world_name", "sydney_regatta")
        self.declare_parameter("model_name_prefix", "gt_ctrv_")
        self.declare_parameter("update_dt", 0.02)
        self.declare_parameter("spawn_delay_sec", 10.0)
        self.declare_parameter("world_service_wait_sec", 1.0)
        self.declare_parameter("create_cli_timeout_sec", 20.0)
        self.declare_parameter("spawn_thread_pool_size", 2)
        self.declare_parameter("fixed_targets_json", "")
        self.declare_parameter("fixed_targets", [])
        self.declare_parameter("waypoint_kinematics", "arc")
        self.declare_parameter("waypoint_arrival_threshold_m", 0.5)
        self.declare_parameter("waypoint_omega_limit", 0.22)
        self.declare_parameter("waypoint_turn_radius_min_m", 10.0)
        self.declare_parameter("waypoint_align_threshold_deg", 12.0)
        self.declare_parameter("cmd_vel_omega_limit", 0.22)
        self.declare_parameter("cmd_vel_turn_radius_min_m", 10.0)
        self.declare_parameter("cmd_vel_align_threshold_deg", 12.0)
        self.declare_parameter("cmd_vel_pose_refresh_interval_sec", 0.05)
        self.declare_parameter("heading_sync_enabled", True)
        self.declare_parameter("heading_sync_interval_sec", 1.0)
        self.declare_parameter("truth_velocity_lowpass_tau", 0.2)
        self.declare_parameter("history_max_points", 500)
        self.declare_parameter("prediction_horizon", 5.0)
        self.declare_parameter("prediction_dt", 0.25)
        self.declare_parameter("cylinder_radius_cap_m", 0.0)
        self.declare_parameter("cylinder_height_cap_m", 0.0)
        self.declare_parameter("collision_topic", "")
        self.declare_parameter("collision_string_topic", "")
        self.declare_parameter("collision_debounce_sec", 0.5)
        self.declare_parameter("remove_retry_interval_sec", 1.0)
        self.declare_parameter("reconcile_interval_sec", 2.0)
        self.declare_parameter("cleanup_stale_models_on_start", True)
        self.declare_parameter("gazebo_target_geometry", "box")
        self.declare_parameter("gazebo_mesh_profile", "")
        self.declare_parameter("contact_collide_bitmask", _COLLIDE_BITMASK_MAX)
        self.declare_parameter("model_mass_kg", 50.0)
        self.declare_parameter("attitude_monitor_enabled", True)
        self.declare_parameter("attitude_max_roll_pitch_deg", 10.0)
        self.declare_parameter("attitude_max_z_error_m", 2.0)
        self.declare_parameter("attitude_monitor_interval_sec", 0.1)
        self.declare_parameter("attitude_respawn_cooldown_sec", 2.0)

    def _get_bool(self, name: str) -> bool:
        value = self.get_parameter(name).value
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return bool(value)
        return str(value).lower() in ("true", "1", "yes")

    def _load_parameters(self) -> None:
        self._frame_id = str(self.get_parameter("frame_id").value).strip() or "map"
        tt = str(self.get_parameter("tracks_topic").value).strip() or "sim/ground_truth"
        mt = str(self.get_parameter("markers_topic").value).strip() or "sim/ground_truth_markers"
        self._tracks_topic = tt.lstrip("/")
        self._markers_topic = mt.lstrip("/")
        self._prefix = str(self.get_parameter("model_name_prefix").value).strip() or "gt_ctrv_"
        self._world = str(self.get_parameter("world_name").value).strip() or "sydney_regatta"
        dt = float(self.get_parameter("update_dt").value)
        self._dt = dt if dt > 0 else 0.02
        self._spawn_delay_sec = max(0.0, float(self.get_parameter("spawn_delay_sec").value))
        self._spawn_gate_open_ns = int(self._spawn_delay_sec * 1e9)
        wsw = max(0.05, float(self.get_parameter("world_service_wait_sec").value))
        self._gz_timeout_ms = max(500, int(wsw * 1000.0))
        self._create_timeout = max(5.0, float(self.get_parameter("create_cli_timeout_sec").value))
        pool = max(1, int(self.get_parameter("spawn_thread_pool_size").value))
        self._pool = ThreadPoolExecutor(max_workers=pool, thread_name_prefix="gt_entity_spawn")
        self._waypoint_arrival_threshold = max(
            0.1, float(self.get_parameter("waypoint_arrival_threshold_m").value)
        )
        self._waypoint_omega_limit = max(
            0.0, float(self.get_parameter("waypoint_omega_limit").value)
        )
        self._waypoint_turn_radius_min = max(
            1.0, float(self.get_parameter("waypoint_turn_radius_min_m").value)
        )
        self._waypoint_align_threshold_deg = max(
            1.0, float(self.get_parameter("waypoint_align_threshold_deg").value)
        )
        self._omega_limit = max(0.0, float(self.get_parameter("cmd_vel_omega_limit").value))
        self._turn_radius_min = max(
            1.0, float(self.get_parameter("cmd_vel_turn_radius_min_m").value)
        )
        self._align_threshold_deg = max(
            1.0, float(self.get_parameter("cmd_vel_align_threshold_deg").value)
        )
        self._pose_refresh_interval = max(
            0.02, float(self.get_parameter("cmd_vel_pose_refresh_interval_sec").value)
        )
        self._heading_sync = self._get_bool("heading_sync_enabled")
        self._heading_sync_interval = max(
            0.2, float(self.get_parameter("heading_sync_interval_sec").value)
        )
        self._velocity_tau = max(
            0.05, float(self.get_parameter("truth_velocity_lowpass_tau").value)
        )
        self._history_max_points = max(1, int(self.get_parameter("history_max_points").value))
        self._prediction_horizon = float(self.get_parameter("prediction_horizon").value)
        self._prediction_dt = float(self.get_parameter("prediction_dt").value)
        self._r_cap = float(self.get_parameter("cylinder_radius_cap_m").value)
        self._h_cap = float(self.get_parameter("cylinder_height_cap_m").value)
        self._collision_debounce = max(
            0.0, float(self.get_parameter("collision_debounce_sec").value)
        )
        self._remove_retry_interval = max(
            0.1, float(self.get_parameter("remove_retry_interval_sec").value)
        )
        self._reconcile_interval = max(
            0.5, float(self.get_parameter("reconcile_interval_sec").value)
        )
        self._cleanup_stale_on_start = self._get_bool("cleanup_stale_models_on_start")
        self._attitude_monitor = self._get_bool("attitude_monitor_enabled")
        self._attitude_max_rp_deg = max(
            0.0, float(self.get_parameter("attitude_max_roll_pitch_deg").value)
        )
        self._attitude_max_z_err = max(
            0.0, float(self.get_parameter("attitude_max_z_error_m").value)
        )
        self._attitude_monitor_interval = max(
            0.02, float(self.get_parameter("attitude_monitor_interval_sec").value)
        )
        self._attitude_respawn_cooldown = max(
            0.0, float(self.get_parameter("attitude_respawn_cooldown_sec").value)
        )
        self._model_mass_kg = max(1.0, float(self.get_parameter("model_mass_kg").value))
        _bm = self.get_parameter("contact_collide_bitmask").value
        try:
            raw_bm = int(str(_bm).strip(), 0) if isinstance(_bm, str) else int(_bm)
        except (TypeError, ValueError):
            raw_bm = _COLLIDE_BITMASK_MAX
        self._contact_bitmask = raw_bm & _COLLIDE_BITMASK_MAX
        _gm = str(self.get_parameter("gazebo_target_geometry").value).strip().lower()
        self._geom_mode = _gm if _gm in ("box", "cylinder", "mesh_profile") else "box"
        self._mesh_profile_path = ""
        self._mesh_profile_data: Optional[dict] = None
        self._mesh_profile_spawn_z = 0.0
        if self._geom_mode == "mesh_profile":
            raw_profile = str(self.get_parameter("gazebo_mesh_profile").value).strip()
            self._mesh_profile_path, self._mesh_profile_data = self._load_mesh_profile(raw_profile)
            self._mesh_profile_spawn_z = self._mesh_profile_default_spawn_z(self._mesh_profile_data)
            self.get_logger().info(
                "mesh_profile=%s uri=%s"
                % (
                    self._mesh_profile_path,
                    str((self._mesh_profile_data.get("mesh") or {}).get("uri", "")).strip(),
                )
            )
        self._remove_svc = f"/world/{self._world}/remove"
        self._set_pose_svc = f"/world/{self._world}/set_pose"
        self._marker_cfg = MarkerPublishConfig(
            frame_id=self._frame_id,
            motion_mode="waypoint",
            waypoint_kinematics=str(self.get_parameter("waypoint_kinematics").value).strip().lower()
            or "arc",
            prediction_horizon=self._prediction_horizon,
            prediction_dt=self._prediction_dt,
            waypoint_arrival_threshold=self._waypoint_arrival_threshold,
            waypoint_omega_limit=self._waypoint_omega_limit,
            waypoint_turn_radius_min=self._waypoint_turn_radius_min,
            waypoint_align_threshold_deg=self._waypoint_align_threshold_deg,
        )
        self._fixed_targets_raw = self.get_parameter("fixed_targets").value
        self._fixed_targets_json = str(self.get_parameter("fixed_targets_json").value or "")
        self._rng = np.random.default_rng(0)
        self._runtimes: Dict[int, EntityTrackRuntime] = {}
        self._drivers: Dict[int, TrackDriver] = {}
        self._spawned_ids: Set[int] = set()
        self._spawn_futures: Dict[int, Future] = {}
        self._collision_suppressed: Set[int] = set()
        self._last_collision_mono: Dict[str, float] = {}
        self._pending_remove_retry_mono: Dict[int, float] = {}
        self._remove_fail_counts: Dict[int, int] = {}
        self._world_pose_cache: Dict[str, ModelWorldPose] = {}
        self._state_lock = threading.RLock()
        self._gz_cli_lock = threading.Lock()
        self._retained_create_sdf_paths: list[str] = []
        self._last_pose_refresh_mono = 0.0
        self._next_attitude_check_mono = 0.0
        self._next_reconcile_mono = 0.0
        self._startup_cleanup_done = False
        self._last_published_track_ids: Set[int] = set()
        self._logged_delay = False
        self._gz_node = gz_transport.Node() if gz_transport is not None else None
        if self._gz_node is None:
            self.get_logger().warn(
                "gz.transport 不可用，cmd_vel 将经 ros_gz_bridge 转发"
            )

    def _init_targets(self) -> None:
        defaults = {
            "speed_min": 2.0,
            "size_width_min": 3.6,
            "size_length_min": 10.0,
            "size_height_min": 2.0,
            "ais_match_probability": 0.4,
        }
        targets = parse_fixed_targets(
            self._fixed_targets_raw,
            self._fixed_targets_json,
            defaults,
            self._rng,
        )
        for t in targets:
            display = WaypointTargetState(
                track_id=t.track_id,
                x=t.x,
                y=t.y,
                speed=t.speed,
                theta=t.theta,
                omega=t.omega,
                size_w=t.size_w,
                size_l=t.size_l,
                size_h=t.size_h,
                is_dark_target=t.is_dark_target,
                is_ais_matched=t.is_ais_matched,
                matched_mmsi=t.matched_mmsi,
                waypoints=list(t.waypoints),
                current_wp_idx=t.current_wp_idx,
                direction=t.direction,
                loop=t.loop,
                waypoint_active=t.waypoint_active,
                history=list(t.history),
            )
            est = EntityVelocityEstimator(tau_sec=self._velocity_tau)
            self._runtimes[int(t.track_id)] = EntityTrackRuntime(
                target=t, display=display, velocity=est
            )

    def _init_publishers(self) -> None:
        self._track_pub = self.create_publisher(GlobalTrackArray, self._tracks_topic, 10)
        from visualization_msgs.msg import MarkerArray

        self._marker_pub = self.create_publisher(MarkerArray, self._markers_topic, 10)

    def _init_collision_subscribers(self) -> None:
        ct = str(self.get_parameter("collision_topic").value).strip().lstrip("/")
        cst = str(self.get_parameter("collision_string_topic").value).strip().lstrip("/")
        if ct and Contacts is not None:
            self.create_subscription(
                Contacts, ct, self._on_contacts, 10, callback_group=self._cb_group
            )
        if cst:
            self.create_subscription(
                String, cst, self._on_collision_string, 10, callback_group=self._cb_group
            )

    def destroy_node(self) -> bool:
        self._pose_stream_stop.set()
        if hasattr(self, "_pose_stream_thread"):
            self._pose_stream_thread.join(timeout=1.0)
        for tid in list(self._drivers.keys()):
            self._teardown_track_driver(tid)
        self._pool.shutdown(wait=False, cancel_futures=True)
        for p in self._retained_create_sdf_paths:
            if p and os.path.isfile(p):
                try:
                    os.unlink(p)
                except OSError:
                    pass
        self._retained_create_sdf_paths.clear()
        return super().destroy_node()

    def _track_entity_name(self, track_id: int) -> str:
        return "%s%d" % (self._prefix, int(track_id))

    def _track_id_from_model_base(self, base: str) -> Optional[int]:
        if not base.startswith(self._prefix):
            return None
        suf = base[len(self._prefix) :]
        return int(suf) if suf.isdigit() else None

    def _runtime_for(self, tid: int) -> Optional[EntityTrackRuntime]:
        return self._runtimes.get(int(tid))

    def _resolve_mesh_profile_path(self, raw_path: str) -> str:
        path_text = str(raw_path or "").strip()
        if not path_text:
            raise RuntimeError("gazebo_target_geometry=mesh_profile 时必须提供 gazebo_mesh_profile")
        if os.path.isabs(path_text) and os.path.isfile(path_text):
            return path_text
        cand = os.path.abspath(path_text)
        if os.path.isfile(cand):
            return cand
        try:
            from ament_index_python.packages import get_package_share_directory

            share_cand = os.path.join(
                get_package_share_directory("usv_sim_full"),
                "description",
                "models",
                "target_ship",
                "10m_mesh_profile.yaml",
            )
            if os.path.isfile(share_cand):
                return share_cand
        except Exception:
            pass
        raise RuntimeError("gazebo_mesh_profile 文件不存在: %s" % path_text)

    def _load_mesh_profile(self, raw_path: str) -> tuple[str, dict]:
        profile_path = self._resolve_mesh_profile_path(raw_path)
        with open(profile_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        if not isinstance(data, dict):
            raise RuntimeError("gazebo_mesh_profile YAML 非法: %s" % profile_path)
        mesh = data.get("mesh") or {}
        if not str(mesh.get("uri", "")).strip():
            raise RuntimeError("gazebo_mesh_profile 缺少 mesh.uri: %s" % profile_path)
        boxes = data.get("boxes") or []
        if not isinstance(boxes, list) or not boxes:
            raise RuntimeError("gazebo_mesh_profile 缺少 boxes[]: %s" % profile_path)
        return profile_path, data

    def _mesh_profile_default_spawn_z(self, profile_data: dict) -> float:
        prof_spawn_z = profile_data.get("spawn_z")
        if prof_spawn_z is not None:
            return float(prof_spawn_z)
        group = profile_data.get("collision_group") or {}
        group_xyz = _as_vec((group.get("pose_offset") or {}).get("xyz"), 3, 0.0)
        min_z = None
        for box in profile_data.get("boxes") or []:
            size = _as_vec(box.get("size_lwh_m"), 3, 0.0)
            pose = _as_vec(box.get("pose_xyz_m"), 3, 0.0)
            bottom = group_xyz[2] + pose[2] - 0.5 * size[2]
            min_z = bottom if min_z is None else min(min_z, bottom)
        return max(0.0, -(0.0 if min_z is None else float(min_z)))

    def _z_center_for_height(self, height_m: float) -> float:
        h = max(float(height_m), 0.1)
        return max(h * 0.5, 0.25)

    def _box_dims(self, rt: EntityTrackRuntime) -> tuple[float, float, float]:
        t = rt.target
        sl = max(float(t.size_l), 0.5)
        sw = max(float(t.size_w), 0.5)
        sh = max(float(t.size_h), 0.5)
        if self._r_cap > 0.0:
            half_diag = 0.5 * math.sqrt(sl * sl + sw * sw)
            if half_diag > self._r_cap:
                s = self._r_cap / half_diag
                sl *= s
                sw *= s
        if self._h_cap > 0.0:
            sh = min(sh, self._h_cap)
        return max(sl, 0.1), max(sw, 0.1), max(sh, 0.5)

    def _spawn_z(self, rt: EntityTrackRuntime) -> float:
        if self._geom_mode == "mesh_profile":
            return self._mesh_profile_spawn_z
        if self._geom_mode == "box":
            _, _, sh = self._box_dims(rt)
            return self._z_center_for_height(sh)
        r = _cylinder_radius(rt.target.size_l, rt.target.size_w)
        h = max(float(rt.target.size_h), 0.5)
        if self._r_cap > 0.0:
            r = min(r, self._r_cap)
        if self._h_cap > 0.0:
            h = min(h, self._h_cap)
        return self._z_center_for_height(h)

    def _color_rgba(self, rt: EntityTrackRuntime) -> str:
        if rt.target.is_ais_matched:
            return "0.15 0.75 0.25 1"
        return "0.95 0.45 0.1 1"

    def _build_mesh_profile_sdf(self, model_name: str) -> str:
        if self._mesh_profile_data is None:
            raise RuntimeError("mesh_profile 数据尚未初始化")
        profile = self._mesh_profile_data
        mesh = profile.get("mesh") or {}
        mesh_uri = str(mesh.get("uri", "")).strip()
        mesh_scale = _as_vec(mesh.get("scale"), 3, 1.0)
        mesh_rgba = _as_vec(mesh.get("material_rgba"), 4, 1.0)
        mesh_pose = mesh.get("pose_offset") or {}
        mesh_xyz = _as_vec(mesh_pose.get("xyz"), 3, 0.0)
        mesh_rpy = _as_vec(mesh_pose.get("rpy"), 3, 0.0)
        group = profile.get("collision_group") or {}
        group_pose = group.get("pose_offset") or {}
        group_xyz = _as_vec(group_pose.get("xyz"), 3, 0.0)
        group_rpy = _as_vec(group_pose.get("rpy"), 3, 0.0)
        collision_xml: list[str] = []
        for idx, box in enumerate(profile.get("boxes") or []):
            box_name = str(box.get("name") or "collision_%d" % idx).strip() or "collision_%d" % idx
            size = _as_vec(box.get("size_lwh_m"), 3, 0.0)
            pose_xyz = _as_vec(box.get("pose_xyz_m"), 3, 0.0)
            pose_rpy = _as_vec(box.get("pose_rpy"), 3, 0.0)
            xyz = [
                group_xyz[0] + pose_xyz[0],
                group_xyz[1] + pose_xyz[1],
                group_xyz[2] + pose_xyz[2],
            ]
            rpy = [
                group_rpy[0] + pose_rpy[0],
                group_rpy[1] + pose_rpy[1],
                group_rpy[2] + pose_rpy[2],
            ]
            collision_xml.append(
                """
      <collision name="%s">
        <pose>%s</pose>
        <geometry>
          <box><size>%.6f %.6f %.6f</size></box>
        </geometry>
%s      </collision>"""
                % (
                    box_name,
                    _fmt_pose_xyzrpy(xyz, rpy),
                    size[0],
                    size[1],
                    size[2],
                    _collision_surface_xml(self._contact_bitmask),
                )
            )
        inertial = _inertial_xml(self._model_mass_kg)
        plugin = _velocity_control_plugin_xml(model_name)
        return """<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="%s">
    <static>false</static>
    <link name="link">
      <gravity>false</gravity>
%s
      <visual name="v">
        <pose>%s</pose>
        <geometry>
          <mesh>
            <uri>%s</uri>
            <scale>%.6f %.6f %.6f</scale>
          </mesh>
        </geometry>
        <material>
          <ambient>%.6f %.6f %.6f %.6f</ambient>
          <diffuse>%.6f %.6f %.6f %.6f</diffuse>
        </material>
      </visual>
%s
    </link>
%s
  </model>
</sdf>
""" % (
            model_name,
            inertial,
            _fmt_pose_xyzrpy(mesh_xyz, mesh_rpy),
            mesh_uri,
            mesh_scale[0],
            mesh_scale[1],
            mesh_scale[2],
            mesh_rgba[0],
            mesh_rgba[1],
            mesh_rgba[2],
            mesh_rgba[3],
            mesh_rgba[0],
            mesh_rgba[1],
            mesh_rgba[2],
            mesh_rgba[3],
            "".join(collision_xml),
            plugin,
        )

    def _fetch_world_pose_info(self) -> tuple[Optional[str], str]:
        gz = _gz_executable()
        cmd = [gz, "topic", "-e", "-n", "12", "-t", f"/world/{self._world}/pose/info"]
        try:
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=max(2.5, self._gz_timeout_ms / 1000.0),
            )
            output = ((result.stdout or "") + "\n" + (result.stderr or "")).strip()
            if not output:
                return None, output or ("returncode=%s" % result.returncode)
            return output, ""
        except Exception as ex:  # pragma: no cover
            return None, str(ex)

    def _apply_pose_info_text(self, output: str) -> None:
        pose_map = _parse_model_poses_from_pose_info(output)
        if not pose_map:
            return
        with self._state_lock:
            for base, pose in pose_map.items():
                if base.startswith(self._prefix):
                    self._world_pose_cache[base] = pose

    def _init_pose_updates(self) -> None:
        self._pose_stream_stop = threading.Event()
        if self._gz_node is not None and Pose_V is not None:
            topic = f"/world/{self._world}/pose/info"
            self._gz_node.subscribe(Pose_V, topic, self._on_gz_pose_v)
            self.get_logger().info("pose/info 订阅: %s (gz.transport)" % topic)
            return
        self._pose_stream_thread = threading.Thread(
            target=self._pose_poll_loop,
            name="gt_entity_pose_poll",
            daemon=True,
        )
        self._pose_stream_thread.start()
        self.get_logger().warn("pose/info 回退为 subprocess 轮询")

    def _on_gz_pose_v(self, msg: Pose_V) -> None:
        with self._state_lock:
            for entry in msg.pose:
                base = _base_model_name(entry.name)
                if not base.startswith(self._prefix):
                    continue
                pos = entry.position
                ori = entry.orientation
                self._world_pose_cache[base] = ModelWorldPose(
                    x=float(pos.x),
                    y=float(pos.y),
                    z=float(pos.z),
                    qx=float(ori.x),
                    qy=float(ori.y),
                    qz=float(ori.z),
                    qw=float(ori.w),
                )
    def _pose_poll_loop(self) -> None:
        """subprocess 回退：后台轮询 pose/info。"""
        while not self._pose_stream_stop.is_set():
            output, _ = self._fetch_world_pose_info()
            if output:
                self._apply_pose_info_text(output)
            self._pose_stream_stop.wait(self._pose_refresh_interval)

    def _refresh_world_pose_cache(self) -> bool:
        with self._state_lock:
            return bool(self._world_pose_cache)

    def _run_create_cli(
        self,
        model_name: str,
        sdf: str,
        x: float,
        y: float,
        z: float,
        roll: float,
        pitch: float,
        yaw: float,
    ) -> tuple[bool, str]:
        tmp_sdf_path = ""
        try:
            fd, tmp_sdf_path = tempfile.mkstemp(prefix="gt_entity_", suffix=".sdf")
            os.close(fd)
            with open(tmp_sdf_path, "w", encoding="utf-8") as f:
                f.write(sdf)
            cmd = [
                "ros2",
                "run",
                "ros_gz_sim",
                "create",
                "-world",
                self._world,
                "-name",
                model_name,
                "-file",
                tmp_sdf_path,
                "-x",
                str(x),
                "-y",
                str(y),
                "-z",
                str(z),
                "-R",
                str(roll),
                "-P",
                str(pitch),
                "-Y",
                str(yaw),
            ]
            with self._gz_cli_lock:
                result = subprocess.run(
                    cmd,
                    capture_output=True,
                    text=True,
                    timeout=self._create_timeout,
                )
            err = (result.stderr or result.stdout or "").strip()
            if result.returncode != 0:
                if tmp_sdf_path and os.path.isfile(tmp_sdf_path):
                    try:
                        os.unlink(tmp_sdf_path)
                    except OSError:
                        pass
                return False, err or "returncode=%s" % result.returncode
            if tmp_sdf_path and os.path.isfile(tmp_sdf_path):
                self._retained_create_sdf_paths.append(tmp_sdf_path)
            return True, ""
        except subprocess.TimeoutExpired:
            if tmp_sdf_path and os.path.isfile(tmp_sdf_path):
                try:
                    os.unlink(tmp_sdf_path)
                except OSError:
                    pass
            return False, "create timeout"
        except Exception as ex:  # pragma: no cover
            return False, str(ex)

    def _submit_spawn(self, tid: int) -> None:
        rt = self._runtime_for(tid)
        if rt is None:
            return
        with self._state_lock:
            if tid in self._spawn_futures or tid in self._spawned_ids:
                return
        name = self._track_entity_name(tid)
        rgba = self._color_rgba(rt)
        if self._geom_mode == "mesh_profile":
            sdf = self._build_mesh_profile_sdf(name)
            z = self._spawn_z(rt)
        elif self._geom_mode == "box":
            sx, sy, sz = self._box_dims(rt)
            sdf = _build_box_sdf(name, sx, sy, sz, rgba, self._contact_bitmask, self._model_mass_kg)
            z = self._z_center_for_height(sz)
        else:
            r = _cylinder_radius(rt.target.size_l, rt.target.size_w)
            h = max(float(rt.target.size_h), 0.5)
            sdf = _build_cylinder_sdf(name, r, h, rgba, self._contact_bitmask, self._model_mass_kg)
            z = self._spawn_z(rt)
        t = rt.target
        yaw = float(t.theta)
        fut = self._pool.submit(
            self._run_create_cli,
            name,
            sdf,
            float(t.x),
            float(t.y),
            z,
            0.0,
            0.0,
            yaw,
        )
        with self._state_lock:
            self._spawn_futures[tid] = fut
        self.get_logger().info(
            "create queued %s at (%.1f, %.1f, %.1f) yaw=%.3f"
            % (name, float(t.x), float(t.y), z, yaw)
        )

    def _drain_spawn_futures(self) -> None:
        to_finish: list[tuple[int, Future]] = []
        with self._state_lock:
            for tid, fut in list(self._spawn_futures.items()):
                if fut.done():
                    to_finish.append((tid, fut))
            for tid, _ in to_finish:
                del self._spawn_futures[tid]
        for tid, fut in to_finish:
            name = self._track_entity_name(tid)
            try:
                ok, err = fut.result()
            except Exception as ex:  # pragma: no cover
                self.get_logger().error("create exception %s: %s" % (name, ex))
                continue
            if not ok:
                self.get_logger().warn("create failed %s: %s" % (name, (err or "")[:200]))
                continue
            with self._state_lock:
                self._spawned_ids.add(tid)
            rt = self._runtime_for(tid)
            initial_yaw = float(rt.target.theta) if rt is not None else 0.0
            self._setup_track_driver(tid, initial_yaw=initial_yaw)
            self.get_logger().info("spawn ready %s" % name)

    def _start_cmd_vel_bridge(self, model_name: str) -> subprocess.Popen:
        cmd = [
            "ros2",
            "run",
            "ros_gz_bridge",
            "parameter_bridge",
            f"/model/{model_name}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
        ]
        return subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def _setup_track_driver(self, tid: int, initial_yaw: float = 0.0) -> None:
        with self._state_lock:
            if tid in self._drivers:
                return
        name = self._track_entity_name(tid)
        topic = f"/model/{name}/cmd_vel"
        ros_pub: Optional[Publisher] = None
        bridge: Optional[subprocess.Popen] = None
        gz_pub = None
        if self._gz_node is not None and GzTwist is not None:
            gz_pub = self._gz_node.advertise(topic, GzTwist)
        else:
            ros_pub = self.create_publisher(Twist, topic, 10)
            bridge = self._start_cmd_vel_bridge(name)
        driver = TrackDriver(
            track_id=tid,
            model_name=name,
            cmd_vel_pub=ros_pub,
            bridge_proc=bridge,
            gz_pub=gz_pub,
            last_yaw=initial_yaw,
            spawn_yaw=initial_yaw,
        )
        with self._state_lock:
            self._drivers[tid] = driver

    def _teardown_track_driver(self, tid: int) -> None:
        with self._state_lock:
            driver = self._drivers.pop(tid, None)
        if driver is None:
            return
        if driver.gz_pub is not None and GzTwist is not None:
            driver.gz_pub.publish(GzTwist())
        elif driver.cmd_vel_pub is not None:
            driver.cmd_vel_pub.publish(Twist())
        if driver.bridge_proc is not None:
            driver.bridge_proc.terminate()
        if driver.cmd_vel_pub is not None:
            try:
                self.destroy_publisher(driver.cmd_vel_pub)
            except Exception:  # pragma: no cover
                pass

    def _remove_entity_gz(self, model_name: str) -> tuple[bool, str]:
        gz = _gz_executable()
        req = 'name: "%s" type: MODEL' % model_name.replace('"', "")
        cmd = [
            gz,
            "service",
            "-s",
            self._remove_svc,
            "--reqtype",
            "gz.msgs.Entity",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            str(self._gz_timeout_ms),
            "--req",
            req,
        ]
        try:
            with self._gz_cli_lock:
                result = subprocess.run(
                    cmd,
                    capture_output=True,
                    text=True,
                    timeout=max(5.0, self._gz_timeout_ms / 500.0),
                )
            output = ((result.stdout or "") + "\n" + (result.stderr or "")).strip()
            normalized = output.lower().replace(" ", "")
            missing = model_name.lower() in output.lower() and "not found" in output.lower()
            ok = missing or (result.returncode == 0 and "data:false" not in normalized)
            return ok, output
        except Exception as ex:  # pragma: no cover
            return False, str(ex)

    def _set_pose_gz(
        self, model_name: str, x: float, y: float, z: float, yaw: float
    ) -> tuple[bool, str]:
        qx, qy, qz, qw = _yaw_to_quaternion(yaw)
        req = (
            f'name: "{model_name.replace(chr(34), "")}"\n'
            f"position {{ x: {x:.9g} y: {y:.9g} z: {z:.9g} }}\n"
            f"orientation {{ x: {qx:.9g} y: {qy:.9g} z: {qz:.9g} w: {qw:.9g} }}"
        )
        gz = _gz_executable()
        cmd = [
            gz,
            "service",
            "-s",
            self._set_pose_svc,
            "--reqtype",
            "gz.msgs.Pose",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            str(self._gz_timeout_ms),
            "--req",
            req,
        ]
        try:
            with self._gz_cli_lock:
                result = subprocess.run(
                    cmd,
                    capture_output=True,
                    text=True,
                    timeout=max(5.0, self._gz_timeout_ms / 500.0),
                )
            output = ((result.stdout or "") + "\n" + (result.stderr or "")).strip()
            normalized = output.lower().replace(" ", "")
            ok = result.returncode == 0 and "data:false" not in normalized
            return ok, output
        except Exception as ex:  # pragma: no cover
            return False, str(ex)

    def _body_yaw(self, driver: TrackDriver) -> float:
        with self._state_lock:
            pose = self._world_pose_cache.get(driver.model_name)
        if pose is not None:
            return _quat_to_yaw(pose.qx, pose.qy, pose.qz, pose.qw)
        return driver.last_yaw

    def _maybe_sync_heading(
        self, rt: EntityTrackRuntime, driver: TrackDriver, truth_yaw: float, now_mono: float
    ) -> None:
        if not self._heading_sync:
            return
        if now_mono - driver.last_heading_sync_mono < self._heading_sync_interval:
            return
        body_yaw = self._body_yaw(driver)
        if abs(normalize_angle(truth_yaw - body_yaw)) > math.radians(self._align_threshold_deg):
            return
        with self._state_lock:
            pose = self._world_pose_cache.get(driver.model_name)
        if pose is None:
            return
        z = self._spawn_z(rt)
        ok, _ = self._set_pose_gz(driver.model_name, pose.x, pose.y, z, truth_yaw)
        if ok:
            driver.last_heading_sync_mono = now_mono

    def _publish_cmd_vel(
        self,
        rt: EntityTrackRuntime,
        driver: TrackDriver,
        desired_vx: float,
        desired_vy: float,
    ) -> None:
        body_yaw = self._body_yaw(driver)
        twist = compute_arc_follow_cmd_vel(
            desired_vx,
            desired_vy,
            body_yaw,
            omega_limit=self._omega_limit,
            turn_radius_min_m=self._turn_radius_min,
            align_threshold_deg=self._align_threshold_deg,
        )
        speed = math.hypot(desired_vx, desired_vy)
        truth_yaw = math.atan2(desired_vy, desired_vx) if speed > 1e-3 else body_yaw
        now_mono = time.monotonic()
        if abs(twist.angular.z) < 1e-6 and speed > 1e-3:
            self._maybe_sync_heading(rt, driver, truth_yaw, now_mono)
            body_yaw = self._body_yaw(driver)
            heading_err = normalize_angle(truth_yaw - body_yaw)
            if abs(heading_err) <= math.radians(self._align_threshold_deg):
                twist.linear.x = speed
                twist.angular.z = 0.0
        driver.last_yaw = body_yaw
        if driver.gz_pub is not None and GzTwist is not None:
            msg = GzTwist()
            msg.linear.x = twist.linear.x
            msg.linear.y = twist.linear.y
            msg.angular.z = twist.angular.z
            driver.gz_pub.publish(msg)
        elif driver.cmd_vel_pub is not None:
            driver.cmd_vel_pub.publish(twist)

    def _reconcile_world_entities(self, desired_ids: Set[int], force: bool = False) -> None:
        now_mono = time.monotonic()
        if not force and now_mono < self._next_reconcile_mono:
            return
        self._next_reconcile_mono = now_mono + self._reconcile_interval
        with self._state_lock:
            bases = {b for b in self._world_pose_cache if b.startswith(self._prefix)}
        if not bases:
            output, _ = self._fetch_world_pose_info()
            if output is None:
                return
            self._apply_pose_info_text(output)
            with self._state_lock:
                bases = {b for b in self._world_pose_cache if b.startswith(self._prefix)}
        if not bases:
            if force and self._cleanup_stale_on_start:
                self._startup_cleanup_done = True
            return
        with self._state_lock:
            pending = set(self._spawn_futures.keys())
            active = set(self._drivers.keys()) | self._spawned_ids | pending
            want = (desired_ids - self._collision_suppressed) | active
            stale = sorted(
                int(tid)
                for base in bases
                if (tid := self._track_id_from_model_base(base)) is not None and tid not in want
            )
        if force and self._cleanup_stale_on_start:
            self._startup_cleanup_done = True
        elif force:
            return
        for tid in stale:
            name = self._track_entity_name(tid)
            self._teardown_track_driver(tid)
            ok, _ = self._remove_entity_gz(name)
            if ok:
                with self._state_lock:
                    self._spawned_ids.discard(tid)
                self.get_logger().info("reconcile removed stale %s" % name)

    def _on_contacts(self, msg: Contacts) -> None:  # type: ignore[valid-type]
        hit: Set[str] = set()
        for c in msg.contacts:
            for ent in (c.collision1, c.collision2):
                base = (getattr(ent, "name", "") or "").split("::", 1)[0]
                if base:
                    hit.add(base)
        for base in hit:
            self._handle_collision_model_base(base)

    def _on_collision_string(self, msg: String) -> None:
        base = (msg.data or "").strip().split("::", 1)[0]
        if base:
            self._handle_collision_model_base(base)

    def _collision_debounced(self, key: str) -> bool:
        if self._collision_debounce <= 0.0:
            return True
        now = time.monotonic()
        last = self._last_collision_mono.get(key, 0.0)
        if now - last < self._collision_debounce:
            return False
        self._last_collision_mono[key] = now
        return True

    def _handle_collision_model_base(self, base: str) -> None:
        if not base.startswith(self._prefix):
            return
        if not self._collision_debounced(base):
            return
        tid = self._track_id_from_model_base(base)
        if tid is None:
            return
        with self._state_lock:
            self._collision_suppressed.add(tid)
        self._teardown_track_driver(tid)
        ok, _ = self._remove_entity_gz(base)
        with self._state_lock:
            self._spawned_ids.discard(tid)
        if ok:
            self.get_logger().info("collision removed %s" % base)

    def _maybe_respawn_for_attitude(self, tid: int, now_mono: float) -> Optional[str]:
        if not self._attitude_monitor:
            return None
        driver = self._drivers.get(tid)
        rt = self._runtime_for(tid)
        if driver is None or rt is None:
            return None
        if now_mono - driver.last_respawn_mono < self._attitude_respawn_cooldown:
            return None
        with self._state_lock:
            pose = self._world_pose_cache.get(driver.model_name)
        if pose is None:
            return None
        return attitude_violation_reason(
            pose, self._spawn_z(rt), self._attitude_max_rp_deg, self._attitude_max_z_err
        )

    def _respawn_track(self, tid: int, reason: str) -> None:
        rt = self._runtime_for(tid)
        if rt is None:
            return
        name = self._track_entity_name(tid)
        with self._state_lock:
            fut = self._spawn_futures.pop(tid, None)
        if fut is not None and not fut.done():
            fut.cancel()
        self._teardown_track_driver(tid)
        self._remove_entity_gz(name)
        with self._state_lock:
            self._spawned_ids.discard(tid)
            self._world_pose_cache.pop(name, None)
        rt.velocity.reset()
        self.get_logger().info("%s: respawn %s" % (reason, name))
        self._submit_spawn(tid)

    def _sync_planner_from_pose(self, rt: EntityTrackRuntime, pose: ModelWorldPose) -> None:
        """用 Gazebo 实体 pose 驱动航路规划，避免内部开环状态超前导致折返/回退。"""
        rt.target.x = float(pose.x)
        rt.target.y = float(pose.y)
        rt.target.theta = _quat_to_yaw(pose.qx, pose.qy, pose.qz, pose.qw)

    def _sync_display_from_pose(self, rt: EntityTrackRuntime, pose: ModelWorldPose) -> None:
        rt.display.x = float(pose.x)
        rt.display.y = float(pose.y)
        pt = Point(x=float(pose.x), y=float(pose.y), z=0.0)
        if not rt.display.history or (
            abs(rt.display.history[-1].x - pt.x) > 1e-4
            or abs(rt.display.history[-1].y - pt.y) > 1e-4
        ):
            rt.display.history.append(pt)
            if len(rt.display.history) > self._history_max_points:
                rt.display.history.pop(0)

    def _tick(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns < self._spawn_gate_open_ns:
            if not self._logged_delay:
                self.get_logger().info(
                    "spawn_delay_sec=%.1f：%.1fs 后再 spawn/发布实体真值"
                    % (self._spawn_delay_sec, (self._spawn_gate_open_ns - now_ns) * 1e-9)
                )
                self._logged_delay = True
            return

        self._drain_spawn_futures()
        desired_ids = set(self._runtimes.keys())
        self._reconcile_world_entities(
            desired_ids,
            force=(self._cleanup_stale_on_start and not self._startup_cleanup_done),
        )

        now_mono = time.monotonic()
        if now_mono - self._last_pose_refresh_mono >= self._pose_refresh_interval:
            self._last_pose_refresh_mono = now_mono

        if self._attitude_monitor and now_mono >= self._next_attitude_check_mono:
            self._next_attitude_check_mono = now_mono + self._attitude_monitor_interval
            for tid in list(self._spawned_ids):
                reason = self._maybe_respawn_for_attitude(tid, now_mono)
                if reason is not None:
                    driver = self._drivers.get(tid)
                    if driver is not None:
                        driver.last_respawn_mono = now_mono
                    self._respawn_track(tid, "attitude (%s)" % reason)

        for tid, rt in self._runtimes.items():
            if tid in self._collision_suppressed:
                continue
            if tid not in self._spawned_ids:
                self._submit_spawn(tid)
                continue
            driver = self._drivers.get(tid)
            if driver is None:
                continue
            with self._state_lock:
                pose = self._world_pose_cache.get(driver.model_name)
            if pose is None:
                continue
            self._sync_planner_from_pose(rt, pose)
            desired_vx, desired_vy = entity_waypoint_desired_velocity(
                rt.target,
                float(pose.x),
                float(pose.y),
                self._waypoint_arrival_threshold,
            )
            with self._state_lock:
                self._sync_display_from_pose(rt, pose)
            vx, vy = rt.velocity.update(pose.x, pose.y, now_ns)
            self._publish_cmd_vel(rt, driver, desired_vx, desired_vy)

        publish_targets: List[WaypointTargetState] = []
        vx_map: Dict[int, tuple[float, float]] = {}
        model_map: Dict[int, str] = {}
        for tid in sorted(self._spawned_ids):
            if tid in self._collision_suppressed:
                continue
            rt = self._runtime_for(tid)
            driver = self._drivers.get(tid)
            if rt is None or driver is None:
                continue
            with self._state_lock:
                pose = self._world_pose_cache.get(driver.model_name)
                if pose is None:
                    continue
                self._sync_display_from_pose(rt, pose)
                publish_targets.append(rt.display)
            vx_map[tid] = (rt.velocity.vx, rt.velocity.vy)
            model_map[tid] = driver.model_name

        if not publish_targets:
            return

        stamp = self.get_clock().now().to_msg()
        array_msg = build_track_array_from_states(
            self._frame_id,
            stamp,
            publish_targets,
            vx_override=vx_map,
            source_model_names=model_map,
        )
        self._track_pub.publish(array_msg)
        markers, self._last_published_track_ids = build_marker_array(
            publish_targets,
            stamp,
            self._marker_cfg,
            self._last_published_track_ids,
        )
        self._marker_pub.publish(markers)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GroundTruthGazeboEntityNode()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("ground_truth_gazebo_entity_node shutting down")
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
