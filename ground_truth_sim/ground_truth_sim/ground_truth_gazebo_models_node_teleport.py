#!/usr/bin/env python3
"""Mirror GlobalTrackArray targets as Gazebo Sim models (visual + collision).

默认按 GlobalTrack 的 **size_l / size_w / size_h** 生成长方体（航向与速度一致）；可选圆柱模式。
Uses gz-transport–aligned tooling (same as dynamic obstacles in scenario_manager):
  - spawn: ``ros2 run ros_gz_sim create`` + temporary SDF file
  - pose: ``gz service`` → ``/world/<world>/set_pose_vector`` (``gz.msgs.Pose_V``, one call per tick)
  - remove: ``gz service`` → ``/world/<world>/remove`` (``gz.msgs.Entity``)

rclpy clients for ``/world/.../SpawnEntity`` often never match the server on some ROS 2 distros; do not rely on them.
Spawns run in a small thread pool so the MultiThreadedExecutor is not blocked by subprocess I/O.
"""

from __future__ import annotations

import math
import os
import re
import shutil
import subprocess
import tempfile
import threading
import time
from concurrent.futures import Future, ThreadPoolExecutor
from typing import Dict, Optional, Set

import rclpy
import yaml
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from usv_interfaces.msg import GlobalTrack, GlobalTrackArray

try:
    from ros_gz_interfaces.msg import Contacts
except ImportError:  # pragma: no cover
    Contacts = None  # type: ignore


def _yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def _yaw_from_track(t: GlobalTrack) -> float:
    spd = math.hypot(t.v_x, t.v_y)
    if spd < 1e-3:
        return 0.0
    return math.atan2(t.v_y, t.v_x)


def _cylinder_radius(size_l: float, size_w: float) -> float:
    """Minimum circle radius in the horizontal plane enclosing an L x W rectangle."""
    sl = max(float(size_l), 0.5)
    sw = max(float(size_w), 0.5)
    return 0.5 * math.sqrt(sl * sl + sw * sw)


# Gazebo Sim / libsdformat 的 <collide_bitmask> 按 16 位处理；0xFFFFFFFF 会报 Out of range。
_COLLIDE_BITMASK_MAX = 0xFFFF


def _collision_surface_xml(collide_bitmask: int) -> str:
    """显式接触 bitmask，避免与场景默认层不碰撞；与视觉同几何的 collision 块内使用。"""
    bm = int(collide_bitmask) & _COLLIDE_BITMASK_MAX
    return (
        "        <surface>\n"
        "          <contact>\n"
        "            <collide_bitmask>%u</collide_bitmask>\n"
        "          </contact>\n"
        "        </surface>\n" % bm
    )


def _build_box_sdf(
    model_name: str, sx: float, sy: float, sz: float, rgba: str, collide_bitmask: int
) -> str:
    """长方体：link 系 X=长(size_l)、Y=宽(size_w)、Z=高(size_h)；模型姿态由 yaw 对齐航向。"""
    surf = _collision_surface_xml(collide_bitmask)
    return f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="{model_name}">
    <static>false</static>
    <link name="link">
      <gravity>false</gravity>
      <inertial>
        <mass>10.0</mass>
        <inertia>
          <ixx>1.0</ixx><iyy>1.0</iyy><izz>1.0</izz>
        </inertia>
      </inertial>
      <collision name="hull">
        <geometry>
          <box><size>{sx:.6f} {sy:.6f} {sz:.6f}</size></box>
        </geometry>
{surf}      </collision>
      <visual name="v">
        <geometry>
          <box><size>{sx:.6f} {sy:.6f} {sz:.6f}</size></box>
        </geometry>
        <material>
          <ambient>{rgba}</ambient>
          <diffuse>{rgba}</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


def _build_cylinder_sdf(
    model_name: str, radius: float, length: float, rgba: str, collide_bitmask: int
) -> str:
    surf = _collision_surface_xml(collide_bitmask)
    return f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="{model_name}">
    <static>false</static>
    <link name="link">
      <gravity>false</gravity>
      <inertial>
        <mass>10.0</mass>
        <inertia>
          <ixx>1.0</ixx><iyy>1.0</iyy><izz>1.0</izz>
        </inertia>
      </inertial>
      <collision name="c">
        <geometry>
          <cylinder><radius>{radius:.6f}</radius><length>{length:.6f}</length></cylinder>
        </geometry>
{surf}      </collision>
      <visual name="v">
        <geometry>
          <cylinder><radius>{radius:.6f}</radius><length>{length:.6f}</length></cylinder>
        </geometry>
        <material>
          <ambient>{rgba}</ambient>
          <diffuse>{rgba}</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


def _tracks_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=20,
    )


def _gz_executable() -> str:
    return shutil.which("gz") or "gz"


def _base_model_name(entity_name: str) -> str:
    if not entity_name:
        return ""
    return entity_name.split("::", 1)[0]


def _fmt_pose_xyzrpy(xyz, rpy) -> str:
    return (
        f"{float(xyz[0]):.6f} {float(xyz[1]):.6f} {float(xyz[2]):.6f} "
        f"{float(rpy[0]):.6f} {float(rpy[1]):.6f} {float(rpy[2]):.6f}"
    )


def _as_vec(raw, length: int, default: float = 0.0) -> list[float]:
    if isinstance(raw, (list, tuple)):
        vals = [float(v) for v in raw[:length]]
    else:
        vals = []
    if len(vals) < length:
        vals.extend([float(default)] * (length - len(vals)))
    return vals


def _extract_model_names(text: str) -> Set[str]:
    out: Set[str] = set()
    if not text:
        return out
    for pat in (
        r'name:\s*"([^"]+)"',
        r'name\[([^\]]+)\]',
        r'Entity named \[([^\]]+)\]',
    ):
        out.update(m for m in re.findall(pat, text) if m)
    return out


class GroundTruthGazeboModelsNode(Node):
    def __init__(self) -> None:
        super().__init__("ground_truth_gazebo_models_node")
        self._cb_group = ReentrantCallbackGroup()
        self.declare_parameter("tracks_topic", "sim/ground_truth")
        self.declare_parameter("world_name", "sydney_regatta")
        self.declare_parameter("model_name_prefix", "gt_ctrv_")
        self.declare_parameter("update_dt", 0.05)
        self.declare_parameter("spawn_delay_sec", 10.0)
        # 兼容旧参数：作为 gz service --timeout 的毫秒值（与 world_service_wait_sec 秒数对应）
        self.declare_parameter("world_service_wait_sec", 1.0)
        self.declare_parameter("create_cli_timeout_sec", 20.0)
        self.declare_parameter("spawn_thread_pool_size", 2)
        # >0 时对 GlobalTrack 推导的半径/高度做上限（米），0 表示不限制
        self.declare_parameter("cylinder_radius_cap_m", 0.0)
        self.declare_parameter("cylinder_height_cap_m", 0.0)
        # 非空时：订阅 Contacts 或 String(model_name)，命中 {prefix}{id} 则从 Gazebo 删除并抑制重生，直至该 track_id 从真值消息中消失
        self.declare_parameter("collision_topic", "")
        self.declare_parameter("collision_string_topic", "")
        self.declare_parameter("collision_debounce_sec", 0.5)
        self.declare_parameter("remove_retry_interval_sec", 1.0)
        self.declare_parameter("reconcile_interval_sec", 2.0)
        self.declare_parameter("cleanup_stale_models_on_start", True)
        # box：与 size_l/size_w/size_h 一致；cylinder：外接圆半径 + 高；mesh_profile：按 YAML 生成 mesh + compound collision
        self.declare_parameter("gazebo_target_geometry", "box")
        self.declare_parameter("gazebo_mesh_profile", "")
        # 与 Gazebo 其它刚体接触；须 ≤0xFFFF（SDF 16 位），默认与官方教程一致
        self.declare_parameter("contact_collide_bitmask", _COLLIDE_BITMASK_MAX)

        self._prefix = str(self.get_parameter("model_name_prefix").value).strip() or "gt_ctrv_"
        self._world = str(self.get_parameter("world_name").value).strip() or "sydney_regatta"
        dt = float(self.get_parameter("update_dt").value)
        self._dt = dt if dt > 0 else 0.05
        delay = max(0.0, float(self.get_parameter("spawn_delay_sec").value))
        self._spawn_delay_sec = delay
        wsw = max(0.05, float(self.get_parameter("world_service_wait_sec").value))
        self._gz_timeout_ms = max(500, int(wsw * 1000.0))
        self._create_timeout = max(5.0, float(self.get_parameter("create_cli_timeout_sec").value))
        pool = max(1, int(self.get_parameter("spawn_thread_pool_size").value))
        self._pool = ThreadPoolExecutor(max_workers=pool, thread_name_prefix="gt_gz_spawn")

        tt = str(self.get_parameter("tracks_topic").value).strip() or "sim/ground_truth"
        if tt.startswith("/"):
            tt = tt.lstrip("/")

        self._set_pose_vector_svc = f"/world/{self._world}/set_pose_vector"
        self._remove_svc = f"/world/{self._world}/remove"

        self._latest: Optional[GlobalTrackArray] = None
        self._spawned_ids: Set[int] = set()
        self._spawn_futures: Dict[int, Future] = {}
        self._warned_frame = False
        self._logged_delay = False
        self._start_ns = self.get_clock().now().nanoseconds
        self._spawn_gate_open_ns = self._start_ns + int(self._spawn_delay_sec * 1e9)
        self._rx_logged = False
        self._pose_fail_count = 0
        self._r_cap = float(self.get_parameter("cylinder_radius_cap_m").value)
        self._h_cap = float(self.get_parameter("cylinder_height_cap_m").value)
        self._collision_debounce = max(0.0, float(self.get_parameter("collision_debounce_sec").value))
        self._remove_retry_interval = max(
            0.1, float(self.get_parameter("remove_retry_interval_sec").value)
        )
        self._reconcile_interval = max(
            0.5, float(self.get_parameter("reconcile_interval_sec").value)
        )
        self._cleanup_stale_on_start = self._get_bool_param("cleanup_stale_models_on_start")
        self._next_reconcile_mono = 0.0
        self._startup_cleanup_done = False
        self._collision_suppressed: Set[int] = set()
        self._last_collision_mono: Dict[str, float] = {}
        self._pending_remove_retry_mono: Dict[int, float] = {}
        self._remove_fail_counts: Dict[int, int] = {}
        self._state_lock = threading.Lock()
        # 多个 create / gz service 同时打 gz-transport 易触发 RecvSrvRequest Host unreachable；全局串行化 CLI。
        self._gz_cli_lock = threading.Lock()
        self._retained_create_sdf_paths: list[str] = []

        _gm = str(self.get_parameter("gazebo_target_geometry").value).strip().lower()
        self._geom_mode = _gm if _gm in ("box", "cylinder", "mesh_profile") else "box"
        if _gm not in ("box", "cylinder", "mesh_profile"):
            self.get_logger().warn(
                "gazebo_target_geometry=%r 无效，使用 box（有效值：box、cylinder、mesh_profile）" % (_gm,)
            )
        _bm = self.get_parameter("contact_collide_bitmask").value
        try:
            raw_bm = int(str(_bm).strip(), 0) if isinstance(_bm, str) else int(_bm)
        except (TypeError, ValueError):
            raw_bm = _COLLIDE_BITMASK_MAX
            self.get_logger().warn(
                "contact_collide_bitmask 无效，使用 0x%X" % _COLLIDE_BITMASK_MAX
            )
        if raw_bm != (raw_bm & _COLLIDE_BITMASK_MAX):
            self.get_logger().warn(
                "contact_collide_bitmask=%s 超出 SDF 16 位上限，已截断为 0x%X"
                % (raw_bm, _COLLIDE_BITMASK_MAX)
            )
        self._contact_bitmask = raw_bm & _COLLIDE_BITMASK_MAX
        self._mesh_profile_path = ""
        self._mesh_profile_data: Optional[dict] = None
        self._mesh_profile_spawn_z = 0.0
        if self._geom_mode == "mesh_profile":
            raw_profile = str(self.get_parameter("gazebo_mesh_profile").value).strip()
            self._mesh_profile_path, self._mesh_profile_data = self._load_mesh_profile(raw_profile)
            self._mesh_profile_spawn_z = self._mesh_profile_default_spawn_z(self._mesh_profile_data)
            self.get_logger().info(
                "ground_truth_gazebo_models_node: using mesh_profile %s (spawn_z=%.3f)"
                % (self._mesh_profile_path, self._mesh_profile_spawn_z)
            )

        self.create_subscription(
            GlobalTrackArray,
            tt,
            self._on_tracks,
            _tracks_qos(),
            callback_group=self._cb_group,
        )
        ct = str(self.get_parameter("collision_topic").value).strip()
        if ct.startswith("/"):
            ct = ct.lstrip("/")
        cst = str(self.get_parameter("collision_string_topic").value).strip()
        if cst.startswith("/"):
            cst = cst.lstrip("/")
        if ct:
            if Contacts is not None:
                self.create_subscription(
                    Contacts,
                    ct,
                    self._on_contacts,
                    10,
                    callback_group=self._cb_group,
                )
                self.get_logger().info(
                    "collision_topic=%s：Contacts 命中 %s* 时从 Gazebo 移除真值模型并抑制重生（须 ros_gz_bridge 桥接 contact）"
                    % (ct, self._prefix)
                )
            else:
                self.get_logger().warn(
                    "collision_topic 已设置但本环境无 ros_gz_interfaces.msg.Contacts，跳过订阅"
                )
        if cst:
            self.create_subscription(
                String,
                cst,
                self._on_collision_string,
                10,
                callback_group=self._cb_group,
            )
            self.get_logger().info(
                "collision_string_topic=%s：data 为模型名（如 gt_ctrv_1）时视同碰撞" % cst
            )

        self.create_timer(self._dt, self._tick, callback_group=self._cb_group)
        self.get_logger().info(
            "ground_truth_gazebo_models_node: world=%s tracks=%s prefix=%s geometry=%s "
            "spawn_delay_sec=%.1f remove_retry_interval_sec=%.1f (create CLI + gz set_pose_vector)"
            % (
                self._world,
                tt,
                self._prefix,
                self._geom_mode,
                self._spawn_delay_sec,
                self._remove_retry_interval,
            )
        )

    def destroy_node(self) -> bool:
        self._pool.shutdown(wait=False, cancel_futures=True)
        for p in self._retained_create_sdf_paths:
            if p and os.path.isfile(p):
                try:
                    os.unlink(p)
                except OSError:
                    pass
        self._retained_create_sdf_paths.clear()
        return super().destroy_node()

    def _get_bool_param(self, name: str) -> bool:
        value = self.get_parameter(name).value
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return bool(value)
        return str(value).lower() in ("true", "1", "yes")

    def _on_tracks(self, msg: GlobalTrackArray) -> None:
        if not self._rx_logged and msg.tracks:
            self._rx_logged = True
            self.get_logger().info(
                "Receiving GlobalTrackArray (%d tracks), mirroring to Gazebo via create/gz"
                % len(msg.tracks)
            )
        self._latest = msg

    def _track_entity_name(self, track_id: int) -> str:
        return "%s%d" % (self._prefix, int(track_id))

    def _z_center_for_height(self, height_m: float) -> float:
        """底面贴 z=0，几何中心高度。"""
        h = max(float(height_m), 0.1)
        return max(h * 0.5, 0.25)

    def _box_dims(self, t: GlobalTrack) -> tuple[float, float, float]:
        """真值长宽高（米），应用与圆柱相同的水平/高度 cap。"""
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
        sh = max(sh, 0.5)
        sl = max(sl, 0.1)
        sw = max(sw, 0.1)
        return sl, sw, sh

    def _cylinder_dims(self, t: GlobalTrack) -> tuple[float, float, float]:
        """圆柱半径、高度、竖直中心 z。"""
        r = _cylinder_radius(t.size_l, t.size_w)
        h = max(float(t.size_h), 0.5)
        if self._r_cap > 0.0:
            r = min(r, self._r_cap)
        if self._h_cap > 0.0:
            h = min(h, self._h_cap)
        h = max(h, 0.5)
        r = max(r, 0.1)
        z = self._z_center_for_height(h)
        return r, h, z

    def _pose_z_for_track(self, t: GlobalTrack) -> float:
        if self._geom_mode == "mesh_profile":
            return self._mesh_profile_spawn_z
        if self._geom_mode == "box":
            _, _, sh = self._box_dims(t)
            return self._z_center_for_height(sh)
        _, h, z = self._cylinder_dims(t)
        return z

    def _track_id_from_model_base(self, base: str) -> Optional[int]:
        if not base.startswith(self._prefix):
            return None
        suf = base[len(self._prefix) :]
        if not suf.isdigit():
            return None
        return int(suf)

    def _resolve_mesh_profile_path(self, raw_path: str) -> str:
        path_text = str(raw_path or "").strip()
        if not path_text:
            raise RuntimeError("gazebo_target_geometry=mesh_profile 时必须提供 gazebo_mesh_profile")
        if os.path.isabs(path_text):
            return path_text
        return os.path.abspath(path_text)

    def _load_mesh_profile(self, raw_path: str) -> tuple[str, dict]:
        profile_path = self._resolve_mesh_profile_path(raw_path)
        if not os.path.isfile(profile_path):
            raise RuntimeError(f"gazebo_mesh_profile 文件不存在: {profile_path}")
        with open(profile_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        if not isinstance(data, dict):
            raise RuntimeError(f"gazebo_mesh_profile YAML 非法: {profile_path}")
        mesh = data.get("mesh") or {}
        if not str(mesh.get("uri", "")).strip():
            raise RuntimeError(f"gazebo_mesh_profile 缺少 mesh.uri: {profile_path}")
        boxes = data.get("boxes") or []
        if not isinstance(boxes, list) or not boxes:
            raise RuntimeError(f"gazebo_mesh_profile 缺少 boxes[]: {profile_path}")
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

    def _missing_model_bases_from_output(self, text: str) -> Set[str]:
        out: Set[str] = set()
        if not text:
            return out
        lower = text.lower()
        for name in _extract_model_names(text):
            base = _base_model_name(name)
            if base.startswith(self._prefix):
                out.add(base)
        if "not found" in lower or "unable to update the pose for entity id:[0]" in lower:
            return out
        return set()

    def _create_output_indicates_existing(self, text: str, model_name: str) -> bool:
        lower = (text or "").lower()
        return (
            model_name.lower() in lower
            and ("already exists" in lower or "entity already exists" in lower)
        )

    def _list_world_model_bases(self) -> tuple[Optional[Set[str]], str]:
        gz = _gz_executable()
        cmd = [
            gz,
            "topic",
            "-e",
            "-n",
            "1",
            "-t",
            f"/world/{self._world}/pose/info",
        ]
        try:
            with self._gz_cli_lock:
                result = subprocess.run(
                    cmd,
                    capture_output=True,
                    text=True,
                    timeout=max(6.0, self._gz_timeout_ms / 250.0),
                )
            output = ((result.stdout or "") + "\n" + (result.stderr or "")).strip()
            if result.returncode != 0:
                return None, output or ("returncode=%s" % result.returncode)
            bases = {
                _base_model_name(name)
                for name in _extract_model_names(result.stdout or "")
                if _base_model_name(name).startswith(self._prefix)
            }
            return bases, output
        except Exception as ex:  # pragma: no cover
            return None, str(ex)

    def _reconcile_world_entities(self, current_ids: Set[int], force: bool = False) -> None:
        now_mono = time.monotonic()
        if not force and now_mono < self._next_reconcile_mono:
            return
        bases, detail = self._list_world_model_bases()
        self._next_reconcile_mono = now_mono + self._reconcile_interval
        if bases is None:
            self.get_logger().warn(
                "世界实体对账失败：%s" % (detail[:300] if detail else "no detail"),
                throttle_duration_sec=10.0,
            )
            return

        world_ids = {
            tid for tid in (self._track_id_from_model_base(base) for base in bases) if tid is not None
        }
        with self._state_lock:
            suppressed_ids = set(self._collision_suppressed)
            desired_ids = current_ids - suppressed_ids
            self._spawned_ids.intersection_update(world_ids)
            self._spawned_ids.update(world_ids & desired_ids)
            stale_ids = sorted(world_ids - desired_ids)

        if force and self._cleanup_stale_on_start:
            self._startup_cleanup_done = True
        elif force:
            return

        for tid in stale_ids:
            name = self._track_entity_name(tid)
            ok, rm_detail = self._remove_entity_gz(name)
            with self._state_lock:
                if ok:
                    self._spawned_ids.discard(tid)
                    self._pending_remove_retry_mono.pop(tid, None)
                    self._remove_fail_counts.pop(tid, None)
            if ok:
                self.get_logger().info("对账清理残留 Gazebo 模型：%s" % name)
            else:
                self.get_logger().warn(
                    "对账清理 %s 失败：%s" % (name, (rm_detail or "no detail")[:300]),
                    throttle_duration_sec=5.0,
                )

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
            box_name = str(box.get("name") or f"collision_{idx}").strip() or f"collision_{idx}"
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
                f"""
      <collision name="{box_name}">
        <pose>{_fmt_pose_xyzrpy(xyz, rpy)}</pose>
        <geometry>
          <box><size>{size[0]:.6f} {size[1]:.6f} {size[2]:.6f}</size></box>
        </geometry>
{_collision_surface_xml(self._contact_bitmask)}      </collision>"""
            )

        return f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="{model_name}">
    <static>false</static>
    <link name="link">
      <gravity>false</gravity>
      <inertial>
        <mass>10.0</mass>
        <inertia>
          <ixx>1.0</ixx><iyy>1.0</iyy><izz>1.0</izz>
        </inertia>
      </inertial>
      <visual name="v">
        <pose>{_fmt_pose_xyzrpy(mesh_xyz, mesh_rpy)}</pose>
        <geometry>
          <mesh>
            <uri>{mesh_uri}</uri>
            <scale>{mesh_scale[0]:.6f} {mesh_scale[1]:.6f} {mesh_scale[2]:.6f}</scale>
          </mesh>
        </geometry>
        <material>
          <ambient>{mesh_rgba[0]:.6f} {mesh_rgba[1]:.6f} {mesh_rgba[2]:.6f} {mesh_rgba[3]:.6f}</ambient>
          <diffuse>{mesh_rgba[0]:.6f} {mesh_rgba[1]:.6f} {mesh_rgba[2]:.6f} {mesh_rgba[3]:.6f}</diffuse>
          <specular>0.15 0.15 0.10 1.0</specular>
        </material>
      </visual>
{''.join(collision_xml)}
    </link>
  </model>
</sdf>
"""

    def _collision_debounced(self, key: str) -> bool:
        if self._collision_debounce <= 0.0:
            return True
        now = time.monotonic()
        last = self._last_collision_mono.get(key, 0.0)
        if now - last < self._collision_debounce:
            return False
        self._last_collision_mono[key] = now
        return True

    def _handle_collision_model_base(self, model_base: str) -> None:
        tid = self._track_id_from_model_base(model_base)
        if tid is None:
            return
        if not self._collision_debounced(model_base):
            return
        name = self._track_entity_name(tid)
        self.get_logger().info(
            "碰撞移除 Gazebo 模型 %s（track_id=%d），抑制重生直至该 ID 从真值列表消失"
            % (name, tid)
        )
        with self._state_lock:
            self._collision_suppressed.add(tid)
            fut = self._spawn_futures.pop(tid, None)
        if fut is not None and not fut.done():
            fut.cancel()
        ok, detail = self._remove_entity_gz(name)
        with self._state_lock:
            if ok:
                self._spawned_ids.discard(tid)
                self._pending_remove_retry_mono.pop(tid, None)
                fail_count = self._remove_fail_counts.pop(tid, 0)
            else:
                self._pending_remove_retry_mono[tid] = time.monotonic() + self._remove_retry_interval
                fail_count = self._remove_fail_counts.get(tid, 0) + 1
                self._remove_fail_counts[tid] = fail_count
        if ok:
            if fail_count > 0:
                self.get_logger().info(
                    "碰撞删除 %s 在 %d 次失败后成功" % (name, fail_count)
                )
        elif fail_count == 1 or fail_count % 5 == 0:
            self.get_logger().warn(
                "碰撞删除 %s 失败，将在 %.1fs 后重试：%s"
                % (name, self._remove_retry_interval, detail or "no detail")
            )

    def _on_contacts(self, msg: Contacts) -> None:  # type: ignore[valid-type]
        hit: Set[str] = set()
        for c in msg.contacts:
            for ent in (c.collision1, c.collision2):
                base = _base_model_name(getattr(ent, "name", "") or "")
                if base:
                    hit.add(base)
        for base in hit:
            self._handle_collision_model_base(base)

    def _on_collision_string(self, msg: String) -> None:
        base = (msg.data or "").strip().split("::", 1)[0]
        if base:
            self._handle_collision_model_base(base)

    def _color_rgba(self, t: GlobalTrack) -> str:
        if t.is_ais_matched:
            return "0.15 0.75 0.25 1"
        return "0.95 0.45 0.1 1"

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
                self.get_logger().error("create CLI exception for %s: %s" % (name, ex))
                continue
            with self._state_lock:
                if ok:
                    self._spawned_ids.add(tid)
            if ok:
                self.get_logger().info("Gazebo spawn OK (create CLI): %s" % name)
            else:
                self.get_logger().error(
                    "Gazebo spawn FAILED (create CLI): %s — %s" % (name, err or "no detail")
                )

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
        tmp_sdf_path = None
        try:
            fd, tmp_sdf_path = tempfile.mkstemp(prefix="gt_gz_", suffix=".sdf")
            os.close(fd)
            with open(tmp_sdf_path, "w") as f:
                f.write(sdf)
            cmd = [
                "ros2",
                "run",
                "ros_gz_sim",
                "create",
                "-world",
                self._world,
                "-file",
                tmp_sdf_path,
                "-name",
                model_name,
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
            env = os.environ.copy()
            with self._gz_cli_lock:
                result = subprocess.run(
                    cmd,
                    capture_output=True,
                    text=True,
                    timeout=self._create_timeout,
                    env=env,
                )
            err = (result.stderr or result.stdout or "").strip()
            if self._create_output_indicates_existing(err, model_name):
                return True, err
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
            return False, "ros_gz_sim create timeout (%.1fs)" % self._create_timeout
        except Exception as ex:  # pragma: no cover
            if tmp_sdf_path and os.path.isfile(tmp_sdf_path):
                try:
                    os.unlink(tmp_sdf_path)
                except OSError:
                    pass
            return False, str(ex)

    def _submit_spawn(self, t: GlobalTrack) -> None:
        tid = int(t.track_id)
        name = self._track_entity_name(tid)
        rgba = self._color_rgba(t)
        bm = self._contact_bitmask
        if self._geom_mode == "mesh_profile":
            sdf = self._build_mesh_profile_sdf(name)
            z = self._mesh_profile_spawn_z
        elif self._geom_mode == "box":
            sx, sy, sz = self._box_dims(t)
            sdf = _build_box_sdf(name, sx, sy, sz, rgba, bm)
            z = self._z_center_for_height(sz)
        else:
            r, h, z = self._cylinder_dims(t)
            sdf = _build_cylinder_sdf(name, r, h, rgba, bm)
        yaw = _yaw_from_track(t)
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
            "create CLI queued for %s at (%.1f, %.1f, %.1f) yaw=%.3f"
            % (name, float(t.x), float(t.y), z, yaw)
        )

    def _pose_v_text_for_tracks(self, tracks: list[GlobalTrack]) -> str:
        parts = []
        for t in tracks:
            name = self._track_entity_name(int(t.track_id))
            z = self._pose_z_for_track(t)
            qx, qy, qz, qw = _yaw_to_quaternion(_yaw_from_track(t))
            parts.append(
                "pose {\n"
                '  name: "%s"\n'
                "  position { x: %.9g y: %.9g z: %.9g }\n"
                "  orientation { x: %.9g y: %.9g z: %.9g w: %.9g }\n"
                "}" % (name, float(t.x), float(t.y), z, qx, qy, qz, qw)
            )
        return "\n".join(parts)

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
            if ok:
                return True, output
            return False, output or ("returncode=%s" % result.returncode)
        except Exception as ex:  # pragma: no cover
            return False, str(ex)

    def _tick(self) -> None:
        self._drain_spawn_futures()

        msg = self._latest
        if msg is None:
            elapsed = (self.get_clock().now().nanoseconds - self._start_ns) * 1e-9
            if elapsed > 5.0:
                self.get_logger().warn(
                    "No GlobalTrackArray received yet on tracks topic.",
                    throttle_duration_sec=15.0,
                )
            return
        if msg.header.frame_id != "map":
            if not self._warned_frame:
                self.get_logger().warn(
                    "Tracks frame_id is '%s' (expected 'map'); poses are still applied in world."
                    % msg.header.frame_id
                )
                self._warned_frame = True

        now_ns = self.get_clock().now().nanoseconds
        if now_ns < self._spawn_gate_open_ns:
            if not self._logged_delay:
                self.get_logger().info(
                    "spawn_delay_sec=%.1f：%.1fs 后再 spawn/set_pose（当前为启动后延迟）"
                    % (
                        self._spawn_delay_sec,
                        (self._spawn_gate_open_ns - now_ns) * 1e-9,
                    )
                )
                self._logged_delay = True
            return

        current_ids = {int(t.track_id) for t in msg.tracks}
        self._reconcile_world_entities(
            current_ids,
            force=(self._cleanup_stale_on_start and not self._startup_cleanup_done),
        )

        remove_names: list[str] = []
        to_spawn: list[GlobalTrack] = []
        pose_tracks: list[GlobalTrack] = []

        with self._state_lock:
            for tid in list(self._collision_suppressed):
                if tid not in current_ids:
                    self._collision_suppressed.discard(tid)
            for tid in list(self._pending_remove_retry_mono):
                if tid in current_ids:
                    self._pending_remove_retry_mono.pop(tid, None)
                    self._remove_fail_counts.pop(tid, None)

            removed_ids = self._spawned_ids - current_ids
            pending_removed = set(self._spawn_futures.keys()) - current_ids
            for tid in pending_removed:
                fut = self._spawn_futures.pop(tid, None)
                if fut is not None and not fut.done():
                    fut.cancel()

            now_mono = time.monotonic()
            for tid in removed_ids:
                retry_due = self._pending_remove_retry_mono.get(tid, 0.0)
                if now_mono >= retry_due:
                    self._pending_remove_retry_mono[tid] = now_mono + self._remove_retry_interval
                    remove_names.append(self._track_entity_name(tid))

            for t in msg.tracks:
                tid = int(t.track_id)
                if tid in self._collision_suppressed:
                    continue
                if tid not in self._spawned_ids and tid not in self._spawn_futures:
                    to_spawn.append(t)
                if tid in self._spawned_ids:
                    pose_tracks.append(t)

        for nm in remove_names:
            tid = self._track_id_from_model_base(nm)
            ok, detail = self._remove_entity_gz(nm)
            if tid is None:
                continue
            with self._state_lock:
                if ok:
                    self._spawned_ids.discard(tid)
                    self._pending_remove_retry_mono.pop(tid, None)
                    fail_count = self._remove_fail_counts.pop(tid, 0)
                else:
                    self._pending_remove_retry_mono[tid] = time.monotonic() + self._remove_retry_interval
                    fail_count = self._remove_fail_counts.get(tid, 0) + 1
                    self._remove_fail_counts[tid] = fail_count
            if ok:
                if fail_count > 0:
                    self.get_logger().info(
                        "Gazebo 删除 %s 在 %d 次失败后成功" % (nm, fail_count)
                    )
            elif fail_count == 1 or fail_count % 5 == 0:
                self.get_logger().warn(
                    "Gazebo 删除 %s 失败，将在 %.1fs 后重试：%s"
                    % (nm, self._remove_retry_interval, detail or "no detail")
                )

        for t in to_spawn:
            self._submit_spawn(t)

        self._apply_poses_vector_from_tracks(pose_tracks)

    def _apply_poses_vector_from_tracks(self, tracks: list[GlobalTrack]) -> None:
        body = self._pose_v_text_for_tracks(tracks)
        if not body:
            return
        gz = _gz_executable()
        cmd = [
            gz,
            "service",
            "-s",
            self._set_pose_vector_svc,
            "--reqtype",
            "gz.msgs.Pose_V",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            str(self._gz_timeout_ms),
            "--req",
            body,
        ]
        try:
            with self._gz_cli_lock:
                result = subprocess.run(
                    cmd,
                    capture_output=True,
                    text=True,
                    timeout=max(5.0, self._gz_timeout_ms / 500.0),
                )
            out = (result.stdout or result.stderr or "").lower()
            ok = result.returncode == 0 and "data: false" not in out.replace(" ", "")
            if not ok:
                self._pose_fail_count += 1
                missing_bases = self._missing_model_bases_from_output(
                    (result.stdout or "") + "\n" + (result.stderr or "")
                )
                if missing_bases:
                    with self._state_lock:
                        for base in missing_bases:
                            tid = self._track_id_from_model_base(base)
                            if tid is None:
                                continue
                            self._spawned_ids.discard(tid)
                            self._pending_remove_retry_mono.pop(tid, None)
                            self._remove_fail_counts.pop(tid, None)
                    self._next_reconcile_mono = 0.0
                if self._pose_fail_count % 100 == 1:
                    self.get_logger().warn(
                        "set_pose_vector may have failed (rc=%s): %s"
                        % (result.returncode, (result.stderr or result.stdout or "")[:300]),
                        throttle_duration_sec=5.0,
                    )
            else:
                self._pose_fail_count = 0
        except Exception as ex:  # pragma: no cover
            self.get_logger().warn(
                "set_pose_vector subprocess error: %s" % ex, throttle_duration_sec=5.0
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GroundTruthGazeboModelsNode()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("ground_truth_gazebo_models_node shutting down")
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
