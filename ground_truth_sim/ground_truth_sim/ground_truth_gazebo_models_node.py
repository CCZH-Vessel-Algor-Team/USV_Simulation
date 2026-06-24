#!/usr/bin/env python3
"""Mirror GlobalTrackArray targets as Gazebo Sim models (visual + collision).

.. deprecated::
    Prefer ``ground_truth_gazebo_entity_node`` when ``gazebo_visual=true``;
    this node subscribes external truth and mirrors to Gazebo (legacy dual-track).

Drive models via Gazebo VelocityControl + cmd_vel (same pattern as scenario_manager_node):
  - spawn: ``ros2 run ros_gz_sim create`` + temporary SDF (VelocityControl plugin embedded)
  - motion: ROS ``/model/{name}/cmd_vel`` → ros_gz_bridge → gz VelocityControl
  - remove: ``gz service`` → ``/world/<world>/remove`` (``gz.msgs.Entity``)
  - optional drift resync: single ``gz service .../set_pose`` when error exceeds threshold

Spawns run in a small thread pool so the MultiThreadedExecutor is not blocked by subprocess I/O.
Teleport-only legacy implementation: ``ground_truth_gazebo_models_node_teleport.py``.
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
from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Set, Tuple

import rclpy
import yaml
from geometry_msgs.msg import Twist
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from ground_truth_sim.arc_follow import compute_arc_follow_cmd_vel
from ground_truth_sim.spawn_ready_sync import (
    DEFAULT_MOTION_READY_TOPIC,
    all_tracks_spawn_ready,
    normalize_motion_ready_topic,
    pending_spawn_track_ids,
)
from usv_interfaces.msg import GlobalTrack, GlobalTrackArray

try:
    from ros_gz_interfaces.msg import Contacts
except ImportError:  # pragma: no cover
    Contacts = None  # type: ignore

try:
    import gz.transport13 as gz_transport
    from gz.msgs10.twist_pb2 import Twist as GzTwist
except ImportError:  # pragma: no cover
    gz_transport = None  # type: ignore
    GzTwist = None  # type: ignore


# Gazebo Sim / libsdformat 的 <collide_bitmask> 按 16 位处理；0xFFFFFFFF 会报 Out of range。
_COLLIDE_BITMASK_MAX = 0xFFFF


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_velocity(
    vx: float, vy: float, last_yaw: float, speed_eps: float = 1e-3
) -> float:
    speed = math.hypot(vx, vy)
    if speed < speed_eps:
        return last_yaw
    return math.atan2(vy, vx)


def compute_omega(yaw: float, last_yaw: float, dt: float) -> float:
    if dt <= 0.0:
        return 0.0
    return normalize_angle(yaw - last_yaw) / dt


def clamp_omega(omega: float, limit: float) -> float:
    lim = max(0.0, float(limit))
    if lim <= 0.0:
        return omega
    return max(-lim, min(lim, omega))


def world_velocity_to_body_twist(
    vx_world: float, vy_world: float, yaw: float, omega: float
) -> Twist:
    """Map 系线速度 + 航向角速度 → 船体系 Twist（与 scenario_manager 一致）。"""
    c = math.cos(yaw)
    s = math.sin(yaw)
    twist = Twist()
    twist.linear.x = c * vx_world + s * vy_world
    twist.linear.y = -s * vx_world + c * vy_world
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = omega
    return twist


def _quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Quaternion (ENU, z-up) → yaw."""
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def _yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def _yaw_from_track(t: GlobalTrack) -> float:
    return yaw_from_velocity(float(t.v_x), float(t.v_y), 0.0)


def _cylinder_radius(size_l: float, size_w: float) -> float:
    sl = max(float(size_l), 0.5)
    sw = max(float(size_w), 0.5)
    return 0.5 * math.sqrt(sl * sl + sw * sw)


def _collision_surface_xml(collide_bitmask: int) -> str:
    bm = int(collide_bitmask) & _COLLIDE_BITMASK_MAX
    return (
        "        <surface>\n"
        "          <contact>\n"
        "            <collide_bitmask>%u</collide_bitmask>\n"
        "          </contact>\n"
        "        </surface>\n" % bm
    )


def _inertial_xml(mass_kg: float) -> str:
    return f"""      <inertial>
        <mass>{mass_kg:.1f}</mass>
        <inertia>
          <ixx>4.0</ixx><iyy>4.0</iyy><izz>6.0</izz>
        </inertia>
      </inertial>"""


def _velocity_control_plugin_xml(model_name: str) -> str:
    return f"""    <plugin filename="gz-sim-velocity-control-system"
            name="gz::sim::systems::VelocityControl">
      <topic>/model/{model_name}/cmd_vel</topic>
    </plugin>"""


def _build_box_sdf(
    model_name: str,
    sx: float,
    sy: float,
    sz: float,
    rgba: str,
    collide_bitmask: int,
    mass_kg: float,
) -> str:
    surf = _collision_surface_xml(collide_bitmask)
    inertial = _inertial_xml(mass_kg)
    plugin = _velocity_control_plugin_xml(model_name)
    return f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="{model_name}">
    <static>false</static>
    <link name="link">
      <gravity>false</gravity>
{inertial}
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
{plugin}
  </model>
</sdf>
"""


def _build_cylinder_sdf(
    model_name: str,
    radius: float,
    length: float,
    rgba: str,
    collide_bitmask: int,
    mass_kg: float,
) -> str:
    surf = _collision_surface_xml(collide_bitmask)
    inertial = _inertial_xml(mass_kg)
    plugin = _velocity_control_plugin_xml(model_name)
    return f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="{model_name}">
    <static>false</static>
    <link name="link">
      <gravity>false</gravity>
{inertial}
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
{plugin}
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


def _quat_to_roll_pitch(qx: float, qy: float, qz: float, qw: float) -> tuple[float, float]:
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi * 0.5, sinp)
    else:
        pitch = math.asin(sinp)
    return roll, pitch


@dataclass(frozen=True)
class ModelWorldPose:
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


def _parse_quat_from_orientation_block(block: str) -> Optional[tuple[float, float, float, float]]:
    """解析 orientation 子块；Gazebo 常省略零分量 x/y。"""
    ori_m = re.search(r"orientation\s*\{([^}]*)\}", block, re.DOTALL)
    if not ori_m:
        return None
    inner = ori_m.group(1)

    def _component(name: str, default: float) -> float:
        comp = re.search(r"%s:\s*([-+0-9.eE]+)" % name, inner)
        return float(comp.group(1)) if comp else default

    return (
        _component("x", 0.0),
        _component("y", 0.0),
        _component("z", 0.0),
        _component("w", 1.0),
    )


def _parse_model_poses_from_pose_info(text: str) -> Dict[str, ModelWorldPose]:
    """从 ``/world/.../pose/info`` 文本解析 model base name → 位姿。"""
    poses: Dict[str, ModelWorldPose] = {}
    if not text:
        return poses
    blocks = re.split(r"(?=pose\s*\{)", text)
    for block in blocks:
        name_m = re.search(r'name:\s*"([^"]+)"', block)
        if not name_m:
            continue
        base = _base_model_name(name_m.group(1))
        pos_m = re.search(
            r"position\s*\{[^}]*x:\s*([-+0-9.eE]+)[^}]*y:\s*([-+0-9.eE]+)[^}]*z:\s*([-+0-9.eE]+)",
            block,
            re.DOTALL,
        )
        quat = _parse_quat_from_orientation_block(block)
        if pos_m and quat is not None:
            poses[base] = ModelWorldPose(
                x=float(pos_m.group(1)),
                y=float(pos_m.group(2)),
                z=float(pos_m.group(3)),
                qx=quat[0],
                qy=quat[1],
                qz=quat[2],
                qw=quat[3],
            )
    return poses


def _parse_model_xy_from_pose_info(text: str) -> Dict[str, Tuple[float, float]]:
    return {
        name: (pose.x, pose.y)
        for name, pose in _parse_model_poses_from_pose_info(text).items()
    }


def attitude_violation_reason(
    pose: ModelWorldPose,
    expected_z: float,
    max_roll_pitch_deg: float,
    max_z_error_m: float,
) -> Optional[str]:
    roll, pitch = _quat_to_roll_pitch(pose.qx, pose.qy, pose.qz, pose.qw)
    max_rad = math.radians(max(0.0, max_roll_pitch_deg))
    if abs(roll) > max_rad or abs(pitch) > max_rad:
        return "roll=%.1f° pitch=%.1f°" % (math.degrees(roll), math.degrees(pitch))
    if max_z_error_m > 0.0 and abs(pose.z - expected_z) > max_z_error_m:
        return "z=%.2fm (expected %.2fm)" % (pose.z, expected_z)
    return None


@dataclass
class TrackDriver:
    track_id: int
    model_name: str
    cmd_vel_pub: Optional[Publisher]
    bridge_proc: Optional[subprocess.Popen]
    gz_pub: Any = None
    last_yaw: float = 0.0
    spawn_yaw: float = 0.0
    last_resync_mono: float = field(default_factory=time.monotonic)
    last_truth_x: Optional[float] = None
    last_truth_y: Optional[float] = None
    last_respawn_mono: float = 0.0
    last_heading_sync_mono: float = 0.0


class GroundTruthGazeboModelsNode(Node):
    def __init__(self) -> None:
        super().__init__("ground_truth_gazebo_models_node")
        self._cb_group = ReentrantCallbackGroup()
        self.declare_parameter("tracks_topic", "sim/ground_truth")
        self.declare_parameter("world_name", "sydney_regatta")
        self.declare_parameter("model_name_prefix", "gt_ctrv_")
        self.declare_parameter("update_dt", 0.02)
        self.declare_parameter("spawn_delay_sec", 10.0)
        self.declare_parameter("world_service_wait_sec", 1.0)
        self.declare_parameter("create_cli_timeout_sec", 20.0)
        self.declare_parameter("spawn_thread_pool_size", 2)
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
        self.declare_parameter("cmd_vel_omega_limit", 0.12)
        self.declare_parameter("cmd_vel_turn_radius_min_m", 25.0)
        self.declare_parameter("cmd_vel_align_threshold_deg", 12.0)
        self.declare_parameter("cmd_vel_pose_refresh_interval_sec", 0.05)
        self.declare_parameter("position_resync_enabled", False)
        self.declare_parameter("position_resync_threshold_m", 15.0)
        self.declare_parameter("position_resync_min_interval_sec", 5.0)
        self.declare_parameter("position_resync_use_set_pose", True)
        self.declare_parameter("fence_jump_respawn_threshold_m", 25.0)
        self.declare_parameter("truth_jump_respawn_enabled", True)
        self.declare_parameter("attitude_monitor_enabled", True)
        self.declare_parameter("attitude_max_roll_pitch_deg", 10.0)
        self.declare_parameter("attitude_max_z_error_m", 2.0)
        self.declare_parameter("attitude_monitor_interval_sec", 0.1)
        self.declare_parameter("attitude_respawn_cooldown_sec", 2.0)
        self.declare_parameter("spawn_post_set_pose_enabled", True)
        self.declare_parameter("heading_sync_enabled", True)
        self.declare_parameter("heading_sync_interval_sec", 1.0)
        self.declare_parameter("spawn_ready_sync_enabled", False)
        self.declare_parameter("motion_ready_topic", DEFAULT_MOTION_READY_TOPIC)
        self.declare_parameter("spawn_ready_timeout_sec", 30.0)

        self._prefix = str(self.get_parameter("model_name_prefix").value).strip() or "gt_ctrv_"
        self._world = str(self.get_parameter("world_name").value).strip() or "sydney_regatta"
        dt = float(self.get_parameter("update_dt").value)
        self._dt = dt if dt > 0 else 0.02
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

        self._remove_svc = f"/world/{self._world}/remove"
        self._set_pose_svc = f"/world/{self._world}/set_pose"

        self._model_mass_kg = max(1.0, float(self.get_parameter("model_mass_kg").value))
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
        self._last_pose_refresh_mono = 0.0
        self._resync_enabled = self._get_bool_param("position_resync_enabled")
        self._resync_threshold = max(0.1, float(self.get_parameter("position_resync_threshold_m").value))
        self._resync_min_interval = max(
            0.5, float(self.get_parameter("position_resync_min_interval_sec").value)
        )
        self._resync_use_set_pose = self._get_bool_param("position_resync_use_set_pose")
        self._fence_jump_threshold = max(
            5.0, float(self.get_parameter("fence_jump_respawn_threshold_m").value)
        )
        self._truth_jump_respawn = self._get_bool_param("truth_jump_respawn_enabled")
        self._attitude_monitor = self._get_bool_param("attitude_monitor_enabled")
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
        self._spawn_post_set_pose = self._get_bool_param("spawn_post_set_pose_enabled")
        self._heading_sync = self._get_bool_param("heading_sync_enabled")
        self._heading_sync_interval = max(
            0.2, float(self.get_parameter("heading_sync_interval_sec").value)
        )
        self._next_attitude_check_mono = 0.0
        self._spawn_ready_sync = self._get_bool_param("spawn_ready_sync_enabled")
        self._spawn_ready_timeout_sec = max(
            0.0, float(self.get_parameter("spawn_ready_timeout_sec").value)
        )
        self._motion_sync_started = not self._spawn_ready_sync
        self._motion_ready_published = False
        self._motion_ready_topic = normalize_motion_ready_topic(
            str(self.get_parameter("motion_ready_topic").value)
        )
        _motion_ready_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._motion_ready_pub: Optional[Publisher] = None
        if self._spawn_ready_sync:
            self._motion_ready_pub = self.create_publisher(
                Bool, self._motion_ready_topic, _motion_ready_qos
            )

        self._latest: Optional[GlobalTrackArray] = None
        self._spawned_ids: Set[int] = set()
        self._spawn_futures: Dict[int, Future] = {}
        self._drivers: Dict[int, TrackDriver] = {}
        self._warned_frame = False
        self._logged_delay = False
        self._start_ns = self.get_clock().now().nanoseconds
        # 与 ground_truth_node 对齐：按仿真时钟 epoch（非节点启动时刻）打开 gate
        self._spawn_gate_open_ns = int(self._spawn_delay_sec * 1e9)
        self._rx_logged = False
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
        self._world_xy_cache: Dict[str, Tuple[float, float]] = {}
        self._world_pose_cache: Dict[str, ModelWorldPose] = {}
        self._state_lock = threading.RLock()
        self._gz_cli_lock = threading.Lock()
        self._retained_create_sdf_paths: list[str] = []
        self._gz_node = gz_transport.Node() if gz_transport is not None else None
        if self._gz_node is None:
            self.get_logger().warn(
                "gz.transport 不可用，cmd_vel 将经 ros_gz_bridge 转发（每船一个 bridge 进程）"
            )

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
                    "collision_topic=%s：Contacts 命中 %s* 时从 Gazebo 移除真值模型并抑制重生"
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
            "update_dt=%.3fs mass=%.1fkg omega_limit=%.3f motion=cmd_vel(%s) resync=%s "
            "attitude_monitor=%s spawn_ready_sync=%s"
            % (
                self._world,
                tt,
                self._prefix,
                self._geom_mode,
                self._dt,
                self._model_mass_kg,
                self._omega_limit,
                "gz-transport" if self._gz_node is not None else "ros_gz_bridge",
                str(self._resync_enabled),
                str(self._attitude_monitor),
                str(self._spawn_ready_sync),
            )
        )

    def destroy_node(self) -> bool:
        with self._state_lock:
            driver_ids = list(self._drivers.keys())
        for tid in driver_ids:
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
                "Receiving GlobalTrackArray (%d tracks), mirroring to Gazebo via create/cmd_vel"
                % len(msg.tracks)
            )
        self._latest = msg

    def _track_entity_name(self, track_id: int) -> str:
        return "%s%d" % (self._prefix, int(track_id))

    def _z_center_for_height(self, height_m: float) -> float:
        h = max(float(height_m), 0.1)
        return max(h * 0.5, 0.25)

    def _box_dims(self, t: GlobalTrack) -> tuple[float, float, float]:
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

    def _create_output_indicates_existing(self, text: str, model_name: str) -> bool:
        lower = (text or "").lower()
        return (
            model_name.lower() in lower
            and ("already exists" in lower or "entity already exists" in lower)
        )

    def _fetch_world_pose_info(self) -> tuple[Optional[str], str]:
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
            if not output:
                err = output or ("returncode=%s" % result.returncode)
                return None, err
            # gz topic -e 常非零退出但仍输出可用 protobuf 文本；有内容则继续解析
            return output, ""
        except Exception as ex:  # pragma: no cover
            return None, str(ex)

    def _refresh_world_pose_cache(self) -> bool:
        output, _ = self._fetch_world_pose_info()
        if output is None:
            return False
        pose_map = _parse_model_poses_from_pose_info(output)
        with self._state_lock:
            for base, pose in pose_map.items():
                if base.startswith(self._prefix):
                    self._world_pose_cache[base] = pose
                    self._world_xy_cache[base] = (pose.x, pose.y)
        return True

    def _list_world_model_bases(self) -> tuple[Optional[Set[str]], str]:
        output, detail = self._fetch_world_pose_info()
        if output is None:
            return None, detail
        pose_map = _parse_model_poses_from_pose_info(output)
        with self._state_lock:
            for base, pose in pose_map.items():
                if base.startswith(self._prefix):
                    self._world_pose_cache[base] = pose
                    self._world_xy_cache[base] = (pose.x, pose.y)
        bases = {base for base in pose_map if base.startswith(self._prefix)}
        return bases, detail

    def _reconcile_world_entities(self, current_ids: Set[int], force: bool = False) -> None:
        now_mono = time.monotonic()
        if not force and now_mono < self._next_reconcile_mono:
            return
        bases, detail = self._list_world_model_bases()
        self._next_reconcile_mono = now_mono + self._reconcile_interval
        if bases is None:
            self.get_logger().debug(
                "世界实体对账跳过（pose/info 暂不可用）：%s"
                % ((detail or "no detail")[:120]),
            )
            return

        world_ids = {
            tid for tid in (self._track_id_from_model_base(base) for base in bases) if tid is not None
        }
        with self._state_lock:
            pending_spawn = set(self._spawn_futures.keys())
            active_ids = set(self._drivers.keys()) | self._spawned_ids | pending_spawn
            desired_ids = (
                (current_ids - set(self._collision_suppressed)) | active_ids
            )
            self._spawned_ids.update(world_ids & desired_ids)
            for tid in list(self._spawned_ids):
                if (
                    tid not in world_ids
                    and tid not in pending_spawn
                    and tid not in self._drivers
                ):
                    self._spawned_ids.discard(tid)
            stale_ids = sorted(world_ids - desired_ids)

        if force and self._cleanup_stale_on_start:
            self._startup_cleanup_done = True
        elif force:
            return

        for tid in stale_ids:
            name = self._track_entity_name(tid)
            self._teardown_track_driver(tid)
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

        inertial = _inertial_xml(self._model_mass_kg)
        plugin = _velocity_control_plugin_xml(model_name)
        return f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="{model_name}">
    <static>false</static>
    <link name="link">
      <gravity>false</gravity>
{inertial}
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
{plugin}
  </model>
</sdf>
"""

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
            try:
                bridge = self._start_cmd_vel_bridge(name)
            except Exception as ex:  # pragma: no cover
                self.destroy_publisher(ros_pub)
                self.get_logger().error("cmd_vel bridge 启动失败 %s: %s" % (name, ex))
                return
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
        via = "gz-transport" if gz_pub is not None else "ros_gz_bridge"
        self.get_logger().info("cmd_vel driver ready: %s (topic=%s via %s)" % (name, topic, via))

    def _publish_zero_cmd_vel(self, driver: TrackDriver) -> None:
        if driver.gz_pub is not None and GzTwist is not None:
            driver.gz_pub.publish(GzTwist())
        elif driver.cmd_vel_pub is not None:
            driver.cmd_vel_pub.publish(Twist())

    def _teardown_track_driver(self, tid: int) -> None:
        with self._state_lock:
            driver = self._drivers.pop(tid, None)
        if driver is None:
            return
        try:
            self._publish_zero_cmd_vel(driver)
        except Exception:  # pragma: no cover
            pass
        if driver.bridge_proc is not None:
            driver.bridge_proc.terminate()
            try:
                driver.bridge_proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                driver.bridge_proc.kill()
        if driver.cmd_vel_pub is not None:
            try:
                self.destroy_publisher(driver.cmd_vel_pub)
            except Exception:  # pragma: no cover
                pass

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
        self._teardown_track_driver(tid)
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
        if ok and fail_count > 0:
            self.get_logger().info("碰撞删除 %s 在 %d 次失败后成功" % (name, fail_count))
        elif not ok and (fail_count == 1 or fail_count % 5 == 0):
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
        to_finish: list[tuple[int, Future, float]] = []
        with self._state_lock:
            for tid, fut in list(self._spawn_futures.items()):
                if fut.done():
                    to_finish.append((tid, fut, 0.0))
            for tid, _, _ in to_finish:
                del self._spawn_futures[tid]
        for tid, fut, _ in to_finish:
            name = self._track_entity_name(tid)
            try:
                ok, err = fut.result()
            except Exception as ex:  # pragma: no cover
                self.get_logger().error("create CLI exception for %s: %s" % (name, ex))
                continue
            if ok:
                with self._state_lock:
                    self._spawned_ids.add(tid)
                initial_yaw = 0.0
                truth_x = truth_y = None
                if self._latest:
                    for tr in self._latest.tracks:
                        if int(tr.track_id) == tid:
                            initial_yaw = _yaw_from_track(tr)
                            truth_x = float(tr.x)
                            truth_y = float(tr.y)
                            break
                self._setup_track_driver(tid, initial_yaw=initial_yaw)
                if truth_x is not None and truth_y is not None:
                    with self._state_lock:
                        drv = self._drivers.get(tid)
                    if drv is not None:
                        drv.last_truth_x = truth_x
                        drv.last_truth_y = truth_y
                    if self._spawn_post_set_pose:
                        z = self._pose_z_for_track(
                            next(tr for tr in self._latest.tracks if int(tr.track_id) == tid)
                        )
                        pose_ok, pose_detail = self._set_pose_gz(
                            name, truth_x, truth_y, z, initial_yaw, force=True
                        )
                        if not pose_ok:
                            self.get_logger().warn(
                                "spawn 后 set_pose 对齐 %s 失败：%s"
                                % (name, (pose_detail or "no detail")[:200]),
                                throttle_duration_sec=5.0,
                            )
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
            with self._gz_cli_lock:
                result = subprocess.run(
                    cmd,
                    capture_output=True,
                    text=True,
                    timeout=self._create_timeout,
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

    def _cancel_pending_spawn(self, tid: int) -> None:
        with self._state_lock:
            fut = self._spawn_futures.pop(tid, None)
        if fut is not None and not fut.done():
            fut.cancel()

    def _respawn_track(self, t: GlobalTrack, reason: str) -> None:
        tid = int(t.track_id)
        name = self._track_entity_name(tid)
        self._cancel_pending_spawn(tid)
        self._teardown_track_driver(tid)
        ok, _ = self._remove_entity_gz(name)
        with self._state_lock:
            self._spawned_ids.discard(tid)
            self._pending_remove_retry_mono.pop(tid, None)
            self._remove_fail_counts.pop(tid, None)
            self._world_pose_cache.pop(name, None)
            self._world_xy_cache.pop(name, None)
        if ok:
            self.get_logger().info(
                "%s：删除并重新 spawn %s → (%.1f, %.1f)"
                % (reason, name, float(t.x), float(t.y))
            )
        else:
            self.get_logger().warn(
                "%s：删除 %s 失败，仍尝试重新 spawn" % (reason, name),
                throttle_duration_sec=5.0,
            )
        self._submit_spawn(t, force=True)

    def _respawn_track_for_fence_jump(self, t: GlobalTrack) -> None:
        self._respawn_track(t, "围栏 ID 复用/真值跳变")

    def _maybe_respawn_for_attitude(
        self, t: GlobalTrack, driver: TrackDriver, now_mono: float
    ) -> Optional[str]:
        """在已持有 ``_state_lock`` 的上下文中调用，仅检测并返回原因，不执行 respawn。"""
        if not self._attitude_monitor:
            return None
        if self._attitude_max_rp_deg <= 0.0 and self._attitude_max_z_err <= 0.0:
            return None
        if now_mono - driver.last_respawn_mono < self._attitude_respawn_cooldown:
            return None
        pose = self._world_pose_cache.get(driver.model_name)
        if pose is None:
            return None
        return attitude_violation_reason(
            pose,
            self._pose_z_for_track(t),
            self._attitude_max_rp_deg,
            self._attitude_max_z_err,
        )

    def _truth_jump_exceeds_threshold(self, t: GlobalTrack, driver: TrackDriver) -> bool:
        x = float(t.x)
        y = float(t.y)
        if driver.last_truth_x is None or driver.last_truth_y is None:
            driver.last_truth_x = x
            driver.last_truth_y = y
            return False
        jump = math.hypot(x - driver.last_truth_x, y - driver.last_truth_y)
        speed = math.hypot(float(t.v_x), float(t.v_y))
        expected = max(speed * self._dt * 4.0, 1.0)
        driver.last_truth_x = x
        driver.last_truth_y = y
        if jump > max(self._fence_jump_threshold, expected):
            return True
        return False

    def _submit_spawn(self, t: GlobalTrack, force: bool = False) -> None:
        tid = int(t.track_id)
        if not force:
            with self._state_lock:
                if tid in self._spawn_futures or tid in self._spawned_ids:
                    return
        else:
            self._cancel_pending_spawn(tid)
        name = self._track_entity_name(tid)
        rgba = self._color_rgba(t)
        bm = self._contact_bitmask
        mass = self._model_mass_kg
        if self._geom_mode == "mesh_profile":
            sdf = self._build_mesh_profile_sdf(name)
            z = self._mesh_profile_spawn_z
        elif self._geom_mode == "box":
            sx, sy, sz = self._box_dims(t)
            sdf = _build_box_sdf(name, sx, sy, sz, rgba, bm, mass)
            z = self._z_center_for_height(sz)
        else:
            r, h, z = self._cylinder_dims(t)
            sdf = _build_cylinder_sdf(name, r, h, rgba, bm, mass)
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

    def _set_pose_gz(
        self,
        model_name: str,
        x: float,
        y: float,
        z: float,
        yaw: float,
        *,
        force: bool = False,
    ) -> tuple[bool, str]:
        if not force and not self._resync_use_set_pose:
            return False, "position_resync_use_set_pose=false"
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
            return ok, output or ("returncode=%s" % result.returncode)
        except Exception as ex:  # pragma: no cover
            return False, str(ex)

    def _body_yaw_for_driver(self, driver: TrackDriver) -> float:
        with self._state_lock:
            pose = self._world_pose_cache.get(driver.model_name)
        if pose is not None:
            return _quat_to_yaw(pose.qx, pose.qy, pose.qz, pose.qw)
        return driver.last_yaw

    def _maybe_sync_heading(
        self,
        t: GlobalTrack,
        driver: TrackDriver,
        truth_yaw: float,
        now_mono: float,
    ) -> None:
        """直线巡航时仅同步航向，避免 Gazebo 物理层持续小角速度空转。"""
        if not self._heading_sync:
            return
        if now_mono - driver.last_heading_sync_mono < self._heading_sync_interval:
            return
        body_yaw = self._body_yaw_for_driver(driver)
        if abs(normalize_angle(truth_yaw - body_yaw)) > math.radians(
            self._align_threshold_deg
        ):
            return
        with self._state_lock:
            pose = self._world_pose_cache.get(driver.model_name)
        if pose is None:
            return
        z = self._pose_z_for_track(t)
        ok, _ = self._set_pose_gz(
            driver.model_name, pose.x, pose.y, z, truth_yaw, force=True
        )
        if ok:
            driver.last_heading_sync_mono = now_mono

    def _publish_cmd_vel_for_track(self, t: GlobalTrack, driver: TrackDriver) -> None:
        vx = float(t.v_x)
        vy = float(t.v_y)
        speed = math.hypot(vx, vy)
        truth_yaw = (
            math.atan2(vy, vx) if speed > 1e-3 else self._body_yaw_for_driver(driver)
        )
        body_yaw = self._body_yaw_for_driver(driver)
        twist = compute_arc_follow_cmd_vel(
            vx,
            vy,
            body_yaw,
            omega_limit=self._omega_limit,
            turn_radius_min_m=self._turn_radius_min,
            align_threshold_deg=self._align_threshold_deg,
        )
        now_mono = time.monotonic()
        if abs(twist.angular.z) < 1e-6 and speed > 1e-3:
            self._maybe_sync_heading(t, driver, truth_yaw, now_mono)
            body_yaw = self._body_yaw_for_driver(driver)
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

    def _maybe_resync_track(
        self, t: GlobalTrack, driver: TrackDriver, now_mono: float
    ) -> None:
        if not self._resync_enabled:
            return
        if now_mono - driver.last_resync_mono < self._resync_min_interval:
            return
        with self._state_lock:
            gz_xy = self._world_xy_cache.get(driver.model_name)
        if gz_xy is None:
            return
        err = math.hypot(float(t.x) - gz_xy[0], float(t.y) - gz_xy[1])
        if err < self._resync_threshold:
            return
        z = self._pose_z_for_track(t)
        # 仅同步 XY；转弯过程中保留 Gazebo 实际航向，避免 set_pose 抹掉掉头动作
        with self._state_lock:
            pose = self._world_pose_cache.get(driver.model_name)
        desired_yaw = _yaw_from_track(t)
        if pose is not None:
            body_yaw = _quat_to_yaw(pose.qx, pose.qy, pose.qz, pose.qw)
            align_rad = math.radians(self._align_threshold_deg)
            if abs(normalize_angle(desired_yaw - body_yaw)) > align_rad:
                yaw = body_yaw
            else:
                yaw = desired_yaw
        else:
            yaw = desired_yaw
        ok, detail = self._set_pose_gz(driver.model_name, float(t.x), float(t.y), z, yaw)
        driver.last_resync_mono = now_mono
        if ok:
            self.get_logger().info(
                "drift resync %s: error=%.1fm → truth (%.1f, %.1f)"
                % (driver.model_name, err, float(t.x), float(t.y))
            )
        else:
            self.get_logger().warn(
                "drift resync failed for %s (error=%.1fm): %s"
                % (driver.model_name, err, (detail or "no detail")[:200]),
                throttle_duration_sec=10.0,
            )

    def _apply_cmd_vel_from_tracks(self, tracks: list[GlobalTrack]) -> None:
        now_mono = time.monotonic()
        if now_mono - self._last_pose_refresh_mono >= self._pose_refresh_interval:
            self._refresh_world_pose_cache()
            self._last_pose_refresh_mono = now_mono
        for t in tracks:
            tid = int(t.track_id)
            with self._state_lock:
                driver = self._drivers.get(tid)
            if driver is None:
                continue
            self._maybe_resync_track(t, driver, now_mono)
            self._publish_cmd_vel_for_track(t, driver)

    def _try_publish_motion_ready(self, expected_ids: Set[int], now_ns: int) -> None:
        if not self._spawn_ready_sync or self._motion_sync_started:
            return
        if not expected_ids:
            return
        with self._state_lock:
            pending = pending_spawn_track_ids(self._spawn_futures)
            ready = all_tracks_spawn_ready(
                expected_ids, self._spawned_ids, self._drivers, pending
            )
        if ready:
            self._start_motion_sync(now_ns, forced=False, track_count=len(expected_ids))
            return
        if self._spawn_ready_timeout_sec <= 0.0:
            return
        elapsed = (now_ns - self._spawn_gate_open_ns) * 1e-9
        if elapsed < self._spawn_ready_timeout_sec:
            return
        self.get_logger().warn(
            "spawn_ready 超时 (%.1fs)：%d/%d track 已就绪，强制 motion_start"
            % (
                self._spawn_ready_timeout_sec,
                len(self._spawned_ids & expected_ids),
                len(expected_ids),
            )
        )
        self._start_motion_sync(now_ns, forced=True, track_count=len(expected_ids))

    def _start_motion_sync(
        self, now_ns: int, *, forced: bool, track_count: int
    ) -> None:
        if self._motion_sync_started:
            return
        self._motion_sync_started = True
        if self._motion_ready_pub is not None and not self._motion_ready_published:
            ready_msg = Bool()
            ready_msg.data = True
            self._motion_ready_pub.publish(ready_msg)
            self._motion_ready_published = True
        suffix = "（超时强制）" if forced else ""
        self.get_logger().info(
            "motion_start sim_time=%.3fs：%d track spawn/driver 就绪%s，已发布 %s"
            % (
                now_ns * 1e-9,
                track_count,
                suffix,
                self._motion_ready_topic,
            )
        )

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
                    "Tracks frame_id is '%s' (expected 'map'); cmd_vel still derived from track velocities."
                    % msg.header.frame_id
                )
                self._warned_frame = True

        now_ns = self.get_clock().now().nanoseconds
        if now_ns < self._spawn_gate_open_ns:
            if not self._logged_delay:
                self.get_logger().info(
                    "spawn_delay_sec=%.1f：%.1fs 后再 spawn/cmd_vel（当前为启动后延迟）"
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

        now_mono = time.monotonic()
        if self._attitude_monitor and now_mono >= self._next_attitude_check_mono:
            self._next_attitude_check_mono = now_mono + self._attitude_monitor_interval
            self._refresh_world_pose_cache()

        remove_names: list[str] = []
        to_spawn: Dict[int, GlobalTrack] = {}
        respawn_tracks: list[GlobalTrack] = []
        attitude_respawn: list[tuple[GlobalTrack, str]] = []
        cmd_tracks: list[GlobalTrack] = []

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
                driver = self._drivers.get(tid)
                if (
                    driver is not None
                    and self._truth_jump_respawn
                    and self._truth_jump_exceeds_threshold(t, driver)
                ):
                    if now_mono - driver.last_respawn_mono >= self._attitude_respawn_cooldown:
                        driver.last_respawn_mono = now_mono
                        respawn_tracks.append(t)
                    continue
                if driver is not None and tid in self._spawned_ids:
                    reason = self._maybe_respawn_for_attitude(t, driver, now_mono)
                    if reason is not None:
                        driver.last_respawn_mono = now_mono
                        attitude_respawn.append((t, reason))
                        continue
                if (
                    tid not in self._spawned_ids
                    and tid not in self._spawn_futures
                    and tid not in self._drivers
                ):
                    to_spawn[tid] = t
                if (
                    tid in self._spawned_ids
                    and tid in self._drivers
                    and (not self._spawn_ready_sync or self._motion_sync_started)
                ):
                    cmd_tracks.append(t)

        for nm in remove_names:
            tid = self._track_id_from_model_base(nm)
            if tid is not None:
                self._teardown_track_driver(tid)
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
            if ok and fail_count > 0:
                self.get_logger().info("Gazebo 删除 %s 在 %d 次失败后成功" % (nm, fail_count))
            elif not ok and (fail_count == 1 or fail_count % 5 == 0):
                self.get_logger().warn(
                    "Gazebo 删除 %s 失败，将在 %.1fs 后重试：%s"
                    % (nm, self._remove_retry_interval, detail or "no detail")
                )

        for t in respawn_tracks:
            self._respawn_track_for_fence_jump(t)

        for t, reason in attitude_respawn:
            self._respawn_track(t, "姿态偏离水平面 (%s)" % reason)

        for t in to_spawn.values():
            self._submit_spawn(t)

        self._try_publish_motion_ready(current_ids, now_ns)
        self._apply_cmd_vel_from_tracks(cmd_tracks)


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
