# Copyright 2026 MurphyChen
"""full_config 多船与 session_manager JSON 解析 — 供各 launch 文件共用。"""
import json
import os
import re
from typing import Optional

import yaml

from ament_index_python.packages import get_package_prefix, get_package_share_directory

ROBOT_SLOT_KEY_RE = re.compile(r'^robot_(\d+)$')


def launch_verbose_enabled(verbose_str: str) -> bool:
    """解析各 launch 中 ``verbose_launch`` 等参数（true/1/yes）。"""
    return str(verbose_str).strip().lower() in ('true', '1', 'yes')


def quiet_ros_node_kwargs(verbose_str: str, base_arguments=None):
    """默认降噪：终端少刷屏，完整日志在 ``~/.ros/log``。``verbose_launch`` 为真时恢复 screen。"""
    args = list(base_arguments or [])
    if launch_verbose_enabled(verbose_str):
        return {'output': 'screen', 'arguments': args}
    return {
        'output': 'log',
        'arguments': args + ['--ros-args', '--log-level', 'warn'],
    }


def session_manager_executable_path():
    """返回已安装的 ``session_manager`` 可执行文件路径。

    勿使用 ``ros2 run usv_sim_full session_manager`` 启动会话生成：该方式会收窄
    子进程 ``AMENT_PREFIX_PATH``，其内再调 xacro 时无法 ``$(find wamv_gazebo)`` 等
    工作区包。直接执行本路径并继承 ``ros2 launch`` 的环境即可。
    """
    prefix = get_package_prefix('usv_sim_full')
    exe = os.path.join(prefix, 'lib', 'usv_sim_full', 'session_manager')
    if not os.path.isfile(exe):
        raise FileNotFoundError(
            f'session_manager 未找到: {exe}；请先 colcon build usv_sim_full 并 source install/setup.bash'
        )
    return exe


def default_radar_nav2_param_yaml(launch_py_dir: str) -> str:
    """默认 ``radar_nav2_param.yaml`` 路径。

    优先 ``usv_sim_full/config``（与 bringup 同源，支持 ``__ROBOT_NS__`` 占位符）；
    再回退 gy_radar 源码树或已安装包。
    """
    launch_py_dir = os.path.abspath(launch_py_dir)
    share_usv = get_package_share_directory('usv_sim_full')
    candidates = [
        os.path.normpath(os.path.join(launch_py_dir, '..', 'config', 'radar_nav2_param.yaml')),
        os.path.join(share_usv, 'config', 'radar_nav2_param.yaml'),
        os.path.normpath(
            os.path.join(
                launch_py_dir,
                '..',
                '..',
                'sensor_plugins',
                'gy_radar_driver-main',
                'config',
                'radar_nav2_param.yaml',
            )
        ),
    ]
    for p in candidates:
        if os.path.isfile(p):
            return p
    try:
        gy = os.path.join(
            get_package_share_directory('gy_radar_driver'), 'config', 'radar_nav2_param.yaml'
        )
        if os.path.isfile(gy):
            return gy
    except Exception:
        pass
    return os.path.join(share_usv, 'config', 'radar_nav2_param.yaml')


def sorted_robot_slot_keys(cfg):
    keys = [k for k in cfg if ROBOT_SLOT_KEY_RE.match(k)]
    return sorted(keys, key=lambda k: int(ROBOT_SLOT_KEY_RE.match(k).group(1)))


def iter_all_block_sensors(cfg):
    """按 robot_1, robot_2… 展开各船 sensors；无 robot_N 时用顶层 sensors 与 robot.sensors。"""
    keys = sorted_robot_slot_keys(cfg)
    if keys:
        for k in keys:
            for s in cfg[k].get('sensors') or []:
                yield s
        return
    for s in cfg.get('sensors') or []:
        yield s
    for s in cfg.get('robot', {}).get('sensors') or []:
        yield s


def ship_config_blocks(cfg):
    keys = sorted_robot_slot_keys(cfg)
    if keys:
        return [cfg[k] for k in keys]
    rb = cfg.get('robot')
    return [rb] if isinstance(rb, dict) and rb else []


def block_has_maritime_radar(block):
    if not isinstance(block, dict):
        return False
    for s in block.get('sensors') or []:
        if not s.get('enabled', True):
            continue
        st = str(s.get('type', '')).lower()
        if st in ('maritime_radar', 'radar'):
            return True
    return False


def block_first_maritime_radar(block):
    if not isinstance(block, dict):
        return 'radar', '/sensors/radar/nav/sector'
    for s in block.get('sensors') or []:
        if not s.get('enabled', True):
            continue
        st = str(s.get('type', '')).lower()
        if st in ('maritime_radar', 'radar'):
            return str(s.get('name', 'radar')), str(
                s.get('override_topic') or '/sensors/radar/nav/sector'
            )
    return 'radar', '/sensors/radar/nav/sector'


def primary_robot_name(user_config):
    """Nav2 / RViz 等默认跟随的第一艘船名称。"""
    keys = sorted_robot_slot_keys(user_config)
    if keys:
        return str(user_config[keys[0]].get('name', 'usv_1')).strip() or 'usv_1'
    rb = user_config.get('robot') or {}
    return str(rb.get('name', 'usv_1')).strip() or 'usv_1'


def parse_session_json_from_stdout(stdout: str) -> dict:
    text = stdout.strip()
    start = text.find('{')
    end = text.rfind('}')
    if start < 0 or end <= start:
        raise ValueError(f'session_manager 输出中未找到 JSON: {text!r}')
    return json.loads(text[start : end + 1])


def _resolve_sensor_config_path(full_config_path: str, user_config: dict) -> str:
    """解析 full_config 引用的 sensor_config.yaml 绝对路径。"""
    cfg_dir = os.path.dirname(os.path.abspath(full_config_path))
    rel = user_config.get(
        'sensor_config_path', 'config/three_vision_one_mmwave/sensor_config.yaml'
    )
    if os.path.isabs(rel):
        candidate = rel
    else:
        candidate = os.path.join(cfg_dir, rel)
        if not os.path.isfile(candidate):
            try:
                share = get_package_share_directory('usv_sim_full')
                candidate = os.path.join(share, rel)
            except Exception:
                pass
    return candidate if os.path.isfile(candidate) else ''


def load_mmwave_sensor_defaults(full_config_path: str, user_config: dict) -> dict:
    """解析 sensor_config.yaml 中 mmwave.default，供 mmwave_4d_cloud_node 参数使用。"""
    candidate = _resolve_sensor_config_path(full_config_path, user_config)
    if not candidate:
        return {}
    with open(candidate, 'r') as f:
        data = yaml.safe_load(f) or {}
    mm = data.get('mmwave') or {}
    return dict(mm.get('default') or {})


def load_mmwave_cluster_defaults(full_config_path: str, user_config: dict) -> dict:
    """解析 sensor_config.yaml 中 mmwave.cluster；缺省回退 usv_mmwave_sim 包内 mmwave_cluster.yaml。"""
    candidate = _resolve_sensor_config_path(full_config_path, user_config)
    if candidate:
        with open(candidate, 'r') as f:
            data = yaml.safe_load(f) or {}
        cluster = (data.get('mmwave') or {}).get('cluster')
        if isinstance(cluster, dict) and cluster:
            return dict(cluster)

    try:
        share = get_package_share_directory('usv_mmwave_sim')
        fallback = os.path.join(share, 'config', 'mmwave_cluster.yaml')
        if os.path.isfile(fallback):
            with open(fallback, 'r') as f:
                data = yaml.safe_load(f) or {}
            params = data.get('/**', {}).get('ros__parameters') or data.get('ros__parameters') or {}
            if isinstance(params, dict):
                return dict(params)
    except Exception:
        pass
    return {}


def mmwave_bridge_topics(sensor: dict, sanitized_ns: str):
    """与 session_manager.generate_bridge_config 中毫米波规则一致：桥接 .../points_gz，最终 .../points。"""
    st = str(sensor.get('type', '')).lower()
    if st not in ('mmwave_radar', 'mmwave'):
        return None
    sensor_name = sensor.get('name', 'mmwave')
    ros_topic = sensor.get('override_topic') or f'/sensors/mmwave/{sensor_name}/points'
    if not ros_topic.startswith('/'):
        ros_topic = '/' + ros_topic
    prefix = f'/{sanitized_ns}'
    if ros_topic.endswith('/points'):
        ros_topic_bridge = ros_topic[: -len('/points')] + '/points_gz'
    else:
        ros_topic_bridge = ros_topic + '_gz'
    input_topic = (
        ros_topic_bridge
        if ros_topic_bridge.startswith(prefix)
        else f'{prefix}{ros_topic_bridge}'
    )
    output_topic = ros_topic if ros_topic.startswith(prefix) else f'{prefix}{ros_topic}'
    return input_topic, output_topic


def mmwave_cluster_topics(sensor: dict, sanitized_ns: str):
    """聚类节点话题：订阅 4D …/points，发布 …/objects（与 mmwave_bridge_topics 命名规则一致）。"""
    pair = mmwave_bridge_topics(sensor, sanitized_ns)
    if pair is None:
        return None
    _, points_topic = pair
    prefix = f'/{sanitized_ns}'
    if points_topic.endswith('/points'):
        objects_topic = points_topic[: -len('/points')] + '/objects'
    else:
        objects_topic = points_topic.rstrip('/') + '/objects'
    if not objects_topic.startswith(prefix):
        objects_topic = f'{prefix}{objects_topic}'
    return points_topic, objects_topic


def _yaml_bool(value, default=False):
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ('true', '1', 'yes')
    return bool(value)


def block_enable_env_dynamics(block, default=True):
    """每船是否启动 usv_env_dynamics（风浪流外力）；缺省 true 与旧版 main 单船始终开启一致。"""
    if not isinstance(block, dict):
        return default
    return _yaml_bool(block.get('enable_env_dynamics'), default)


def block_env_dynamics_k_gains(block):
    """env_dynamics.k_wind / k_current，与 main.launch 默认一致。"""
    if not isinstance(block, dict):
        return 1.5, 250.0
    ed = block.get('env_dynamics')
    if not isinstance(ed, dict):
        return 1.5, 250.0
    return (
        float(ed.get('k_wind', 1.5)),
        float(ed.get('k_current', 250.0)),
    )


def build_mmwave_4d_cloud_parameters(mm_defs: dict, input_topic: str, output_topic: str, odom_topic: str):
    """mmwave_4d_cloud_node 参数字典（与 sensor_config mmwave.default 对齐）。"""
    md = mm_defs or {}
    return {
        'use_sim_time': True,
        'input_topic': input_topic,
        'output_topic': output_topic,
        'odom_topic': odom_topic,
        'world_frame': 'map',
        'base_rcs': float(md.get('base_rcs', 12.0)),
        'rcs_distance_decay': float(md.get('rcs_distance_decay', 0.01)),
        'perception_range_limit_m': float(md.get('perception_range_limit_m', 300.0)),
        'enable_sea_clutter': _yaml_bool(md.get('enable_sea_clutter'), False),
        'sea_clutter_probability': float(md.get('sea_clutter_probability', 0.0)),
        'sea_clutter_amplitude': float(md.get('sea_clutter_amplitude', 0.0)),
        'enable_range_measurement_error': _yaml_bool(
            md.get('enable_range_measurement_error'), False
        ),
        'enable_azimuth_measurement_error': _yaml_bool(
            md.get('enable_azimuth_measurement_error'), False
        ),
        'range_error_at_reference_m': float(md.get('range_error_at_reference_m', 0.66)),
        'range_error_reference_m': float(md.get('range_error_reference_m', 300.0)),
        'azimuth_error_std_deg': float(md.get('azimuth_error_std_deg', 0.5)),
        'output_use_reliable_qos': _yaml_bool(md.get('output_use_reliable_qos'), True),
    }


def build_mmwave_cluster_parameters(
    cluster_defs: dict, input_topic: str, output_topic: str, radar_id: str
):
    """mmwave_cluster_node 参数字典（与 sensor_config mmwave.cluster 对齐）。"""
    cd = cluster_defs or {}
    return {
        'use_sim_time': True,
        'input_topic': input_topic,
        'output_topic': output_topic,
        'radar_id': radar_id,
        'cluster_mode': str(cd.get('cluster_mode', 'xyz')),
        'cluster_eps_m': float(cd.get('cluster_eps_m', 4.0)),
        'cluster_min_points': int(cd.get('cluster_min_points', 3)),
        'min_rcs': float(cd.get('min_rcs', 2.0)),
        'max_range_m': float(cd.get('max_range_m', 300.0)),
        'enable_doppler_prefilter': _yaml_bool(cd.get('enable_doppler_prefilter'), False),
        'doppler_prefilter_threshold': float(cd.get('doppler_prefilter_threshold', 0.05)),
        'min_cluster_rcs_mean': float(cd.get('min_cluster_rcs_mean', 0.0)),
        'max_clusters': int(cd.get('max_clusters', 0)),
        'motion_speed_threshold': float(cd.get('motion_speed_threshold', 0.3)),
        'min_size_h_m': float(cd.get('min_size_h_m', 0.5)),
        'output_use_reliable_qos': _yaml_bool(cd.get('output_use_reliable_qos'), True),
    }


# full_config `ground_truth_sim:` 中仅 launch 层使用的键（不写入任何 ROS 节点参数）
GROUND_TRUTH_SIM_LAUNCH_ONLY_KEYS = frozenset(
    {
        'enabled',
        'params_file',
        'gazebo_visual',
        'gazebo_model_prefix',
    }
)

# kinematic ground_truth_node 不接收的键（Gazebo 实体 / 旧 models 节点专用）
GROUND_TRUTH_SIM_NODE_EXCLUDE_KEYS = frozenset(
    {
        *GROUND_TRUTH_SIM_LAUNCH_ONLY_KEYS,
        'gazebo_mesh_profile',
        'world_service_wait_sec',
        'collision_topic',
        'collision_string_topic',
        'collision_debounce_sec',
        'cylinder_radius_cap_m',
        'cylinder_height_cap_m',
        'gazebo_target_geometry',
        'contact_collide_bitmask',
        'create_cli_timeout_sec',
        'spawn_thread_pool_size',
        'update_dt',
        'model_mass_kg',
        'cmd_vel_omega_limit',
        'cmd_vel_turn_radius_min_m',
        'cmd_vel_align_threshold_deg',
        'cmd_vel_pose_refresh_interval_sec',
        'position_resync_enabled',
        'position_resync_threshold_m',
        'position_resync_min_interval_sec',
        'position_resync_use_set_pose',
        'fence_jump_respawn_threshold_m',
        'truth_jump_respawn_enabled',
        'attitude_monitor_enabled',
        'attitude_max_roll_pitch_deg',
        'attitude_max_z_error_m',
        'attitude_monitor_interval_sec',
        'attitude_respawn_cooldown_sec',
        'spawn_post_set_pose_enabled',
        'heading_sync_enabled',
        'heading_sync_interval_sec',
        'spawn_ready_sync_enabled',
        'motion_ready_topic',
        'spawn_ready_timeout_sec',
        'wait_for_spawn_ready',
        'truth_velocity_lowpass_tau',
    }
)

# 兼容旧名：kinematic 节点参数 YAML 仍用此集合过滤
GROUND_TRUTH_SIM_META_KEYS = GROUND_TRUTH_SIM_NODE_EXCLUDE_KEYS


def _resolve_gazebo_mesh_profile(v, full_config_path: str):
    vp = str(v).strip()
    if vp and full_config_path and not os.path.isabs(vp):
        base = os.path.dirname(os.path.abspath(full_config_path))
        return os.path.normpath(os.path.join(base, vp))
    return v


def merge_ground_truth_gazebo_entity_params(
    world_name: str,
    scen_gt_cfg: dict,
    tracks_topic: str,
    model_prefix: str,
    gz_spawn_delay: float,
    gz_svc_wait: float,
    full_config_path: str = "",
) -> dict:
    """组装 scenario_ground_truth_gazebo_entity 参数字典（Gazebo 实体权威真值）。"""
    p = dict(DEFAULT_GROUND_TRUTH_ENTITY_PARAMS)
    for k, v in scen_gt_cfg.items():
        if k in GROUND_TRUTH_SIM_LAUNCH_ONLY_KEYS:
            continue
        if v is None or v == '':
            continue
        if k == 'gazebo_mesh_profile':
            p['gazebo_mesh_profile'] = _resolve_gazebo_mesh_profile(v, full_config_path)
            continue
        p[k] = v
    p['world_name'] = world_name
    p['tracks_topic'] = tracks_topic.lstrip('/')
    p['model_name_prefix'] = str(model_prefix).strip() or p.get('model_name_prefix', 'gt_ctrv_')
    p['spawn_delay_sec'] = gz_spawn_delay
    p['world_service_wait_sec'] = gz_svc_wait
    fixed = scen_gt_cfg.get('fixed_targets')
    if isinstance(fixed, list) and fixed and not str(p.get('fixed_targets_json') or '').strip():
        p['fixed_targets_json'] = json.dumps(fixed, ensure_ascii=False)
    # Launch 内联 dict 无法序列化嵌套 list；仅保留 fixed_targets_json。
    p.pop('fixed_targets', None)
    return p


def merge_ground_truth_gazebo_models_params(
    world_name: str,
    scen_gt_cfg: dict,
    tracks_topic: str,
    model_prefix: str,
    gz_spawn_delay: float,
    gz_svc_wait: float,
    full_config_path: str = "",
) -> dict:
    """组装 scenario_ground_truth_gazebo_models 的参数字典（含 scenario.ground_truth_sim 中的可选扩展）。"""
    p = {
        'world_name': world_name,
        'tracks_topic': tracks_topic,
        'model_name_prefix': model_prefix,
        'spawn_delay_sec': gz_spawn_delay,
        'world_service_wait_sec': gz_svc_wait,
    }
    for k in (
        'collision_topic',
        'collision_string_topic',
        'collision_debounce_sec',
        'cylinder_radius_cap_m',
        'cylinder_height_cap_m',
        'gazebo_target_geometry',
        'gazebo_mesh_profile',
        'contact_collide_bitmask',
        'create_cli_timeout_sec',
        'spawn_thread_pool_size',
        'update_dt',
        'model_mass_kg',
        'cmd_vel_omega_limit',
        'cmd_vel_turn_radius_min_m',
        'cmd_vel_align_threshold_deg',
        'cmd_vel_pose_refresh_interval_sec',
        'position_resync_enabled',
        'position_resync_threshold_m',
        'position_resync_min_interval_sec',
        'position_resync_use_set_pose',
        'fence_jump_respawn_threshold_m',
        'truth_jump_respawn_enabled',
        'attitude_monitor_enabled',
        'attitude_max_roll_pitch_deg',
        'attitude_max_z_error_m',
        'attitude_monitor_interval_sec',
        'attitude_respawn_cooldown_sec',
        'spawn_post_set_pose_enabled',
        'heading_sync_enabled',
        'heading_sync_interval_sec',
        'spawn_ready_sync_enabled',
        'motion_ready_topic',
        'spawn_ready_timeout_sec',
    ):
        if k not in scen_gt_cfg:
            continue
        v = scen_gt_cfg[k]
        if v is None or v == '':
            continue
        if k == 'gazebo_mesh_profile' and full_config_path:
            v = _resolve_gazebo_mesh_profile(v, full_config_path)
        p[k] = v
    return p

# Gazebo 实体权威真值节点默认参数
DEFAULT_GROUND_TRUTH_ENTITY_PARAMS = {
    'update_dt': 0.02,
    'frame_id': 'map',
    'tracks_topic': 'sim/ground_truth',
    'markers_topic': 'sim/ground_truth_markers',
    'model_name_prefix': 'gt_ctrv_',
    'spawn_delay_sec': 10.0,
    'world_service_wait_sec': 1.0,
    'create_cli_timeout_sec': 20.0,
    'spawn_thread_pool_size': 4,
    'fixed_targets_json': '',
    'motion_mode': 'waypoint',
    'waypoint_kinematics': 'arc',
    'waypoint_arrival_threshold_m': 0.5,
    'waypoint_omega_limit': 0.22,
    'waypoint_turn_radius_min_m': 10.0,
    'waypoint_align_threshold_deg': 12.0,
    'cmd_vel_omega_limit': 0.22,
    'cmd_vel_turn_radius_min_m': 10.0,
    'cmd_vel_align_threshold_deg': 12.0,
    'cmd_vel_pose_refresh_interval_sec': 0.05,
    'heading_sync_enabled': True,
    'heading_sync_interval_sec': 1.0,
    'truth_velocity_lowpass_tau': 0.2,
    'history_max_points': 500,
    'prediction_horizon': 5.0,
    'prediction_dt': 0.25,
    'model_mass_kg': 50.0,
    'gazebo_target_geometry': 'box',
    'attitude_monitor_enabled': True,
    'attitude_max_roll_pitch_deg': 10.0,
    'attitude_max_z_error_m': 2.0,
    'attitude_monitor_interval_sec': 0.1,
    'attitude_respawn_cooldown_sec': 2.0,
    'cleanup_stale_models_on_start': True,
    'reconcile_interval_sec': 2.0,
    'collision_debounce_sec': 0.5,
    'remove_retry_interval_sec': 1.0,
}

# 与 ground_truth_sim/config/ground_truth_params.yaml 对齐；scenario 集成时由 full_config 覆盖 frame_id/reference_*
DEFAULT_GROUND_TRUTH_NODE_PARAMS = {
    'update_dt': 0.02,
    'frame_id': 'map',
    'reference_robot': '',
    'reference_frame': 'map',
    'reference_child_frame': 'base_link',
    'reference_tf_timeout_sec': 25.0,
    'target_count': 5,
    'annulus_radius_min': 50.0,
    'annulus_radius_max': 500.0,
    'speed_min': 2.0,
    'speed_max': 12.0,
    'size_width_min': 2.0,
    'size_width_max': 10.0,
    'size_length_min': 5.0,
    'size_length_max': 50.0,
    'size_height_min': 2.0,
    'size_height_max': 15.0,
    'ais_match_probability': 0.4,
    'omega_noise_std': 0.005,
    'omega_decay': 0.99,
    'omega_limit': 0.1,
    'prediction_horizon': 5.0,
    'prediction_dt': 0.25,
    'history_max_points': 500,
    'rng_seed': -1,
    'tracks_topic': 'sim/ground_truth',
    'markers_topic': 'sim/ground_truth_markers',
    'fence_enabled': False,
    'fence_min_x': -500.0,
    'fence_max_x': 500.0,
    'fence_min_y': -500.0,
    'fence_max_y': 500.0,
    'fence_maintain_target_count': True,
    'motion_mode': 'ctrv',
    'fixed_targets_json': '',
    'waypoint_arrival_threshold_m': 0.5,
    'spawn_delay_sec': 10.0,
    'waypoint_kinematics': 'arc',
    'waypoint_omega_limit': 0.22,
    'waypoint_turn_radius_min_m': 10.0,
    'waypoint_align_threshold_deg': 10.0,
}


def scenario_ground_truth_sim_config(user_config: dict) -> dict:
    """解析 `scenario.ground_truth_sim`；缺省 enabled=False。"""
    if not isinstance(user_config, dict):
        return {'enabled': False}
    scen = user_config.get('scenario')
    if not isinstance(scen, dict):
        return {'enabled': False}
    raw = scen.get('ground_truth_sim')
    if raw is None:
        return {'enabled': False}
    if isinstance(raw, bool):
        return {'enabled': bool(raw)}
    if not isinstance(raw, dict):
        return {'enabled': False}
    out = dict(raw)
    out['enabled'] = _yaml_bool(raw.get('enabled'), False)
    return out


def ground_truth_gazebo_visual_enabled(gt_cfg: dict) -> bool:
    """`scenario.ground_truth_sim.gazebo_visual` → 是否启动 Gazebo 柱状实体镜像节点。"""
    if not isinstance(gt_cfg, dict):
        return False
    return _yaml_bool(gt_cfg.get('gazebo_visual'), False)


def resolve_ground_truth_user_params_path(full_config_path: str, params_file) -> str:
    """ground_truth_sim.params_file 相对于 full_config 所在目录解析。"""
    if params_file is None:
        return ''
    p = str(params_file).strip()
    if not p:
        return ''
    if os.path.isabs(p):
        return p if os.path.isfile(p) else ''
    base = os.path.dirname(os.path.abspath(full_config_path))
    cand = os.path.normpath(os.path.join(base, p))
    return cand if os.path.isfile(cand) else ''


def write_ground_truth_node_params_yaml(
    gt_cfg: dict,
    dest_path: str,
    user_config: Optional[dict] = None,
    *,
    ros_node_name: str = "scenario_ground_truth_node",
) -> None:
    """写入临时 YAML（scenario.ground_truth_sim）。

    ros_node_name 必须与 launch 里 Node(name=...) 一致，否则参数不会加载（曾导致 frame_id 等
    全部回落到 declare_parameter 默认值，例如误用 base_link）。
    """
    ros_params = dict(DEFAULT_GROUND_TRUTH_NODE_PARAMS)
    for k, v in gt_cfg.items():
        if k in GROUND_TRUTH_SIM_META_KEYS:
            continue
        ros_params[k] = v
    if user_config is not None and 'reference_robot' not in gt_cfg:
        ros_params['reference_robot'] = primary_robot_name(user_config)
    # ROS 2 params YAML 不支持 fixed_targets 嵌套序列，转为 JSON 字符串传递。
    fixed = ros_params.get('fixed_targets')
    if isinstance(fixed, list) and fixed:
        if not str(ros_params.get('fixed_targets_json') or '').strip():
            ros_params['fixed_targets_json'] = json.dumps(fixed, ensure_ascii=False)
    ros_params.pop('fixed_targets', None)
    payload = {ros_node_name: {'ros__parameters': ros_params}}
    with open(dest_path, 'w', encoding='utf-8') as f:
        yaml.safe_dump(payload, f, sort_keys=False, allow_unicode=True)


def write_ground_truth_entity_params_yaml(
    gt_cfg: dict,
    dest_path: str,
    *,
    full_config_path: str = "",
    ros_node_name: str = "scenario_ground_truth_gazebo_entity",
) -> None:
    """写入 Gazebo 实体权威真值节点临时参数 YAML。"""
    ros_params = dict(DEFAULT_GROUND_TRUTH_ENTITY_PARAMS)
    for k, v in gt_cfg.items():
        if k in GROUND_TRUTH_SIM_LAUNCH_ONLY_KEYS:
            continue
        if v is None or v == '':
            continue
        if k == 'gazebo_mesh_profile':
            ros_params['gazebo_mesh_profile'] = _resolve_gazebo_mesh_profile(v, full_config_path)
            continue
        ros_params[k] = v
    fixed = gt_cfg.get('fixed_targets')
    if isinstance(fixed, list) and fixed:
        if not str(ros_params.get('fixed_targets_json') or '').strip():
            ros_params['fixed_targets_json'] = json.dumps(fixed, ensure_ascii=False)
    ros_params.pop('fixed_targets', None)
    payload = {ros_node_name: {'ros__parameters': ros_params}}
    with open(dest_path, 'w', encoding='utf-8') as f:
        yaml.safe_dump(payload, f, sort_keys=False, allow_unicode=True)


def resolve_session_robots(session_info: dict, user_config: dict):
    """合并 session_manager 的 robots 列表；旧 JSON 无 robots 时回退单船。"""
    robots = session_info.get('robots')
    if robots:
        return robots
    rc = user_config.get('robot') or {}
    return [{
        'name': rc.get('name', 'usv'),
        'urdf_path': session_info['urdf_path'],
        'bridge_yaml_path': session_info['bridge_yaml_path'],
        'spawn_pose': rc.get('spawn_pose', [0.0, 0.0, 0.5, 0.0, 0.0, 0.0]),
    }]
