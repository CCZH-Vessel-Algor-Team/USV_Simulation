"""CCS 认证会遇：将 certificate_case 合并进 CCS 配置后，拉起完整 CCS + Nav2 环境。

与 certifi_launch 的区别：
- 基底默认 three_vision_one_mmwave/ccs_config.yaml（三视觉/毫米波/Nav2 栈）
- 本船运行航速由 Nav2 给定，不启 certi_own_ship_cmd_vel
- 目标船由 scenario_manager 驱动，经 /certificate_case/tracked_ships
  → GT bridge → /sim/ground_truth/_src/scenario → CCS merger → 感知/融合

用法（待当前测试完成后再拉）：

  ros2 launch usv_sim_full ccs_certificate_encounter.launch.py \\
    case_config:=src/usv_simulation/usv_sim_full/config/certificate_case/C1-001.yaml
"""

from __future__ import annotations

import os
import subprocess
import sys
from typing import Optional

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_pkg_paths():
    try:
        share = get_package_share_directory('usv_sim_full')
    except Exception:
        share = None
    launch_dir = os.path.dirname(os.path.abspath(__file__))
    src_root = os.path.normpath(os.path.join(launch_dir, '..'))
    return share, src_root


def _merge_script_path(share: Optional[str], src_root: str) -> str:
    candidates = []
    if share:
        candidates.append(os.path.join(share, 'tools', 'merge_certi_config.py'))
    candidates.append(os.path.join(src_root, 'tools', 'merge_certi_config.py'))
    for path in candidates:
        if os.path.isfile(path):
            return path
    raise FileNotFoundError('merge_certi_config.py not found')


def _config_path(share: Optional[str], src_root: str, *parts: str) -> str:
    rel = os.path.join(*parts)
    if share:
        path = os.path.join(share, 'config', rel)
        if os.path.isfile(path):
            return path
    return os.path.join(src_root, 'config', rel)


def _resolve_user_path(path: str, src_root: str) -> str:
    if not path:
        return path
    if os.path.isabs(path) and os.path.isfile(path):
        return path
    if os.path.isfile(path):
        return os.path.abspath(path)
    cand = os.path.normpath(os.path.join(src_root, path))
    if os.path.isfile(cand):
        return cand
    return os.path.abspath(path)


def _case_id_from_yaml(case_data: dict, case_path: str) -> str:
    if case_data.get('scenario_id'):
        return str(case_data['scenario_id'])
    meta = case_data.get('meta') or {}
    if meta.get('case_id'):
        return str(meta['case_id'])
    return os.path.splitext(os.path.basename(case_path))[0]


def _forward_launch_args(*names: str) -> dict:
    """把同名 LaunchConfiguration 透传给 CCS launch。"""
    return {name: LaunchConfiguration(name) for name in names}


def launch_setup(context, *args, **kwargs):
    share, src_root = _resolve_pkg_paths()

    base_config = LaunchConfiguration('base_config').perform(context).strip()
    case_config = LaunchConfiguration('case_config').perform(context).strip()
    merged_config_arg = LaunchConfiguration('merged_config').perform(context).strip()
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    enable_cert_gt = (
        LaunchConfiguration('enable_certificate_gt_bridge').perform(context).strip().lower()
        in ('true', '1', 'yes')
    )

    if not base_config or not os.path.isfile(base_config):
        base_config = _config_path(
            share, src_root, 'three_vision_one_mmwave', 'ccs_config.yaml'
        )
    case_config = _resolve_user_path(case_config, src_root)
    if not os.path.isfile(case_config):
        case_config = _config_path(share, src_root, 'certificate_case', 'C1-001.yaml')

    merge_script = _merge_script_path(share, src_root)
    with open(case_config, 'r', encoding='utf-8') as f:
        case_data = yaml.safe_load(f) or {}
    case_id = _case_id_from_yaml(case_data, case_config)

    if merged_config_arg:
        out_path = merged_config_arg
    else:
        out_dir = os.path.join(os.path.dirname(os.path.abspath(base_config)), 'generated')
        os.makedirs(out_dir, exist_ok=True)
        out_path = os.path.join(out_dir, f'{case_id}.ccs.merged.yaml')

    cmd = [
        sys.executable, merge_script,
        '--base', base_config, '--case', case_config, '--out', out_path,
    ]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0 or not os.path.isfile(out_path):
        raise RuntimeError(
            f'merge_certi_config failed ({result.returncode}):\n'
            f'{result.stdout}\n{result.stderr}'
        )
    if result.stdout.strip():
        print(result.stdout.strip())
    if result.stderr.strip():
        print(result.stderr.strip(), file=sys.stderr)

    with open(out_path, 'r', encoding='utf-8') as f:
        merged = yaml.safe_load(f) or {}

    # generated/ 子目录会导致相对路径 sensor_config.yaml 解析失败；写成绝对路径
    base_dir = os.path.dirname(os.path.abspath(base_config))
    sensor_rel = str(merged.get('sensor_config_path') or 'sensor_config.yaml').strip()
    if sensor_rel and not os.path.isabs(sensor_rel):
        sensor_abs = os.path.normpath(os.path.join(base_dir, sensor_rel))
        if not os.path.isfile(sensor_abs):
            sensor_abs = os.path.normpath(
                os.path.join(os.path.dirname(out_path), '..', sensor_rel)
            )
        if os.path.isfile(sensor_abs):
            merged['sensor_config_path'] = sensor_abs
            with open(out_path, 'w', encoding='utf-8') as f:
                yaml.safe_dump(
                    merged, f, default_flow_style=False, allow_unicode=True, sort_keys=False
                )

    runtime = merged.get('certificate_runtime') or {}
    case_id = runtime.get('case_id', case_id)
    sm_cfg = ((merged.get('scenario') or {}).get('scenario_manager') or {})
    tracked_topic = str(
        sm_cfg.get('tracked_ships_topic') or '/certificate_case/tracked_ships'
    )

    ccs_launch = os.path.join(
        share or src_root,
        'launch',
        'CCS_Certified_Simulation_Environment.launch.py',
    )
    if not os.path.isfile(ccs_launch):
        ccs_launch = os.path.join(
            src_root, 'launch', 'CCS_Certified_Simulation_Environment.launch.py'
        )

    # 透传 CCS 常用开关；config_path 强制为合并产物
    ccs_args = _forward_launch_args(
        'use_sim_time',
        'enable_dynamic_ship_gt_bridge',
        'enable_ais_sim',
        'enable_ais_aggregator',
        'enable_safety',
        'enable_route_planner',
        'enable_camera_rtsp_streaming',
        'enable_map_rtsp_streamer',
        'enable_gazebo_camera_follow',
        'enable_nav2',
        'nav2_min_wait_sec',
        'nav2_readiness_timeout_sec',
    )
    ccs_args['config_path'] = out_path

    actions = [
        LogInfo(msg=[
            f'[ccs_certificate_encounter] case={case_id} merged={out_path}'
        ]),
        LogInfo(msg=[
            '[ccs_certificate_encounter] Nav2 owns own-ship speed; '
            'certi_own_ship_cmd_vel is NOT started'
        ]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ccs_launch),
            launch_arguments=ccs_args.items(),
        ),
    ]

    if enable_cert_gt:
        actions.extend([
            LogInfo(msg=[
                f'[ccs_certificate_encounter] certificate GT bridge: '
                f'{tracked_topic} → /sim/ground_truth/_src/scenario'
            ]),
            Node(
                package='ground_truth_sensor_sim',
                executable='dynamic_ship_to_ground_truth',
                name='certificate_case_to_ground_truth',
                output='log',
                parameters=[{
                    'use_sim_time': use_sim_time.lower() == 'true',
                    'input_topic': tracked_topic,
                    'output_topic': '/sim/ground_truth/_src/scenario',
                    'frame_id': 'map',
                    'size_w': 3.6,
                    'size_l': 10.0,
                    'size_h': 2.0,
                    'is_dark_target': True,
                    'is_ais_matched': False,
                    'matched_mmsi': 0,
                    'source_model_name': 'certificate_case',
                }],
            ),
        ])
    else:
        actions.append(
            LogInfo(msg='[ccs_certificate_encounter] enable_certificate_gt_bridge:=false')
        )

    return actions


def generate_launch_description():
    share, src_root = _resolve_pkg_paths()
    default_base = _config_path(
        share, src_root, 'three_vision_one_mmwave', 'ccs_config.yaml'
    )
    default_case = _config_path(share, src_root, 'certificate_case', 'C1-001.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'base_config',
            default_value=default_base,
            description='CCS 基底配置（默认 three_vision_one_mmwave/ccs_config.yaml）',
        ),
        DeclareLaunchArgument(
            'case_config',
            default_value=default_case,
            description='certificate_case 场景 YAML',
        ),
        DeclareLaunchArgument(
            'merged_config',
            default_value='',
            description='合并输出路径；空则写入 base 旁 generated/<id>.ccs.merged.yaml',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock',
        ),
        DeclareLaunchArgument(
            'enable_certificate_gt_bridge',
            default_value='true',
            description=(
                'true：将 /certificate_case/tracked_ships 转到 '
                '/sim/ground_truth/_src/scenario，供 CCS merger 汇入融合链'
            ),
        ),
        # 以下参数透传 CCS_Certified_Simulation_Environment
        DeclareLaunchArgument('enable_dynamic_ship_gt_bridge', default_value='true'),
        DeclareLaunchArgument('enable_ais_sim', default_value='true'),
        DeclareLaunchArgument('enable_ais_aggregator', default_value='true'),
        DeclareLaunchArgument('enable_safety', default_value='true'),
        DeclareLaunchArgument('enable_route_planner', default_value='true'),
        DeclareLaunchArgument(
            'enable_camera_rtsp_streaming',
            default_value='false',
            description='认证会遇默认关闭相机 RTSP，避免刷屏拖死 Nav2 lifecycle',
        ),
        DeclareLaunchArgument(
            'enable_map_rtsp_streamer',
            default_value='false',
            description='认证会遇默认关闭地图 RTSP',
        ),
        DeclareLaunchArgument(
            'enable_gazebo_camera_follow',
            default_value='true',
        ),
        # 透传给底层 three_vision bringup（CCS include 会再透传）
        DeclareLaunchArgument(
            'nav2_min_wait_sec',
            default_value='8.0',
            description='Nav2 readiness 最短等待，给仿真 TF/实体更多稳定时间',
        ),
        DeclareLaunchArgument(
            'nav2_readiness_timeout_sec',
            default_value='180.0',
            description='Nav2 readiness 总超时',
        ),
        DeclareLaunchArgument(
            'enable_nav2',
            default_value='true',
            description='是否启动 Nav2',
        ),
        OpaqueFunction(function=launch_setup),
    ])
