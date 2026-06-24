"""
认证会遇仿真入口：合并 certi 基底 + certificate_case → 启动 main + 本船 cmd_vel 链。
本船运动链（cmd_vel_to_thruster + certi_own_ship_cmd_vel）在此 launch 内强制拉起，不依赖 Nav2 bringup。
"""

import os
import subprocess
import sys

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from usv_sim_full.launch_config_helpers import launch_verbose_enabled


def _resolve_pkg_paths():
    """share 安装路径与源码树回退。"""
    try:
        share = get_package_share_directory('usv_sim_full')
    except Exception:
        share = None
    launch_dir = os.path.dirname(os.path.abspath(__file__))
    src_root = os.path.normpath(os.path.join(launch_dir, '..'))
    return share, src_root


def _merge_script_path(share: str, src_root: str) -> str:
    candidates = []
    if share:
        candidates.append(os.path.join(share, 'tools', 'merge_certi_config.py'))
    candidates.append(os.path.join(src_root, 'tools', 'merge_certi_config.py'))
    for p in candidates:
        if os.path.isfile(p):
            return p
    raise FileNotFoundError(
        'merge_certi_config.py not found in install share or source tree'
    )


def _config_path(share: str, src_root: str, rel: str) -> str:
    if share:
        p = os.path.join(share, 'config', rel)
        if os.path.isfile(p):
            return p
    return os.path.join(src_root, 'config', rel)


def _resolve_user_path(path: str, src_root: str) -> str:
    """将相对路径解析为绝对路径（优先 cwd，再源码包根）。"""
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


def _merged_path_from_stdout(stdout: str) -> str:
    for line in (stdout or '').splitlines():
        if '[merge_certi_config] OK:' in line:
            return line.split('OK:', 1)[1].strip()
    return ''


def launch_setup(context, *args, **kwargs):
    share, src_root = _resolve_pkg_paths()

    base_config = LaunchConfiguration('base_config').perform(context)
    case_config = LaunchConfiguration('case_config').perform(context)
    merged_config_arg = LaunchConfiguration('merged_config').perform(context).strip()
    verbose_s = LaunchConfiguration('verbose_launch').perform(context)
    robot_ns_arg = LaunchConfiguration('robot_namespace').perform(context).strip()
    thrust_delay_s = float(LaunchConfiguration('thrust_chain_delay').perform(context))

    if not os.path.isfile(base_config):
        base_config = _config_path(share, src_root, 'certi_senario.yaml')
    case_config = _resolve_user_path(case_config, src_root)
    if not os.path.isfile(case_config):
        case_config = _config_path(share, src_root, 'certificate_case/C1-001.yaml')

    merge_script = _merge_script_path(share, src_root)
    out_path = merged_config_arg
    cmd = [sys.executable, merge_script, '--base', base_config, '--case', case_config]
    if out_path:
        cmd.extend(['--out', out_path])

    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(
            f'merge_certi_config failed ({result.returncode}):\n'
            f'{result.stdout}\n{result.stderr}'
        )
    if result.stdout.strip():
        print(result.stdout.strip())
    if result.stderr.strip():
        print(result.stderr.strip(), file=sys.stderr)

    if not out_path:
        out_path = _merged_path_from_stdout(result.stdout)
    if not out_path or not os.path.isfile(out_path):
        with open(case_config, 'r', encoding='utf-8') as f:
            case_data = yaml.safe_load(f) or {}
        case_id = _case_id_from_yaml(case_data, case_config)
        out_path = os.path.join(
            os.path.dirname(os.path.abspath(base_config)),
            'generated',
            f'{case_id}.merged.yaml',
        )

    if not os.path.isfile(out_path):
        raise RuntimeError(f'merged config not found: {out_path}')

    with open(out_path, 'r', encoding='utf-8') as f:
        merged = yaml.safe_load(f) or {}

    runtime = merged.get('certificate_runtime', {})
    own_vel = runtime.get('own_ship_velocity', {})
    if not own_vel.get('enabled', False):
        raise RuntimeError(
            f'certificate_runtime.own_ship_velocity.enabled must be true in {out_path}'
        )

    robot_ns = robot_ns_arg or str(merged.get('robot_1', {}).get('name', 'usv_1')).strip('/')
    speed_mps = float(own_vel.get('speed_mps', 4.0))
    course_deg = float(own_vel.get('course_deg', 0.0))

    share_dir = share or src_root
    main_launch = os.path.join(
        get_package_share_directory('usv_sim_full') if share else src_root,
        'launch',
        'main.launch.py',
    )
    if not os.path.isfile(main_launch):
        main_launch = os.path.join(src_root, 'launch', 'main.launch.py')

    out_mode = 'screen' if launch_verbose_enabled(verbose_s) else 'log'

    actions = [
        LogInfo(msg=[f'[certifi_launch] merged config: {out_path}']),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(main_launch),
            launch_arguments={
                'config_path': out_path,
                'verbose_launch': verbose_s,
            }.items(),
        ),
    ]

    thrust_chain = [
        ExecuteProcess(
            cmd=[
                'ros2',
                'run',
                'usv_sim_full',
                'cmd_vel_to_thruster',
                '--ros-args',
                '-r',
                f'__ns:=/{robot_ns}',
                '-p',
                f'namespace:={robot_ns}',
            ],
            name='cmd_vel_to_thruster',
            output=out_mode,
        ),
        Node(
            package='usv_sim_full',
            executable='certi_own_ship_cmd_vel',
            name='certi_own_ship_cmd_vel',
            namespace=robot_ns,
            parameters=[{
                'namespace': robot_ns,
                'speed_mps': speed_mps,
                'course_deg': course_deg,
            }],
            output=out_mode,
        ),
        LogInfo(msg=[
            f'[certifi_launch] thrust chain: cmd_vel_to_thruster + '
            f'certi_own_ship_cmd_vel @ {robot_ns} {speed_mps} m/s',
        ]),
    ]

    actions.append(
        TimerAction(
            period=max(thrust_delay_s, 0.0),
            actions=thrust_chain,
        )
    )

    return actions


def generate_launch_description():
    share, src_root = _resolve_pkg_paths()
    default_base = _config_path(share, src_root, 'certi_senario.yaml')
    default_case = _config_path(share, src_root, 'certificate_case/C1-001.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'base_config',
            default_value=default_base,
            description='认证仿真基底 certi_senario.yaml',
        ),
        DeclareLaunchArgument(
            'case_config',
            default_value=default_case,
            description='certificate_case 场景 YAML（含 encounter）',
        ),
        DeclareLaunchArgument(
            'merged_config',
            default_value='',
            description='合并输出路径；空则写入 config/generated/<case_id>.merged.yaml',
        ),
        DeclareLaunchArgument(
            'robot_namespace',
            default_value='',
            description='本船命名空间；空则从 merged robot_1.name 读取',
        ),
        DeclareLaunchArgument(
            'verbose_launch',
            default_value='false',
            description='true：详细日志输出到终端',
        ),
        DeclareLaunchArgument(
            'thrust_chain_delay',
            default_value='3.0',
            description='spawn 后延时启动 cmd_vel 链（秒）',
        ),
        OpaqueFunction(function=launch_setup),
    ])
