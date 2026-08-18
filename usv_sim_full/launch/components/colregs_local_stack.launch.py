"""COLREGS Local Planner Server 版 Nav2 导航栈（单船实例）。

等价于 nav2_thruster_bringup.launch.py + nav2_bringup/navigation_launch.py 的组合，
但用 nav2_colregs_local_planner_server（内置 VO-RRT* + TS 决策 + colregs_costmap，
进程内编排 colregs_ts_state 子节点）替代标准 planner_server + TS 三节点子系统。

启动内容（均位于 namespace 下，默认 usv_1）：
  clearing_scan_publisher
  controller_server / smoother_server / colregs_local_planner_server /
  behavior_server / bt_navigator / waypoint_follower / velocity_smoother
  lifecycle_manager_navigation（node_names 中 planner_server 替换为
  colregs_local_planner_server）
  cmd_vel_to_thruster 推力桥（进程外 ros2 run）

参数文件默认 config/radar_nav2_param_colregs_local.yaml：
  __ROBOT_NS__ 占位符在此替换；control_params（ALOS/PID）深合并（剔除 /** 段）；
  处理后的临时 yaml 经 RewrittenYaml(root_key=namespace) 注入各节点。
"""

import os
import tempfile
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    LogInfo,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from usv_sim_full.launch_config_helpers import launch_verbose_enabled, quiet_ros_node_kwargs


def _subst_robot_ns(obj, ns: str):
    """将参数 yaml 中的 __ROBOT_NS__ 替换为实际船名（无首尾 /）。"""
    if isinstance(obj, dict):
        return {k: _subst_robot_ns(v, ns) for k, v in obj.items()}
    if isinstance(obj, list):
        return [_subst_robot_ns(x, ns) for x in obj]
    if isinstance(obj, str):
        return obj.replace('__ROBOT_NS__', ns)
    return obj


def _deep_merge(base, override):
    """递归合并 override dict 到 base dict，返回 base（原地修改）。"""
    for k, v in override.items():
        if k in base and isinstance(base[k], dict) and isinstance(v, dict):
            _deep_merge(base[k], v)
        else:
            base[k] = v
    return base


def _default_colregs_params_file(launch_dir: str) -> str:
    """默认参数文件：优先 install/share，回退源码树。"""
    share = os.path.join(
        get_package_share_directory('usv_sim_full'),
        'config', 'radar_nav2_param_colregs_local.yaml',
    )
    if os.path.isfile(share):
        return share
    src = os.path.normpath(os.path.join(
        os.path.dirname(launch_dir), 'config', 'radar_nav2_param_colregs_local.yaml'))
    return src


def generate_launch_description():
    usv_sim_full_pkg = get_package_share_directory('usv_sim_full')
    launch_dir = os.path.dirname(os.path.abspath(__file__))
    default_params_file = _default_colregs_params_file(launch_dir)
    default_control_params_file = os.path.join(
        usv_sim_full_pkg, 'config', 'control_params.yaml'
    )

    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')
    control_params_file = LaunchConfiguration('control_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    verbose_launch = LaunchConfiguration('verbose_launch')
    enable_tf_namespace_relay = LaunchConfiguration('enable_tf_namespace_relay')

    def launch_tf_relay_node(context, *args, **kwargs):
        if enable_tf_namespace_relay.perform(context).lower() != 'true':
            return [
                LogInfo(
                    msg=(
                        'enable_tf_namespace_relay:=false，跳过 tf_namespace_relay '
                        '（假定 main.launch 或其它进程已提供 /{ns}/tf）'
                    )
                ),
            ]
        v = verbose_launch.perform(context)
        ns = namespace.perform(context).strip().strip('/') or 'usv_1'
        return [
            Node(
                package='usv_sim_full',
                executable='tf_namespace_relay',
                name='tf_namespace_relay',
                namespace=ns,
                parameters=[{
                    'namespace': ns,
                    'use_sim_time': use_sim_time,
                }],
                **quiet_ros_node_kwargs(v),
            )
        ]

    def _launch_colregs_stack(context, *args, **kwargs):
        resolved_ns = namespace.perform(context).strip().strip('/')
        resolved_params_file = params_file.perform(context)
        resolved_use_sim_time = use_sim_time.perform(context)

        prefix_logs = []

        with open(resolved_params_file, 'r') as f:
            nav2_params = yaml.safe_load(f) or {}

        nav2_params = _subst_robot_ns(nav2_params, resolved_ns)

        # 静态地图话题统一为 /map（map_server 在根命名空间发布）
        map_topic = '/map'
        try:
            nav2_params['colregs_costmap']['colregs_costmap']['ros__parameters'][
                'static_layer']['map_topic'] = map_topic
        except KeyError:
            pass

        # deep-merge control_params（剔除 /** 通配段，仅合并具名节点参数）
        resolved_control_params_file = control_params_file.perform(context).strip()
        if resolved_control_params_file and os.path.isfile(resolved_control_params_file):
            with open(resolved_control_params_file, 'r') as f:
                control_params = yaml.safe_load(f) or {}
            control_params = _subst_robot_ns(control_params, resolved_ns)
            control_part = {k: v for k, v in control_params.items() if k != '/**'}
            _deep_merge(nav2_params, control_part)
            prefix_logs.append(
                LogInfo(msg=f'已合并控制参数: {resolved_control_params_file}')
            )

        gcp = nav2_params.get('colregs_costmap', {}).get(
            'colregs_costmap', {}).get('ros__parameters', {})
        robot_bf = str(gcp.get('robot_base_frame', '?'))

        tmp_file = tempfile.NamedTemporaryFile(
            mode='w',
            prefix=f'nav2_colregs_{resolved_ns}_',
            suffix='.yaml',
            delete=False,
        )
        with tmp_file:
            yaml.safe_dump(nav2_params, tmp_file, sort_keys=False)

        info = LogInfo(
            msg=(
                'COLREGS Nav2 参数已按船名注入: namespace='
                + resolved_ns
                + ' colregs_costmap.robot_base_frame='
                + robot_bf
                + ' static_layer.map_topic='
                + map_topic
            )
        )

        configured_params = ParameterFile(
            RewrittenYaml(
                source_file=tmp_file.name,
                root_key=namespace,
                param_rewrites={'use_sim_time': use_sim_time},
                convert_types=True),
            allow_substs=True)

        # 与 navigation_launch.py 相同的 tf 重映射（/tf → <ns>/tf，经 relay 提供）
        remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

        lifecycle_nodes = [
            'controller_server',
            'smoother_server',
            'colregs_local_planner_server',
            'behavior_server',
            'bt_navigator',
            'waypoint_follower',
            'velocity_smoother',
        ]

        stack = GroupAction([
            PushRosNamespace(namespace=namespace),
            Node(
                package='usv_sim_full',
                executable='clearing_scan_publisher',
                name='clearing_scan_publisher',
                parameters=[{
                    'topic': '/clearing_scan',
                    'frame_id': resolved_ns + '/base_link',
                    'max_range': 200.0,
                    'num_rays': 3600,
                    'publish_rate': 5.0,
                    'use_sim_time': resolved_use_sim_time == 'true' or resolved_use_sim_time == 'True',
                }],
                output='screen',
            ),
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                parameters=[configured_params],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                parameters=[configured_params],
                remappings=remappings),
            Node(
                package='nav2_colregs_local_planner_server',
                executable='colregs_local_planner_server',
                name='colregs_local_planner_server',
                output='screen',
                parameters=[configured_params],
                remappings=remappings),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                parameters=[configured_params],
                remappings=remappings),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[configured_params],
                remappings=remappings),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                parameters=[configured_params],
                remappings=remappings),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                parameters=[configured_params],
                remappings=remappings +
                    [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': True,
                    'node_names': lifecycle_nodes,
                }]),
        ])

        return [*[p for p in prefix_logs if p is not None], info, stack]

    def launch_thruster_bridge(context, *args, **kwargs):
        ns = namespace.perform(context).strip().strip('/')
        v = verbose_launch.perform(context)
        out = 'screen' if launch_verbose_enabled(v) else 'log'
        cmd = [
            'ros2',
            'run',
            'usv_sim_full',
            'cmd_vel_to_thruster',
            '--ros-args',
            '-r',
            f'__ns:=/{ns}',
            '-p',
            f'namespace:={ns}',
        ]
        resolved_control_params_file = control_params_file.perform(context).strip()
        if resolved_control_params_file and os.path.isfile(resolved_control_params_file):
            cmd += ['--params-file', resolved_control_params_file]
        return [
            ExecuteProcess(
                cmd=cmd,
                name='cmd_vel_to_thruster',
                output=out,
            )
        ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='usv_1',
            description=(
                '本船 ROS 命名空间，须与 full_config 中该船 name 及 TF 前缀一致。'
            ),
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description=(
                'Nav2 参数文件（默认 config/radar_nav2_param_colregs_local.yaml，'
                'planner_server/global_costmap 由 colregs_local_planner_server 替代）'
            ),
        ),
        DeclareLaunchArgument(
            'control_params_file',
            default_value=default_control_params_file,
            description='整船控制参数 YAML（ALOS + PID），合并到 Nav2 参数并传给 cmd_vel→推力桥',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock',
        ),
        DeclareLaunchArgument(
            'verbose_launch',
            default_value='false',
            description='true：TF 中继与 cmd_vel→桨 进程输出到终端；false：写入日志文件',
        ),
        DeclareLaunchArgument(
            'enable_tf_namespace_relay',
            default_value='false',
            description=(
                'true：本 launch 内启动 tf_namespace_relay。'
                '经 nav2_sim_colregs_local_bringup 启动时应为 false'
                '（relay 已在 main.launch 随仿真常驻）'
            ),
        ),
        OpaqueFunction(function=launch_tf_relay_node),
        LogInfo(msg=['Starting COLREGS Nav2 navigation stack for ', namespace, '...']),
        OpaqueFunction(function=_launch_colregs_stack),
        LogInfo(msg=['Starting cmd_vel to thruster bridge for ', namespace, '...']),
        OpaqueFunction(function=launch_thruster_bridge),
    ])
