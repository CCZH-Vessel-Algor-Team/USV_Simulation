import os
import subprocess
import yaml

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from usv_sim_full.launch_config_helpers import (
    default_radar_nav2_param_yaml,
    primary_robot_name,
    ship_config_blocks,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_nav2_namespace(requested_ns: str, cfg_path: str) -> tuple[str, list[str], dict]:
    """解析 Nav2 命名空间，返回 (resolved_ns, known_names, cfg)。"""
    cfg = {}
    try:
        with open(cfg_path, 'r') as f:
            cfg = yaml.safe_load(f) or {}
    except Exception:
        pass

    resolved_ns = requested_ns.strip()
    if resolved_ns in ('', 'auto'):
        resolved_ns = 'usv_1'
        try:
            resolved_ns = primary_robot_name(cfg)
        except Exception:
            resolved_ns = 'usv_1'

    known_names = []
    for block in ship_config_blocks(cfg):
        if isinstance(block, dict) and block.get('name'):
            known_names.append(str(block['name']).strip())

    return resolved_ns, known_names, cfg


def _nav2_bringup_available() -> bool:
    try:
        get_package_share_directory('nav2_bringup')
        return True
    except PackageNotFoundError:
        return False


_NAV2_INSTALL_HINT = (
    '未找到 nav2_bringup（Nav2 导航栈）。仿真将继续运行，但不会启动 Nav2。'
    '请先在 usv_ws 内构建 usv_nav：'
    ' cd src/usv_nav && colcon build --symlink-install'
    ' 或在工作区根目录： colcon build --packages-up-to usv_sim_full --symlink-install；'
    ' 然后 source install/setup.bash（若使用独立 usv_nav 安装，另 source src/usv_nav/install/setup.bash）。'
)


def generate_launch_description():
    usv_sim_full_pkg = get_package_share_directory('usv_sim_full')
    main_launch_file = os.path.join(usv_sim_full_pkg, 'launch', 'main.launch.py')

    nav2_thruster_launch_file = os.path.join(
        usv_sim_full_pkg, 'launch', 'nav2_thruster_bringup.launch.py'
    )

    launch_dir = os.path.dirname(os.path.abspath(__file__))
    default_nav2_params_file = default_radar_nav2_param_yaml(launch_dir)

    default_config_path = os.path.join(usv_sim_full_pkg, 'config', 'full_config.yaml')
    default_localization_params = os.path.join(
        usv_sim_full_pkg, 'config', 'robot_localization_gps.yaml'
    )
    default_map_yaml = os.path.join(usv_sim_full_pkg, 'maps', 'sydney_map2.yaml')

    _rviz_share = os.path.join(usv_sim_full_pkg, 'rviz', 'default.rviz')
    _rviz_src = os.path.normpath(
        os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'rviz', 'default.rviz')
    )
    default_rviz_config = _rviz_share if os.path.isfile(_rviz_share) else _rviz_src

    config_path = LaunchConfiguration('config_path')
    nav2_namespace = LaunchConfiguration('nav2_namespace')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    nav2_min_wait_sec = LaunchConfiguration('nav2_min_wait_sec')
    nav2_readiness_timeout_sec = LaunchConfiguration('nav2_readiness_timeout_sec')
    nav2_poll_period_sec = LaunchConfiguration('nav2_poll_period_sec')
    nav2_tf_stable_checks = LaunchConfiguration('nav2_tf_stable_checks')
    nav2_require_map = LaunchConfiguration('nav2_require_map')
    nav2_map_topic = LaunchConfiguration('nav2_map_topic')
    nav2_start_on_gate_failure = LaunchConfiguration('nav2_start_on_gate_failure')
    enable_nav2 = LaunchConfiguration('enable_nav2')
    auto_cleanup = LaunchConfiguration('auto_cleanup')
    cleanup_fastdds_shm = LaunchConfiguration('cleanup_fastdds_shm')
    enable_robot_localization = LaunchConfiguration('enable_robot_localization')
    localization_params_file = LaunchConfiguration('localization_params_file')
    use_static_map_odom_tf = LaunchConfiguration('use_static_map_odom_tf')
    enable_tf_namespace_relay = LaunchConfiguration('enable_tf_namespace_relay')
    disable_fastdds_shm = LaunchConfiguration('disable_fastdds_shm')
    gz_headless = LaunchConfiguration('gz_headless')
    map_yaml = LaunchConfiguration('map_yaml')
    verbose_launch = LaunchConfiguration('verbose_launch')

    def disable_fastdds_shm_env(context, *args, **kwargs):
        """Gazebo + 多 ROS 节点时 Fast DDS 默认共享内存易抖动，可触发 lifecycle 服务响应超时。"""
        if disable_fastdds_shm.perform(context).lower() != 'true':
            return []
        return [
            SetEnvironmentVariable(
                name='RMW_FASTRTPS_USE_SHM',
                value='0',
            ),
        ]

    def prelaunch_cleanup(context, *args, **kwargs):
        if auto_cleanup.perform(context).lower() != 'true':
            return []

        kill_pattern = (
            'nav2_thruster_bringup.launch.py|'
            'navigation_launch.py|'
            'main.launch.py|'
            'gz sim|'
            'ros_gz_bridge/parameter_bridge|'
            'odom_tf_broadcaster|'
            'controller_server|planner_server|bt_navigator|behavior_server|'
            'waypoint_follower|velocity_smoother|smoother_server|'
            'lifecycle_manager_navigation|lifecycle_manager_map|map_server|cmd_vel_to_thruster.py|'
            'ekf_node|navsat_transform_node|radar_gz_bridge|'
            'adaptive_radar_grid_map_node|usv_sim_wrapper|scenario_manager_node|'
            'nav2_tf_readiness_gate'
        )
        subprocess.run(
            ['bash', '-lc', f'pkill -9 -f "{kill_pattern}" || true; sleep 1'],
            check=False,
        )

        if cleanup_fastdds_shm.perform(context).lower() == 'true':
            subprocess.run(
                ['bash', '-lc', 'rm -rf /dev/shm/fastrtps* 2>/dev/null || true'],
                check=False,
            )

        return []

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(main_launch_file),
        launch_arguments={
            'config_path': config_path,
            'enable_robot_localization': enable_robot_localization,
            'localization_params_file': localization_params_file,
            'use_static_map_odom_tf': use_static_map_odom_tf,
            'enable_tf_namespace_relay': enable_tf_namespace_relay,
            'gz_headless': gz_headless,
            'use_sim_time': use_sim_time,
            'rviz_config_path_override': default_rviz_config,
            'verbose_launch': verbose_launch,
            'nav2_namespace': nav2_namespace,
        }.items(),
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_yaml,
            'frame_id': 'map',
            'topic_name': 'map',
            'use_sim_time': use_sim_time,
        }],
    )

    map_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server'],
        }],
    )

    def nav2_readiness_and_deferred_launch(context, *args, **kwargs):
        if enable_nav2.perform(context).lower() != 'true':
            return [
                LogInfo(msg='enable_nav2:=false，仅运行仿真，不启动 Nav2 readiness gate。'),
            ]

        if not _nav2_bringup_available():
            return [
                LogInfo(msg='[WARN] ' + _NAV2_INSTALL_HINT),
            ]

        cfg_path = config_path.perform(context)
        resolved_ns, known_names, _cfg = _resolve_nav2_namespace(
            nav2_namespace.perform(context), cfg_path
        )

        prefix = []
        if known_names and resolved_ns not in known_names:
            prefix.append(
                LogInfo(
                    msg=(
                        '[WARN] nav2_namespace='
                        + resolved_ns
                        + ' 不在 full_config 的船名列表 '
                        + repr(known_names)
                        + ' 中；Nav2 TF/雷达话题可能与仿真不一致。'
                    )
                )
            )

        use_sim = use_sim_time.perform(context).lower() == 'true'
        readiness_gate = Node(
            package='usv_sim_full',
            executable='nav2_tf_readiness_gate',
            name='nav2_tf_readiness_gate',
            output='screen',
            parameters=[{
                'namespace': resolved_ns,
                'use_sim_time': use_sim,
                'min_wait_sec': float(nav2_min_wait_sec.perform(context)),
                'readiness_timeout_sec': float(nav2_readiness_timeout_sec.perform(context)),
                'poll_period_sec': float(nav2_poll_period_sec.perform(context)),
                'tf_stable_checks': int(float(nav2_tf_stable_checks.perform(context))),
                'require_map': nav2_require_map.perform(context).lower() == 'true',
                'map_topic': nav2_map_topic.perform(context),
            }],
        )

        nav2_thruster_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_thruster_launch_file),
            launch_arguments={
                'namespace': resolved_ns,
                'params_file': params_file.perform(context),
                'use_sim_time': use_sim_time.perform(context),
                'verbose_launch': verbose_launch.perform(context),
                'enable_tf_namespace_relay': 'false',
            }.items(),
        )

        def on_readiness_gate_exit(event, _context):
            if event.returncode != 0:
                if nav2_start_on_gate_failure.perform(_context).lower() == 'true':
                    if not _nav2_bringup_available():
                        return [
                            LogInfo(msg='[ERROR] ' + _NAV2_INSTALL_HINT),
                        ]
                    return [
                        LogInfo(
                            msg=(
                                '[WARN] Nav2 TF readiness gate 失败 (code='
                                + str(event.returncode)
                                + ')，nav2_start_on_gate_failure:=true，仍启动 Nav2'
                            )
                        ),
                        nav2_thruster_launch,
                    ]
                return [
                    LogInfo(
                        msg=(
                            '[ERROR] Nav2 TF readiness gate 失败 (code='
                            + str(event.returncode)
                            + ')，已跳过 Nav2。可加大 nav2_readiness_timeout_sec '
                            '或设 nav2_start_on_gate_failure:=true 强制启动。'
                        )
                    ),
                ]

            if not _nav2_bringup_available():
                return [
                    LogInfo(
                        msg=(
                            '[WARN] Nav2 TF readiness gate 已通过，但 '
                            + _NAV2_INSTALL_HINT
                        )
                    ),
                ]

            return [
                LogInfo(
                    msg=(
                        'Nav2 TF readiness gate 通过，启动 Nav2 (namespace='
                        + resolved_ns
                        + ')'
                    )
                ),
                nav2_thruster_launch,
            ]

        gate_exit_handler = RegisterEventHandler(
            OnProcessExit(
                target_action=readiness_gate,
                on_exit=on_readiness_gate_exit,
            )
        )

        return [
            *prefix,
            LogInfo(
                msg=(
                    'Nav2 将在 TF/地图就绪后启动 (namespace='
                    + resolved_ns
                    + ', timeout='
                    + nav2_readiness_timeout_sec.perform(context)
                    + 's)'
                )
            ),
            readiness_gate,
            gate_exit_handler,
        ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_path',
            default_value=default_config_path,
            description='Path to usv_sim_full full_config.yaml',
        ),
        DeclareLaunchArgument(
            'nav2_namespace',
            default_value='auto',
            description=(
                'Nav2 与 cmd_vel→桨 所跟船的 ROS 命名空间，必须与 full_config 中该船 '
                'robot_*.name 完全一致（如 usv_1），以便 TF 帧 {name}/odom、{name}/base_link '
                '与 radar_nav2_param 中 __ROBOT_NS__ 替换一致。'
                'auto 或空：取 robot_1（按 slot 数字排序后的首船）。'
            ),
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_nav2_params_file,
            description='Nav2 parameters file path',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock',
        ),
        DeclareLaunchArgument(
            'nav2_min_wait_sec',
            default_value='5.0',
            description='Nav2 readiness gate：启动后最短等待（秒），再开始检测 TF/地图',
        ),
        DeclareLaunchArgument(
            'nav2_readiness_timeout_sec',
            default_value='120.0',
            description='Nav2 readiness gate：总超时（秒）；超时则默认不启动 Nav2',
        ),
        DeclareLaunchArgument(
            'nav2_poll_period_sec',
            default_value='0.5',
            description='Nav2 readiness gate：轮询周期（秒）',
        ),
        DeclareLaunchArgument(
            'nav2_tf_stable_checks',
            default_value='3',
            description='Nav2 readiness gate：连续 TF 检测成功次数（防抖）',
        ),
        DeclareLaunchArgument(
            'nav2_require_map',
            default_value='true',
            description='Nav2 readiness gate：是否等待 map_topic 至少收到一条地图',
        ),
        DeclareLaunchArgument(
            'nav2_map_topic',
            default_value='/map',
            description='Nav2 readiness gate：静态地图话题',
        ),
        DeclareLaunchArgument(
            'nav2_start_on_gate_failure',
            default_value='false',
            description='true：readiness gate 超时/失败时仍强制启动 Nav2',
        ),
        DeclareLaunchArgument(
            'enable_nav2',
            default_value='true',
            description='false：仅仿真，不运行 readiness gate 与 Nav2 导航栈',
        ),
        DeclareLaunchArgument(
            'auto_cleanup',
            default_value='true',
            description='Kill stale sim/Nav2 related processes before launching',
        ),
        DeclareLaunchArgument(
            'cleanup_fastdds_shm',
            default_value='false',
            description='Additionally clean /dev/shm/fastrtps* before launching',
        ),
        DeclareLaunchArgument(
            'disable_fastdds_shm',
            default_value='true',
            description=(
                'true：子进程设置 RMW_FASTRTPS_USE_SHM=0，减轻 planner_server change_state 等与 '
                'Fast DDS 共享内存相关的超时/丢响应'
            ),
        ),
        DeclareLaunchArgument(
            'enable_robot_localization',
            default_value='false',
            description='Enable robot_localization in usv_sim_full main launch',
        ),
        DeclareLaunchArgument(
            'localization_params_file',
            default_value=default_localization_params,
            description='robot_localization parameter yaml path',
        ),
        DeclareLaunchArgument(
            'use_static_map_odom_tf',
            default_value='true',
            description='Publish static identity map->odom TF in main launch',
        ),
        DeclareLaunchArgument(
            'enable_tf_namespace_relay',
            default_value='true',
            description=(
                'true：main.launch 为每条船启动 tf_namespace_relay（/tf -> /{robot}/tf）；'
                'Nav2 readiness gate 与命名空间 Nav2 依赖此路径'
            ),
        ),
        DeclareLaunchArgument(
            'verbose_launch',
            default_value='false',
            description='透传 main / nav2_thruster：true 时终端详细输出（默认降噪）',
        ),
        DeclareLaunchArgument(
            'gz_headless',
            default_value='false',
            description='true 时 Gazebo 以 server-only 模式运行（无 GUI 渲染窗口）',
        ),
        DeclareLaunchArgument(
            'map_yaml',
            default_value=default_map_yaml,
            description='PGM map yaml file for nav2_map_server',
        ),
        LogInfo(msg=['Starting simulation bringup from: ', config_path]),
        OpaqueFunction(function=disable_fastdds_shm_env),
        OpaqueFunction(function=prelaunch_cleanup),
        sim_launch,
        map_server,
        map_lifecycle_manager,
        OpaqueFunction(function=nav2_readiness_and_deferred_launch),
    ])
