"""COLREGS Local Planner Server 全栈仿真 bringup（nav2_sim_three_vision_mmwave_bringup 的独立变体）。

与原 three_vision launch 的差异（其余仿真/感知/融合/监控链完全一致）：
  - Nav2 栈改为 components/colregs_local_stack.launch.py：
      colregs_local_planner_server（内置 VO-RRT* 避障规划 + TS 决策 + colregs_costmap）
      替代标准 planner_server 与 TS 三节点子系统（ts_state_manager /
      avoidance_point_node / barrier_node 已在上游 usv_nav 中删除并内化）
  - 默认参数 config/radar_nav2_param_colregs_local.yaml（planner_server /
      global_costmap 段移除，新增 colregs_* 三段；BT xml 为
      navigate_to_pose_colregs_local.xml，planner_id=RRTStar）
  - 默认 RViz rviz/three_vision_one_mmwave_colregs.rviz（cpa_markers 显示改为
    /usv_1/cpa_markers，随 colregs_ts_state 子节点命名空间）

原 launch 保持不动，供仍需旧栈的同事使用。
"""

import os
import subprocess
import yaml

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from usv_sim_full.launch_config_helpers import (
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
        get_package_share_directory('nav2_colregs_local_planner_server')
        return True
    except PackageNotFoundError:
        return False


_NAV2_INSTALL_HINT = (
    '未找到 nav2_colregs_local_planner_server。仿真将继续运行，但不会启动 Nav2。'
    '请先在 usv_ws 内构建 usv_nav（feat/rrt-star-local-planner-server-humble 分支）：'
    ' cd src/usv_nav && colcon build --symlink-install --packages-select '
    ' nav2_colregs_msgs nav2_colregs_ts_manager nav2_colregs_local_planner_server'
    ' nav2_colregs_costmap_layers nav2_colregs_alos_controller'
    '；然后 source 对应 install/setup.zsh。'
)


def _three_vision_sensor_sim_nodes(context, *args, **kwargs):
    """启动 ground_truth_sensor_sim 三视觉 + 前向毫米波节点。"""
    enable = LaunchConfiguration('enable_gt_sensor_sim').perform(context)
    if enable.lower() != 'true':
        return [
            LogInfo(msg='enable_gt_sensor_sim:=false，跳过 ground_truth_sensor_sim。'),
        ]

    use_sim = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    params_file = LaunchConfiguration('sensor_params_file').perform(context)
    verbose_s = LaunchConfiguration('verbose_launch').perform(context)
    output = 'screen' if verbose_s.lower() in ('true', '1', 'yes') else 'log'

    cfg_path = LaunchConfiguration('config_path').perform(context)
    resolved_ns, _, _ = _resolve_nav2_namespace(
        LaunchConfiguration('nav2_namespace').perform(context), cfg_path
    )
    body_frame = f'{resolved_ns}/base_link'
    vision_frame_ids = {
        'front': f'{resolved_ns}/front_cam_link',
        'left': f'{resolved_ns}/left_cam_link',
        'right': f'{resolved_ns}/right_cam_link',
    }
    vision_mounts = {
        'front': (0.25, 0.0),
        'left': (0.0, 0.25),
        'right': (0.0, -0.25),
    }

    nodes = [
        LogInfo(
            msg=(
                '启动 ground_truth_sensor_sim（订阅 /sim/ground_truth，'
                f'参数 {params_file}，frame 前缀 {resolved_ns}/）'
            )
        ),
    ]
    for suffix in ('front', 'left', 'right'):
        mount_x, mount_y = vision_mounts[suffix]
        nodes.append(
            Node(
                package='ground_truth_sensor_sim',
                executable='sim_vision_node',
                name=f'sim_vision_{suffix}',
                output=output,
                parameters=[
                    {'use_sim_time': use_sim},
                    params_file,
                    {
                        'frame_id': vision_frame_ids[suffix],
                        'body_frame': body_frame,
                        'use_tf_transform': True,
                        'camera_mount_x_m': mount_x,
                        'camera_mount_y_m': mount_y,
                    },
                ],
            )
        )
    nodes.append(
        Node(
            package='ground_truth_sensor_sim',
            executable='sim_mmwave_node',
            name='sim_mmwave_front',
            output=output,
            parameters=[
                {'use_sim_time': use_sim},
                params_file,
                {
                    'frame_id': f'{resolved_ns}/mmwave_front_link',
                    'body_frame': body_frame,
                    'use_tf_transform': True,
                    'radar_mount_x_m': 0.25,
                    'radar_mount_y_m': 0.0,
                },
            ],
        )
    )
    return nodes


def _late_fusion_node(context, *args, **kwargs):
    """启动 usv_late_fusion（默认 event_fusion_three_sensor_io.yaml）。"""
    enable = LaunchConfiguration('enable_late_fusion').perform(context)
    if enable.lower() != 'true':
        return [
            LogInfo(msg='enable_late_fusion:=false，跳过 usv_late_fusion。'),
        ]

    use_sim = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    verbose_s = LaunchConfiguration('verbose_launch').perform(context)
    output = 'screen' if verbose_s.lower() in ('true', '1', 'yes') else 'log'

    override = LaunchConfiguration('fusion_params_file').perform(context).strip()
    if override:
        param_files = [{'use_sim_time': use_sim}, override]
    else:
        io_file = LaunchConfiguration('fusion_io_params_file').perform(context).strip()
        algo_file = LaunchConfiguration('fusion_algorithm_params_file').perform(context).strip()
        param_files = [{'use_sim_time': use_sim}, io_file, algo_file]

    cfg_path = LaunchConfiguration('config_path').perform(context)
    resolved_ns, _, _ = _resolve_nav2_namespace(
        LaunchConfiguration('nav2_namespace').perform(context), cfg_path
    )
    output_frame_id = f'{resolved_ns}/base_link'

    return [
        LogInfo(
            msg=f'启动 usv_late_fusion late_fusion_node（output_frame_id={output_frame_id}）'
        ),
        Node(
            package='usv_late_fusion',
            executable='late_fusion_node',
            name='late_fusion_node',
            output=output,
            parameters=param_files + [{'output_frame_id': output_frame_id}],
        ),
    ]


def _convert_to_trackship_node(context, *args, **kwargs):
    """启动 convert_to_trackship：/fusion/snapshot → /fusion/tracked_ship。"""
    enable = LaunchConfiguration('enable_convert_to_trackship').perform(context)
    if enable.lower() != 'true':
        return [
            LogInfo(msg='enable_convert_to_trackship:=false，跳过 convert_to_trackship。'),
        ]

    use_sim = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    verbose_s = LaunchConfiguration('verbose_launch').perform(context)
    output = 'screen' if verbose_s.lower() in ('true', '1', 'yes') else 'log'

    cfg_path = LaunchConfiguration('config_path').perform(context)
    resolved_ns, _, _ = _resolve_nav2_namespace(
        LaunchConfiguration('nav2_namespace').perform(context), cfg_path
    )
    frame_id = f'{resolved_ns}/base_link'
    params_file = LaunchConfiguration('convert_to_trackship_params_file').perform(context)

    return [
        LogInfo(
            msg=(
                '启动 convert_to_trackship（/fusion/snapshot → /fusion/tracked_ship，'
                f'frame_id={frame_id}）'
            )
        ),
        Node(
            package='convert_to_trackship',
            executable='target_snapshot_to_tracked_ship',
            name='target_snapshot_to_tracked_ship',
            output=output,
            parameters=[
                {'use_sim_time': use_sim},
                params_file,
                {'frame_id': frame_id},
            ],
        ),
    ]


def _maritime_situation_monitor_node(context, *args, **kwargs):
    """启动 maritime_situation_monitor：真值 TrackedShipList → SituationReportArray。"""
    enable = LaunchConfiguration('enable_maritime_situation_monitor').perform(context)
    if enable.lower() != 'true':
        return [
            LogInfo(msg='enable_maritime_situation_monitor:=false，跳过 maritime_situation_monitor。'),
        ]

    use_sim = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    verbose_s = LaunchConfiguration('verbose_launch').perform(context)
    output = 'screen' if verbose_s.lower() in ('true', '1', 'yes') else 'log'

    cfg_path = LaunchConfiguration('config_path').perform(context)
    resolved_ns, _, _ = _resolve_nav2_namespace(
        LaunchConfiguration('nav2_namespace').perform(context), cfg_path
    )
    params_file = LaunchConfiguration('monitor_params_file').perform(context)

    tracked_ship_topic = LaunchConfiguration(
        'monitor_tracked_ship_topic').perform(context).strip()
    global_frame = LaunchConfiguration('monitor_global_frame').perform(context).strip()
    odom_topic = LaunchConfiguration('monitor_odom_topic').perform(context).strip()
    if not odom_topic:
        odom_topic = f'/{resolved_ns}/odom'
    base_frame = LaunchConfiguration('monitor_base_frame').perform(context).strip()
    if not base_frame:
        base_frame = f'{resolved_ns}/base_link'

    return [
        LogInfo(
            msg=(
                '启动 maritime_situation_monitor（tracked_ship_topic='
                f'{tracked_ship_topic}，odom_topic={odom_topic}，'
                f'global_frame={global_frame}，base_frame={base_frame}）'
            )
        ),
        Node(
            package='nav2_maritime_situation_monitor',
            executable='maritime_situation_monitor',
            name='maritime_situation_monitor',
            output=output,
            parameters=[
                {'use_sim_time': use_sim},
                params_file,
                {
                    'tracked_ship_topic': tracked_ship_topic,
                    'odom_topic': odom_topic,
                    'global_frame': global_frame,
                    'base_frame': base_frame,
                    'publish_frequency': float(
                        LaunchConfiguration('monitor_publish_frequency').perform(context)
                    ),
                    'target_timeout': float(
                        LaunchConfiguration('monitor_target_timeout').perform(context)
                    ),
                    'ownship_timeout': float(
                        LaunchConfiguration('monitor_ownship_timeout').perform(context)
                    ),
                    'transform_timeout': float(
                        LaunchConfiguration('monitor_transform_timeout').perform(context)
                    ),
                },
            ],
        ),
    ]


def generate_launch_description():
    usv_sim_full_pkg = get_package_share_directory('usv_sim_full')
    main_launch_file = os.path.join(usv_sim_full_pkg, 'launch', 'main.launch.py')
    colregs_local_stack_file = os.path.join(
        usv_sim_full_pkg, 'launch', 'components', 'colregs_local_stack.launch.py'
    )

    launch_dir = os.path.dirname(os.path.abspath(__file__))

    # 默认参数：COLREGS Local Planner Server 版（share 优先，源码树回退）
    _params_share = os.path.join(
        usv_sim_full_pkg, 'config', 'radar_nav2_param_colregs_local.yaml'
    )
    _params_src = os.path.normpath(os.path.join(
        os.path.dirname(os.path.abspath(__file__)), '..', 'config',
        'radar_nav2_param_colregs_local.yaml'))
    default_nav2_params_file = (
        _params_share if os.path.isfile(_params_share) else _params_src
    )

    cfg_dir = os.path.join(usv_sim_full_pkg, 'config', 'three_vision_one_mmwave')
    default_config_path = os.path.join(cfg_dir, 'full_config.yaml')
    default_sensor_params = os.path.join(cfg_dir, 'ground_truth_sensor_sim_params.yaml')

    fusion_pkg = get_package_share_directory('usv_late_fusion')
    fusion_cfg_dir = os.path.join(fusion_pkg, 'config')
    default_fusion_io = os.path.join(fusion_cfg_dir, 'event_fusion_three_sensor_io.yaml')
    default_fusion_algo = os.path.join(fusion_cfg_dir, 'event_fusion_algorithm.yaml')

    trackship_pkg = get_package_share_directory('convert_to_trackship')
    default_convert_to_trackship_params = os.path.join(
        trackship_pkg, 'config', 'target_snapshot_to_tracked_ship.yaml'
    )

    monitor_pkg = get_package_share_directory('nav2_maritime_situation_monitor')
    default_monitor_params = os.path.join(
        monitor_pkg, 'config', 'maritime_situation_monitor.yaml'
    )

    default_vector_object_params = os.path.join(
        usv_sim_full_pkg, 'config', 'vector_object_server_params.yaml'
    )

    default_control_params_file = os.path.join(
        usv_sim_full_pkg, 'config', 'control_params.yaml'
    )

    default_localization_params = os.path.join(
        usv_sim_full_pkg, 'config', 'robot_localization_gps.yaml'
    )
    default_map_yaml = os.path.join(usv_sim_full_pkg, 'maps', 'sydney_map2.yaml')

    _rviz_name = 'three_vision_one_mmwave_colregs.rviz'
    _rviz_share = os.path.join(usv_sim_full_pkg, 'rviz', _rviz_name)
    _rviz_src = os.path.normpath(
        os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'rviz', _rviz_name)
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
    control_params_file = LaunchConfiguration('control_params_file')
    disable_fastdds_shm = LaunchConfiguration('disable_fastdds_shm')
    gz_headless = LaunchConfiguration('gz_headless')
    map_yaml = LaunchConfiguration('map_yaml')
    verbose_launch = LaunchConfiguration('verbose_launch')
    rviz_config_path = LaunchConfiguration('rviz_config_path')

    def disable_fastdds_shm_env(context, *args, **kwargs):
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
            'colregs_local_stack.launch.py|colregs_local_planner_server|'
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
            'dynamic_ship_manager_node|'
            'nav2_tf_readiness_gate|'
            'sim_vision_node|sim_mmwave_node|late_fusion_node|'
            'target_snapshot_to_tracked_ship|'
            'vector_object_server|keepout_costmap_filter_info_server|lifecycle_manager_keepout_zone|'
            'scenario_ground_truth_node|ground_truth_gazebo_entity|ground_truth_gazebo_models'
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
            'rviz_config_path_override': rviz_config_path,
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

        colregs_stack_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(colregs_local_stack_file),
            launch_arguments={
                'namespace': resolved_ns,
                'params_file': params_file.perform(context),
                'control_params_file': control_params_file.perform(context),
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
                        colregs_stack_launch,
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
                        'Nav2 TF readiness gate 通过，启动 COLREGS Nav2 (namespace='
                        + resolved_ns
                        + ')'
                    )
                ),
                colregs_stack_launch,
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

    def vector_object_server_launch(context, *args, **kwargs):
        if LaunchConfiguration('enable_keepout_filter').perform(context).lower() != 'true':
            return [
                LogInfo(msg='enable_keepout_filter:=false，跳过 vector_object_server。'),
            ]
        params = LaunchConfiguration('vector_object_params_file').perform(context)
        use_sim = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
        return [
            LogInfo(
                msg=(
                    '启动 vector_object_server（静态禁航区，默认空 mask；'
                    f'参数 {params}；service /vector_object_server/add_shapes 可动态注入）'
                )
            ),
            Node(
                package='nav2_colregs_vector_object_server',
                executable='vector_object_server',
                name='vector_object_server',
                output='screen',
                parameters=[params],
            ),
            Node(
                package='nav2_map_server',
                executable='costmap_filter_info_server',
                name='keepout_costmap_filter_info_server',
                output='screen',
                parameters=[params],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_keepout_zone',
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim},
                    {'autostart': True},
                    {'node_names': ['vector_object_server',
                                    'keepout_costmap_filter_info_server']},
                ],
            ),
        ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_path',
            default_value=default_config_path,
            description='Path to three_vision_one_mmwave/full_config.yaml（默认 waypoint 3 目标固定航路）',
        ),
        DeclareLaunchArgument(
            'sensor_params_file',
            default_value=default_sensor_params,
            description='ground_truth_sensor_sim 参数（三视觉 + 前向毫米波）',
        ),
        DeclareLaunchArgument(
            'fusion_io_params_file',
            default_value=default_fusion_io,
            description='usv_late_fusion I/O：event_fusion_three_sensor_io.yaml',
        ),
        DeclareLaunchArgument(
            'fusion_algorithm_params_file',
            default_value=default_fusion_algo,
            description='usv_late_fusion 算法参数',
        ),
        DeclareLaunchArgument(
            'fusion_params_file',
            default_value='',
            description='非空时单文件覆盖 fusion io/algorithm 默认',
        ),
        DeclareLaunchArgument(
            'enable_gt_sensor_sim',
            default_value='true',
            description='false：不启动 ground_truth_sensor_sim',
        ),
        DeclareLaunchArgument(
            'enable_late_fusion',
            default_value='true',
            description='false：不启动 usv_late_fusion',
        ),
        DeclareLaunchArgument(
            'enable_convert_to_trackship',
            default_value='true',
            description='false：不启动 convert_to_trackship（融合快照 → TrackedShipList）',
        ),
        DeclareLaunchArgument(
            'convert_to_trackship_params_file',
            default_value=default_convert_to_trackship_params,
            description='convert_to_trackship 参数（input/output topic 等）',
        ),
        DeclareLaunchArgument(
            'enable_maritime_situation_monitor',
            default_value='true',
            description='false：不启动 maritime_situation_monitor（态势评估报告）',
        ),
        DeclareLaunchArgument(
            'monitor_params_file',
            default_value=default_monitor_params,
            description='maritime_situation_monitor 参数（阈值类参数在此文件配置）',
        ),
        DeclareLaunchArgument(
            'monitor_tracked_ship_topic',
            default_value='/dynamic_ship/tracked_ships',
            description='maritime_situation_monitor 订阅的 TrackedShipList 话题',
        ),
        DeclareLaunchArgument(
            'monitor_odom_topic',
            default_value='',
            description='maritime_situation_monitor 订阅的 odom 话题；空则取 /{nav2_namespace}/odom',
        ),
        DeclareLaunchArgument(
            'monitor_global_frame',
            default_value='map',
            description='maritime_situation_monitor 报告输出坐标系',
        ),
        DeclareLaunchArgument(
            'monitor_base_frame',
            default_value='',
            description='maritime_situation_monitor 本船坐标系；空则取 {nav2_namespace}/base_link',
        ),
        DeclareLaunchArgument(
            'monitor_publish_frequency',
            default_value='0.5',
            description='maritime_situation_monitor 评估发布频率（Hz）',
        ),
        DeclareLaunchArgument(
            'monitor_target_timeout',
            default_value='3.0',
            description='maritime_situation_monitor 目标快照过期阈值（秒）',
        ),
        DeclareLaunchArgument(
            'monitor_ownship_timeout',
            default_value='1.0',
            description='maritime_situation_monitor 本船快照过期阈值（秒）',
        ),
        DeclareLaunchArgument(
            'monitor_transform_timeout',
            default_value='0.2',
            description='maritime_situation_monitor TF 查询超时（秒）',
        ),
        DeclareLaunchArgument(
            'enable_keepout_filter',
            default_value='true',
            description=(
                'true：启动 vector_object_server + keepout costmap filter（默认空 mask，'
                '可经 /vector_object_server/add_shapes 动态注入静态障碍）；false：不启动'
            ),
        ),
        DeclareLaunchArgument(
            'vector_object_params_file',
            default_value=default_vector_object_params,
            description='vector_object_server 参数（默认 config/vector_object_server_params.yaml，空禁航区）',
        ),
        DeclareLaunchArgument(
            'nav2_namespace',
            default_value='auto',
            description=(
                'Nav2 与 cmd_vel→桨 所跟船的 ROS 命名空间，必须与 full_config 中该船 '
                'robot_*.name 完全一致（如 usv_1），以便 TF 帧 {name}/odom、{name}/base_link '
                '与参数文件中 __ROBOT_NS__ 替换一致。'
                'auto 或空：取 robot_1（按 slot 数字排序后的首船）。'
            ),
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_nav2_params_file,
            description=(
                'Nav2 参数文件（默认 config/radar_nav2_param_colregs_local.yaml，'
                'colregs_local_planner_server 替代 planner_server + TS 子系统）'
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
            description='透传 main / nav2 栈：true 时终端详细输出（默认降噪）',
        ),
        DeclareLaunchArgument(
            'rviz_config_path',
            default_value=default_rviz_config,
            description=(
                'RViz 配置文件路径；透传 main.launch rviz_config_path_override。'
                '默认 rviz/three_vision_one_mmwave_colregs.rviz（colregs_costmap/cpa_markers 适配）。'
            ),
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
        LogInfo(msg=['Starting COLREGS local planner bringup from: ', config_path]),
        OpaqueFunction(function=disable_fastdds_shm_env),
        OpaqueFunction(function=prelaunch_cleanup),
        sim_launch,
        OpaqueFunction(function=_three_vision_sensor_sim_nodes),
        OpaqueFunction(function=_late_fusion_node),
        OpaqueFunction(function=_convert_to_trackship_node),
        map_server,
        map_lifecycle_manager,
        Node(
            package='usv_sim_full',
            executable='dynamic_ship_manager_node',
            name='dynamic_ship_manager',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'world_name': 'sydney_regatta',
                'config_base_dir': os.path.join(usv_sim_full_pkg, 'config', 'three_vision_one_mmwave'),
                'default_mesh_profile': os.path.join(
                    usv_sim_full_pkg, 'description', 'models',
                    'target_ship', '10m_mesh_profile.yaml'),
            }],
        ),
        OpaqueFunction(function=_maritime_situation_monitor_node),
        OpaqueFunction(function=vector_object_server_launch),
        OpaqueFunction(function=nav2_readiness_and_deferred_launch),
    ])
