"""
******************************************************************************************
*  Copyright (C) 2026 MurphyChen, All Rights Reserved                                  *
*                                                                                        *
*  @brief    全量仿真主入口 — session_manager 多船、毫米波/海事雷达、按船环境动力学       *
*  @author   MurphyChen                                                                *
*  @version  1.0.0                                                                       *
*  @date     2026.1.21                                                                 *
******************************************************************************************
"""

import os
import re
import shutil
import subprocess
import tempfile
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from usv_sim_full.launch_config_helpers import (
    block_first_maritime_radar,
    block_has_maritime_radar,
    build_mmwave_4d_cloud_parameters,
    build_mmwave_cluster_parameters,
    iter_all_block_sensors,
    load_mmwave_cluster_defaults,
    load_mmwave_sensor_defaults,
    launch_verbose_enabled,
    mmwave_bridge_topics,
    mmwave_cluster_topics,
    parse_session_json_from_stdout,
    ground_truth_gazebo_visual_enabled,
    merge_ground_truth_gazebo_entity_params,
    merge_ground_truth_gazebo_models_params,
    primary_robot_name,
    quiet_ros_node_kwargs,
    resolve_ground_truth_user_params_path,
    resolve_session_robots,
    scenario_ground_truth_sim_config,
    session_manager_executable_path,
    ship_config_blocks,
    write_ground_truth_entity_params_yaml,
    write_ground_truth_node_params_yaml,
)


def _ensure_list(x):
    if x is None:
        return []
    if isinstance(x, list):
        return x
    return [x]


def _append_displays_to_rviz_config(rviz_config_path: str, primary_name: str) -> None:
    """
    将额外 Display 安全地插入 session.rviz（YAML）中，而不是粗暴追加文本。
    这样可避免把 Display 块写到 Window Geometry 后导致 RViz 无法加载。
    """
    try:
        with open(rviz_config_path, 'r', encoding='utf-8') as f:
            rviz_cfg = yaml.safe_load(f) or {}
    except Exception as e:
        print(f"Warning: Could not read rviz config as YAML ({rviz_config_path}): {e}")
        return

    if not isinstance(rviz_cfg, dict):
        print(f"Warning: rviz config is not a YAML mapping: {rviz_config_path}")
        return

    vm = rviz_cfg.get('Visualization Manager')
    if not isinstance(vm, dict):
        vm = {}
        rviz_cfg['Visualization Manager'] = vm

    displays = vm.get('Displays')
    if not isinstance(displays, list):
        displays = _ensure_list(displays)
        vm['Displays'] = displays

    def has_display(name: str) -> bool:
        for d in displays:
            if isinstance(d, dict) and d.get('Name') == name:
                return True
        return False

    # Radar occupancy grid
    if not has_display('Radar OccupancyGrid'):
        displays.append({
            'Class': 'rviz_default_plugins/Map',
            'Name': 'Radar OccupancyGrid',
            'Topic': {
                'Value': f'/{primary_name}/map/navradar/occupancy_grid',
                'Depth': 5,
            },
            'Update Topic': {
                'Value': f'/{primary_name}/map/navradar/occupancy_grid_updates',
                'Depth': 5,
            },
            'Enabled': True,
            'Color Scheme': 'map',
            'Draw Behind': False,
            'Use Timestamp': False,
        })

    # Front camera (a simpler, always-on view separate from the per-sensor Image displays)
    if not has_display('Front Camera'):
        displays.append({
            'Class': 'rviz_default_plugins/Image',
            'Name': 'Front Camera',
            'Topic': {
                'Value': f'/{primary_name}/sensors/camera/front_cam/image_raw',
                'Depth': 5,
            },
            'Enabled': True,
            'Normalize Range': False,
        })

    try:
        with open(rviz_config_path, 'w', encoding='utf-8') as f:
            yaml.dump(
                rviz_cfg,
                f,
                default_flow_style=False,
                indent=2,
                sort_keys=False,
                allow_unicode=True,
            )
    except Exception as e:
        print(f"Warning: Could not write updated rviz config ({rviz_config_path}): {e}")


def launch_setup(context, *args, **kwargs):
    config_path = LaunchConfiguration('config_path').perform(context)
    verbose_s = LaunchConfiguration('verbose_launch').perform(context)
    gz_headless_s = LaunchConfiguration('gz_headless').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_robot_localization = LaunchConfiguration('enable_robot_localization').perform(
        context
    )
    localization_params_file = LaunchConfiguration('localization_params_file').perform(
        context
    )
    localization_start_delay = LaunchConfiguration('localization_start_delay').perform(
        context
    )

    with open(config_path, 'r') as f:
        user_config = yaml.safe_load(f)

    raw_nav2_namespace = LaunchConfiguration('nav2_namespace').perform(context).strip()
    if raw_nav2_namespace in ('', 'auto'):
        raw_nav2_namespace = primary_robot_name(user_config)
    nav2_namespace = raw_nav2_namespace

    world_name = user_config.get('environment', {}).get('world_name', 'sydney_regatta')
    launch_rviz = user_config.get('visualization', {}).get('launch_rviz', True)

    radar_processing_enabled = False
    mmwave_enabled = False
    for sensor in iter_all_block_sensors(user_config):
        if not sensor.get('enabled', True):
            continue
        st = str(sensor.get('type', '')).lower()
        if st in ('maritime_radar', 'radar'):
            radar_processing_enabled = True
        if st in ('mmwave_radar', 'mmwave'):
            mmwave_enabled = True

    try:
        sm_cmd = [
            session_manager_executable_path(),
            '--config-path', config_path,
        ]
        if launch_verbose_enabled(verbose_s):
            sm_cmd.append('--verbose')
        result = subprocess.run(sm_cmd, capture_output=True, text=True, check=True)

        session_output = result.stdout.strip()
        session_info = parse_session_json_from_stdout(session_output)
        if launch_verbose_enabled(verbose_s):
            print(f"Full session manager output: '{session_output}'")
        else:
            n_robots = len(session_info.get('robots') or [])
            print(
                f'[usv_sim_full] session_manager OK: {n_robots} robot(s), '
                f'session_dir={session_info.get("session_path", "?")}'
            )
        rviz_config_path = session_info['rviz_config_path']
        obstacle_layout_path = session_info['obstacle_layout_path']

        # 使用外部底稿（如 default.rviz）时不再向文件末尾追加 Display：追加写在
        # Window Geometry 之后会破坏 YAML，RViz 可能空白/异常；default.rviz 已含 Nav2/相机等。
        rviz_skip_session_append = False
        rviz_override = LaunchConfiguration('rviz_config_path_override').perform(context).strip()
        if rviz_override:
            if os.path.isfile(rviz_override):
                fd, tmp_rviz = tempfile.mkstemp(prefix='usv_rviz_', suffix='.rviz')
                os.close(fd)
                shutil.copy2(rviz_override, tmp_rviz)
                rviz_config_path = tmp_rviz
                rviz_skip_session_append = True
                print(
                    '[usv_sim_full] RViz: using rviz_config_path_override -> '
                    f'{tmp_rviz} (source {rviz_override})'
                )
            else:
                print(
                    f"Warning: rviz_config_path_override not found ({rviz_override!r}); "
                    'using session_manager RViz config.'
                )
    except subprocess.CalledProcessError as e:
        print(f"Session manager failed with error: {e}")
        print(f"Stdout: {e.stdout}")
        print(f"Stderr: {e.stderr}")
        raise e
    except Exception as e:
        print(f"Error running session manager: {e}")
        raise e

    session_robots = resolve_session_robots(session_info, user_config)
    ship_blocks = ship_config_blocks(user_config)
    primary_name = session_robots[0]['name']

    # --- 追加雷达地图与相机显示（仅 session.rviz 结构；见 rviz_skip_session_append） ---
    if not rviz_skip_session_append:
        try:
            _append_displays_to_rviz_config(rviz_config_path, primary_name)
        except Exception as e:
            print(f"Warning: Could not modify rviz config with radar displays: {e}")

    infra_sim_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('usv_sim_full'),
            '/launch/components/infra_sim.launch.py'
        ]),
        launch_arguments={
            'world_name': world_name,
            'verbose_launch': verbose_s,
            'gz_headless': gz_headless_s,
        }.items()
    )

    launch_items = []
    if not launch_verbose_enabled(verbose_s):
        launch_items.append(
            SetEnvironmentVariable(
                name='RCUTILS_LOGGING_SEVERITY',
                value='WARN',
            )
        )
    launch_items.append(infra_sim_include)

    # session.rviz 的 Fixed Frame 为 map；发布 map->{robot}/odom，否则点云/相机等无法变换到 map
    for ship in session_robots:
        rname = ship['name']
        sanitized = re.sub(r"[^A-Za-z0-9_\-]", '_', str(rname))
        launch_items.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'map_to_odom_tf_{sanitized}',
                # use_sim_time 须与 Gazebo /clock 一致，否则 TF 墙钟戳与 Nav2 仿真时刻不一致
                parameters=[{'use_sim_time': True}],
                condition=IfCondition(LaunchConfiguration('use_static_map_odom_tf')),
                **quiet_ros_node_kwargs(
                    verbose_s,
                    [
                        '--frame-id',
                        'map',
                        '--child-frame-id',
                        f'{sanitized}/odom',
                    ],
                ),
            )
        )

    for idx, ship in enumerate(session_robots):
        block = ship_blocks[idx] if idx < len(ship_blocks) else {}
        ship_maritime = block_has_maritime_radar(block)
        radar_sensor_name, radar_output_topic = block_first_maritime_radar(block)
        spawn_pose = ship.get('spawn_pose', [0.0, 0.0, 0.5, 0.0, 0.0, 0.0])
        pe = spawn_pose + [0.0] * (6 - len(spawn_pose))

        robot_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('usv_sim_full'),
                '/launch/components/robot_bringup.launch.py'
            ]),
            launch_arguments={
                'robot_name': ship['name'],
                'urdf_path': ship['urdf_path'],
                'bridge_config_path': ship['bridge_yaml_path'],
                'obstacle_layout_path': obstacle_layout_path,
                'radar_sensor_name': radar_sensor_name,
                'radar_ros_topic': radar_output_topic,
                'enable_maritime_radar_bridge': (
                    'true' if ship_maritime else 'false'
                ),
                'enable_obstacle_spawner': 'true' if idx == 0 else 'false',
                'spawn_stagger_sec': str(float(idx) * 2.0),
                'gz_world_name': world_name,
                'enable_robot_localization': enable_robot_localization,
                'localization_params_file': localization_params_file,
                'localization_start_delay': localization_start_delay,
                'x': str(pe[0]),
                'y': str(pe[1]),
                'z': str(pe[2]),
                'R': str(pe[3]),
                'P': str(pe[4]),
                'Y': str(pe[5]),
                'use_sim_time': 'true',
                'verbose_launch': verbose_s,
            }.items()
        )
        launch_items.append(robot_launch)

    enable_tf_relay_s = LaunchConfiguration('enable_tf_namespace_relay').perform(
        context
    )
    if enable_tf_relay_s.lower() == 'true':
        for ship in session_robots:
            rname = ship['name']
            sanitized = re.sub(r"[^A-Za-z0-9_\-]", '_', str(rname))
            launch_items.append(
                Node(
                    package='usv_sim_full',
                    executable='tf_namespace_relay',
                    name=f'tf_namespace_relay_{sanitized}',
                    namespace=sanitized,
                    parameters=[{
                        'namespace': rname,
                        'use_sim_time': True,
                    }],
                    **quiet_ros_node_kwargs(verbose_s),
                )
            )

    scen_gt_cfg = scenario_ground_truth_sim_config(user_config)
    if scen_gt_cfg.get('enabled'):
        gz_visual = ground_truth_gazebo_visual_enabled(scen_gt_cfg)
        if gz_visual:
            tt = str(scen_gt_cfg.get('tracks_topic') or 'sim/ground_truth').strip().lstrip('/')
            prefix = str(scen_gt_cfg.get('gazebo_model_prefix') or 'gt_ctrv_').strip() or 'gt_ctrv_'
            gz_spawn_delay = float(scen_gt_cfg.get('spawn_delay_sec', 10.0))
            gz_svc_wait = float(scen_gt_cfg.get('world_service_wait_sec', 1.0))
            fd_ent, ent_gen_path = tempfile.mkstemp(prefix='usv_scenario_gt_entity_', suffix='.yaml')
            os.close(fd_ent)
            write_ground_truth_entity_params_yaml(
                scen_gt_cfg,
                ent_gen_path,
                full_config_path=config_path,
                ros_node_name='scenario_ground_truth_gazebo_entity',
            )
            ent_params = merge_ground_truth_gazebo_entity_params(
                world_name, scen_gt_cfg, tt, prefix, gz_spawn_delay, gz_svc_wait, config_path
            )
            launch_items.append(
                Node(
                    package='ground_truth_sim',
                    executable='ground_truth_gazebo_entity_node',
                    name='scenario_ground_truth_gazebo_entity',
                    parameters=[
                        {'use_sim_time': True},
                        ent_gen_path,
                        ent_params,
                    ],
                    **quiet_ros_node_kwargs(verbose_s),
                )
            )
        else:
            fd_gt, gt_gen_path = tempfile.mkstemp(prefix='usv_scenario_gt_', suffix='.yaml')
            os.close(fd_gt)
            write_ground_truth_node_params_yaml(scen_gt_cfg, gt_gen_path, user_config)
            gt_user_path = resolve_ground_truth_user_params_path(
                config_path, scen_gt_cfg.get('params_file')
            )
            gt_params = [{'use_sim_time': True}, gt_gen_path]
            if gt_user_path:
                gt_params.append(gt_user_path)
            launch_items.append(
                Node(
                    package='ground_truth_sim',
                    executable='ground_truth_node',
                    name='scenario_ground_truth_node',
                    parameters=gt_params,
                    **quiet_ros_node_kwargs(verbose_s),
                )
            )

    mm_defs = load_mmwave_sensor_defaults(config_path, user_config)
    mm_cluster_defs = load_mmwave_cluster_defaults(config_path, user_config)
    if mmwave_enabled:
        for idx, ship in enumerate(session_robots):
            block = ship_blocks[idx] if idx < len(ship_blocks) else {}
            sanitized = ship.get('sanitized_name') or re.sub(
                r"[^A-Za-z0-9_\-]", '_', str(ship['name'])
            )
            for sensor in block.get('sensors') or []:
                if not sensor.get('enabled', True):
                    continue
                pair = mmwave_bridge_topics(sensor, sanitized)
                if pair is None:
                    continue
                input_topic, output_topic = pair
                sname = sensor.get('name', 'mmwave')
                safe = re.sub(r'[^a-zA-Z0-9_]', '_', f'{sanitized}_{sname}')
                launch_items.append(
                    Node(
                        package='usv_mmwave_sim',
                        executable='mmwave_4d_cloud_node',
                        name=f'mmwave_4d_{safe}',
                        parameters=[
                            build_mmwave_4d_cloud_parameters(
                                mm_defs,
                                input_topic,
                                output_topic,
                                f'/{sanitized}/odom',
                            )
                        ],
                        **quiet_ros_node_kwargs(verbose_s),
                    )
                )
                cluster_pair = mmwave_cluster_topics(sensor, sanitized)
                if cluster_pair is None:
                    continue
                cluster_in, cluster_out = cluster_pair
                launch_items.append(
                    Node(
                        package='usv_mmwave_sim',
                        executable='mmwave_cluster_node',
                        name=f'mmwave_cluster_{safe}',
                        parameters=[
                            build_mmwave_cluster_parameters(
                                mm_cluster_defs,
                                cluster_in,
                                cluster_out,
                                sname,
                            )
                        ],
                        **quiet_ros_node_kwargs(verbose_s),
                    )
                )

    viz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('usv_sim_full'),
            '/launch/components/visualization.launch.py'
        ]),
        launch_arguments={
            'rviz_config_path': rviz_config_path,
            'verbose_launch': verbose_s,
            'use_sim_time': use_sim_time,
            'nav2_namespace': nav2_namespace,
        }.items()
    )

    if launch_rviz:
        launch_items.append(viz_launch)

    scenario_manager_node = Node(
        package="usv_sim_full",
        executable="scenario_manager_node",
        name="scenario_manager_node",
        parameters=[{
            "config_path": config_path
        }],
        **quiet_ros_node_kwargs(verbose_s),
    )
    launch_items.append(scenario_manager_node)

    for ship in session_robots:
        rname = ship['name']
        sanitized = re.sub(r"[^A-Za-z0-9_\-]", '_', str(rname))
        wrapper_node = Node(
            package='usv_sim_full',
            executable='usv_sim_wrapper',
            name=f'usv_sim_wrapper_{sanitized}',
            namespace=rname,
            parameters=[{
                'odom_topic': f'/{sanitized}/odom',
                'gps_topic': f'/{sanitized}/sensors/gps/gps_sensor/data'
            }],
            remappings=[
                ('/usv/state/vessel', f'/{sanitized}/state/vessel'),
            ],
            **quiet_ros_node_kwargs(verbose_s),
        )
        launch_items.append(wrapper_node)

    if radar_processing_enabled:
        for idx, ship in enumerate(session_robots):
            block = ship_blocks[idx] if idx < len(ship_blocks) else {}
            if not block_has_maritime_radar(block):
                continue
            robot_name = ship['name']
            radar_sensor_name, radar_output_topic = block_first_maritime_radar(block)
            mapping_input_topic = radar_output_topic
            if not mapping_input_topic.startswith('/'):
                mapping_input_topic = '/' + mapping_input_topic
            if not mapping_input_topic.startswith(f'/{robot_name}/'):
                mapping_input_topic = f'/{robot_name}{mapping_input_topic}'
            radar_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    get_package_share_directory('gy_radar_driver'),
                    '/launch/radar_controller.launch.py'
                ]),
                launch_arguments={
                    'namespace': robot_name,
                    'enable_control': 'false',
                    'enable_data': 'false',
                    'enable_arpa': 'false',
                    'enable_tf': 'false',
                    'enable_mapping': 'true',
                    'enable_converter': 'true',
                    'mapping_input_topic': mapping_input_topic,
                    'converter_input_topic': mapping_input_topic,
                    'converter_output_topic': f'/{robot_name}/sensors/radar/nav/points',
                    'mapping_output_gridmap_topic': f'/{robot_name}/map/navradar/gridmap',
                    'mapping_output_occupancy_topic': (
                        f'/{robot_name}/map/navradar/occupancy_grid'
                    ),
                    'use_sim_time': 'true'
                }.items()
            )
            launch_items.append(radar_launch)

    return launch_items


def generate_launch_description():
    pkg_share = get_package_share_directory('usv_sim_full')
    default_config_path = os.path.join(
        pkg_share, 'config', 'three_vision_one_mmwave', 'full_config.yaml'
    )
    default_localization_params = os.path.join(
        pkg_share, 'config', 'robot_localization_gps.yaml'
    )
    default_rviz_config_path = os.path.join(
        pkg_share, 'rviz', 'three_vision_one_mmwave.rviz'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_path',
            default_value=default_config_path,
            description='Path to the full_config.yaml file'
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),

        DeclareLaunchArgument(
            'nav2_namespace',
            default_value='',
            description='Nav2 动作服务命名空间；非空时透传给 visualization.launch.py 供 RViz remap'
        ),

        DeclareLaunchArgument(
            'enable_robot_localization',
            default_value='false',
            description='各船 robot_bringup 是否启用 robot_localization（EKF + navsat）'
        ),
        DeclareLaunchArgument(
            'localization_params_file',
            default_value=default_localization_params,
            description='robot_localization 参数文件路径'
        ),
        DeclareLaunchArgument(
            'localization_start_delay',
            default_value='5.0',
            description='robot_localization 节点启动延时（秒）'
        ),
        DeclareLaunchArgument(
            'use_static_map_odom_tf',
            default_value='true',
            description='发布静态 map->{robot}/odom（与 session RViz Fixed Frame=map 配套；多船各一条）'
        ),
        DeclareLaunchArgument(
            'enable_tf_namespace_relay',
            default_value='true',
            description=(
                '为每条船启动 tf_namespace_relay（/tf -> /{robot}/tf），'
                '供命名空间 Nav2 使用；nav2_sim_full_bringup 默认开启'
            ),
        ),
        DeclareLaunchArgument(
            'rviz_config_path_override',
            default_value=default_rviz_config_path,
            description=(
                'RViz 配置底稿；默认使用 rviz/three_vision_one_mmwave.rviz。'
                '非空时不使用 session_manager 生成的 session.rviz。'
            ),
        ),
        DeclareLaunchArgument(
            'verbose_launch',
            default_value='false',
            description=(
                'true：各节点输出到终端并保留 session_manager/infra 详细 print 与 INFO 日志；'
                'false（默认）：降噪（桥接/RViz 等写入 ~/.ros/log，RCUTILS 默认 WARN）'
            ),
        ),
        DeclareLaunchArgument(
            'gz_headless',
            default_value='false',
            description='true 时 Gazebo 以 server-only 运行，不启动 GUI 渲染窗口'
        ),
        OpaqueFunction(function=launch_setup)
    ])
