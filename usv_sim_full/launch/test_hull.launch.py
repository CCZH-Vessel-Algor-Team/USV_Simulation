"""
******************************************************************************************
*  Copyright (C) 2026 MurphyChen, All Rights Reserved                                  *
*                                                                                        *
*  @brief    船体测试环境 - 用于验证船体参数的简化仿真环境（支持 robot_N 多船）          *
*  @author   MurphyChen                                                                *
*  @version  1.0.0                                                                       *
*  @date     2026.1.21                                                                 *
******************************************************************************************
"""

import os
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml
import subprocess

from usv_sim_full.launch_config_helpers import (
    block_first_maritime_radar,
    block_has_maritime_radar,
    ground_truth_gazebo_visual_enabled,
    launch_verbose_enabled,
    merge_ground_truth_gazebo_entity_params,
    parse_session_json_from_stdout,
    quiet_ros_node_kwargs,
    resolve_ground_truth_user_params_path,
    resolve_session_robots,
    scenario_ground_truth_sim_config,
    session_manager_executable_path,
    ship_config_blocks,
    write_ground_truth_entity_params_yaml,
    write_ground_truth_node_params_yaml,
)


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('usv_sim_full')
    config_path = LaunchConfiguration('config_path').perform(context)
    verbose_s = LaunchConfiguration('verbose_launch').perform(context)

    with open(config_path, 'r') as f:
        user_config = yaml.safe_load(f)

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
                f'[test_hull] session_manager OK: {n_robots} robot(s), '
                f'session_dir={session_info.get("session_path", "?")}'
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
    obstacle_layout_path = session_info.get('obstacle_layout_path', '')
    rviz_config_path = session_info['rviz_config_path']
    # test_hull 固定加载 simple_water.sdf，Gazebo 世界名须与 robot_bringup 的 gz_world_name 一致
    test_hull_gz_world = 'simple_water'

    simple_world_path = os.path.join(pkg_share, 'test_env', 'simple_water.sdf')
    gz_out = 'screen' if launch_verbose_enabled(verbose_s) else 'log'
    gz_sim_process = ExecuteProcess(
        cmd=['gz', 'sim', '-r', simple_world_path],
        output=gz_out
    )

    launch_items = []
    if not launch_verbose_enabled(verbose_s):
        launch_items.append(
            SetEnvironmentVariable(
                name='RCUTILS_LOGGING_SEVERITY',
                value='WARN',
            )
        )
    launch_items.append(gz_sim_process)

    for idx, ship in enumerate(session_robots):
        block = ship_blocks[idx] if idx < len(ship_blocks) else {}
        ship_maritime = block_has_maritime_radar(block)
        radar_sensor_name, radar_output_topic = block_first_maritime_radar(block)
        spawn_pose = ship.get('spawn_pose', [0.0, 0.0, 0.5, 0.0, 0.0, 0.0])
        pe = spawn_pose + [0.0] * (6 - len(spawn_pose))

        bringup = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch/components/robot_bringup.launch.py')
            ),
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
                'gz_world_name': 'simple_water',
                'enable_robot_localization': 'false',
                'localization_params_file': '',
                'localization_start_delay': '5.0',
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
        launch_items.append(bringup)

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
                test_hull_gz_world, scen_gt_cfg, tt, prefix, gz_spawn_delay, gz_svc_wait, config_path
            )
            launch_items.append(
                Node(
                    package='ground_truth_sim',
                    executable='ground_truth_gazebo_entity_node',
                    name='scenario_ground_truth_gazebo_entity',
                    parameters=[{'use_sim_time': True}, ent_gen_path, ent_params],
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

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': True}],
        **quiet_ros_node_kwargs(verbose_s, ['-d', rviz_config_path]),
    )
    launch_items.append(rviz_node)

    return launch_items


def generate_launch_description():
    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value=os.path.join(
            get_package_share_directory('usv_sim_full'),
            'config',
            'full_config.yaml'
        ),
        description='用户配置文件路径'
    )

    return LaunchDescription([
        config_path_arg,
        DeclareLaunchArgument(
            'verbose_launch',
            default_value='false',
            description='true：详细终端输出；false（默认）：降噪'
        ),
        OpaqueFunction(function=launch_setup)
    ])
