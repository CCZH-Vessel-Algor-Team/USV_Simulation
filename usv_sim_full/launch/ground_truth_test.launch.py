#!/usr/bin/env python3
"""隔离测试 ground_truth_sim：Gazebo 基础设施 + kinematic 或 Gazebo 实体权威真值。"""

from __future__ import annotations

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from usv_sim_full.launch_config_helpers import (
    ground_truth_gazebo_visual_enabled,
    launch_verbose_enabled,
    merge_ground_truth_gazebo_entity_params,
    quiet_ros_node_kwargs,
    resolve_ground_truth_user_params_path,
    scenario_ground_truth_sim_config,
    write_ground_truth_entity_params_yaml,
    write_ground_truth_node_params_yaml,
)


def launch_setup(context, *args, **kwargs):
    config_path = LaunchConfiguration('config_path').perform(context)
    verbose_s = LaunchConfiguration('verbose_launch').perform(context)
    gz_headless_s = LaunchConfiguration('gz_headless').perform(context)
    use_rviz_s = LaunchConfiguration('use_rviz').perform(context)

    with open(config_path, 'r', encoding='utf-8') as f:
        user_config = yaml.safe_load(f) or {}

    world_name = user_config.get('environment', {}).get('world_name', 'sydney_regatta')
    scen_gt_cfg = scenario_ground_truth_sim_config(user_config)
    if not scen_gt_cfg.get('enabled'):
        raise RuntimeError(
            'ground_truth_test 需要 scenario.ground_truth_sim.enabled: true；'
            f'请检查配置：{config_path}'
        )

    launch_rviz_cfg = user_config.get('visualization', {}).get('launch_rviz', True)
    use_rviz = use_rviz_s.lower() in ('true', '1', 'yes') if use_rviz_s else bool(launch_rviz_cfg)

    usv_pkg = get_package_share_directory('usv_sim_full')
    gt_pkg = get_package_share_directory('ground_truth_sim')
    default_rviz = os.path.join(gt_pkg, 'rviz', 'ground_truth_view.rviz')
    rviz_override = LaunchConfiguration('rviz_config_path').perform(context).strip()
    rviz_config = rviz_override if rviz_override else default_rviz

    launch_items = []
    if not launch_verbose_enabled(verbose_s):
        launch_items.append(
            SetEnvironmentVariable(name='RCUTILS_LOGGING_SEVERITY', value='WARN')
        )

    launch_items.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(usv_pkg, 'launch', 'components', 'infra_sim.launch.py')
            ),
            launch_arguments={
                'world_name': world_name,
                'verbose_launch': verbose_s,
                'gz_headless': gz_headless_s,
            }.items(),
        )
    )

    gz_visual = ground_truth_gazebo_visual_enabled(scen_gt_cfg)
    if gz_visual:
        tt = str(scen_gt_cfg.get('tracks_topic') or 'sim/ground_truth').strip().lstrip('/')
        prefix = str(scen_gt_cfg.get('gazebo_model_prefix') or 'gt_ctrv_').strip() or 'gt_ctrv_'
        gz_spawn_delay = float(scen_gt_cfg.get('spawn_delay_sec', 10.0))
        gz_svc_wait = float(scen_gt_cfg.get('world_service_wait_sec', 1.0))
        fd_ent, ent_gen_path = tempfile.mkstemp(prefix='usv_gt_entity_test_', suffix='.yaml')
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
                parameters=[{'use_sim_time': True}, ent_gen_path, ent_params],
                **quiet_ros_node_kwargs(verbose_s),
            )
        )
    else:
        fd_gt, gt_gen_path = tempfile.mkstemp(prefix='usv_gt_test_', suffix='.yaml')
        os.close(fd_gt)
        write_ground_truth_node_params_yaml(
            scen_gt_cfg,
            gt_gen_path,
            user_config,
            ros_node_name='scenario_ground_truth_node',
        )
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

    if use_rviz:
        launch_items.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='ground_truth_test_rviz',
                parameters=[{'use_sim_time': True}],
                **quiet_ros_node_kwargs(verbose_s, ['-d', rviz_config]),
            )
        )

    print(
        '[ground_truth_test] world=%s target_count=%s gazebo_visual=%s rviz=%s config=%s'
        % (
            world_name,
            scen_gt_cfg.get('target_count', '?'),
            ground_truth_gazebo_visual_enabled(scen_gt_cfg),
            use_rviz,
            config_path,
        )
    )
    return launch_items


def generate_launch_description():
    pkg_share = get_package_share_directory('usv_sim_full')
    default_config = os.path.join(pkg_share, 'config', 'full_config.yaml')
    gt_pkg = get_package_share_directory('ground_truth_sim')
    default_rviz = os.path.join(gt_pkg, 'rviz', 'ground_truth_view.rviz')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'config_path',
                default_value=default_config,
                description='含 scenario.ground_truth_sim.enabled 的 full_config 路径（默认 full_config.yaml）',
            ),
            DeclareLaunchArgument(
                'use_rviz',
                default_value='',
                description='空=跟随 YAML visualization.launch_rviz；true/false 强制覆盖',
            ),
            DeclareLaunchArgument(
                'rviz_config_path',
                default_value=default_rviz,
                description='RViz 配置文件',
            ),
            DeclareLaunchArgument(
                'verbose_launch',
                default_value='false',
                description='true 时节点输出到终端',
            ),
            DeclareLaunchArgument(
                'gz_headless',
                default_value='false',
                description='true 时 Gazebo 无 GUI（便于无显示环境跑计数测试）',
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
