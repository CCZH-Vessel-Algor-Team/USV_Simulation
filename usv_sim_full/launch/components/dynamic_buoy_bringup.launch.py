"""Launch dynamic buoy manager, COLREGS merger, and ground-truth bridges."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _world_name_from_config(config_path: str, default: str = 'sydney_regatta') -> str:
    try:
        with open(config_path, 'r', encoding='utf-8') as f:
            cfg = yaml.safe_load(f) or {}
        return cfg.get('environment', {}).get('world_name', default)
    except Exception:
        return default


def _dynamic_buoy_stack(context, *args, **kwargs):
    enable = LaunchConfiguration('enable_dynamic_buoy_manager').perform(context).strip().lower()
    if enable not in ('true', '1', 'yes'):
        return [LogInfo(msg='enable_dynamic_buoy_manager:=false，跳过动态浮标模块。')]

    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    config_path = LaunchConfiguration('config_path').perform(context).strip()
    enable_gt = LaunchConfiguration('enable_dynamic_buoy_gt_bridge').perform(context).strip().lower()
    remap_ship_tracked = LaunchConfiguration('remap_dynamic_ship_tracked').perform(context).strip().lower()

    world_name = LaunchConfiguration('world_name').perform(context).strip()
    if not world_name and config_path:
        world_name = _world_name_from_config(config_path)

    actions = [
        LogInfo(msg='启动 dynamic_buoy_manager 与 tracked_ship_list_merger。'),
        Node(
            package='usv_sim_full',
            executable='dynamic_buoy_manager_node',
            name='dynamic_buoy_manager',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'world_name': world_name or 'sydney_regatta',
                'frame_id': 'map',
            }],
        ),
        Node(
            package='usv_sim_full',
            executable='tracked_ship_list_merger',
            name='tracked_ship_list_merger',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'input_topics': [
                    '/dynamic_ship/tracked_ships/_internal',
                    '/dynamic_buoy/tracked_ships',
                ],
                'output_topic': '/dynamic_ship/tracked_ships',
                'frame_id': 'map',
            }],
        ),
    ]

    if enable_gt in ('true', '1', 'yes'):
        actions.extend([
            LogInfo(msg='启动 dynamic_buoy_to_ground_truth 与 ground_truth_track_merger。'),
            Node(
                package='ground_truth_sensor_sim',
                executable='dynamic_buoy_to_ground_truth',
                name='dynamic_buoy_to_ground_truth',
                output='log',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'input_topic': '/dynamic_buoy/buoys',
                    'output_topic': '/sim/ground_truth/_src/dynamic_buoys',
                    'frame_id': 'map',
                }],
            ),
            Node(
                package='ground_truth_sensor_sim',
                executable='ground_truth_track_merger',
                name='ground_truth_track_merger',
                output='log',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'input_topics': [
                        '/sim/ground_truth/_src/scenario',
                        '/sim/ground_truth/_src/dynamic_ships',
                        '/sim/ground_truth/_src/dynamic_buoys',
                    ],
                    'output_topic': '/sim/ground_truth',
                    'frame_id': 'map',
                }],
            ),
        ])

    if remap_ship_tracked in ('true', '1', 'yes'):
        actions.insert(0, LogInfo(
            msg='dynamic_ship/tracked_ships 将由 merger 汇总；请 remap dynamic_ship_manager 到 _internal。'
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_dynamic_buoy_manager',
            default_value='true',
            description='true：启动动态浮标管理、COLREGS 合并与 ground_truth 桥接。',
        ),
        DeclareLaunchArgument(
            'enable_dynamic_buoy_gt_bridge',
            default_value='true',
            description='true：启动 BuoyArray -> /sim/ground_truth 合并链。',
        ),
        DeclareLaunchArgument(
            'remap_dynamic_ship_tracked',
            default_value='false',
            description='true：提示 dynamic_ship_manager 发布到 _internal 话题。',
        ),
        DeclareLaunchArgument(
            'config_path',
            default_value='',
            description='仿真 YAML，用于解析 world_name。',
        ),
        DeclareLaunchArgument(
            'world_name',
            default_value='',
            description='Gazebo world 名称；留空时从 config_path 读取。',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='是否使用仿真时钟。',
        ),
        OpaqueFunction(function=_dynamic_buoy_stack),
    ])
