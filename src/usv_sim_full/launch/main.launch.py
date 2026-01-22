"""
******************************************************************************************
*  Copyright (C) 2026 MurphyChen, All Rights Reserved                                  *
*                                                                                        *
*  @brief    主启动文件 - 组装各个模块                                                 *
*  @author   MurphyChen                                                                *
*  @version  1.0.0                                                                       *
*  @date     2026.1.21                                                                 *
******************************************************************************************
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml
import subprocess
import json


def launch_setup(context, *args, **kwargs):
    # 获取包路径
    pkg_share = get_package_share_directory('usv_sim_full')
    
    # 获取用户配置路径 (通过LaunchConfiguration)
    config_path = LaunchConfiguration('config_path').perform(context)
    
    # 读取用户配置获取世界名称
    with open(config_path, 'r') as f:
        user_config = yaml.safe_load(f)
    
    world_name = user_config.get('environment', {}).get('world_name', 'sydney_regatta')
    
    # 检查是否启用RViz
    launch_rviz = user_config.get('visualization', {}).get('launch_rviz', True)
    
    # 执行会话管理器脚本，获取URDF、桥接配置、RViz配置和障碍物布局路径
    try:
        result = subprocess.run([
            'python3', 
            os.path.join(pkg_share, 'scripts', 'session_manager.py'),
            '--config-path', config_path
        ], capture_output=True, text=True, check=True)
        
        session_output = result.stdout.strip()
        print(f"Full session manager output: '{session_output}'")
        
        # 提取JSON部分 - 从 { 开始到最后
        lines = session_output.split('\n')
        json_line = None
        for line in lines:
            line = line.strip()
            if line.startswith('{') and line.endswith('}'):
                json_line = line
                break
                
        if not json_line:
            raise ValueError(f"No JSON found in output: {session_output}")
        
        # 解析返回的JSON
        session_info = json.loads(json_line)
        urdf_path = session_info['urdf_path']
        bridge_yaml_path = session_info['bridge_yaml_path']
        rviz_config_path = session_info['rviz_config_path']
        obstacle_layout_path = session_info['obstacle_layout_path']
    except subprocess.CalledProcessError as e:
        print(f"Session manager failed with error: {e}")
        print(f"Stdout: {e.stdout}")
        print(f"Stderr: {e.stderr}")
        raise e
    except Exception as e:
        print(f"Error running session manager: {e}")
        raise e

    # 获取机器人配置
    robot_config = user_config.get('robot', {})
    robot_name = robot_config.get('name', 'usv')  # 默认使用'usv'作为机器人名
    spawn_pose = robot_config.get('spawn_pose', [0.0, 0.0, 0.5, 0.0, 0.0, 0.0])
    
    # 包含基础设施仿真
    infra_sim_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('usv_sim_full'),
            '/launch/components/infra_sim.launch.py'
        ]),
        launch_arguments={
            'world_name': world_name
        }.items()
    )
    
    # 启动机器人系统（状态发布、实体创建、传感器桥接）
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('usv_sim_full'),
            '/launch/components/robot_bringup.launch.py'
        ]),
        launch_arguments={
            'robot_name': robot_name,
            'urdf_path': urdf_path,
            'bridge_config_path': bridge_yaml_path,
            'obstacle_layout_path': obstacle_layout_path,
            'x': str(spawn_pose[0]),
            'y': str(spawn_pose[1]),
            'z': str(spawn_pose[2]),
            'R': str(spawn_pose[3]),
            'P': str(spawn_pose[4]),
            'Y': str(spawn_pose[5]),
            'use_sim_time': 'true'
        }.items()
    )
    
    # 启动可视化（RViz）
    viz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('usv_sim_full'),
            '/launch/components/visualization.launch.py'
        ]),
        launch_arguments={
            'rviz_config_path': rviz_config_path
        }.items()
    )
    
    # 返回所有启动项
    return [
        infra_sim_include,
        robot_launch,
        viz_launch
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])