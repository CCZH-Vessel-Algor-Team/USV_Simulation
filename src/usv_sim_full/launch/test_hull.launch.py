"""
******************************************************************************************
*  Copyright (C) 2026 MurphyChen, All Rights Reserved                                  *
*                                                                                        *
*  @brief    船体测试环境 - 用于验证船体参数的简化仿真环境                              *
*  @author   MurphyChen                                                                *
*  @version  1.0.0                                                                       *
*  @date     2026.1.21                                                                 *
******************************************************************************************
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml
import subprocess


def launch_setup(context, *args, **kwargs):
    # 获取包路径
    pkg_share = get_package_share_directory('usv_sim_full')
    
    # 获取用户配置路径
    config_path = LaunchConfiguration('config_path').perform(context)
    
    # 读取用户配置
    with open(config_path, 'r') as f:
        user_config = yaml.safe_load(f)
    
    # 执行会话管理器脚本，获取URDF
    try:
        result = subprocess.run([
            'python3', 
            os.path.join(pkg_share, 'scripts', 'session_manager.py'),
            '--config-path', config_path
        ], capture_output=True, text=True, check=True)
        
        session_output = result.stdout.strip()
        print(f"Full session manager output: '{session_output}'")
        
        # 提取JSON部分
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
        session_info = yaml.safe_load(json_line)  # Using yaml.safe_load as it can handle JSON
        urdf_path = session_info['urdf_path']
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
    robot_name = robot_config.get('name', 'wamv_test')
    spawn_pose = robot_config.get('spawn_pose', [0.0, 0.0, 0.5, 0.0, 0.0, 0.0])

    # 启动Gazebo仿真 - 使用安装后的路径
    simple_world_path = os.path.join(pkg_share, 'test_env', 'simple_water.sdf')
    gz_sim_process = ExecuteProcess(
        cmd=['gz', 'sim', '-r', simple_world_path],
        output='screen'
    )
    
    # 启动机器人专属的传感器桥接节点
    bridge_config_path = session_info['bridge_yaml_path']
    sensor_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='sensor_bridge',
        parameters=[{
            'config_file': bridge_config_path,
            'use_sim_time': True
        }],
        output='screen'
    )
    
    # 启动robot_state_publisher
    from launch.substitutions import Command
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['cat ', urdf_path]),
            'use_sim_time': True
        }],
        output='screen'
    )
    
    # 启动Gazebo实体创建（生成机器人）
    create_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '-name', robot_name,
            '-x', str(spawn_pose[0]),
            '-y', str(spawn_pose[1]),
            '-z', str(spawn_pose[2]),
            '-R', str(spawn_pose[3]),  # roll
            '-P', str(spawn_pose[4]),  # pitch
            '-Y', str(spawn_pose[5]),  # yaw
            '-file', urdf_path
        ]
    )
    
    # 启动RViz2
    rviz_config_path = session_info['rviz_config_path']
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return [
        gz_sim_process,
        sensor_bridge_node,
        robot_state_publisher_node,
        create_entity_node,
        rviz_node
    ]


def generate_launch_description():
    # 声明launch参数
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
        OpaqueFunction(function=launch_setup)
    ])