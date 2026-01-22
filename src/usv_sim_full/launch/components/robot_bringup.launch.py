"""
******************************************************************************************
*  Copyright (C) 2026 MurphyChen, All Rights Reserved                                  *
*                                                                                        *
*  @brief    单体机器人容器 - 封装所有与特定机器人相关的节点                            *
*  @author   MurphyChen                                                                *
*  @version  1.0.0                                                                       *
*  @date     2026.1.21                                                                 *
******************************************************************************************
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
import os


def generate_launch_description():
    # 声明launch参数
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='wamv',
        description='机器人名称'
    )
    
    urdf_path_arg = DeclareLaunchArgument(
        'urdf_path',
        default_value='',
        description='编译好的URDF文件绝对路径'
    )
    
    bridge_config_path_arg = DeclareLaunchArgument(
        'bridge_config_path',
        default_value='',
        description='传感器桥接配置文件的绝对路径'
    )
    
    obstacle_layout_path_arg = DeclareLaunchArgument(
        'obstacle_layout_path',
        default_value='',
        description='障碍物布局文件的绝对路径'
    )
    
    x_arg = DeclareLaunchArgument(
        'x', default_value='0.0',
        description='初始X坐标'
    )
    
    y_arg = DeclareLaunchArgument(
        'y', default_value='0.0',
        description='初始Y坐标'
    )
    
    z_arg = DeclareLaunchArgument(
        'z', default_value='0.5',
        description='初始Z坐标'
    )
    
    roll_arg = DeclareLaunchArgument(
        'R', default_value='0.0',
        description='初始Roll角'
    )
    
    pitch_arg = DeclareLaunchArgument(
        'P', default_value='0.0',
        description='初始Pitch角'
    )
    
    yaw_arg = DeclareLaunchArgument(
        'Y', default_value='0.0',
        description='初始Yaw角'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='是否使用仿真时间'
    )

    # 获取launch配置
    robot_name = LaunchConfiguration('robot_name')
    urdf_path = LaunchConfiguration('urdf_path')
    bridge_config_path = LaunchConfiguration('bridge_config_path')
    obstacle_layout_path = LaunchConfiguration('obstacle_layout_path')
    x_pose = LaunchConfiguration('x')
    y_pose = LaunchConfiguration('y')
    z_pose = LaunchConfiguration('z')
    roll = LaunchConfiguration('R')
    pitch = LaunchConfiguration('P')
    yaw = LaunchConfiguration('Y')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 启动robot_state_publisher - 使用Command substitution来读取URDF文件
    from launch.substitutions import Command
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['cat ', urdf_path]),
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )
    
    # 启动Gazebo实体创建（生成机器人）- 使用ros_gz_sim的create节点
    create_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-name', robot_name,
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-R', roll,
            '-P', pitch,
            '-Y', yaw,
            '-file', urdf_path
        ]
    )
    
    # 启动机器人专属的传感器桥接节点
    sensor_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='sensor_bridge',
        parameters=[{
            'config_file': bridge_config_path,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )
    
    # 启动障碍物生成器节点（如果提供了有效路径）
    # 使用ExecuteProcess来运行通过console_scripts安装的脚本
    # 需要使用OpaqueFunction来正确处理LaunchConfiguration
    from launch.actions import OpaqueFunction
    def launch_obstacle_spawner(context, *args, **kwargs):
        # 获取实际的障碍物布局路径
        actual_path = obstacle_layout_path.perform(context)
        
        if actual_path and actual_path.strip():  # 如果路径不为空
            obstacle_spawner_process = ExecuteProcess(
                cmd=['obstacle_spawner', actual_path],
                output='screen'
            )
            return [obstacle_spawner_process]
        else:
            # 如果路径为空，不启动任何东西
            return []

    # TODO: 在此处插入路径规划 (Nav2) 和定位 (EKF) 节点
    # 示例预留位置:
    # nav2_nodes = IncludeLaunchDescription(...)  # Nav2导航栈
    # ekf_localization_node = Node(...)         # EKF定位节点

    return LaunchDescription([
        robot_name_arg,
        urdf_path_arg,
        bridge_config_path_arg,
        obstacle_layout_path_arg,
        x_arg,
        y_arg,
        z_arg,
        roll_arg,
        pitch_arg,
        yaw_arg,
        use_sim_time_arg,
        robot_state_publisher_node,  # 启动state publisher
        create_entity_node,          # 创建实体
        sensor_bridge_node,          # 传感器桥接
        OpaqueFunction(function=launch_obstacle_spawner)  # 障碍物生成器（条件启动）
    ])