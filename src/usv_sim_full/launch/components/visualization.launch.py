"""
******************************************************************************************
*  Copyright (C) 2026 MurphyChen, All Rights Reserved                                  *
*                                                                                        *
*  @brief    可视化组件 - 负责启动RViz2                                               *
*  @author   MurphyChen                                                                *
*  @version  1.0.0                                                                       *
*  @date     2026.1.21                                                                 *
******************************************************************************************
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 声明launch参数
    rviz_config_path_arg = DeclareLaunchArgument(
        'rviz_config_path',
        default_value='',
        description='RViz配置文件的绝对路径'
    )
    
    # 获取launch配置
    rviz_config_path = LaunchConfiguration('rviz_config_path')

    # 启动RViz2节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        rviz_config_path_arg,
        rviz_node
    ])