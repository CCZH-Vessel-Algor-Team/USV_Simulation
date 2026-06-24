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
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from usv_sim_full.launch_config_helpers import quiet_ros_node_kwargs


def generate_launch_description():
    rviz_config_path_arg = DeclareLaunchArgument(
        'rviz_config_path',
        default_value='',
        description='RViz配置文件的绝对路径'
    )

    verbose_launch_arg = DeclareLaunchArgument(
        'verbose_launch',
        default_value='false',
        description='为 true 时 RViz 输出到终端（默认写入 ~/.ros/log）'
    )

    rviz_config_path = LaunchConfiguration('rviz_config_path')
    verbose_launch = LaunchConfiguration('verbose_launch')

    def launch_rviz(context, *args, **kwargs):
        cfg = rviz_config_path.perform(context)
        v = verbose_launch.perform(context)
        kw = quiet_ros_node_kwargs(v, ['-d', cfg])
        return [
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                # **kw,
                arguments=['-d', LaunchConfiguration('rviz_config_path')],
                parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
                }],
                output='screen'
            ),
        ]

    return LaunchDescription([
        rviz_config_path_arg,
        verbose_launch_arg,
        OpaqueFunction(function=launch_rviz),
    ])
