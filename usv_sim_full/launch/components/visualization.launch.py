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

    nav2_namespace_arg = DeclareLaunchArgument(
        'nav2_namespace',
        default_value='',
        description=(
            'Nav2 动作服务所在的命名空间（如 usv_1）。非空时 RViz 整体进入该命名空间运行，'
            '使 Nav2 Panel 的反馈/ETA 显示、状态与动作客户端自动解析到 '
            '/{nav2_namespace}/navigate_to_pose 等（RViz 显示节点不应用 topic remap，'
            '只能用命名空间）；空则 RViz 在根命名空间运行，行为不变。'
        )
    )
    nav2_namespace = LaunchConfiguration('nav2_namespace')

    def launch_rviz(context, *args, **kwargs):
        cfg = rviz_config_path.perform(context)
        v = verbose_launch.perform(context)
        ns = nav2_namespace.perform(context).strip().strip('/')
        kw = quiet_ros_node_kwargs(v, ['-d', cfg])
        return [
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                namespace=ns,
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
        nav2_namespace_arg,
        OpaqueFunction(function=launch_rviz),
    ])
