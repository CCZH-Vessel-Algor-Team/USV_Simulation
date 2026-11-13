"""独立启动无 Gazebo 的动态目标船模块。

模块只启动 dynamic_ship_manager_node 的 virtual 后端，发布与仿真一致的
/dynamic_ship/tracked_ships 与 /dynamic_ship/names，不启动 Gazebo、感知、
导航或 RViz。使用时不得同时运行 simulation.launch.py 中的同名 manager。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = ParameterValue(
        LaunchConfiguration('use_sim_time'), value_type=bool)
    publish_markers = ParameterValue(
        LaunchConfiguration('publish_markers'), value_type=bool)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='是否使用 ROS 仿真时钟；实船保持 false。',
        ),
        DeclareLaunchArgument(
            'publish_markers',
            default_value='true',
            description='是否发布 /dynamic_ship/markers 供 RViz 显示虚拟船。',
        ),
        Node(
            package='usv_sim_full',
            executable='dynamic_ship_manager_node',
            name='dynamic_ship_manager',
            output='screen',
            parameters=[{
                'backend': 'virtual',
                'use_sim_time': use_sim_time,
                'publish_markers': publish_markers,
            }],
        ),
    ])
