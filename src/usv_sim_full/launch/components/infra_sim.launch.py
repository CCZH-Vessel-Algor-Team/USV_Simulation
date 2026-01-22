"""
******************************************************************************************
*  Copyright (C) 2026 MurphyChen, All Rights Reserved                                  *
*                                                                                        *
*  @brief    基础设施仿真 - 仅负责环境基础设施                                         *
*  @author   MurphyChen                                                                *
*  @version  1.0.0                                                                       *
*  @date     2026.1.21                                                                 *
******************************************************************************************
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 声明launch参数
    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='sydney_regatta',
        description='仿真环境名称'
    )
    
    # 获取launch配置
    world_name = LaunchConfiguration('world_name')

    # 设置GZ_SIM_RESOURCE_PATH环境变量，确保能找到wamv_description
    try:
        wamv_path = get_package_share_directory('wamv_gazebo')
        wamv_parent_path = os.path.dirname(os.path.dirname(wamv_path))  # 获取install目录
    except:
        # 如果找不到wamv_description包，使用默认路径
        wamv_parent_path = "/home/cczh/simulation/vrx_ws/install"

    gz_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    new_resource_path = f"{wamv_parent_path}:{gz_resource_path}" if gz_resource_path else wamv_parent_path
    
    set_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=new_resource_path
    )
    
    # 启动Gazebo仿真 - 使用ros_gz_sim的内置启动文件
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    
    # 获取vrx_gz路径
    try:
        vrx_gz_path = get_package_share_directory('vrx_gz')
    except:
        vrx_gz_path = "/home/cczh/simulation/vrx_ws/install/vrx_gz/share/vrx_gz"
    
    # 构建世界文件路径
    world_file = os.path.join(vrx_gz_path, "worlds", "sydney_regatta.sdf")
    
    # 使用ros_gz_sim的gz_sim.launch.py启动Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('ros_gz_sim'),
            '/launch/gz_sim.launch.py'
        ]),
        launch_arguments={
            'gz_args': f'-r {world_file}'
        }.items()
    )
    
    # 启动全局桥接节点（仅包含/clock和系统控制话题）
    global_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='global_bridge',
        parameters=[{
            'config_file': os.path.join(get_package_share_directory('usv_sim_full'), 'config', 'global_bridge.yaml')
        }],
        output='screen'
    )

    return LaunchDescription([
        world_name_arg,
        set_resource_path,
        gazebo_launch,
        global_bridge_node
    ])