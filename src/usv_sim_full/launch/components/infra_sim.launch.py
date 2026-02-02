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

    # 设置GZ_SIM_RESOURCE_PATH环境变量，确保能找到模型文件
    usv_sim_path = get_package_share_directory('usv_sim_full')
    usv_models_path = os.path.join(usv_sim_path, 'description')
    
    # 获取当前环境变量
    gz_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    
    # 构造新的资源路径，将我们的模型路径放在最前面
    if gz_resource_path:
        new_resource_path = f"{usv_models_path}:{gz_resource_path}"
    else:
        new_resource_path = usv_models_path
    
    print(f"Setting GZ_SIM_RESOURCE_PATH to: {new_resource_path}")
    
    set_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=new_resource_path
    )
    
    # 同时设置GAZEBO_MODEL_PATH以兼容旧版本
    gazebo_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    if gazebo_model_path:
        new_model_path = f"{usv_models_path}:{gazebo_model_path}"
    else:
        new_model_path = usv_models_path
    
    print(f"Setting GAZEBO_MODEL_PATH to: {new_model_path}")
    
    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=new_model_path
    )
    
    # 启动Gazebo仿真 - 使用ros_gz_sim的内置启动文件
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    
    # 构建世界文件路径，直接使用usv_sim_full包内的worlds目录
    world_file = os.path.join(usv_sim_path, "worlds", "sydney_regatta.sdf")
    
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
        set_model_path,  # 添加GAZEBO_MODEL_PATH设置
        gazebo_launch,
        global_bridge_node
    ])