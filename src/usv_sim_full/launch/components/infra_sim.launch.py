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
        wamv_path = get_package_share_directory('wamv_description')
        wamv_models_path = os.path.join(wamv_path, 'models')  # 直接指向models目录
    except:
        # 如果找不到wamv_description包，使用默认路径
        wamv_models_path = "/home/cczh/USV_ROS/src/USV_Simulation/install/wamv_description/share/wamv_description/models"

    gz_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    # 添加模型路径到Gazebo资源路径
    new_resource_path = f"{wamv_models_path}:{gz_resource_path}" if gz_resource_path else wamv_models_path
    
    set_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=new_resource_path
    )
    
    # 同时设置GAZEBO_MODEL_PATH以兼容旧版本
    gazebo_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    new_model_path = f"{os.path.dirname(wamv_models_path)}:{gazebo_model_path}" if gazebo_model_path else os.path.dirname(wamv_models_path)
    
    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=new_model_path
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
        set_model_path,  # 添加GAZEBO_MODEL_PATH设置
        gazebo_launch,
        global_bridge_node
    ])