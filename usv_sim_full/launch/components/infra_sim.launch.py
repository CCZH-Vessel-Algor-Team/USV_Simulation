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
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from usv_sim_full.launch_config_helpers import quiet_ros_node_kwargs
import os


def generate_launch_description():
    # 声明launch参数
    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='sydney_regatta',
        description='仿真环境名称'
    )

    verbose_launch_arg = DeclareLaunchArgument(
        'verbose_launch',
        default_value='false',
        description='为 true 时打印 GZ 资源路径同步信息，全局桥接节点输出到终端'
    )

    gz_headless_arg = DeclareLaunchArgument(
        'gz_headless',
        default_value='false',
        description='true 时以 server-only 方式启动 Gazebo（不启动 GUI 渲染窗口）'
    )
    
    # 获取launch配置
    world_name = LaunchConfiguration('world_name')
    verbose_launch = LaunchConfiguration('verbose_launch')
    gz_headless = LaunchConfiguration('gz_headless')

    # 设置GZ_SIM_RESOURCE_PATH环境变量，确保能找到模型文件
    usv_sim_path = get_package_share_directory('usv_sim_full')
    usv_worlds_path = os.path.join(usv_sim_path, 'worlds')
    usv_world_models_dir = os.path.join(usv_sim_path, 'worlds', 'models')
    usv_models_path = os.path.join(usv_sim_path, 'description')
    usv_models_dir = os.path.join(usv_sim_path, 'description', 'models')
    
    # 尝试获取 wamv_description 包路径的父目录，以便解决 model://wamv_description/... 报错
    extra_paths = [usv_worlds_path, usv_world_models_dir, usv_models_path, usv_models_dir]
    try:
        wamv_desc_path = get_package_share_directory('wamv_description')
        wamv_share_base = os.path.dirname(wamv_desc_path)
        extra_paths.append(wamv_share_base)
    except Exception:
        pass
    
    # 获取当前环境变量
    gz_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    
    # 构造新的资源路径
    paths_to_add = ":".join(extra_paths)
    if gz_resource_path:
        new_resource_path = f"{paths_to_add}:{gz_resource_path}"
    else:
        new_resource_path = paths_to_add
    
    # 同时设置GAZEBO_MODEL_PATH以兼容旧版本
    gazebo_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    if gazebo_model_path:
        new_model_path = f"{paths_to_add}:{gazebo_model_path}"
    else:
        new_model_path = paths_to_add
    
    # 设置 GZ_SIM_SYSTEM_PLUGIN_PATH 用以加载雷达等外挂系统插件
    gz_plugin_path = os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
    new_plugin_path = gz_plugin_path
    _plugin_setup_error = None
    set_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=gz_plugin_path
    )
    try:
        usv_sim_prefix = get_package_prefix('usv_sim_full')
        radar_plugin_lib_path = os.path.join(os.path.dirname(usv_sim_prefix), 'gz_maritime_radar_plugin', 'lib')
        plugin_paths = [radar_plugin_lib_path]
        try:
            plugin_paths.append(os.path.join(get_package_prefix('usv_mmwave_sim'), 'lib'))
        except Exception:
            pass

        plugin_paths = [p for p in plugin_paths if p and os.path.isdir(p)]
        combined_plugin_paths = ':'.join(plugin_paths)
        if gz_plugin_path:
            new_plugin_path = f"{combined_plugin_paths}:{gz_plugin_path}" if combined_plugin_paths else gz_plugin_path
        else:
            new_plugin_path = combined_plugin_paths
        set_plugin_path = SetEnvironmentVariable(
            name='GZ_SIM_SYSTEM_PLUGIN_PATH',
            value=new_plugin_path
        )
    except Exception as e:
        # 如果找不到包就设空，以避免报错阻断（异常信息仅在 verbose_launch 时打印）
        _plugin_setup_error = e
        set_plugin_path = SetEnvironmentVariable(
            name='GZ_SIM_SYSTEM_PLUGIN_PATH',
            value=gz_plugin_path
        )

    def log_infra_paths_if_verbose(context, *args, **kwargs):
        v = verbose_launch.perform(context).lower()
        if v not in ('true', '1', 'yes'):
            return []
        print(f"Setting GZ_SIM_RESOURCE_PATH to: {new_resource_path}")
        print(f"Setting GAZEBO_MODEL_PATH to: {new_model_path}")
        print(f"Setting GZ_SIM_SYSTEM_PLUGIN_PATH to: {new_plugin_path}")
        if _plugin_setup_error is not None:
            print(f"Could not find gz_maritime_radar_plugin: {_plugin_setup_error}")
        return []
    
    set_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=new_resource_path
    )
    
    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=new_model_path
    )

    # ros_gz_sim 的 gz_sim.launch.py 在 OpaqueFunction 里用 os.environ（而非 launch context）
    # 拼接 additional_env 中的 GZ_SIM_SYSTEM_PLUGIN_PATH，并覆盖子进程环境里的该变量。
    # SetEnvironmentVariable 只写入 context.environment，若不先同步到 os.environ，
    # 上述拼接会以「空的前缀 + LD_LIBRARY_PATH」覆盖掉我们在 context 里设好的插件路径，
    # 导致海事雷达等 .so 无法加载。毫米波主链路已改为 gpu_ray + usv_mmwave_sim 节点；独立验证仍可用包内 Gazebo 插件。
    def sync_gz_system_plugin_path_to_os_environ(context, *args, **kwargs):
        key = 'GZ_SIM_SYSTEM_PLUGIN_PATH'
        if key in context.environment:
            os.environ[key] = context.environment[key]
            v = verbose_launch.perform(context).lower()
            if v in ('true', '1', 'yes'):
                print(f"Synced {key} to os.environ for gz_sim.launch.py (value length={len(os.environ[key])})")
        return []

    # 启动Gazebo仿真 - 使用ros_gz_sim的内置启动文件
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource

    def launch_gazebo_with_selected_world(context, *args, **kwargs):
        selected_world_name = world_name.perform(context)
        headless = gz_headless.perform(context).lower() in ('true', '1', 'yes')
        worlds_dir = os.path.join(usv_sim_path, "worlds")
        world_file_sdf = os.path.join(worlds_dir, f"{selected_world_name}.sdf")
        world_file_world = os.path.join(worlds_dir, f"{selected_world_name}.world")

        if os.path.exists(world_file_sdf):
            world_file = world_file_sdf
        elif os.path.exists(world_file_world):
            world_file = world_file_world
        else:
            world_file = world_file_sdf

        if not os.path.exists(world_file):
            available_worlds = []
            if os.path.isdir(worlds_dir):
                available_worlds = sorted([
                    filename.rsplit('.', 1)[0]
                    for filename in os.listdir(worlds_dir)
                    if filename.endswith('.sdf') or filename.endswith('.world')
                ])
            raise FileNotFoundError(
                f"World not found: {world_file}. Available world_name values: {available_worlds}"
            )

        gz_args = f'-r -s {world_file}' if headless else f'-r {world_file}'
        gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('ros_gz_sim'),
                '/launch/gz_sim.launch.py'
            ]),
            launch_arguments={
                'gz_args': gz_args
            }.items()
        )
        return [gazebo_launch]
    
    def launch_global_bridge(context, *args, **kwargs):
        v = verbose_launch.perform(context)
        kw = quiet_ros_node_kwargs(v)
        return [
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='global_bridge',
                parameters=[{
                    'config_file': os.path.join(get_package_share_directory('usv_sim_full'), 'config', 'global_bridge.yaml')
                }],
                **kw,
            )
        ]

    return LaunchDescription([
        world_name_arg,
        verbose_launch_arg,
        gz_headless_arg,
        set_resource_path,
        set_model_path,  # 添加GAZEBO_MODEL_PATH设置
        set_plugin_path,  # 注册仿真插件路径（launch context）
        OpaqueFunction(function=log_infra_paths_if_verbose),
        OpaqueFunction(function=sync_gz_system_plugin_path_to_os_environ),
        OpaqueFunction(function=launch_gazebo_with_selected_world),
        OpaqueFunction(function=launch_global_bridge),
    ])
