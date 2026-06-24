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
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction, TimerAction
from launch_ros.descriptions import ParameterValue
from usv_sim_full.launch_config_helpers import quiet_ros_node_kwargs
import os
import re


def _convert_model_uri_to_package_uri(urdf_content):
    """将 Gazebo 专用的 model:// URI 转换为 RViz 可识别的 package:// URI。"""
    from ament_index_python.packages import get_package_share_directory

    model_to_pkg = {}

    try:
        wamv_desc_share = get_package_share_directory('wamv_description')
        wamv_models_dir = os.path.join(wamv_desc_share, 'models')
        if os.path.isdir(wamv_models_dir):
            for name in os.listdir(wamv_models_dir):
                if os.path.isdir(os.path.join(wamv_models_dir, name)):
                    model_to_pkg[name] = f'package://wamv_description/models/{name}/'
    except Exception:
        pass

    try:
        usv_sim_share = get_package_share_directory('usv_sim_full')
        usv_models_dir = os.path.join(usv_sim_share, 'description', 'models')
        if os.path.isdir(usv_models_dir):
            for name in os.listdir(usv_models_dir):
                if os.path.isdir(os.path.join(usv_models_dir, name)):
                    model_to_pkg[name] = f'package://usv_sim_full/description/models/{name}/'
    except Exception:
        pass

    def replace_model_uri(match):
        model_name = match.group(1)
        sub_path = match.group(2)
        if model_name in model_to_pkg:
            return model_to_pkg[model_name] + sub_path
        return match.group(0)

    return re.sub(r'model://([^/]+)/(.+)', replace_model_uri, urdf_content)


def generate_launch_description():
    # 声明launch参数
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='',
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

    radar_sensor_name_arg = DeclareLaunchArgument(
        'radar_sensor_name',
        default_value='radar',
        description='导航雷达传感器名称（用于拼接GZ原生话题）'
    )

    radar_ros_topic_arg = DeclareLaunchArgument(
        'radar_ros_topic',
        default_value='/sensors/radar/nav/sector',
        description='导航雷达最终ROS输出话题（推荐来自 full_config.override_topic）'
    )

    enable_maritime_radar_bridge_arg = DeclareLaunchArgument(
        'enable_maritime_radar_bridge',
        default_value='true',
        description='是否启动 radar_gz_bridge（仅海事扫描雷达 spokes→ROS；毫米波由 Gazebo 插件直发，不经此桥）'
    )

    enable_robot_localization_arg = DeclareLaunchArgument(
        'enable_robot_localization',
        default_value='false',
        description='启用 robot_localization (EKF + navsat_transform) 计算 map->odom'
    )

    localization_params_file_arg = DeclareLaunchArgument(
        'localization_params_file',
        default_value='',
        description='robot_localization 参数文件绝对路径'
    )

    localization_start_delay_arg = DeclareLaunchArgument(
        'localization_start_delay',
        default_value='5.0',
        description='Delay seconds before starting robot_localization nodes'
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

    enable_obstacle_spawner_arg = DeclareLaunchArgument(
        'enable_obstacle_spawner',
        default_value='true',
        description='是否启动 obstacle_spawner（多船时仅第一艘船应为 true，避免重复生成障碍物）'
    )

    spawn_stagger_sec_arg = DeclareLaunchArgument(
        'spawn_stagger_sec',
        default_value='0.0',
        description=(
            '世界与 /clock 就绪后的额外错开时间（秒）；多船时第 idx 艘传 idx*2.0'
        ),
    )

    spawn_wait_timeout_sec_arg = DeclareLaunchArgument(
        'spawn_wait_timeout_sec',
        default_value='120.0',
        description='等待 Gazebo create 服务与仿真时钟的最大墙钟时间（秒）',
    )

    gz_world_name_arg = DeclareLaunchArgument(
        'gz_world_name',
        default_value='',
        description='Gazebo 世界名（与 SDF 中 <world name="..."> 一致，如 sydney_regatta）；空则交给 create 默认'
    )

    verbose_launch_arg = DeclareLaunchArgument(
        'verbose_launch',
        default_value='false',
        description='为 true 时本组件各节点恢复 screen + INFO 级日志（默认降噪写入 ~/.ros/log）'
    )

    # 获取launch配置
    robot_name = LaunchConfiguration('robot_name')
    urdf_path = LaunchConfiguration('urdf_path')
    bridge_config_path = LaunchConfiguration('bridge_config_path')
    obstacle_layout_path = LaunchConfiguration('obstacle_layout_path')
    radar_sensor_name = LaunchConfiguration('radar_sensor_name')
    radar_ros_topic = LaunchConfiguration('radar_ros_topic')
    enable_maritime_radar_bridge = LaunchConfiguration('enable_maritime_radar_bridge')
    enable_robot_localization = LaunchConfiguration('enable_robot_localization')
    localization_params_file = LaunchConfiguration('localization_params_file')
    localization_start_delay = LaunchConfiguration('localization_start_delay')
    x_pose = LaunchConfiguration('x')
    y_pose = LaunchConfiguration('y')
    z_pose = LaunchConfiguration('z')
    roll = LaunchConfiguration('R')
    pitch = LaunchConfiguration('P')
    yaw = LaunchConfiguration('Y')
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_obstacle_spawner = LaunchConfiguration('enable_obstacle_spawner')
    spawn_stagger_sec = LaunchConfiguration('spawn_stagger_sec')
    spawn_wait_timeout_sec = LaunchConfiguration('spawn_wait_timeout_sec')
    gz_world_name = LaunchConfiguration('gz_world_name')
    verbose_launch = LaunchConfiguration('verbose_launch')

    # 启动robot_state_publisher - 使用 OpaqueFunction 在运行时读取并转换 URDF 路径
    # 将 session_manager 输出的 model:// URI 还原为 RViz 可识别的 package:// URI
    def launch_robot_state_publisher(context, *args, **kwargs):
        actual_urdf_path = urdf_path.perform(context)
        use_sim_time_val = use_sim_time.perform(context).lower() == 'true'
        v = verbose_launch.perform(context)
        with open(actual_urdf_path, 'r') as f:
            robot_desc = _convert_model_uri_to_package_uri(f.read())
        kw = quiet_ros_node_kwargs(v)
        node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_desc,
                'use_sim_time': use_sim_time_val
            }],
            **kw,
        )
        return [node]

    # 等待 /clock 与 /world/<name>/create 就绪后再 spawn（替代固定 create_entity_delay）
    def launch_spawn_robot_when_ready(context, *args, **kwargs):
        rn = robot_name.perform(context)
        world_gz = gz_world_name.perform(context).strip()
        v = verbose_launch.perform(context)
        safe = re.sub(r'[^a-zA-Z0-9_]', '_', rn)
        spawn_kw = quiet_ros_node_kwargs(v)
        # 就绪检测用墙钟轮询，避免 use_sim_time 在 /clock 未到时卡住
        return [
            Node(
                package='usv_sim_full',
                executable='gz_spawn_robot_when_ready',
                name=f'gz_spawn_{safe}',
                parameters=[{
                    'world_name': world_gz,
                    'entity_name': rn,
                    'sdf_file': urdf_path.perform(context),
                    'x': ParameterValue(x_pose, value_type=float),
                    'y': ParameterValue(y_pose, value_type=float),
                    'z': ParameterValue(z_pose, value_type=float),
                    'roll': ParameterValue(roll, value_type=float),
                    'pitch': ParameterValue(pitch, value_type=float),
                    'yaw': ParameterValue(yaw, value_type=float),
                    'spawn_stagger_sec': ParameterValue(
                        spawn_stagger_sec, value_type=float
                    ),
                    'wait_timeout_sec': ParameterValue(
                        spawn_wait_timeout_sec, value_type=float
                    ),
                }],
                **spawn_kw,
            )
        ]

    def launch_unnamespaced_gz_ros_stack(context, *args, **kwargs):
        """避免 PushRosNamespace 对 parameter_bridge 的副作用，保证 /{robot}/odom 等与 YAML 一致。"""
        rn = robot_name.perform(context)
        safe = re.sub(r'[^a-zA-Z0-9_]', '_', rn)
        use_sim_time_val = use_sim_time.perform(context).lower() == 'true'
        v = verbose_launch.perform(context)
        bridge_kw = quiet_ros_node_kwargs(v)
        odom_kw = quiet_ros_node_kwargs(v)
        bridge_path = bridge_config_path.perform(context)
        bridge_on = enable_maritime_radar_bridge.perform(context).lower() == 'true'
        radar_sensor_name_str = radar_sensor_name.perform(context)
        radar_ros_topic_str = radar_ros_topic.perform(context)
        if not radar_ros_topic_str.startswith('/'):
            radar_ros_topic_str = '/' + radar_ros_topic_str
        if not radar_ros_topic_str.startswith(f'/{rn}/'):
            radar_ros_topic_str = f'/{rn}{radar_ros_topic_str}'

        nodes = [
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name=f'param_bridge_{safe}',
                parameters=[{
                    'config_file': bridge_path,
                    'use_sim_time': use_sim_time_val,
                }],
                **bridge_kw,
            ),
            Node(
                package='usv_sim_full',
                executable='odom_tf_broadcaster',
                name=f'odom_tf_broadcaster_{safe}',
                parameters=[{
                    'odom_topic': f'/{rn}/odom',
                    'use_sim_time': use_sim_time_val,
                }],
                **odom_kw,
            ),
        ]
        if bridge_on:
            radar_kw = quiet_ros_node_kwargs(v)
            nodes.append(
                Node(
                    package='radar_gz_bridge',
                    executable='radar_gz_bridge',
                    name=f'radar_gz_bridge_{safe}',
                    parameters=[{
                        'gz_topic': f'/{rn}/{radar_sensor_name_str}/spokes',
                        'ros_topic': radar_ros_topic_str,
                        'frame_id': (
                            f'{rn}/{radar_sensor_name_str}_base_link'
                        ),
                        'use_sim_time': use_sim_time_val,
                    }],
                    **radar_kw,
                )
            )
        return nodes
    
    # 启动障碍物生成器节点（如果提供了有效路径）
    # 使用ExecuteProcess来运行通过console_scripts安装的脚本
    # 需要使用OpaqueFunction来正确处理LaunchConfiguration
    def launch_obstacle_spawner(context, *args, **kwargs):
        if enable_obstacle_spawner.perform(context).lower() != 'true':
            return []
        # 获取实际的障碍物布局路径
        actual_path = obstacle_layout_path.perform(context)

        if actual_path and actual_path.strip():  # 如果路径不为空
            v = verbose_launch.perform(context)
            obs_kw = quiet_ros_node_kwargs(v, [actual_path])
            obstacle_spawner_process = Node(
                package='usv_sim_full',
                executable='obstacle_spawner',
                **obs_kw,
            )
            return [obstacle_spawner_process]
        else:
            # 如果路径为空，不启动任何东西
            return []

    # TODO: 在此处插入路径规划 (Nav2) 和定位 (EKF) 节点
    # 示例预留位置:
    # nav2_nodes = IncludeLaunchDescription(...)  # Nav2导航栈
    # ekf_localization_node = Node(...)         # EKF定位节点

    def launch_robot_localization(context, *args, **kwargs):
        robot_name_str = robot_name.perform(context)
        use_sim_time_val = use_sim_time.perform(context).lower() == 'true'
        localization_params = localization_params_file.perform(context)
        v = verbose_launch.perform(context)
        loc_kw = quiet_ros_node_kwargs(v)

        gps_topic = f'/{robot_name_str}/sensors/gps/gps_sensor/data'
        imu_topic = f'/{robot_name_str}/sensors/imu/imu_sensor/data'
        odom_topic = f'/{robot_name_str}/odom'
        odom_gps_topic = f'/{robot_name_str}/odometry/gps'
        odom_global_topic = f'/{robot_name_str}/odometry/global'

        navsat_node = Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            parameters=[
                localization_params,
                {'use_sim_time': use_sim_time_val}
            ],
            remappings=[
                ('gps/fix', gps_topic),
                ('imu/data', imu_topic),
                ('odometry/filtered', odom_topic),
                ('odometry/gps', odom_gps_topic),
            ],
            condition=IfCondition(enable_robot_localization),
            **loc_kw,
        )

        ekf_map_node = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_map_node',
            parameters=[
                localization_params,
                {
                    'use_sim_time': use_sim_time_val,
                    'base_link_frame': f'{robot_name_str}/base_link',
                    'odom_frame': f'{robot_name_str}/odom',
                    'odom0': odom_topic,
                    'imu0': imu_topic,
                    'odom1': odom_gps_topic,
                }
            ],
            remappings=[
                ('odometry/filtered', odom_global_topic),
            ],
            condition=IfCondition(enable_robot_localization),
            **loc_kw,
        )

        return [navsat_node, ekf_map_node]

    robot_scoped_group = GroupAction(
        actions=[
            PushRosNamespace(namespace=robot_name),
            OpaqueFunction(function=launch_robot_state_publisher),
            TimerAction(
                period=localization_start_delay,
                actions=[OpaqueFunction(function=launch_robot_localization)]
            ),
        ]
    )

    return LaunchDescription([
        robot_name_arg,
        urdf_path_arg,
        bridge_config_path_arg,
        obstacle_layout_path_arg,
        radar_sensor_name_arg,
        radar_ros_topic_arg,
        enable_maritime_radar_bridge_arg,
        enable_robot_localization_arg,
        localization_params_file_arg,
        localization_start_delay_arg,
        x_arg,
        y_arg,
        z_arg,
        roll_arg,
        pitch_arg,
        yaw_arg,
        use_sim_time_arg,
        enable_obstacle_spawner_arg,
        spawn_stagger_sec_arg,
        spawn_wait_timeout_sec_arg,
        gz_world_name_arg,
        verbose_launch_arg,
        OpaqueFunction(function=launch_spawn_robot_when_ready),
        OpaqueFunction(function=launch_obstacle_spawner),  # 障碍物生成器（条件启动）
        robot_scoped_group,
        OpaqueFunction(function=launch_unnamespaced_gz_ros_stack),
    ])