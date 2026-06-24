import os
import re
import subprocess
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from usv_sim_full.launch_config_helpers import (
    launch_verbose_enabled,
    parse_session_json_from_stdout,
    quiet_ros_node_kwargs,
    resolve_session_robots,
    session_manager_executable_path,
)


def _convert_model_uri_to_package_uri(urdf_content):
    """
    将 Gazebo 专用的 model:// URI 反向还原回 RViz 可识别的 package:// URI。
    session_manager.py 在生成 URDF 时会将 package:// 替换成 model:// 以供
    Gazebo 使用，此函数执行逆操作，以便在纯 RViz 环境中正确加载网格。
    """
    # 动态扫描两个 package 下的 models 子目录，构建 (目录名 -> package://) 映射表
    # usv_sim_full 的自定义模型优先级高于 wamv_description
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
                    # usv_sim_full 自定义模型覆盖同名的 wamv_description 模型
                    model_to_pkg[name] = f'package://usv_sim_full/description/models/{name}/'
    except Exception:
        pass

    def replace_model_uri(match):
        model_name = match.group(1)
        sub_path = match.group(2)
        if model_name in model_to_pkg:
            return model_to_pkg[model_name] + sub_path
        # 找不到映射则保留原样，避免破坏 URDF 结构
        return match.group(0)

    return re.sub(r'model://([^/]+)/(.+)', replace_model_uri, urdf_content)


def launch_setup(context, *args, **kwargs):
    # 1. 获取用户配置的路径
    config_path = LaunchConfiguration('config_path').perform(context)
    robot_index = int(LaunchConfiguration('robot_index').perform(context).strip() or '0')
    verbose_s = LaunchConfiguration('verbose_launch').perform(context)
    pkg_share = get_package_share_directory('usv_sim_full')

    with open(config_path, 'r') as f:
        user_config = yaml.safe_load(f)

    # 2. 调用 session_manager 获取 URDF（多船时由 robot_index 选择）
    try:
        sm_cmd = [
            session_manager_executable_path(),
            '--config-path', config_path,
        ]
        if launch_verbose_enabled(verbose_s):
            sm_cmd.append('--verbose')
        result = subprocess.run(sm_cmd, capture_output=True, text=True, check=True)

        session_output = result.stdout.strip()
        session_info = parse_session_json_from_stdout(session_output)
        robots = resolve_session_robots(session_info, user_config)
        if not robots:
            raise ValueError('session_manager 未返回任何机器人')
        idx = max(0, min(robot_index, len(robots) - 1))
        urdf_path = robots[idx]['urdf_path']

    except Exception as e:
        print(f"执行 session manager 时发生错误: {e}")
        raise e

    # 读取生成的 URDF 文本内容，并将 model:// 路径还原为 RViz 可识别的 package:// 路径
    with open(urdf_path, 'r') as infp:
        robot_desc = _convert_model_uri_to_package_uri(infp.read())

    # 3. 启动 robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
        **quiet_ros_node_kwargs(verbose_s),
    )

    # 4. 启动 joint_state_publisher_gui 提供活动关节的滑动条
    joint_state_pub_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        **quiet_ros_node_kwargs(verbose_s),
    )

    # 5. 启动 RViz 测试环境
    rviz_config_file = os.path.join(pkg_share, 'config', 'tf_tune.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        **quiet_ros_node_kwargs(verbose_s, ['-d', rviz_config_file]),
    )

    # 返回纯 TF/URDF 相关的基础节点，不含任何 gazebo/物理引擎 的逻辑
    out = [
        robot_state_publisher_node,
        joint_state_pub_gui_node,
        rviz_node
    ]
    if not launch_verbose_enabled(verbose_s):
        return [
            SetEnvironmentVariable(
                name='RCUTILS_LOGGING_SEVERITY',
                value='WARN',
            ),
            *out,
        ]
    return out


def generate_launch_description():
    pkg_share = get_package_share_directory('usv_sim_full')
    default_config_path = os.path.join(pkg_share, 'config', 'full_config.yaml')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_path',
            default_value=default_config_path,
            description='Path to the full_config.yaml file'
        ),
        DeclareLaunchArgument(
            'robot_index',
            default_value='0',
            description='多船时选择第几条船（0 起，对应 session robots 顺序）'
        ),
        DeclareLaunchArgument(
            'verbose_launch',
            default_value='false',
            description='true：详细终端输出；false（默认）：降噪'
        ),
        OpaqueFunction(function=launch_setup)
    ])
