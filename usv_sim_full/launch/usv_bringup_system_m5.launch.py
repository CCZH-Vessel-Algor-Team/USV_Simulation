"""M5 identified planar model, full-stack default entry (delivered in-repo).

Includes the unmodified usv_bringup system entrypoint with this package's
config/system/m5_sim.yaml profile: the company module composition on the
M5 dynamics layer (m5.urdf.xacro robot, power-channel thrusters, power-mode
cmd_vel_to_thruster). The ordinary bringup entrypoint is unchanged.

Requires a compatible usv_bringup and its dependencies to be installed;
this package only contributes the model, template wiring and the profile.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from usv_bringup.launch_utils import include_launch


def generate_launch_description():
    """Include the ordinary system entrypoint with an explicit M5 profile.

    :return: M5-specific system launch description.
    """
    share = get_package_share_directory('usv_sim_full')
    profile = os.path.join(share, 'config', 'system', 'm5_sim.yaml')
    arguments = [
        DeclareLaunchArgument(
            'system_config_file', default_value=profile,
            description='系统模块 profile；默认使用随包安装的 M5 配置。'),
        DeclareLaunchArgument(
            'gz_headless', default_value='',
            description='覆盖 Gazebo 无界面开关；空值沿用 m5_sim.yaml（false）。'),
        DeclareLaunchArgument(
            'launch_rviz', default_value='',
            description='覆盖 RViz 开关；空值沿用场景配置。'),
        DeclareLaunchArgument(
            'fastdds_profile_file', default_value='',
            description='空值保留调用方 DDS 环境（容器已验证的 UDPv4 配置）。'),
    ]
    system = include_launch('usv_bringup', 'system.launch.py', {
        'system_config_file': LaunchConfiguration('system_config_file'),
        'gz_headless': LaunchConfiguration('gz_headless'),
        'launch_rviz': LaunchConfiguration('launch_rviz'),
        'fastdds_profile_file': LaunchConfiguration('fastdds_profile_file'),
    })
    return LaunchDescription(arguments + [system])
