"""Deprecated compatibility entry point for the generic full simulation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    sim_share = get_package_share_directory('usv_sim_full')
    bringup_share = get_package_share_directory('usv_bringup')
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_path',
            default_value=os.path.join(
                sim_share, 'config', 'three_vision_one_mmwave', 'full_config.yaml'
            ),
        ),
        DeclareLaunchArgument('nav2_namespace', default_value='usv_1'),
        DeclareLaunchArgument(
            'map_yaml',
            default_value=os.path.join(sim_share, 'maps', 'CN441122_enc_5km.yaml'),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, 'launch', 'full_sim.launch.py')
            ),
            launch_arguments={
                'config_path': LaunchConfiguration('config_path'),
                'namespace': LaunchConfiguration('nav2_namespace'),
                'map_yaml': LaunchConfiguration('map_yaml'),
            }.items(),
        ),
    ])
