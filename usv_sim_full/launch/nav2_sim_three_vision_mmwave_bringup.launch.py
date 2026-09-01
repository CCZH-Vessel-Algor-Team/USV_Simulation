"""Deprecated compatibility entry point for the CCS perception simulation."""

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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, 'launch', 'ccs_certified_sim.launch.py')
            ),
            launch_arguments={
                'config_path': LaunchConfiguration('config_path'),
                'enable_safety': 'false',
                'enable_ais_sim': 'false',
                'enable_ais_aggregator': 'false',
                'enable_camera_rtsp_streaming': 'false',
                'enable_map_rtsp_streamer': 'false',
                'enable_route_planner': 'false',
            }.items(),
        ),
    ])
