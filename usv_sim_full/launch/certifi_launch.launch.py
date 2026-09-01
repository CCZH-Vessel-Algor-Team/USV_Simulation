"""Deprecated compatibility entry point for CCS encounter certification."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    bringup_share = get_package_share_directory('usv_bringup')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    bringup_share,
                    'launch',
                    'ccs_encounter_certification.launch.py',
                )
            )
        ),
    ])
