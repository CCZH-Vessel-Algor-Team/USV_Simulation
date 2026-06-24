"""
******************************************************************************************
*  Copyright (C) 2026 MurphyChen, All Rights Reserved                                  *
*                                                                                        *
*  @brief    10m.dae 缩放测试环境入口                                                    *
*  @author   GPT-5.4                                                                   *
*  @version  1.0.0                                                                     *
*  @date     2026.4.22                                                                 *
******************************************************************************************
"""

import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory("usv_sim_full")
    verbose_launch = LaunchConfiguration("verbose_launch")

    worlds_path = os.path.join(pkg_share, "worlds")
    worlds_models_path = os.path.join(worlds_path, "models")
    description_path = os.path.join(pkg_share, "description")
    description_models_path = os.path.join(description_path, "models")
    world_file = os.path.join(worlds_path, "target_ship_scale_test.sdf")

    extra_paths = [
        worlds_path,
        worlds_models_path,
        description_path,
        description_models_path,
    ]

    try:
        wamv_desc_path = get_package_share_directory("wamv_description")
        extra_paths.append(os.path.dirname(wamv_desc_path))
    except Exception:
        pass

    gz_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    new_resource_path = ":".join(extra_paths + ([gz_resource_path] if gz_resource_path else []))

    gazebo_model_path = os.environ.get("GAZEBO_MODEL_PATH", "")
    new_model_path = ":".join(extra_paths + ([gazebo_model_path] if gazebo_model_path else []))

    gz_plugin_path = os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")
    plugin_paths = []
    try:
        usv_sim_prefix = get_package_prefix("usv_sim_full")
        radar_plugin_lib_path = os.path.join(
            os.path.dirname(usv_sim_prefix), "gz_maritime_radar_plugin", "lib"
        )
        if os.path.isdir(radar_plugin_lib_path):
            plugin_paths.append(radar_plugin_lib_path)
    except Exception:
        pass
    try:
        mmwave_lib_path = os.path.join(get_package_prefix("usv_mmwave_sim"), "lib")
        if os.path.isdir(mmwave_lib_path):
            plugin_paths.append(mmwave_lib_path)
    except Exception:
        pass

    new_plugin_path = ":".join(plugin_paths + ([gz_plugin_path] if gz_plugin_path else []))

    def log_paths_if_verbose(context, *args, **kwargs):
        v = verbose_launch.perform(context).lower()
        if v not in ("true", "1", "yes"):
            return []
        print(f"[target_ship_scale_test] world: {world_file}")
        print(f"[target_ship_scale_test] GZ_SIM_RESOURCE_PATH={new_resource_path}")
        print(f"[target_ship_scale_test] GAZEBO_MODEL_PATH={new_model_path}")
        print(f"[target_ship_scale_test] GZ_SIM_SYSTEM_PLUGIN_PATH={new_plugin_path}")
        return []

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "verbose_launch",
                default_value="false",
                description="true 时打印 GZ 资源路径和世界文件路径",
            ),
            SetEnvironmentVariable(name="GZ_SIM_RESOURCE_PATH", value=new_resource_path),
            SetEnvironmentVariable(name="GAZEBO_MODEL_PATH", value=new_model_path),
            SetEnvironmentVariable(name="GZ_SIM_SYSTEM_PLUGIN_PATH", value=new_plugin_path),
            OpaqueFunction(function=log_paths_if_verbose),
            ExecuteProcess(
                cmd=["gz", "sim", "-r", world_file],
                output="screen",
            ),
        ]
    )
