"""Launch the full simulation + Nav2 + grounding warning nodes."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_robot_namespace(config_path: str) -> str:
    ns = "usv_1"
    try:
        from usv_sim_full.launch_config_helpers import primary_robot_name

        ns = primary_robot_name(config_path)
    except Exception:  # noqa: BLE001 - fallback to default
        pass
    return ns or "usv_1"


def _grounding_warning_include(context, *args, **kwargs):
    config_path = LaunchConfiguration("config_path").perform(context)
    requested_ns = LaunchConfiguration("nav2_namespace").perform(context).strip()
    ns = requested_ns if requested_ns and requested_ns != "auto" else _resolve_robot_namespace(
        config_path
    )

    gw_pkg_share = get_package_share_directory("enc_grounding_warning")
    gw_launch_file = os.path.join(
        gw_pkg_share, "launch", "enc_grounding_warning.launch.py"
    )
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gw_launch_file),
            launch_arguments={
                "namespace": ns,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "depth_grid_file": LaunchConfiguration("depth_grid_file"),
                "params_file": LaunchConfiguration("gw_params_file"),
                "robot_base_frame": f"{ns}/base_link",
            }.items(),
        )
    ]


def generate_launch_description():
    usv_sim_full_pkg = get_package_share_directory("usv_sim_full")
    sim_launch_file = os.path.join(
        usv_sim_full_pkg,
        "launch",
        "nav2_sim_full_bringup.launch.py",
    )
    gw_pkg_share = get_package_share_directory("enc_grounding_warning")
    default_config_path = os.path.join(
        usv_sim_full_pkg, "config", "three_vision_one_mmwave", "full_config.yaml"
    )
    default_map_yaml = os.path.join(usv_sim_full_pkg, "maps", "sydney_map2.yaml")
    default_rviz_config = os.path.join(usv_sim_full_pkg, "rviz", "default.rviz")
    default_nav2_params = os.path.join(
        usv_sim_full_pkg, "config", "radar_nav2_param.yaml"
    )
    default_control_params = os.path.join(
        usv_sim_full_pkg, "config", "control_params.yaml"
    )
    default_localization_params = os.path.join(
        usv_sim_full_pkg, "config", "robot_localization_gps.yaml"
    )
    default_gw_params = os.path.join(
        gw_pkg_share, "config", "grounding_warning_params.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("config_path", default_value=default_config_path),
            DeclareLaunchArgument("nav2_namespace", default_value="auto"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("verbose_launch", default_value="false"),
            DeclareLaunchArgument("gz_headless", default_value="false"),
            DeclareLaunchArgument("map_yaml", default_value=default_map_yaml),
            DeclareLaunchArgument("enable_nav2", default_value="true"),
            DeclareLaunchArgument("enable_gt_sensor_sim", default_value="false"),
            DeclareLaunchArgument("enable_late_fusion", default_value="false"),
            DeclareLaunchArgument("enable_convert_to_trackship", default_value="false"),
            DeclareLaunchArgument("enable_keepout_filter", default_value="true"),
            DeclareLaunchArgument("auto_cleanup", default_value="false"),
            DeclareLaunchArgument("disable_fastdds_shm", default_value="true"),
            DeclareLaunchArgument("enable_rviz", default_value="true"),
            DeclareLaunchArgument("enable_robot_localization", default_value="false"),
            DeclareLaunchArgument("use_static_map_odom_tf", default_value="true"),
            DeclareLaunchArgument("enable_tf_namespace_relay", default_value="true"),
            DeclareLaunchArgument("params_file", default_value=default_nav2_params),
            DeclareLaunchArgument(
                "control_params_file", default_value=default_control_params
            ),
            DeclareLaunchArgument(
                "localization_params_file", default_value=default_localization_params
            ),
            DeclareLaunchArgument("depth_grid_file", default_value=""),
            DeclareLaunchArgument("gw_params_file", default_value=default_gw_params),
            SetEnvironmentVariable(
                name="PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION",
                value="python",
            ),
            SetEnvironmentVariable(
                name="RMW_FASTRTPS_USE_SHM",
                value="0",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sim_launch_file),
                launch_arguments={
                    "config_path": LaunchConfiguration("config_path"),
                    "nav2_namespace": LaunchConfiguration("nav2_namespace"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "verbose_launch": LaunchConfiguration("verbose_launch"),
                    "gz_headless": LaunchConfiguration("gz_headless"),
                    "map_yaml": LaunchConfiguration("map_yaml"),
                    "enable_nav2": LaunchConfiguration("enable_nav2"),
                    "enable_gt_sensor_sim": LaunchConfiguration("enable_gt_sensor_sim"),
                    "enable_late_fusion": LaunchConfiguration("enable_late_fusion"),
                    "enable_convert_to_trackship": LaunchConfiguration(
                        "enable_convert_to_trackship"
                    ),
                    "enable_keepout_filter": LaunchConfiguration("enable_keepout_filter"),
                    "auto_cleanup": LaunchConfiguration("auto_cleanup"),
                    "disable_fastdds_shm": LaunchConfiguration("disable_fastdds_shm"),
                    "enable_robot_localization": LaunchConfiguration(
                        "enable_robot_localization"
                    ),
                    "use_static_map_odom_tf": LaunchConfiguration(
                        "use_static_map_odom_tf"
                    ),
                    "enable_tf_namespace_relay": LaunchConfiguration(
                        "enable_tf_namespace_relay"
                    ),
                    "params_file": LaunchConfiguration("params_file"),
                    "control_params_file": LaunchConfiguration("control_params_file"),
                    "localization_params_file": LaunchConfiguration(
                        "localization_params_file"
                    ),
                }.items(),
            ),
            OpaqueFunction(function=_grounding_warning_include),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_rviz")),
                arguments=["-d", default_rviz_config],
            ),
        ]
    )
