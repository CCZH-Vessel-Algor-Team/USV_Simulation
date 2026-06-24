#!/usr/bin/env python3
"""Launch RViz-ready ground truth simulation pipeline."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_PACKAGE_SHARE = get_package_share_directory("ground_truth_sim")
_SENSOR_SHARE = get_package_share_directory("ground_truth_sensor_sim")
_DEFAULT_RVIZ_CONFIG = os.path.join(_PACKAGE_SHARE, "rviz", "ground_truth_view.rviz")
_DEFAULT_PARAMS_FILE = os.path.join(_PACKAGE_SHARE, "config", "ground_truth_params.yaml")
_DEFAULT_SENSOR_PARAMS = os.path.join(_SENSOR_SHARE, "config", "single_sensor_params.yaml")


def generate_launch_description() -> LaunchDescription:
    parent_frame_arg = DeclareLaunchArgument(
        "parent_frame", default_value="map", description="Parent TF frame name"
    )
    child_frame_arg = DeclareLaunchArgument(
        "child_frame", default_value="base_link", description="Child TF frame name"
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Whether to auto-start RViz with the provided config",
    )
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=_DEFAULT_RVIZ_CONFIG,
        description="Absolute path to an RViz configuration file",
    )
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=_DEFAULT_PARAMS_FILE,
        description="YAML file that stores ground truth node parameters",
    )
    sensor_params_file_arg = DeclareLaunchArgument(
        "sensor_params_file",
        default_value=_DEFAULT_SENSOR_PARAMS,
        description="YAML for ground_truth_sensor_sim nodes",
    )
    start_vision_arg = DeclareLaunchArgument(
        "start_vision_node",
        default_value="true",
        description="Whether to launch ground_truth_sensor_sim vision node",
    )
    start_mmwave_arg = DeclareLaunchArgument(
        "start_mmwave_node",
        default_value="true",
        description="Whether to launch ground_truth_sensor_sim mmWave node",
    )
    start_ais_arg = DeclareLaunchArgument(
        "start_ais_node",
        default_value="false",
        description="Deprecated: AIS simulator not provided by ground_truth_sensor_sim",
    )
    start_nav_radar_arg = DeclareLaunchArgument(
        "start_nav_radar_node",
        default_value="false",
        description="Deprecated: nav radar simulator not provided by ground_truth_sensor_sim",
    )

    static_tf_node = Node(
        package="ground_truth_sim",
        executable="static_tf_broadcaster",
        name="static_tf_broadcaster",
        parameters=[
            {
                "parent_frame": LaunchConfiguration("parent_frame"),
                "child_frame": LaunchConfiguration("child_frame"),
            }
        ],
    )

    ground_truth_node = Node(
        package="ground_truth_sim",
        executable="ground_truth_node",
        name="ground_truth_node",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="ground_truth_rviz",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    sim_vision_node = Node(
        package="ground_truth_sensor_sim",
        executable="sim_vision_node",
        name="sim_vision_node",
        output="screen",
        parameters=[LaunchConfiguration("sensor_params_file")],
        condition=IfCondition(LaunchConfiguration("start_vision_node")),
    )

    sim_mmwave_node = Node(
        package="ground_truth_sensor_sim",
        executable="sim_mmwave_node",
        name="sim_mmwave_node",
        output="screen",
        parameters=[LaunchConfiguration("sensor_params_file")],
        condition=IfCondition(LaunchConfiguration("start_mmwave_node")),
    )

    return LaunchDescription(
        [
            parent_frame_arg,
            child_frame_arg,
            use_rviz_arg,
            rviz_config_arg,
            params_file_arg,
            sensor_params_file_arg,
            start_vision_arg,
            start_ais_arg,
            start_nav_radar_arg,
            start_mmwave_arg,
            static_tf_node,
            ground_truth_node,
            sim_vision_node,
            sim_mmwave_node,
            rviz_node,
        ]
    )
