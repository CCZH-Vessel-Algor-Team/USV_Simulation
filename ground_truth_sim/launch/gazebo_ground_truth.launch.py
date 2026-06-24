#!/usr/bin/env python3
"""Launch Gazebo-oriented ground truth bridge + optional static TF + optional ground_truth_sensor_sim.

Do not run this together with ground_truth_sim.launch.py's ground_truth_node on the same
/sim/ground_truth topic — use either legacy simulation or this bridge.
"""

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
_DEFAULT_BRIDGE_PARAMS = os.path.join(_PACKAGE_SHARE, "config", "gazebo_ground_truth_bridge.yaml")
_DEFAULT_RVIZ_CONFIG = os.path.join(_PACKAGE_SHARE, "rviz", "ground_truth_view.rviz")
_DEFAULT_SENSOR_PARAMS = os.path.join(_SENSOR_SHARE, "config", "single_sensor_params.yaml")


def generate_launch_description() -> LaunchDescription:
    parent_frame_arg = DeclareLaunchArgument(
        "parent_frame", default_value="map", description="Static TF parent (if start_static_tf is true)"
    )
    child_frame_arg = DeclareLaunchArgument(
        "child_frame",
        default_value="base_link",
        description="Static TF child for USV / sensor tree",
    )
    bridge_params_arg = DeclareLaunchArgument(
        "bridge_params_file",
        default_value=_DEFAULT_BRIDGE_PARAMS,
        description="YAML for gazebo_ground_truth_bridge_node",
    )
    sensor_params_file_arg = DeclareLaunchArgument(
        "sensor_params_file",
        default_value=_DEFAULT_SENSOR_PARAMS,
        description="YAML for ground_truth_sensor_sim nodes",
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="false",
        description="Whether to start RViz (Gazebo often has its own view)",
    )
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=_DEFAULT_RVIZ_CONFIG,
        description="RViz config path when use_rviz is true",
    )
    start_static_tf_arg = DeclareLaunchArgument(
        "start_static_tf",
        default_value="true",
        description="Publish map->base_link and sensor frames (disable if Gazebo already provides them)",
    )
    start_vision_arg = DeclareLaunchArgument(
        "start_vision_node",
        default_value="false",
        description="Launch ground_truth_sensor_sim sim_vision_node",
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
    start_mmwave_arg = DeclareLaunchArgument(
        "start_mmwave_node",
        default_value="false",
        description="Launch ground_truth_sensor_sim sim_mmwave_node",
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
        condition=IfCondition(LaunchConfiguration("start_static_tf")),
    )

    bridge_node = Node(
        package="ground_truth_sim",
        executable="gazebo_ground_truth_bridge_node",
        name="gazebo_ground_truth_bridge_node",
        output="screen",
        parameters=[LaunchConfiguration("bridge_params_file")],
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
            bridge_params_arg,
            sensor_params_file_arg,
            use_rviz_arg,
            rviz_config_arg,
            start_static_tf_arg,
            start_vision_arg,
            start_ais_arg,
            start_nav_radar_arg,
            start_mmwave_arg,
            static_tf_node,
            bridge_node,
            sim_vision_node,
            sim_mmwave_node,
            rviz_node,
        ]
    )
