"""Launch grounding warning nodes (depth provider / UKC / look-ahead)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    pkg_share = get_package_share_directory("enc_grounding_warning")
    default_depth_grid = os.path.join(pkg_share, "config", "sim_depth_grid.yaml")
    default_params = os.path.join(pkg_share, "config", "grounding_warning_params.yaml")

    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    depth_grid_file = LaunchConfiguration("depth_grid_file")
    params_file = LaunchConfiguration("params_file")
    robot_base_frame = LaunchConfiguration("robot_base_frame")
    enable_depth_provider = LaunchConfiguration("enable_depth_provider")
    enable_ukc = LaunchConfiguration("enable_ukc")
    enable_grounding_warning = LaunchConfiguration("enable_grounding_warning")
    enable_route_depth_publisher = LaunchConfiguration("enable_route_depth_publisher")
    publish_identity_map_odom_tf = LaunchConfiguration("publish_identity_map_odom_tf")

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="usv_1"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("depth_grid_file", default_value=default_depth_grid),
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("robot_base_frame", default_value="usv_1/base_link"),
            DeclareLaunchArgument("enable_depth_provider", default_value="true"),
            DeclareLaunchArgument("enable_ukc", default_value="true"),
            DeclareLaunchArgument("enable_grounding_warning", default_value="true"),
            DeclareLaunchArgument("enable_route_depth_publisher", default_value="true"),
            DeclareLaunchArgument("publish_identity_map_odom_tf", default_value="true"),
            GroupAction(
                [
                    PushRosNamespace(namespace),
                    Node(
                        package="tf2_ros",
                        executable="static_transform_publisher",
                        name="gw_map_to_odom_tf",
                        output="log",
                        condition=IfCondition(publish_identity_map_odom_tf),
                        arguments=[
                            "--frame-id",
                            "map",
                            "--child-frame-id",
                            "usv_1/odom",
                        ],
                    ),
                    Node(
                        package="enc_grounding_warning",
                        executable="depth_provider_node",
                        name="depth_provider_node",
                        output="screen",
                        condition=IfCondition(enable_depth_provider),
                        parameters=[
                            {
                                "use_sim_time": use_sim_time,
                                "depth_grid_file": depth_grid_file,
                                "robot_base_frame": robot_base_frame,
                            }
                        ],
                    ),
                    Node(
                        package="enc_grounding_warning",
                        executable="ukc_estimator_node",
                        name="ukc_estimator_node",
                        output="screen",
                        condition=IfCondition(enable_ukc),
                        parameters=[
                            {
                                "use_sim_time": use_sim_time,
                                "params_file": params_file,
                                "depth_grid_file": depth_grid_file,
                                "robot_base_frame": robot_base_frame,
                            }
                        ],
                    ),
                    Node(
                        package="enc_grounding_warning",
                        executable="grounding_warning_node",
                        name="grounding_warning_node",
                        output="screen",
                        condition=IfCondition(enable_grounding_warning),
                        parameters=[
                            {
                                "use_sim_time": use_sim_time,
                                "params_file": params_file,
                                "depth_grid_file": depth_grid_file,
                                "robot_base_frame": robot_base_frame,
                            }
                        ],
                    ),
                    Node(
                        package="enc_grounding_warning",
                        executable="route_depth_publisher_node",
                        name="route_depth_publisher_node",
                        output="screen",
                        condition=IfCondition(enable_route_depth_publisher),
                        parameters=[
                            {
                                "use_sim_time": use_sim_time,
                                "params_file": params_file,
                                "depth_grid_file": depth_grid_file,
                            }
                        ],
                    ),
                ]
            ),
        ]
    )
