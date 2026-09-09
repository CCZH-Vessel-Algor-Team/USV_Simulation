"""Launch the ENC dual-route planner."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def require_humble(context):
    """Reject action definitions that do not match the Humble Nav2 server."""
    del context
    ros_distro = os.environ.get("ROS_DISTRO", "")
    if ros_distro != "humble":
        raise RuntimeError(
            "usv_route_planner must run with ROS 2 Humble because the target "
            f"Nav2 server uses Humble action definitions; got ROS_DISTRO={ros_distro!r}"
        )
    return []


def generate_launch_description() -> LaunchDescription:
    """Create the route planner node with a replaceable parameter file."""
    default_params = PathJoinSubstitution(
        [FindPackageShare("usv_route_planner"), "config", "route_planner.yaml"]
    )
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    return LaunchDescription(
        [
            OpaqueFunction(function=require_humble),
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="usv_route_planner",
                executable="route_planner_node",
                name="usv_route_planner",
                output="screen",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
            ),
        ]
    )
