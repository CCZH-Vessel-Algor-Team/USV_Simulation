"""Launch the simulation-owned TS service chain with explicit parameter wiring."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Create TS manager, avoidance-point and barrier nodes.

    :return: The TS subsystem launch description.
    """
    params_file = LaunchConfiguration('ts_params_file')
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument(
            'ts_params_file',
            default_value=os.path.join(
                get_package_share_directory('usv_sim_full'), 'config', 'ts_subsystem.yaml'),
            description='TS service parameters; independent of the Nav2 params_file'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'tracked_ship_topic', default_value='/dynamic_ship/tracked_ships'),
        DeclareLaunchArgument('robot_base_frame', default_value='usv_1/base_link'),
        DeclareLaunchArgument('odom_topic', default_value='/usv_1/odom'),
        Node(
            package='nav2_colregs_ts_manager',
            executable='ts_state_manager',
            name='ts_state_manager',
            namespace='',
            output='screen',
            parameters=[params_file, {
                'use_sim_time': use_sim_time,
                'tracked_ship_topic': LaunchConfiguration('tracked_ship_topic'),
                'robot_base_frame': LaunchConfiguration('robot_base_frame'),
                'odom_topic': LaunchConfiguration('odom_topic'),
            }]),
        Node(
            package='nav2_colregs_ts_manager',
            executable='avoidance_point_node',
            name='avoidance_point_node',
            namespace='',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}]),
        Node(
            package='nav2_colregs_ts_manager',
            executable='barrier_node',
            name='barrier_node',
            namespace='',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}]),
    ])
