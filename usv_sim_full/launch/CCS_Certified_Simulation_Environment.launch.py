"""CCS certified simulation environment with camera and map RTSP streams.

This launch includes every action from
nav2_sim_three_vision_mmwave_bringup.launch.py, selects ccs_config.yaml by
default, and adds three camera streams plus the radar occupancy-map stream.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    usv_sim_full_share = get_package_share_directory('usv_sim_full')
    usv_vision_share = get_package_share_directory('usv_vision')
    map_streamer_share = get_package_share_directory('usv_map_rtsp_streamer')

    base_launch_file = os.path.join(
        usv_sim_full_share,
        'launch',
        'nav2_sim_three_vision_mmwave_bringup.launch.py',
    )
    default_ccs_config = os.path.join(
        usv_sim_full_share,
        'config',
        'three_vision_one_mmwave',
        'ccs_config.yaml',
    )
    rtsp_params_file = os.path.join(
        usv_vision_share,
        'config',
        'rtsp_stream.yaml',
    )
    map_streamer_launch_file = os.path.join(
        map_streamer_share,
        'launch',
        'usv_map_rtsp_streamer.launch.py',
    )

    config_path = LaunchConfiguration('config_path')
    enable_camera_rtsp = LaunchConfiguration('enable_camera_rtsp_streaming')
    enable_map_rtsp = LaunchConfiguration('enable_map_rtsp_streamer')
    camera_width = LaunchConfiguration('camera_stream_width')
    camera_height = LaunchConfiguration('camera_stream_height')
    camera_fps = LaunchConfiguration('camera_stream_fps')

    base_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch_file),
        launch_arguments={
            'config_path': config_path,
        }.items(),
    )

    camera_streams = []
    camera_definitions = (
        (
            'front_cam',
            '/usv_1/sensors/camera/front_cam/image_raw',
            LaunchConfiguration('front_camera_rtsp_url'),
        ),
        (
            'left_cam',
            '/usv_1/sensors/camera/left_cam/image_raw',
            LaunchConfiguration('left_camera_rtsp_url'),
        ),
        (
            'right_cam',
            '/usv_1/sensors/camera/right_cam/image_raw',
            LaunchConfiguration('right_camera_rtsp_url'),
        ),
    )
    for sensor_name, input_topic, rtsp_url in camera_definitions:
        camera_streams.append(
            Node(
                package='usv_vision',
                executable='rtsp_stream_node',
                name=f'rtsp_stream_{sensor_name}',
                output='screen',
                condition=IfCondition(enable_camera_rtsp),
                parameters=[
                    rtsp_params_file,
                    {
                        'sensor_name': sensor_name,
                        'camera_id': sensor_name,
                        'input_image_topic': input_topic,
                        'rtsp_url': rtsp_url,
                        'width': camera_width,
                        'height': camera_height,
                        'fps': camera_fps,
                    },
                ],
            )
        )

    map_streamer = GroupAction(
        condition=IfCondition(enable_map_rtsp),
        actions=[
            SetRemap(
                src='/map/navradar/occupancy_grid',
                dst=LaunchConfiguration('map_rtsp_input_topic'),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(map_streamer_launch_file),
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_path',
            default_value=default_ccs_config,
            description='CCS simulation configuration file',
        ),
        DeclareLaunchArgument(
            'enable_camera_rtsp_streaming',
            default_value='true',
            description='Start RTSP streaming for the three simulated cameras',
        ),
        DeclareLaunchArgument(
            'front_camera_rtsp_url',
            default_value='rtsp://127.0.0.1:8554/cam1',
            description='Front camera RTSP destination URL',
        ),
        DeclareLaunchArgument(
            'left_camera_rtsp_url',
            default_value='rtsp://127.0.0.1:8554/cam2',
            description='Left camera RTSP destination URL',
        ),
        DeclareLaunchArgument(
            'right_camera_rtsp_url',
            default_value='rtsp://127.0.0.1:8554/cam3',
            description='Right camera RTSP destination URL',
        ),
        DeclareLaunchArgument(
            'camera_stream_width',
            default_value='640',
            description='Camera stream width; matches the CCS camera configuration',
        ),
        DeclareLaunchArgument(
            'camera_stream_height',
            default_value='480',
            description='Camera stream height; matches the CCS camera configuration',
        ),
        DeclareLaunchArgument(
            'camera_stream_fps',
            default_value='23',
            description='Camera stream frame rate; matches the CCS camera configuration',
        ),
        DeclareLaunchArgument(
            'enable_map_rtsp_streamer',
            default_value='true',
            description='Start the radar occupancy-map RTSP streamer',
        ),
        DeclareLaunchArgument(
            'map_rtsp_input_topic',
            default_value='/usv_1/map/navradar/occupancy_grid',
            description='Actual occupancy-grid topic remapped into the map streamer',
        ),
        LogInfo(msg=['Starting CCS certified simulation from: ', config_path]),
        base_bringup,
        *camera_streams,
        map_streamer,
    ])
