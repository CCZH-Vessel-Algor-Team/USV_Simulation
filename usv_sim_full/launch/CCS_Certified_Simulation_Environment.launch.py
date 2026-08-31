"""CCS certified simulation environment with camera and map RTSP streams.

This launch includes every action from
nav2_sim_three_vision_mmwave_bringup.launch.py, selects ccs_config.yaml by
default, and adds three camera streams plus the radar occupancy-map stream.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def _resolve_robot_namespace(config_path: str) -> str:
    """从 CCS full_config 解析首船 ROS 命名空间，供 safety 节点使用。"""
    try:
        with open(config_path, 'r') as f:
            cfg = yaml.safe_load(f) or {}
        from usv_sim_full.launch_config_helpers import primary_robot_name

        ns = primary_robot_name(cfg)
        return ns or 'usv_1'
    except Exception:
        return 'usv_1'


def _dynamic_ship_ground_truth_bridge(context, *args, **kwargs):
    """可选启动 dynamic_ship/tracked_ships → /sim/ground_truth 转换节点。"""
    enable = LaunchConfiguration('enable_dynamic_ship_gt_bridge').perform(context).strip().lower()
    if enable not in ('true', '1', 'yes'):
        return [
            LogInfo(msg='enable_dynamic_ship_gt_bridge:=false，跳过动态船真值转换节点。'),
        ]

    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    return [
        LogInfo(
            msg='启动 dynamic_ship_to_ground_truth（/dynamic_ship/tracked_ships/_internal -> /sim/ground_truth/_src/dynamic_ships）。'
        ),
        Node(
            package='ground_truth_sensor_sim',
            executable='dynamic_ship_to_ground_truth',
            name='dynamic_ship_to_ground_truth',
            output='log',
            parameters=[{
                'use_sim_time': use_sim_time,
                'input_topic': '/dynamic_ship/tracked_ships/_internal',
                'output_topic': '/sim/ground_truth/_src/dynamic_ships',
                'frame_id': 'map',
                'size_w': 3.6,
                'size_l': 10.0,
                'size_h': 2.0,
                'is_dark_target': True,
                'is_ais_matched': False,
                'matched_mmsi': 0,
                'source_model_name': 'dynamic_ship',
            }],
        ),
    ]


def _safety_include(context, *args, **kwargs):
    """可选启动 enc_grounding_warning 搁浅预警节点。"""
    enable_safety = LaunchConfiguration('enable_safety').perform(context).strip().lower()
    if enable_safety not in ('true', '1', 'yes'):
        return [
            LogInfo(msg='enable_safety:=false，跳过 enc_grounding_warning 搁浅预警。'),
        ]

    config_path = LaunchConfiguration('config_path').perform(context)
    ns = _resolve_robot_namespace(config_path)
    gw_pkg_share = get_package_share_directory('enc_grounding_warning')
    gw_launch_file = os.path.join(
        gw_pkg_share,
        'launch',
        'enc_grounding_warning.launch.py',
    )
    return [
        LogInfo(
            msg=[
                'Starting enc_grounding_warning safety nodes, namespace=',
                ns,
                ', config=',
                LaunchConfiguration('config_path'),
            ]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gw_launch_file),
            launch_arguments={
                'namespace': ns,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'depth_grid_file': LaunchConfiguration('safety_depth_grid_file'),
                'params_file': LaunchConfiguration('safety_params_file'),
                'robot_base_frame': f'{ns}/base_link',
            }.items(),
        ),
    ]


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
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_camera_rtsp = LaunchConfiguration('enable_camera_rtsp_streaming')
    enable_map_rtsp = LaunchConfiguration('enable_map_rtsp_streamer')
    camera_width = LaunchConfiguration('camera_stream_width')
    camera_height = LaunchConfiguration('camera_stream_height')
    camera_fps = LaunchConfiguration('camera_stream_fps')

    base_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch_file),
        launch_arguments={
            'config_path': config_path,
            'use_sim_time': use_sim_time,
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
            'use_sim_time',
            default_value='true',
            description='Use simulation clock; forwarded to the base simulation and safety nodes',
        ),
        DeclareLaunchArgument(
            'enable_dynamic_ship_gt_bridge',
            default_value='true',
            description='true：启动 /dynamic_ship/tracked_ships -> /sim/ground_truth 转换节点',
        ),
        DeclareLaunchArgument(
            'enable_safety',
            default_value='true',
            description='Start the enc_grounding_warning grounding safety stack',
        ),
        DeclareLaunchArgument(
            'safety_depth_grid_file',
            default_value='',
            description=(
                'Override the enc_grounding_warning depth grid file; '
                'empty uses the safety package default'
            ),
        ),
        DeclareLaunchArgument(
            'safety_params_file',
            default_value='',
            description=(
                'Override the enc_grounding_warning parameter file; '
                'empty uses the safety package default'
            ),
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
        OpaqueFunction(function=_dynamic_ship_ground_truth_bridge),
        *camera_streams,
        OpaqueFunction(function=_safety_include),
        map_streamer,
    ])
