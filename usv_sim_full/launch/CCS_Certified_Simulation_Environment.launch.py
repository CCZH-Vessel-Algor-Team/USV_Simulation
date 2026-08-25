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
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
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
            msg='启动 dynamic_ship_to_ground_truth（/dynamic_ship/tracked_ships -> /sim/ground_truth）。'
        ),
        Node(
            package='ground_truth_sensor_sim',
            executable='dynamic_ship_to_ground_truth',
            name='dynamic_ship_to_ground_truth',
            output='log',
            parameters=[{
                'use_sim_time': use_sim_time,
                'input_topic': '/dynamic_ship/tracked_ships',
                'output_topic': '/sim/ground_truth',
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


def _sim_ais_node(context, *args, **kwargs):
    """可选启动 /sim/ground_truth 到 AIS 报文的仿真节点。"""
    enable = LaunchConfiguration('enable_ais_sim').perform(context).strip().lower()
    if enable not in ('true', '1', 'yes'):
        return [LogInfo(msg='enable_ais_sim:=false，跳过 AIS 仿真节点。')]

    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    return [
        LogInfo(msg='启动 sim_ais_node（/sim/ground_truth -> /perception/ais/*）。'),
        Node(
            package='ground_truth_sensor_sim',
            executable='sim_ais_node',
            name='sim_ais_node',
            output='log',
            parameters=[{
                'use_sim_time': use_sim_time,
                'input_topic': '/sim/ground_truth',
                'origin_latitude': LaunchConfiguration('ais_origin_latitude'),
                'origin_longitude': LaunchConfiguration('ais_origin_longitude'),
                'location_period_sec': LaunchConfiguration('ais_location_period_sec'),
                'only_ais_matched': False,
                'auto_assign_mmsi': True,
                'vessel_name_prefix': 'CCS_SIM_',
            }],
        ),
    ]


def _ais_aggregator_node(context, *args, **kwargs):
    """可选启动 AIS 报文汇聚节点，生成 snapshot、catalog 与 tracks。"""
    enable = LaunchConfiguration('enable_ais_aggregator').perform(context).strip().lower()
    if enable not in ('true', '1', 'yes'):
        return [LogInfo(msg='enable_ais_aggregator:=false，跳过 AIS 汇聚节点。')]

    config_path = LaunchConfiguration('config_path').perform(context)
    namespace = _resolve_robot_namespace(config_path)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    return [
        LogInfo(msg='启动 ais_aggregator_node（AIS 报文 -> snapshot/catalog/tracks）。'),
        Node(
            package='ais_perception',
            executable='ais_aggregator_node',
            name='ais_aggregator_node',
            output='log',
            parameters=[
                LaunchConfiguration('ais_aggregator_params_file'),
                {
                    'use_sim_time': use_sim_time,
                    'os_name': namespace,
                },
            ],
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
                # CCS publishes the only map->odom transform below.
                'publish_identity_map_odom_tf': 'false',
            }.items(),
        ),
    ]


def _ccs_map_to_odom_tf(context, *args, **kwargs):
    """Connect chart/ENU frames and provide Humble Nav2's base_link alias."""
    config_path = LaunchConfiguration('config_path').perform(context)
    ns = _resolve_robot_namespace(config_path)
    return [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='ccs_map_to_odom_tf',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            arguments=[
                '--x', LaunchConfiguration('ccs_map_to_odom_x'),
                '--y', LaunchConfiguration('ccs_map_to_odom_y'),
                '--z', '0.0',
                '--roll', '0.0',
                '--pitch', '0.0',
                '--yaw', LaunchConfiguration('ccs_map_to_odom_yaw'),
                '--frame-id', 'map',
                '--child-frame-id', f'{ns}/odom',
            ],
        ),
        # Humble NavigateThroughPoses internally requests the conventional
        # un-namespaced base_link even when bt_navigator is namespaced.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='ccs_base_link_compat_tf',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            arguments=[
                '--x', '0.0',
                '--y', '0.0',
                '--z', '0.0',
                '--roll', '0.0',
                '--pitch', '0.0',
                '--yaw', '0.0',
                '--frame-id', f'{ns}/base_link',
                '--child-frame-id', 'base_link',
            ],
        ),
    ]


def _gazebo_camera_follow(context, *args, **kwargs):
    """Make the Gazebo GUI camera follow the CCS vessel after it spawns."""
    enable = (
        LaunchConfiguration('enable_gazebo_camera_follow')
        .perform(context)
        .strip()
        .lower()
    )
    if enable not in ('true', '1', 'yes'):
        return []

    ns = _resolve_robot_namespace(
        LaunchConfiguration('config_path').perform(context)
    )
    return [
        TimerAction(
            period=8.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'gz', 'service',
                        '-s', '/gui/follow',
                        '--reqtype', 'gz.msgs.StringMsg',
                        '--reptype', 'gz.msgs.Boolean',
                        '--timeout', '5000',
                        '--req', f'data: "{ns}"',
                    ],
                    output='log',
                ),
            ],
        ),
    ]


def generate_launch_description():
    usv_sim_full_share = get_package_share_directory('usv_sim_full')
    usv_vision_share = get_package_share_directory('usv_vision')
    map_streamer_share = get_package_share_directory('usv_map_rtsp_streamer')
    ais_perception_share = get_package_share_directory('ais_perception')

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
            # CCS publishes the map -> odom transform below.
            'use_static_map_odom_tf': 'false',
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

    route_planner_launch_file = os.path.join(
        get_package_share_directory('usv_route_planner'),
        'launch',
        'route_planner.launch.py',
    )
    # 显式指定 route planner 的参数文件：父链中的 params_file 已被 Nav2
    # bringup 占用（radar_nav2_param.yaml），同名 LaunchConfiguration 会覆盖
    # route_planner.launch.py 的默认值，导致节点加载错误参数。
    route_planner_params_file = os.path.join(
        get_package_share_directory('usv_route_planner'),
        'config',
        'route_planner.yaml',
    )
    route_planner_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(route_planner_launch_file),
        condition=IfCondition(LaunchConfiguration('enable_route_planner')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': route_planner_params_file,
        }.items(),
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
            'ccs_map_to_odom_x',
            default_value='0.0',
            description='ENC-local ENU map to Gazebo odom translation (x)',
        ),
        DeclareLaunchArgument(
            'ccs_map_to_odom_y',
            default_value='0.0',
            description='ENC-local ENU map to Gazebo odom translation (y)',
        ),
        DeclareLaunchArgument(
            'ccs_map_to_odom_yaw',
            default_value='0.0',
            description='ENC-local ENU map to Gazebo odom yaw in radians',
        ),
        DeclareLaunchArgument(
            'enable_dynamic_ship_gt_bridge',
            default_value='true',
            description='true：启动 /dynamic_ship/tracked_ships -> /sim/ground_truth 转换节点',
        ),
        DeclareLaunchArgument(
            'enable_ais_sim',
            default_value='true',
            description='true：启动 /sim/ground_truth 到 AIS 报文的仿真节点',
        ),
        DeclareLaunchArgument(
            'ais_origin_latitude',
            default_value='34.692120',
            description='AIS 仿真 ENU 原点纬度（WGS84）',
        ),
        DeclareLaunchArgument(
            'ais_origin_longitude',
            default_value='119.481403',
            description='AIS 仿真 ENU 原点经度（WGS84）',
        ),
        DeclareLaunchArgument(
            'ais_location_period_sec',
            default_value='1.0',
            description='AIS 位置报告发布周期（秒）',
        ),
        DeclareLaunchArgument(
            'enable_ais_aggregator',
            default_value='true',
            description='true：汇聚 AIS 报文并发布 snapshot/catalog/tracks',
        ),
        DeclareLaunchArgument(
            'ais_aggregator_params_file',
            default_value=os.path.join(
                ais_perception_share, 'config', 'ais_params.yaml'
            ),
            description='ais_aggregator_node 参数文件',
        ),
        DeclareLaunchArgument(
            'enable_gazebo_camera_follow',
            default_value='true',
            description='Follow the CCS vessel in the Gazebo GUI; press Esc to release',
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
        DeclareLaunchArgument(
            'enable_route_planner',
            default_value='true',
            description='true：随仿真一起启动 usv_route_planner 选路节点',
        ),
        LogInfo(msg=['Starting CCS certified simulation from: ', config_path]),
        base_bringup,
        route_planner_include,
        OpaqueFunction(function=_ccs_map_to_odom_tf),
        OpaqueFunction(function=_gazebo_camera_follow),
        OpaqueFunction(function=_dynamic_ship_ground_truth_bridge),
        OpaqueFunction(function=_sim_ais_node),
        OpaqueFunction(function=_ais_aggregator_node),
        *camera_streams,
        OpaqueFunction(function=_safety_include),
        map_streamer,
    ])
