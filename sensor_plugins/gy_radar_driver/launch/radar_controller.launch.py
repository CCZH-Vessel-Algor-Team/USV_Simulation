"""
GY雷达驱动完整启动文件
启动所有雷达相关节点：控制、数据、ARPA、TF、建图
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('gy_radar_driver')
    default_params_file = os.path.join(pkg_dir, 'config', 'radar_params.yaml')

    # ==================== Launch 参数声明 ====================
    declared_args = [
        DeclareLaunchArgument('params_file', default_value=default_params_file,
                              description='参数文件路径'),
        DeclareLaunchArgument('namespace', default_value='',
                      description='机器人命名空间，例如 usv_1'),
        DeclareLaunchArgument('enable_control', default_value='true',
                              description='启用雷达控制节点'),
        DeclareLaunchArgument('enable_data', default_value='true',
                              description='启用雷达数据节点'),
        DeclareLaunchArgument('enable_arpa', default_value='true',
                              description='启用ARPA目标节点'),
        DeclareLaunchArgument('enable_tf', default_value='true',
                              description='启用TF发布节点'),
        DeclareLaunchArgument('enable_mapping', default_value='true',
                              description='启用建图节点'),
        DeclareLaunchArgument('enable_converter', default_value='true',
                      description='启用 RadarSector->PointCloud2 转换节点'),
        DeclareLaunchArgument('mapping_input_topic', default_value='/sensors/radar/nav/sector',
                      description='建图节点输入的RadarSector话题'),
        DeclareLaunchArgument('converter_input_topic', default_value='/sensors/radar/nav/sector',
                      description='转换节点输入的RadarSector话题'),
        DeclareLaunchArgument('converter_output_topic', default_value='/sensors/radar/nav/points',
                      description='转换节点输出PointCloud2话题'),
        DeclareLaunchArgument('mapping_output_gridmap_topic', default_value='map/navradar/gridmap',
                  description='建图节点输出GridMap话题'),
        DeclareLaunchArgument('mapping_output_occupancy_topic', default_value='map/navradar/occupancy_grid',
                  description='建图节点输出OccupancyGrid话题'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='使用仿真时间'),
    ]

    # 获取配置
    params_file = LaunchConfiguration('params_file')
    namespace = LaunchConfiguration('namespace')
    mapping_input_topic = LaunchConfiguration('mapping_input_topic')
    converter_input_topic = LaunchConfiguration('converter_input_topic')
    converter_output_topic = LaunchConfiguration('converter_output_topic')
    mapping_output_gridmap_topic = LaunchConfiguration('mapping_output_gridmap_topic')
    mapping_output_occupancy_topic = LaunchConfiguration('mapping_output_occupancy_topic')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ==================== 节点定义 ====================
    
    # 1. 雷达控制节点
    radar_control_node = Node(
        package='gy_radar_driver',
        executable='radar_control_node',
        name='radar_control_node',
        namespace=namespace,
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('enable_control')),
    )

    # 2. 雷达数据节点 (延迟1秒启动，等待控制节点初始化)
    radar_data_node = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='gy_radar_driver',
                executable='radar_data_node',
                name='radar_data_node',
                namespace=namespace,
                output='screen',
                parameters=[params_file, {'use_sim_time': use_sim_time}],
                condition=IfCondition(LaunchConfiguration('enable_data')),
            )
        ]
    )

    # 3. ARPA目标节点 (延迟2秒启动)
    arpa_receiver_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='gy_radar_driver',
                executable='arpa_receiver_node',
                name='arpa_receiver_node',
                namespace=namespace,
                output='screen',
                parameters=[params_file, {'use_sim_time': use_sim_time}],
                condition=IfCondition(LaunchConfiguration('enable_arpa')),
            )
        ]
    )

    # 4. TF发布节点
    radar_tf_node = TimerAction(
        period=2.5,
        actions=[
            Node(
                package='gy_radar_driver',
                executable='radar_tf_node',
                name='radar_tf_node',
                namespace=namespace,
                output='screen',
                parameters=[params_file, {'use_sim_time': use_sim_time}],
                condition=IfCondition(LaunchConfiguration('enable_tf')),
            )
        ]
    )

    # 5. 自适应建图节点 (延迟3秒启动，等待数据节点就绪)
    mapping_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='gy_radar_driver',
                executable='adaptive_radar_grid_map_node',
                name='adaptive_radar_grid_map_node',
                namespace=namespace,
                output='screen',
                parameters=[params_file, {'use_sim_time': use_sim_time}],
                remappings=[
                    ('/sensors/radar/nav/sector', mapping_input_topic),
                    ('/map/navradar/gridmap', mapping_output_gridmap_topic),
                    ('/map/navradar/occupancy_grid', mapping_output_occupancy_topic),
                ],
                condition=IfCondition(LaunchConfiguration('enable_mapping')),
            )
        ]
    )

    # 6. 雷达扇区转点云节点
    radar_converter_node = TimerAction(
        period=3.2,
        actions=[
            Node(
                package='gy_radar_driver',
                executable='radar_converter_node',
                name='radar_converter',
                namespace=namespace,
                output='screen',
                parameters=[params_file, {'use_sim_time': use_sim_time}],
                remappings=[
                    ('/sensors/radar/nav/sector', converter_input_topic),
                    ('/sensors/radar/nav/points', converter_output_topic),
                ],
                condition=IfCondition(LaunchConfiguration('enable_converter')),
            )
        ]
    )

    # ==================== 静态TF  ====================
    # static_tf_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='radar_static_tf',
    #     arguments=['0', '0', '2.0', '0', '0', '0', 'base_link', 'radar_link'],
    # )

    radar_nodes_group = GroupAction(
        actions=[
            PushRosNamespace(namespace=namespace),
            radar_control_node,
            radar_data_node,
            arpa_receiver_node,
            radar_tf_node,
            mapping_node,
            radar_converter_node,
        ]
    )

    return LaunchDescription(
        declared_args + [
            radar_nodes_group,
            # static_tf_node,
        ]
    )