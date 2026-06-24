from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 参数声明
        DeclareLaunchArgument(
            'gz_topic',
            default_value='/blueboat/radar/spokes',
            description='Gazebo radar spokes topic'
        ),
        DeclareLaunchArgument(
            'ros_topic',
            default_value='/sensors/radar/nav/sector',
            description='ROS2 RadarSector output topic'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='nav_radar_link',
            description='TF frame ID for radar data'
        ),
        DeclareLaunchArgument(
            'range_min',
            default_value='5.0',
            description='Minimum range (m)'
        ),
        DeclareLaunchArgument(
            'range_max',
            default_value='500.0',
            description='Maximum range (m)'
        ),
        DeclareLaunchArgument(
            'rotation_period',
            default_value='2.5',
            description='Radar rotation period (seconds)'
        ),

        # 桥接节点
        Node(
            package='radar_gz_bridge',
            executable='radar_gz_bridge',
            name='radar_gz_bridge',
            output='screen',
            parameters=[{
                'gz_topic': LaunchConfiguration('gz_topic'),
                'ros_topic': LaunchConfiguration('ros_topic'),
                'frame_id': LaunchConfiguration('frame_id'),
                'range_min': LaunchConfiguration('range_min'),
                'range_max': LaunchConfiguration('range_max'),
                'rotation_period': LaunchConfiguration('rotation_period'),
            }],
        ),
    ])
