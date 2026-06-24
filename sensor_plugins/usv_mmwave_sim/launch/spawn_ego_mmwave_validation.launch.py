"""
在已运行的 Gazebo Sim 世界中 spawn 静态毫米波探测体（FourDRadarPlugin 直发 ROS 点云）。

须保证启动 gz-sim 的进程已设置 GZ_SIM_SYSTEM_PLUGIN_PATH（含本包 install/.../lib），
否则新插入的模型无法加载 libusv_4d_radar_plugin.so。整机仿真见 usv_sim_full 的 infra_sim。

在已运行 Gz 世界中调试时可单独启动本 launch（参数见下方 DeclareLaunchArgument）。
"""
import os
import tempfile

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import SetEnvironmentVariable
from launch.actions import TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_PROBE_SDF_TEMPLATE = """<?xml version="1.0"?>
<sdf version="1.10">
  <model name="{spawn_name}">
    <static>true</static>
    <link name="base_link">
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.1</ixx><iyy>0.1</iyy><izz>0.1</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <collision name="c">
        <geometry><box><size>0.2 0.2 0.1</size></box></geometry>
      </collision>
      <visual name="v">
        <geometry><box><size>0.2 0.2 0.1</size></box></geometry>
        <material><diffuse>0.2 0.6 0.9 1</diffuse></material>
      </visual>
    </link>
    <plugin filename="libusv_4d_radar_plugin.so" name="usv_4d_radar_gz::FourDRadarPlugin">
      <topic>{topic}</topic>
      <frame_id>world</frame_id>
      <ego_link_name>base_link</ego_link_name>
      <horizontal_fov_deg>100.0</horizontal_fov_deg>
      <vertical_fov_deg>30.0</vertical_fov_deg>
      <azimuth_resolution_deg>2.0</azimuth_resolution_deg>
      <elevation_resolution_deg>2.0</elevation_resolution_deg>
      <min_range>0.8</min_range>
      <max_range>120.0</max_range>
      <update_rate_hz>10.0</update_rate_hz>
      <base_rcs>12.0</base_rcs>
      <rcs_distance_decay>0.01</rcs_distance_decay>
      <enable_sea_clutter>false</enable_sea_clutter>
      <sea_clutter_probability>0.0</sea_clutter_probability>
      <sea_clutter_amplitude>0.0</sea_clutter_amplitude>
      <perception_range_limit_m>300.0</perception_range_limit_m>
      <enable_range_measurement_error>false</enable_range_measurement_error>
      <enable_azimuth_measurement_error>false</enable_azimuth_measurement_error>
    </plugin>
  </model>
</sdf>
"""


def _spawn_probe(context, *args, **kwargs):
    world = LaunchConfiguration('world').perform(context)
    topic = LaunchConfiguration('topic').perform(context)
    spawn_name = LaunchConfiguration('spawn_name').perform(context)
    delay_s = float(LaunchConfiguration('delay_s').perform(context))
    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    z = LaunchConfiguration('z').perform(context)
    roll = LaunchConfiguration('R').perform(context)
    pitch = LaunchConfiguration('P').perform(context)
    yaw = LaunchConfiguration('Y').perform(context)

    sdf = _PROBE_SDF_TEMPLATE.format(spawn_name=spawn_name, topic=topic)
    fd, path = tempfile.mkstemp(prefix='ego_mmwave_', suffix='.sdf', text=True)
    with os.fdopen(fd, 'w') as f:
        f.write(sdf)

    safe = ''.join(c if c.isalnum() or c == '_' else '_' for c in spawn_name)
    arg_list = [
        '-world', world,
        '-name', spawn_name,
        '-x', x, '-y', y, '-z', z,
        '-R', roll, '-P', pitch, '-Y', yaw,
        '-file', path,
    ]
    create = Node(
        package='ros_gz_sim',
        executable='create',
        name=f'gz_create_mmwave_probe_{safe}',
        output='screen',
        arguments=arg_list,
    )
    return [TimerAction(period=delay_s, actions=[create])]


def generate_launch_description():
    pkg_prefix = get_package_prefix('usv_mmwave_sim')
    plugin_lib = os.path.join(pkg_prefix, 'lib')
    old = os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
    merged = plugin_lib if not old else plugin_lib + os.pathsep + old

    return LaunchDescription([
        SetEnvironmentVariable(name='GZ_SIM_SYSTEM_PLUGIN_PATH', value=merged),
        DeclareLaunchArgument(
            'world',
            default_value='sydney_regatta',
            description='与当前运行中的 gz 世界名一致（SDF <world name="...">）',
        ),
        DeclareLaunchArgument('topic', default_value='/mmwave_spawn_test/points'),
        DeclareLaunchArgument('spawn_name', default_value='mmwave_spawn_test'),
        DeclareLaunchArgument('x', default_value='15.0'),
        DeclareLaunchArgument('y', default_value='15.0'),
        DeclareLaunchArgument('z', default_value='3.0'),
        DeclareLaunchArgument('R', default_value='0.0'),
        DeclareLaunchArgument('P', default_value='0.0'),
        DeclareLaunchArgument('Y', default_value='0.0'),
        DeclareLaunchArgument(
            'delay_s',
            default_value='3.0',
            description='延迟后再 create，避免 gz 尚未就绪',
        ),
        OpaqueFunction(function=_spawn_probe),
    ])
