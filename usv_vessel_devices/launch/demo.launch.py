"""Standalone lights/mast test deck, independent of navigation and perception."""
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def launch_demo(context):
    share = get_package_share_directory('usv_vessel_devices')
    namespace = LaunchConfiguration('namespace').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    description = xacro.process_file(
        os.path.join(share, 'description', 'demo.urdf.xacro'),
        mappings={'namespace': namespace,
                  'config_file': LaunchConfiguration('config_file').perform(context)},
    ).toxml()
    world = os.path.join(share, 'worlds', 'devices_demo.sdf')
    plugin_dir = os.path.join(get_package_prefix('usv_vessel_devices'), 'lib')
    # ros_gz_sim reads os.environ when assembling its server environment.
    os.environ['GZ_SIM_SYSTEM_PLUGIN_PATH'] = os.pathsep.join(filter(None, [
        plugin_dir, os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''),
    ]))
    arguments = f'-r -s --headless-rendering {world}' if headless else f'-r {world}'
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': arguments}.items(),
        ),
        Node(package='ros_gz_sim', executable='create', arguments=[
            '-world', 'devices_demo', '-name', namespace, '-string', description,
        ], output='screen'),
    ]


def generate_launch_description():
    share = get_package_share_directory('usv_vessel_devices')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='usv_1'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('config_file', default_value=os.path.join(share, 'config', 'devices.yaml')),
        OpaqueFunction(function=launch_demo),
    ])
