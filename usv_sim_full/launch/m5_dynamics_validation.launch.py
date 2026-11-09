"""M5 identified-planar-model dynamics validation entry (v1, no control chain).

Materializes a private world (company CCS open-water scene) with the M5 model
embedded, mirroring the proven fossen_model2 pattern: package URIs rewritten to
the installed share, model pose configurable for floating-equilibrium tuning.

Validation drives the boat by publishing per-side power commands on
/usv_1/thrusters/{left,right}/power (kW) and records /usv_1/fossen/state
telemetry plus /usv_1/odom; a task-local RK4 reference (outside this repo)
cross-checks the closed model.

The Gazebo GUI is optional and off by default; validation is data-driven.
"""

import os
import tempfile
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.actions import OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    """Build the private world, bridges and sim processes at launch time.

    :param context: Launch context.
    :return: Actions for the validated M5 world and bridges.
    """
    value = lambda key: LaunchConfiguration(key).perform(context)  # noqa: E731
    share = get_package_share_directory('usv_sim_full')

    world = ET.parse(os.path.join(share, 'worlds', 'ccs_open_water_crop.sdf'))
    model = ET.parse(
        os.path.join(share, 'description', 'models', 'm5', 'model.sdf')
    ).find('model')
    for uri in model.findall('.//uri'):
        uri.text = uri.text.replace('package://usv_sim_full', share)
    # Floating equilibrium: buoyancy points sit ~0.28 m below the waterline,
    # so the model pose z drives the settling behaviour during validation.
    pose = model.find('pose')
    parts = pose.text.split()
    parts[2] = value('spawn_z')
    pose.text = ' '.join(parts)
    # The Surface plugin materializes its SDF wavefield (PMS gain 0.3) even
    # without an external publisher, and the company world additionally runs
    # a vrx::PublisherPlugin re-publishing wavefield parameters (gain 0.3)
    # every 2 s, which overrides the model block. The RK4 reference is a
    # calm-water model, so validation rewrites BOTH to wave_gain (default 0);
    # scene reuse passes 0.3 to keep the company behaviour.
    gain = model.find('.//wave/gain')
    if gain is not None:
        gain.text = value('wave_gain')
    import re as _re
    for message in world.iter('message'):
        if message.get('topic', '') == '/vrx/wavefield/parameters':
            message.text = _re.sub(
                r'(key: "gain" value \{ type: DOUBLE double_value: )[0-9.]+',
                r'\g<1>' + value('wave_gain'), message.text or '')
    world.find('world').append(model)

    session = tempfile.mkdtemp(prefix='m5_validation_')
    world_path = os.path.join(session, 'world.sdf')
    world.write(world_path, encoding='unicode', xml_declaration=True)

    # model:// assets of the CCS scene (coast_waves visual etc.) live in the
    # vrx_gz share; mesh URIs were made absolute above.
    vrx_models = os.path.join(
        os.path.dirname(get_package_share_directory('vrx_gz')), 'vrx_gz', 'models')
    resource = ':'.join(path for path in (
        vrx_models,
        os.path.join(share, 'worlds'), os.path.join(share, 'worlds', 'models'),
        os.path.join(share, 'description'),
        os.path.join(share, 'description', 'models'),
        os.environ.get('GZ_SIM_RESOURCE_PATH', '')) if path)

    actions = [
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=resource),
        SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=resource),
        ExecuteProcess(cmd=['gz', 'sim', '-r', '-s', world_path], output='screen'),
    ]
    if value('gui').lower() == 'true':
        actions.append(ExecuteProcess(cmd=['gz', 'sim', '-g'], output='screen'))

    bridge = [
        {'ros_topic_name': '/clock', 'gz_topic_name': '/clock',
         'ros_type_name': 'rosgraph_msgs/msg/Clock', 'gz_type_name': 'gz.msgs.Clock',
         'direction': 'GZ_TO_ROS'},
        {'ros_topic_name': '/usv_1/odom', 'gz_topic_name': '/usv_1/odom',
         'ros_type_name': 'nav_msgs/msg/Odometry', 'gz_type_name': 'gz.msgs.Odometry',
         'direction': 'GZ_TO_ROS'},
    ]
    for side in ('left', 'right'):
        topic = '/usv_1/thrusters/' + side + '/power'
        bridge.append({'ros_topic_name': topic, 'gz_topic_name': topic,
                       'ros_type_name': 'std_msgs/msg/Float64',
                       'gz_type_name': 'gz.msgs.Double', 'direction': 'ROS_TO_GZ'})
    bridge_path = os.path.join(session, 'bridge.yaml')
    with open(bridge_path, 'w', encoding='utf-8') as stream:
        yaml.safe_dump(bridge, stream)
    actions.append(Node(package='ros_gz_bridge', executable='parameter_bridge',
                        parameters=[{'config_file': bridge_path, 'use_sim_time': True}],
                        output='screen'))
    actions.append(Node(package='usv_sim_full', executable='odom_tf_broadcaster',
                        parameters=[{'use_sim_time': True}], output='screen'))
    return actions


def generate_launch_description():
    """Declare arguments and defer world materialization to launch time."""
    arguments = [
        DeclareLaunchArgument(
            'gui', default_value='false',
            description='Start the Gazebo GUI alongside the server (default off).'),
        DeclareLaunchArgument(
            'spawn_z', default_value='-0.28',
            description='Model spawn z in metres; -0.28 is the measured '
                        'buoyancy equilibrium for the 1800 kg carrier.'),
        DeclareLaunchArgument(
            'wave_gain', default_value='0.0',
            description='Surface PMS wavefield gain; 0.0 = calm water for the '
                        'RK4 reference comparison, 0.3 = company scene reuse.'),
    ]
    return LaunchDescription(arguments + [OpaqueFunction(function=_setup)])
