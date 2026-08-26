"""Regression tests for the generated three-axis gimbal model and bridges."""

import os
import tempfile

import yaml
from usv_interfaces.msg import GimbalState

from usv_sim_full.scripts.session_manager import (
    generate_bridge_config,
    generate_sensors_overlay,
)


def _config():
    return {
        'robot': {'name': 'usv_1'},
        'sensors': [{
            'name': 'payload_gimbal',
            'type': 'gimbal',
            'parent_link': 'base_link',
            'xyz': [0.45, 0.0, 2.2],
            'rpy': [0.0, 0.0, 0.0],
            'image_topic': '/sensors/camera/payload_gimbal/image_raw',
            'imu_topic': '/sensors/imu/payload_gimbal/data',
            'enabled': True,
        }],
    }


def test_gimbal_overlay_contains_px4_axis_order():
    with tempfile.TemporaryDirectory() as session_dir:
        params = os.path.join(session_dir, 'sensor_params.xacro')
        with open(params, 'w', encoding='utf-8') as stream:
            stream.write(
                '<robot xmlns:xacro="http://www.ros.org/wiki/xacro"/>'
            )
        overlay = generate_sensors_overlay(_config(), session_dir, params)
        with open(overlay, encoding='utf-8') as stream:
            content = stream.read()
    assert '<xacro:gimbal_macro' in content
    assert 'command_prefix="command/payload_gimbal"' in content


def test_gimbal_bridge_has_commands_feedback_and_sensor_topics():
    bridges = generate_bridge_config(_config())
    by_ros_topic = {bridge['ros_topic_name']: bridge for bridge in bridges}
    joint_state_bridge = by_ros_topic['/usv_1/joint_states']
    assert joint_state_bridge['gz_topic_name'] == (
        '/world/sydney_regatta/model/usv_1/joint_state'
    )
    for axis in ('roll', 'pitch', 'yaw'):
        bridge = by_ros_topic[
            f'/usv_1/gimbal/payload_gimbal/{axis}/cmd_pos'
        ]
        assert bridge['gz_topic_name'] == (
            f'/model/usv_1/command/payload_gimbal/{axis}'
        )
        assert bridge['direction'] == 'ROS_TO_GZ'
    assert '/usv_1/sensors/camera/payload_gimbal/image_raw' in by_ros_topic
    assert '/usv_1/sensors/camera/payload_gimbal/camera_info' in by_ros_topic
    assert '/usv_1/sensors/imu/payload_gimbal/data' in by_ros_topic


def test_three_vision_config_enables_the_payload_gimbal():
    config_path = os.path.join(
        os.path.dirname(__file__),
        '..',
        'config',
        'three_vision_one_mmwave',
        'full_config.yaml',
    )
    with open(config_path, encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    gimbals = [
        sensor for sensor in config['robot_1']['sensors']
        if sensor.get('type') == 'gimbal' and sensor.get('enabled')
    ]
    assert [sensor['name'] for sensor in gimbals] == ['payload_gimbal']


def test_px4_gimbal_visual_meshes_are_packaged():
    mesh_dir = os.path.join(
        os.path.dirname(__file__),
        '..',
        'description',
        'models',
        'px4_gimbal',
        'meshes',
    )
    for mesh_name in (
        'cgo3_mount_remeshed_v1.stl',
        'cgo3_vertical_arm_remeshed_v1.stl',
        'cgo3_horizontal_arm_remeshed_v1.stl',
        'cgo3_camera_remeshed_v1.stl',
    ):
        assert os.path.isfile(os.path.join(mesh_dir, mesh_name))


def test_gimbal_state_accepts_float32_rate_feedback():
    message = GimbalState()
    message.angle_rt_rate = [0.0, 1.0, -1.0]
    message.angle_rate_set = [float(rate) for rate in message.angle_rt_rate]
    assert list(message.angle_rate_set) == [0.0, 1.0, -1.0]
