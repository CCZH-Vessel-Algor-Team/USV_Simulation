"""Attachments preserve legacy propulsion; outboard tilt is explicitly opt-in."""
from pathlib import Path
import copy
import math
import subprocess
import xml.etree.ElementTree as ET

import pytest
import yaml

from usv_sim_full.scripts import session_manager as session

PACKAGE = Path(__file__).resolve().parents[2] / 'usv_vessel_devices'


@pytest.fixture
def device_share(monkeypatch):
    original = session.get_package_share_directory
    monkeypatch.setattr(session, 'get_package_share_directory',
                        lambda name: str(PACKAGE) if name == 'usv_vessel_devices' else original(name))


def test_legacy_configuration_does_not_load_devices(monkeypatch, tmp_path):
    def unexpected_lookup(_):
        raise AssertionError('disabled attachments must not resolve the new package')
    monkeypatch.setattr(session, 'get_package_share_directory', unexpected_lookup)
    assert session.generate_vessel_devices_config({}, str(tmp_path), 'boat.urdf') is None
    assert session.generate_vessel_devices_config(
        {'vessel_devices': {'enabled': False}}, str(tmp_path), 'boat.urdf') is None


def test_per_boat_config_merges_defaults_and_uses_unique_paths(device_share, tmp_path):
    config = {'xacro_template': 'wamv_no_battery.urdf.xacro',
              'vessel_devices': {'enabled': True, 'mast_speed': 0.2}}
    one = session.generate_vessel_devices_config(config, str(tmp_path), 'boat_1.urdf')
    two = session.generate_vessel_devices_config(config, str(tmp_path), 'boat_2.urdf')
    assert one != two
    loaded = yaml.safe_load(Path(one).read_text())
    assert loaded['mast_speed'] == 0.2
    assert loaded['left_y'] > loaded['right_y']
    assert loaded['roof_lights_enabled'] is True


@pytest.mark.parametrize('settings', [
    {'enabled': 'false'}, {'enabled': True, 'mast_speed': -1},
    {'enabled': True, 'roof_z': float('nan')}, {'enabled': True, 'mast_xyz': [0, 1]},
    {'enabled': True, 'mast_xyz': 'wrong'}, {'enabled': True, 'mast_length': True},
    {'enabled': True, 'mast_max_angle': 10}, {'enabled': True, 'mast_tolerance': 2},
    {'enabled': True, 'typo': 1}, {'enabled': True, 'left_y': -1},
    {'enabled': True, 'mast_use_cad': 'true'}, {'enabled': True, 'lamp_use_cad': 1},
    {'enabled': True, 'lamp_scale': 0}, {'enabled': True, 'mast_base_mass': -0.1},
    {'enabled': True, 'lights_on_mast': 'true'}, {'enabled': True, 'mast_lamp_scale': 0},
    {'enabled': True, 'extra_lights_enabled': 'true'},
    {'enabled': True, 'roof_lights_enabled': 'true'},
    {'enabled': True, 'extra_light_colors': None},
    {'enabled': True, 'extra_light_colors': {'typo': 'red'}},
    {'enabled': True, 'extra_light_colors': {'aft_lower': 'white'}},
    {'enabled': True, 'extra_light_colors': {'aft_lower': True}},
    {'enabled': True, 'extra_light_colors': {1: 'red'}},
    {'enabled': True, 'thruster_tilt_enabled': 'true'},
    {'enabled': True, 'thruster_tilt_max_angle': 2.0},
    {'enabled': True, 'thruster_tilt_max_angle': math.radians(75.001)},
    {'enabled': True, 'thruster_tilt_max_angle': math.pi / 2},
    {'enabled': True, 'thruster_tilt_tolerance': 2.0},
    {'enabled': True, 'thruster_tilt_speed': 0},
    {'enabled': True, 'thruster_tilt_effort': -1},
    {'enabled': True, 'thruster_tilt_speed': float('inf')},
    {'enabled': True, 'thruster_tilt_max_angle': float('nan')},
])
def test_invalid_config_is_rejected(device_share, tmp_path, settings):
    with pytest.raises(ValueError):
        session.generate_vessel_devices_config(
            {'xacro_template': 'wamv_no_battery.urdf.xacro', 'vessel_devices': settings},
            str(tmp_path), 'boat.urdf')


def test_unsupported_template_fails_explicitly(device_share, tmp_path):
    with pytest.raises(ValueError, match='requires'):
        session.generate_vessel_devices_config(
            {'vessel_devices': {'enabled': True}}, str(tmp_path), 'boat.urdf')


@pytest.mark.parametrize('layout', ['H', 'T', 'X'])
def test_tilt_rejects_unsupported_thruster_layout(device_share, tmp_path, layout):
    with pytest.raises(ValueError, match='CUSTOM'):
        session.generate_vessel_devices_config({
            'xacro_template': 'wamv_no_battery.urdf.xacro', 'thruster_config': layout,
            'vessel_devices': {'enabled': True, 'thruster_tilt_enabled': True},
        }, str(tmp_path), 'boat.urdf')


def test_tilt_rejects_yaml_thruster_override(device_share, tmp_path):
    with pytest.raises(ValueError, match='CUSTOM'):
        session.generate_vessel_devices_config({
            'xacro_template': 'wamv_no_battery.urdf.xacro', 'thruster_config': 'CUSTOM',
            'overrides': {'yaml_thruster_generation': True},
            'vessel_devices': {'enabled': True, 'thruster_tilt_enabled': True},
        }, str(tmp_path), 'boat.urdf')


def test_extra_color_overrides_merge_defaults(device_share, tmp_path):
    path = session.generate_vessel_devices_config({
        'xacro_template': 'wamv_no_battery.urdf.xacro',
        'vessel_devices': {'enabled': True, 'extra_light_colors': {'aft_lower': 'green'}},
    }, str(tmp_path), 'boat.urdf')
    colors = yaml.safe_load(Path(path).read_text())['extra_light_colors']
    assert len(colors) == 7
    assert colors['aft_lower'] == colors['aft_middle'] == 'green'
    assert colors['forward_upper'] == 'red'


def test_propulsion_bridges_are_unchanged_and_joint_feedback_not_duplicated():
    legacy = {'robot': {'name': 'test_boat'}, 'visualization': {'enable_telemetry': False}}
    enabled = {**legacy, 'robot': {'name': 'test_boat', 'vessel_devices': {'enabled': True}}}
    old_bridges = session.generate_bridge_config(legacy)
    new_bridges = session.generate_bridge_config(enabled)
    assert [b for b in new_bridges if '/thrusters/' in b['ros_topic_name']] == [
        b for b in old_bridges if '/thrusters/' in b['ros_topic_name']]
    assert len([b for b in new_bridges if b['ros_type_name'] == 'sensor_msgs/msg/JointState']) == 1
    enabled['visualization']['enable_telemetry'] = True
    assert len([b for b in session.generate_bridge_config(enabled)
                if b['ros_type_name'] == 'sensor_msgs/msg/JointState']) == 1


def test_real_boat_template_keeps_existing_joints_and_sensor_mounts(device_share, tmp_path):
    full = yaml.safe_load(Path(session.resolve_package_asset('config/full_config.yaml')).read_text())
    effective = session.build_effective_robot_config(full, full['robot_1'])
    params, _ = session.generate_session_sensor_params_xacro(
        full, session.resolve_package_asset('config/full_config.yaml'), str(tmp_path))
    session.generate_sensors_overlay(effective, str(tmp_path), params)
    enabled_path = session.compile_xacro_to_urdf(None, effective, str(tmp_path),
                                                urdf_filename='with_devices.urdf')
    legacy = copy.deepcopy(effective)
    legacy['robot'].pop('vessel_devices', None)
    legacy_path = session.compile_xacro_to_urdf(None, legacy, str(tmp_path),
                                               urdf_filename='without_devices.urdf')
    enabled_root = ET.parse(enabled_path).getroot()
    legacy_root = ET.parse(legacy_path).getroot()
    def canonical(element):
        return ET.canonicalize(ET.tostring(element).decode(), strip_text=True)

    for original in legacy_root.findall('joint'):
        unchanged = enabled_root.find(f"joint[@name='{original.attrib['name']}']")
        assert canonical(unchanged) == canonical(original)
    for original in legacy_root.findall('gazebo'):
        # All existing dynamics plugins and sensor descriptions are untouched.
        assert any(canonical(candidate) == canonical(original)
                   for candidate in enabled_root.findall('gazebo'))
    assert enabled_root.find("joint[@name='usv_1/mast_joint']") is not None
    assert legacy_root.find("joint[@name='usv_1/mast_joint']") is None
    converted = subprocess.run(['gz', 'sdf', '-p', enabled_path],
                               capture_output=True, text=True, check=True)
    assert ET.fromstring(converted.stdout).find("model/joint[@name='usv_1/mast_joint']") is not None


def test_tilt_is_opt_in_and_preserves_lowered_geometry_and_steering(device_share, tmp_path):
    full = yaml.safe_load(Path(session.resolve_package_asset('config/full_config.yaml')).read_text())
    effective = session.build_effective_robot_config(full, full['robot_1'])
    effective['robot']['vessel_devices']['thruster_tilt_enabled'] = False
    disabled = ET.parse(session.compile_xacro_to_urdf(
        None, effective, str(tmp_path), urdf_filename='tilt_off.urdf')).getroot()
    effective['robot']['vessel_devices']['thruster_tilt_enabled'] = True
    enabled_path = session.compile_xacro_to_urdf(
        None, effective, str(tmp_path), urdf_filename='tilt_on.urdf')
    enabled = ET.parse(enabled_path).getroot()
    steering_names = {f'usv_1/{side}_chassis_engine_joint' for side in ('left', 'right')}

    def canonical(element):
        return ET.canonicalize(ET.tostring(element).decode(), strip_text=True)

    for original in disabled.findall('link'):
        assert canonical(enabled.find(f"link[@name='{original.get('name')}']")) == canonical(original)
    for original in disabled.findall('joint'):
        if original.get('name') not in steering_names:
            assert canonical(enabled.find(f"joint[@name='{original.get('name')}']")) == canonical(original)
    for side in ('left', 'right'):
        assert disabled.find(f"joint[@name='usv_1/{side}_tilt_joint']") is None
        old_steering = disabled.find(f"joint[@name='usv_1/{side}_chassis_engine_joint']")
        steering = enabled.find(f"joint[@name='usv_1/{side}_chassis_engine_joint']")
        tilt = enabled.find(f"joint[@name='usv_1/{side}_tilt_joint']")
        assert float(tilt.find('limit').get('upper')) <= math.radians(75.0)
        assert tilt.get('type') == 'revolute'
        assert tilt.find('axis').get('xyz') == '0 1 0'
        assert tilt.find('origin').attrib == old_steering.find('origin').attrib
        assert tilt.find('parent').get('link') == 'usv_1/base_link'
        assert tilt.find('child').get('link') == steering.find('parent').get('link')
        assert steering.find('origin').get('xyz') == '0 0 0'
        assert steering.find('origin').get('rpy') == '0 0 0'
        assert canonical(steering.find('axis')) == canonical(old_steering.find('axis'))
        assert canonical(steering.find('limit')) == canonical(old_steering.find('limit'))
    # The tilt plugin owns propulsion when enabled; native plugins must not
    # also apply the same force. Legacy models keep their original plugins.
    original_plugins = disabled.findall("gazebo/plugin[@name='gz::sim::systems::Thruster']")
    new_plugins = enabled.findall("gazebo/plugin[@name='gz::sim::systems::Thruster']")
    assert len(original_plugins) == 2
    assert not new_plugins
    old_steering = disabled.findall("gazebo/plugin[@name='gz::sim::systems::JointPositionController']")
    new_steering = enabled.findall("gazebo/plugin[@name='gz::sim::systems::JointPositionController']")
    assert list(map(canonical, old_steering)) == list(map(canonical, new_steering))
    assert len(enabled.findall("gazebo/plugin[@name='usv_vessel_devices::ThrusterTilt']")) == 1
    converted = subprocess.run(['gz', 'sdf', '-p', enabled_path],
                               capture_output=True, text=True, check=True)
    sdf = ET.fromstring(converted.stdout).find('model')
    for side in ('left', 'right'):
        assert sdf.find(f"joint[@name='usv_1/{side}_tilt_joint']") is not None
