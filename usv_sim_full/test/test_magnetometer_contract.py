"""CCS SI magnetic measurements and per-sensor noise overrides."""

from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from usv_sim_full.scripts.session_manager import generate_sensors_overlay


@pytest.mark.parametrize('explicit_noise', [False, True])
def test_magnetometer_overlay_preserves_optional_noise_overrides(tmp_path, explicit_noise):
    sensor = {'name': 'mag_sensor', 'type': 'magnetometer', 'update_rate': 50}
    overrides = {
        'field_tesla_x': 0.0, 'field_tesla_y': 0.0, 'field_tesla_z': 0.0,
        'noise_stddev': 4.0e-11,
    }
    if explicit_noise:
        sensor.update(overrides)
    path = generate_sensors_overlay(
        {'sensors': [sensor]}, str(tmp_path), str(tmp_path / 'params.xacro'),
    )
    call = ET.parse(path).getroot().find('{http://www.ros.org/wiki/xacro}magnetometer_macro')
    assert call is not None
    for key, value in overrides.items():
        if explicit_noise:
            assert float(call.attrib[key]) == value
        else:
            assert key not in call.attrib


def test_ccs_magnetometer_system_uses_enu_tesla():
    world = Path(__file__).parents[1] / 'worlds/ccs_open_water_crop.sdf'
    plugin = ET.parse(world).getroot().find(
        "world/plugin[@name='gz::sim::systems::Magnetometer']",
    )
    assert plugin is not None
    assert plugin.findtext('use_units_gauss') == 'false'
    assert plugin.findtext('use_earth_frame_ned') == 'false'
