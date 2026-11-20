"""CCS water changes shading, not the geometry, wave interface or ray masks."""
import hashlib
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import pytest


PACKAGE = Path(__file__).resolve().parents[1]
WATER = PACKAGE / 'worlds/models/ccs_water'
VRX = PACKAGE.parent / 'vrx/vrx_gz/models/coast_waves'


def test_ccs_world_selects_local_water_without_new_wave_parameters():
    world = ET.parse(PACKAGE / 'worlds/ccs_open_water_crop.sdf').getroot().find('world')
    include = world.find("include[name='CCS Crop Waves']")
    assert include.findtext('uri') == 'model://ccs_water'
    assert include.findtext('pose') == '0 0 0 0 0 0'
    publisher = world.find("plugin[@name='vrx::PublisherPlugin']/message")
    assert publisher.get('topic') == '/vrx/wavefield/parameters'
    assert publisher.get('every') == '2.0'
    for key, value in (('direction', '0.0'), ('gain', '0.3'), ('period', '5'), ('steepness', '0')):
        assert f'key: "{key}" value {{ type: DOUBLE double_value: {value} }}' in publisher.text


def test_water_reuses_legacy_geometry_plugin_topic_and_visibility():
    model = ET.parse(WATER / 'model.sdf').getroot().find('model')
    old = ET.parse(VRX / 'model_36km.sdf').getroot().find('model')
    visual = model.find('link/visual')
    original = old.find('link/visual')
    assert model.findtext('static') == old.findtext('static') == 'true'
    assert model.find('link').get('name') == old.find('link').get('name')
    assert visual.get('name') == original.get('name')
    assert visual.findtext('geometry/mesh/uri') == 'model://coast_waves/meshes/waterlow.dae'
    assert visual.findtext('geometry/mesh/scale') == original.findtext('geometry/mesh/scale') == '6 6 1'
    assert visual.findtext('visibility_flags') == original.findtext('visibility_flags') == '8'
    assert model.findall('.//collision') == []
    plugin = visual.find('plugin')
    assert plugin.attrib == original.find('plugin').attrib
    assert len(model.findall('.//plugin')) == 1
    assert plugin.findtext('wavefield/topic') == '/vrx/wavefield/parameters'
    assert plugin.find('wavefield/wave') is None
    assert ET.parse(WATER / 'model.config').getroot().findtext('sdf') == 'model.sdf'


@pytest.mark.parametrize('name', ['wave_normals.dds', 'skybox_lowres.dds'])
def test_bundled_textures_are_identical_to_existing_vrx_assets(name):
    new = WATER / 'materials/textures' / name
    old = VRX / 'materials/textures' / name
    assert new.read_bytes()[:4] == b'DDS '
    assert hashlib.sha256(new.read_bytes()).digest() == hashlib.sha256(old.read_bytes()).digest()


def without_comments(source):
    source = re.sub(r'/\*.*?\*/', '', source, flags=re.S)
    return re.sub(r'//[^\n]*', '', source)


def test_vertex_displacement_is_unchanged():
    name = 'materials/programs/GerstnerWaves_vs_330.glsl'
    new = without_comments((WATER / name).read_text())
    old = without_comments((VRX / name).read_text())
    start = 'WaveParameters waves[3];'
    end = 'gl_Position = worldviewproj_matrix * P;'
    extract = lambda text: re.sub(r'\s+', '', text[text.index(start):text.index(end) + len(end)])
    assert extract(new) == extract(old)


def test_restored_texture_shaders_match_the_existing_plugin_interface():
    folder = WATER / 'materials/programs'
    vertex = without_comments((folder / 'GerstnerWaves_vs_330.glsl').read_text())
    fragment = without_comments((folder / 'GerstnerWaves_fs_330.glsl').read_text())
    assert 'uniform vec2 bumpScale;' in vertex
    assert 'uniform vec2 bumpSpeed;' in vertex
    assert 'outVs.bumpCoord = uv0.xy*bumpScale + drift;' in vertex
    assert 'currentAdvectionEnabled != 0 ? -currentUvOffset : t*bumpSpeed' in vertex
    assert 'uniform sampler2D bumpMap;' in fragment
    assert 'uniform samplerCube cubeMap;' in fragment
    assert 'texture(bumpMap, inPs.bumpCoord)' in fragment
    assert 'texture(cubeMap, R)' in fragment
    # The linker requires the same interpolated interface in both stages.
    interface = r'block\s*\{([^}]+)\}'
    assert re.search(interface, vertex).group(1) == re.search(interface, fragment).group(1)


def test_water_resources_and_rendering_parameters_are_local_and_finite():
    plugin = ET.parse(WATER / 'model.sdf').getroot().find('.//plugin')
    for stage in ('vertex', 'fragment'):
        assert (WATER / plugin.findtext('shader/' + stage)).is_file()
    parameters = plugin.find('shader/parameters')
    for color in ('shallowColor', 'deepColor'):
        rgba = list(map(float, parameters.findtext(color).split()))
        assert len(rgba) == 4 and all(0 <= value <= 1 for value in rgba)
    assert float(parameters.findtext('rescale')) > 0
    assert all(float(value) > 0 for value in parameters.findtext('bumpScale').split())


def test_current_advection_is_opt_in_and_reuses_the_physical_current_topic():
    plugin = ET.parse(WATER / 'model.sdf').getroot().find('.//plugin')
    assert plugin.findtext('current/topic') == '/ocean_current'
    assert plugin.findtext('current/size') == '36000 36000'
    assert plugin.findtext('current/uv_sign') == '1 -1'
    for name in ('model.sdf', 'model_36km.sdf'):
        assert ET.parse(VRX / name).getroot().find('.//plugin/current') is None
    # A velocity callback must not reload textures or reset the wave phase.
    source = (VRX.parents[1] / 'src/WaveVisual.cc').read_text()
    callback = without_comments(source.split('void WaveVisualPrivate::OnCurrent', 1)[1])
    assert 'paramsSet = false' not in callback
    assert 'ResetPhase' not in callback
    assert 'SetVelocity' in callback
