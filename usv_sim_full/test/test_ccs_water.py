"""CCS water geometry and shading must not affect physics or sensor ray masks."""
from collections import Counter
import hashlib
from pathlib import Path
import re
import runpy
import xml.etree.ElementTree as ET

import pytest


PACKAGE = Path(__file__).resolve().parents[1]
WATER = PACKAGE / 'worlds/models/ccs_water'
VRX = PACKAGE.parent / 'vrx/vrx_gz/models/coast_waves'
MESH = WATER / 'meshes/ccs_water.dae'
COLLADA = {'d': 'http://www.collada.org/2005/11/COLLADASchema'}


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


def test_water_keeps_one_visual_with_legacy_plugin_topic_and_visibility():
    model = ET.parse(WATER / 'model.sdf').getroot().find('model')
    old = ET.parse(VRX / 'model_36km.sdf').getroot().find('model')
    visual = model.find('link/visual')
    original = old.find('link/visual')
    assert model.findtext('static') == old.findtext('static') == 'true'
    assert model.find('link').get('name') == old.find('link').get('name')
    assert visual.get('name') == original.get('name')
    assert len(model.findall('.//visual')) == 1
    assert visual.findtext('geometry/mesh/uri') == 'model://ccs_water/meshes/ccs_water.dae'
    assert visual.find('geometry/mesh/scale') is None  # Mesh coordinates are already metres.
    assert visual.findtext('visibility_flags') == original.findtext('visibility_flags') == '8'
    assert model.findall('.//collision') == []
    plugin = visual.find('plugin')
    assert plugin.attrib == original.find('plugin').attrib
    assert len(model.findall('.//plugin')) == 1
    assert plugin.findtext('wavefield/topic') == '/vrx/wavefield/parameters'
    assert plugin.find('wavefield/wave') is None
    assert ET.parse(WATER / 'model.config').getroot().findtext('sdf') == 'model.sdf'


def test_graded_water_mesh_is_reproducible_and_within_previous_triangle_budget(tmp_path):
    generator = runpy.run_path(str(PACKAGE / 'tools/generate_ccs_water_mesh.py'))
    vertices, triangles = generator['generate']()
    regenerated = tmp_path / MESH.name
    generator['write_collada'](regenerated, vertices, triangles)
    assert regenerated.read_bytes() == MESH.read_bytes()
    legacy = ET.parse(VRX / 'meshes/waterlow.dae').find('.//d:triangles', COLLADA)
    assert len(triangles) < int(legacy.get('count'))

    half_width = 18000
    area_twice = 0
    edges = Counter()
    for a, b, c in triangles:
        x0, y0 = vertices[a]
        x1, y1 = vertices[b]
        x2, y2 = vertices[c]
        area = (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0)
        assert area > 0  # Nondegenerate and facing up.
        area_twice += area
        edges.update(tuple(sorted(edge)) for edge in ((a, b), (b, c), (c, a)))
    assert area_twice == pytest.approx(2 * (2 * half_width) ** 2)
    for (a, b), count in edges.items():
        x0, y0 = vertices[a]
        x1, y1 = vertices[b]
        perimeter = (
            (x0 == x1 and abs(x0) == half_width) or (y0 == y1 and abs(y0) == half_width)
        )
        assert count == (1 if perimeter else 2)  # No interior T-junctions or holes.
        radius = max(abs(x0), abs(y0), abs(x1), abs(y1))
        if radius <= 384:
            # The first stitch between 3 m and 6 m rows can span both steps.
            spacing = 3 if radius <= 96 else (9 if radius == 102 else 6)
            assert max(abs(x0 - x1), abs(y0 - y1)) <= spacing


@pytest.mark.parametrize('name', ['wave_normals.dds', 'skybox_lowres.dds'])
def test_bundled_textures_are_identical_to_existing_vrx_assets(name):
    new = WATER / 'materials/textures' / name
    old = VRX / 'materials/textures' / name
    assert new.read_bytes()[:4] == b'DDS '
    assert hashlib.sha256(new.read_bytes()).digest() == hashlib.sha256(old.read_bytes()).digest()


def without_comments(source):
    source = re.sub(r'/\*.*?\*/', '', source, flags=re.S)
    return re.sub(r'//[^\n]*', '', source)


def test_restored_texture_shaders_match_the_existing_plugin_interface():
    folder = WATER / 'materials/programs'
    vertex = without_comments((folder / 'GerstnerWaves_vs_330.glsl').read_text())
    fragment = without_comments((folder / 'GerstnerWaves_fs_330.glsl').read_text())
    for uniform in ('amplitude', 'wavenumber', 'omega', 'phase', 'steepness'):
        assert f'uniform vec3 {uniform};' in vertex
    assert 'uniform vec2 bumpScale;' in vertex
    assert 'uniform vec2 bumpSpeed;' in vertex
    assert 'outVs.bumpCoord = uv*bumpScale + drift;' in vertex
    assert 'vertex.xy * vec2(1.0, -1.0) / 36000.0' in vertex
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
