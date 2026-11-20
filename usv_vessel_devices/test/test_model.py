"""Check the real expanded model and Gazebo's URDF conversion."""
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import pytest
import xacro
import yaml
import json

PACKAGE = Path(__file__).resolve().parents[1]
PROFILE = json.loads((PACKAGE / 'description/meshes/cad_assets.json').read_text())
LEGACY = ['left_front', 'left_rear', 'right_front', 'right_rear', 'mast']
ROOF_NAMES = ['roof_left_front', 'roof_left_rear', 'roof_right_front', 'roof_right_rear']
MAST_NAMES = LEGACY + PROFILE['mast']['extra_light_order']
NAMES = MAST_NAMES + ROOF_NAMES


@pytest.fixture
def urdf():
    return xacro.process_file(str(PACKAGE / 'description/demo.urdf.xacro')).toxml()


def test_five_preserved_lights_and_rearward_folding_axis(urdf):
    root = ET.fromstring(urdf)
    names = NAMES
    for name in names:
        assert root.find(f"link[@name='usv_1/light_{name}_link']") is not None
        assert root.find(f"gazebo[@reference='usv_1/light_{name}_joint']/preserveFixedJoint").text == 'true'
    joint = root.find("joint[@name='usv_1/mast_joint']")
    assert joint.attrib['type'] == 'revolute'
    assert joint.find('axis').attrib['xyz'] == '0 -1 0'
    assert float(joint.find('limit').attrib['upper']) == pytest.approx(1.5707963267948966)
    mast_lamp = root.find("joint[@name='usv_1/light_mast_joint']")
    assert mast_lamp.find('parent').attrib['link'] == 'usv_1/mast_link'
    plugin = root.find("gazebo/plugin[@filename='libVesselDevices.so']")
    assert [lamp.findtext('color') for lamp in plugin.findall('lamp')] == [
        '1 0 0 1', '1 0 0 1', '0 1 0 1', '0 1 0 1', '1 1 1 1',
        '0 1 0 1', '1 0 0 1', '0 1 0 1', '1 0 0 1',
        '1 0 0 1', '0 1 0 1', '1 0 0 1',
        '1 0 0 1', '1 0 0 1', '0 1 0 1', '0 1 0 1',
    ]
    assert [lamp.findtext('name') for lamp in plugin.findall('lamp')] == NAMES


def test_gazebo_keeps_visual_names_and_folding_joint(urdf, tmp_path):
    path = tmp_path / 'deck.urdf'
    path.write_text(urdf)
    result = subprocess.run(['gz', 'sdf', '-p', str(path)], capture_output=True, text=True, check=True)
    model = ET.fromstring(result.stdout).find('model')
    assert model.find("joint[@name='usv_1/mast_joint']") is not None
    for name in NAMES:
        link = model.find(f"link[@name='usv_1/light_{name}_link']")
        assert link is not None
        # sdformat renames visuals even when their fixed parent joint is preserved.
        assert len(link.findall('visual')) == (1 if name == 'mast' else 2)


def test_cad_base_stays_fixed_and_only_lenses_are_controlled(urdf):
    import json
    root = ET.fromstring(urdf)
    profile = json.loads((PACKAGE / 'description/meshes/cad_assets.json').read_text())
    joint = root.find("joint[@name='usv_1/mast_joint']")
    assert joint.find('parent').attrib['link'] == 'usv_1/mast_base_link'
    assert float(joint.find('origin').attrib['xyz'].split()[2]) == pytest.approx(
        2 * profile['mast']['hinge_height_m'])
    base = root.find("joint[@name='usv_1/mast_base_joint']")
    assert base.attrib['type'] == 'fixed'
    assert base.find('parent').attrib['link'] == 'usv_1/base_link'
    for name in ['left_front', 'left_rear', 'right_front', 'right_rear']:
        assert root.find(f"joint[@name='usv_1/light_{name}_joint']/parent").attrib['link'] == 'usv_1/mast_link'
        link = root.find(f"link[@name='usv_1/light_{name}_link']")
        assert link.find("visual[@name='lamp']/geometry/mesh").attrib['filename'].endswith(
            '/navigation_lamp_lens.stl')
        assert link.find("visual[@name='housing']/geometry/mesh").attrib['filename'].endswith(
            '/navigation_lamp_housing.stl')
    assert root.find("link[@name='usv_1/light_mast_link']/visual/geometry/mesh").attrib[
        'filename'].endswith('/mast_lens.stl')
    # Detailed CAD is visual-only: physics keeps simple, stable shapes.
    assert not root.findall('link/collision/geometry/mesh')


def test_primitive_fallback_remains_available(tmp_path):
    config = yaml.safe_load((PACKAGE / 'config/devices.yaml').read_text())
    config.update(mast_use_cad=False, lamp_use_cad=False, mast_length=1.0)
    path = tmp_path / 'primitive.yaml'
    path.write_text(yaml.safe_dump(config))
    root = ET.fromstring(xacro.process_file(
        str(PACKAGE / 'description/demo.urdf.xacro'), mappings={'config_file': str(path)}).toxml())
    assert root.find("link[@name='usv_1/mast_base_link']") is None
    assert root.find("joint[@name='usv_1/mast_joint']/parent").attrib['link'] == 'usv_1/base_link'
    assert not root.findall('link/visual/geometry/mesh')
    assert root.find("link[@name='usv_1/light_mast_link']/visual/geometry/sphere") is not None
    assert len(root.findall('gazebo/plugin/lamp')) == 5


def test_cad_uniform_scaling_keeps_mount_datums(tmp_path):
    import json
    profile = json.loads((PACKAGE / 'description/meshes/cad_assets.json').read_text())
    config = yaml.safe_load((PACKAGE / 'config/devices.yaml').read_text())
    config.update(mast_length=profile['mast']['native_height_m'] * 2, lamp_scale=0.5)
    path = tmp_path / 'scaled.yaml'
    path.write_text(yaml.safe_dump(config))
    root = ET.fromstring(xacro.process_file(
        str(PACKAGE / 'description/demo.urdf.xacro'), mappings={'config_file': str(path)}).toxml())
    assert root.find("link[@name='usv_1/mast_link']/visual/geometry/mesh").attrib['scale'] == '2.0 2.0 2.0'
    assert float(root.find("joint[@name='usv_1/mast_joint']/origin").attrib['xyz'].split()[2]) == pytest.approx(
        2 * profile['mast']['hinge_height_m'])
    assert root.find("joint[@name='usv_1/mast_base_joint']/origin").attrib['xyz'] == '-0.35 0.0 1.55'
    radius = root.find("link[@name='usv_1/mast_link']/collision/geometry/cylinder").attrib['radius']
    assert float(radius) == pytest.approx(2 * config['mast_radius'])
    link = root.find("link[@name='usv_1/light_left_front_link']")
    expected_scale = 0.5 * config['mast_lamp_scale'] * 2
    for mesh in link.findall('visual/geometry/mesh'):
        assert list(map(float, mesh.attrib['scale'].split())) == pytest.approx([expected_scale] * 3)
    expected_xyz = [value * 2 for value in profile['mast']['navigation_light_mounts_from_hinge_m']['left_front']]
    xyz = root.find("joint[@name='usv_1/light_left_front_joint']/origin").attrib['xyz']
    assert list(map(float, xyz.split())) == pytest.approx(expected_xyz)


def test_four_lamps_match_two_crossbars_without_overlap(urdf):
    import json
    profile = json.loads((PACKAGE / 'description/meshes/cad_assets.json').read_text())
    config = yaml.safe_load((PACKAGE / 'config/devices.yaml').read_text())
    root = ET.fromstring(urdf)
    points = {}
    for name, anchor in profile['mast']['navigation_light_mounts_from_hinge_m'].items():
        joint = root.find(f"joint[@name='usv_1/light_{name}_joint']")
        assert joint.find('parent').attrib['link'] == 'usv_1/mast_link'
        points[name] = list(map(float, joint.find('origin').attrib['xyz'].split()))
        assert points[name] == pytest.approx([value * 2 for value in anchor])
    assert points['left_front'][1] > 0 and points['right_front'][1] < 0
    assert points['left_rear'][1] > 0 and points['right_rear'][1] < 0
    assert points['left_front'][2] == pytest.approx(points['right_front'][2])
    assert points['left_rear'][2] == pytest.approx(points['right_rear'][2])
    height = profile['navigation_lamp']['meshes']['navigation_lamp_housing.stl']['bounds_m'][5]
    height *= config['lamp_scale'] * config['mast_lamp_scale'] * 2
    assert points['left_rear'][2] + height < points['left_front'][2]


def test_roof_mounting_can_be_selected_explicitly(tmp_path):
    config = yaml.safe_load((PACKAGE / 'config/devices.yaml').read_text())
    config.update(lights_on_mast=False, lamp_scale=0.5)
    path = tmp_path / 'roof.yaml'
    path.write_text(yaml.safe_dump(config))
    root = ET.fromstring(xacro.process_file(
        str(PACKAGE / 'description/demo.urdf.xacro'), mappings={'config_file': str(path)}).toxml())
    for name in ['left_front', 'left_rear', 'right_front', 'right_rear']:
        assert root.find(f"joint[@name='usv_1/light_{name}_joint']/parent").attrib['link'] == 'usv_1/base_link'
    joint = root.find("joint[@name='usv_1/light_left_front_joint']")
    assert list(map(float, joint.find('origin').attrib['xyz'].split())) == pytest.approx(
        [config['front_x'], config['left_y'], config['roof_z']])
    link = root.find("link[@name='usv_1/light_left_front_link']")
    assert all(mesh.attrib['scale'] == '0.5 0.5 0.5' for mesh in link.findall('visual/geometry/mesh'))
    assert root.find("joint[@name='usv_1/light_mast_joint']/parent").attrib['link'] == 'usv_1/mast_link'
    assert not any(root.find(f"link[@name='usv_1/light_{name}_link']") is not None for name in ROOF_NAMES)


def test_default_doubles_entire_assembly_and_extra_mounts(urdf):
    root = ET.fromstring(urdf)
    cfg = yaml.safe_load((PACKAGE / 'config/devices.yaml').read_text())
    assert cfg['mast_length'] == pytest.approx(2 * PROFILE['mast']['native_height_m'])
    for link in ['mast_base', 'mast', 'light_mast']:
        mesh = root.find(f"link[@name='usv_1/{link}_link']/visual/geometry/mesh")
        assert list(map(float, mesh.attrib['scale'].split())) == [2, 2, 2]
    for name, mount in PROFILE['mast']['extra_light_mounts_from_hinge_m'].items():
        joint = root.find(f"joint[@name='usv_1/light_{name}_joint']")
        assert joint.find('parent').attrib['link'] == 'usv_1/mast_link'
        assert list(map(float, joint.find('origin').attrib['xyz'].split())) == pytest.approx(
            [2 * value for value in mount['xyz']])
        scale = 2 * cfg['mast_lamp_scale'] * cfg['lamp_scale'] * mount['size_factor']
        for mesh in root.findall(f"link[@name='usv_1/light_{name}_link']/visual/geometry/mesh"):
            assert list(map(float, mesh.attrib['scale'].split())) == pytest.approx([scale] * 3)
        # Centreline fixture envelope stays clear of the shaft at default fit.
        if name != 'right_lower':
            half_width = 0.076 * scale
            assert abs(2 * mount['xyz'][0]) - half_width > 2 * cfg['mast_radius']


@pytest.mark.parametrize('settings', [
    {'extra_lights_enabled': False}, {'mast_use_cad': False},
])
def test_extra_lights_can_be_disabled(tmp_path, settings):
    cfg = yaml.safe_load((PACKAGE / 'config/devices.yaml').read_text())
    cfg.update(settings)
    path = tmp_path / 'disabled.yaml'
    path.write_text(yaml.safe_dump(cfg))
    root = ET.fromstring(xacro.process_file(str(PACKAGE / 'description/demo.urdf.xacro'),
                                         mappings={'config_file': str(path)}).toxml())
    names = LEGACY + (ROOF_NAMES if cfg['mast_use_cad'] else [])
    assert [lamp.findtext('name') for lamp in root.findall('gazebo/plugin/lamp')] == names
    assert len([link for link in root.findall('link') if '/light_' in link.attrib['name']]) == len(names)


def test_extra_colors_support_partial_overrides(tmp_path):
    cfg = yaml.safe_load((PACKAGE / 'config/devices.yaml').read_text())
    cfg['extra_light_colors'] = {'aft_lower': 'green'}
    path = tmp_path / 'colors.yaml'
    path.write_text(yaml.safe_dump(cfg))
    root = ET.fromstring(xacro.process_file(str(PACKAGE / 'description/demo.urdf.xacro'),
                                         mappings={'config_file': str(path)}).toxml())
    colors = {lamp.findtext('name'): lamp.findtext('color') for lamp in root.findall('gazebo/plugin/lamp')}
    assert colors['aft_lower'] == colors['aft_middle'] == '0 1 0 1'
    assert colors['aft_upper'] == '1 0 0 1'


@pytest.mark.parametrize('mast_scale', [1, 2, 3])
@pytest.mark.parametrize('lamp_cad', [True, False])
def test_additional_roof_lights_keep_original_roof_positions_and_size(tmp_path, mast_scale, lamp_cad):
    cfg = yaml.safe_load((PACKAGE / 'config/devices.yaml').read_text())
    cfg.update(mast_length=PROFILE['mast']['native_height_m'] * mast_scale,
               lamp_use_cad=lamp_cad, lamp_scale=0.5,
               front_x=0.7, rear_x=-0.7, left_y=0.55, right_y=-0.55, roof_z=1.6)
    path = tmp_path / 'roof.yaml'
    path.write_text(yaml.safe_dump(cfg))
    root = ET.fromstring(xacro.process_file(str(PACKAGE / 'description/demo.urdf.xacro'),
                                         mappings={'config_file': str(path)}).toxml())
    for name in ROOF_NAMES:
        joint = root.find(f"joint[@name='usv_1/light_{name}_joint']")
        assert joint.find('parent').attrib['link'] == 'usv_1/base_link'
        assert joint.attrib['type'] == 'fixed'
        assert list(map(float, joint.find('origin').attrib['xyz'].split())) == pytest.approx([
            cfg['front_x'] if 'front' in name else cfg['rear_x'],
            cfg['left_y'] if 'left' in name else cfg['right_y'], cfg['roof_z'],
        ])
        link = root.find(f"link[@name='usv_1/light_{name}_link']")
        if lamp_cad:
            meshes = link.findall('visual/geometry/mesh')
            assert len(meshes) == 2
            assert all(mesh.attrib['scale'] == '0.5 0.5 0.5' for mesh in meshes)
        else:
            assert not link.findall('visual/geometry/mesh')
            assert float(link.find('visual/geometry/sphere').attrib['radius']) == cfg['lamp_radius']


def test_roof_lights_can_move_aft_15cm_without_moving_mast_devices(urdf, tmp_path):
    cfg = yaml.safe_load((PACKAGE / 'config/devices.yaml').read_text())
    # Test the configuration contract, not a tunable user's installation layout.
    previous = {**cfg, 'front_x': cfg['front_x'] + 0.15, 'rear_x': cfg['rear_x'] + 0.15}
    path = tmp_path / 'previous_roof_positions.yaml'
    path.write_text(yaml.safe_dump(previous))
    before = ET.fromstring(xacro.process_file(
        str(PACKAGE / 'description/demo.urdf.xacro'),
        mappings={'config_file': str(path)}).toxml())
    after = ET.fromstring(urdf)
    roof_joints = {f'usv_1/light_{name}_joint' for name in ROOF_NAMES}
    for joint in after.findall('joint'):
        old = before.find(f"joint[@name='{joint.attrib['name']}']")
        assert old is not None
        if joint.attrib['name'] in roof_joints:
            old_xyz = list(map(float, old.find('origin').attrib['xyz'].split()))
            new_xyz = list(map(float, joint.find('origin').attrib['xyz'].split()))
            assert new_xyz == pytest.approx([
                old_xyz[0] - 0.15, old_xyz[1], old_xyz[2],
            ])
        else:
            assert ET.tostring(joint) == ET.tostring(old)
    for link in after.findall('link'):
        assert ET.tostring(link) == ET.tostring(
            before.find(f"link[@name='{link.attrib['name']}']"))


def test_roof_lights_can_be_disabled_without_changing_twelve_mast_lamps(tmp_path):
    cfg = yaml.safe_load((PACKAGE / 'config/devices.yaml').read_text())
    cfg['roof_lights_enabled'] = False
    path = tmp_path / 'no_roof.yaml'
    path.write_text(yaml.safe_dump(cfg))
    root = ET.fromstring(xacro.process_file(str(PACKAGE / 'description/demo.urdf.xacro'),
                                         mappings={'config_file': str(path)}).toxml())
    assert [lamp.findtext('name') for lamp in root.findall('gazebo/plugin/lamp')] == MAST_NAMES
    for name in ROOF_NAMES:
        assert root.find(f"link[@name='usv_1/light_{name}_link']") is None
    for name in MAST_NAMES:
        assert root.find(f"joint[@name='usv_1/light_{name}_joint']/parent").attrib['link'] == 'usv_1/mast_link'
