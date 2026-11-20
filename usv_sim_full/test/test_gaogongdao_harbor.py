"""The connected harbor preserves sources and changes only documented local geometry."""
from collections import Counter
from functools import lru_cache
import importlib.util
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

shapely = pytest.importorskip('shapely', minversion='2.1')
from shapely.affinity import translate


PACKAGE = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('harbor_preparation',
                                            PACKAGE / 'tools/prepare_gaogongdao_harbor.py')
generator = importlib.util.module_from_spec(spec)
spec.loader.exec_module(generator)
CHART, HARBOR = generator.CHART, generator.HARBOR
REPORT = json.loads((CHART / 'gaogongdao_variant.json').read_text())


@lru_cache(maxsize=12)
def mesh(path):
    return generator.read_obj(path)


def pose():
    return np.fromstring(ET.parse(CHART / 'model.sdf').getroot().findtext(
        "model/link[@name='chart_obstacle_001']/pose"), sep=' ')[:3]


def canonical(element):
    return ET.canonicalize(ET.tostring(element, encoding='unicode'), strip_text=True)


def test_actual_bringup_world_selects_quay_variant_and_only_one_harbor():
    bringup = PACKAGE.parents[1] / 'usv_bringup'
    system_file = bringup / 'config/system/system_sim.yaml'
    system = yaml.safe_load(system_file.read_text())
    scene_file = (system_file.parent / system['launches']['simulation'][
        'arguments']['config_path']['path']).resolve()
    scene = yaml.safe_load(scene_file.read_text())
    world = ET.parse(PACKAGE / 'worlds' / (
        scene['environment']['world_name'] + '.sdf')).getroot().find('world')
    assert world.get('name') == 'ccs_open_water_crop'
    assert world.findtext("include[name='CCS Crop Waves']/uri") == 'model://ccs_water'
    assert world.findtext("include[name='CCS ENC Chart Obstacles']/uri") == (
        'model://ccs_chart_obstacles_enc_5km/model_gaogongdao.sdf')
    harbors = [i for i in world.findall('include') if i.findtext('name') == 'gaogongdao_local_harbor_v1']
    assert len(harbors) == 1
    include = harbors[0]
    assert include.findtext('pose') == '-170 270 0 0 0 0'
    # The nested file URI works with the EXISTING worlds/models resource root.
    uri = include.findtext('uri').removeprefix('model://')
    assert PACKAGE / 'worlds/models' / uri == CHART / generator.HARBOR_VARIANT
    assert (PACKAGE / 'worlds/models' / uri).is_file()
    datum = world.find('spherical_coordinates')
    assert float(datum.findtext('latitude_deg')) == 34.692120
    assert float(datum.findtext('longitude_deg')) == 119.481403
    assert world.find('plugin[@name="vrx::PublisherPlugin"]') is not None


def test_variant_preserves_every_other_chart_link_and_original_link_pose():
    original = ET.parse(CHART / 'model.sdf').getroot().find('model')
    variant = ET.parse(CHART / generator.VARIANT).getroot().find('model')
    assert original.attrib == variant.attrib
    assert original.findtext('static') == variant.findtext('static') == 'true'
    assert len(original.findall('link')) == len(variant.findall('link'))
    for link in original.findall('link'):
        replacement = variant.find(f"link[@name='{link.get('name')}']")
        if link.get('name') != 'chart_obstacle_001':
            assert canonical(replacement) == canonical(link)
        else:
            assert link.findtext('pose') == replacement.findtext('pose')
            assert len(replacement.findall('collision')) == 2
            assert len(replacement.findall('visual')) == 4
            names = [child.get('name') for child in replacement if child.get('name')]
            assert len(names) == len(set(names))
            assert replacement.findtext("visual[@name='vegetation']/cast_shadows") == 'false'


@pytest.mark.parametrize('stem', generator.STEMS)
def test_outside_mesh_attributes_faces_and_material_order_are_exactly_retained(stem):
    attributes, records, faces, vertices = mesh(CHART / 'meshes' / (stem + '.obj'))
    new_attributes, new_records, new_faces, new_vertices = mesh(
        CHART / 'meshes' / (stem + '_without_harbor.obj'))
    removed = generator.quay_faces(vertices, faces, pose())
    assert int(removed.sum()) == REPORT['meshes'][stem]['removed_faces']
    assert len(new_faces) == int((~removed).sum())
    used = [set(), set(), set()]
    retained = []
    for face, drop in zip(faces, removed):
        if not drop:
            retained.append(face)
            for ref in face:
                for column, index in enumerate(ref):
                    if index:
                        used[column].add(index)
    maps = [{old: new for new, old in enumerate(sorted(indices), start=1)} for indices in used]
    for column, name in enumerate(('v', 'vt', 'vn')):
        assert new_attributes[name] == [attributes[name][i - 1] for i in sorted(used[column])]
    for old_face, new_face in zip(retained, new_faces):
        assert new_face == [tuple(maps[column][index] if index else 0
                                 for column, index in enumerate(ref)) for ref in old_face]
    assert [r for r in records if r[0] == 'directive' and not r[1].startswith('mtllib ')] == [
        r for r in new_records if r[0] == 'directive' and not r[1].startswith('mtllib ')]
    assert not generator.quay_faces(new_vertices, new_faces, pose()).any()
    mtllib = next(value.split(maxsplit=1)[1] for kind, value in new_records
                  if kind == 'directive' and value.startswith('mtllib '))
    assert (CHART / 'meshes' / mtllib).is_file()


@pytest.mark.parametrize('name', ['green_island_001_without_harbor',
                                  'green_island_001_collision_without_harbor',
                                  'gaogongdao_foundation'])
def test_remaining_terrain_and_foundation_are_closed_solids(name):
    _, _, faces, vertices = mesh(CHART / 'meshes' / (name + '.obj'))
    edges = Counter(tuple(sorted((face[i][0], face[(i + 1) % len(face)][0])))
                    for face in faces for i in range(len(face)))
    assert set(edges.values()) == {2}, Counter(edges.values())
    assert vertices[:, 2].min() == -5


def surface_projection(vertices, faces, group_records):
    group, roofs = None, []
    for kind, value in group_records:
        if kind == 'directive' and value.startswith('g '):
            group = value.split()[1]
        elif kind == 'f' and group == 'terrain_surface':
            roofs.append(shapely.Polygon(vertices[[ref[0] - 1 for ref in faces[value]], :2]))
    return shapely.union_all(roofs)


def test_replaced_quay_base_preserves_original_outline_before_adding_causeway():
    _, _, source_faces, source_vertices = mesh(CHART / 'meshes/chart_obstacle_001.obj')
    chart = generator.roof_footprint(source_vertices, source_faces)
    _, _, foundation_faces, foundation_vertices = mesh(CHART / 'meshes/gaogongdao_foundation.obj')
    foundation = generator.roof_footprint(foundation_vertices, foundation_faces)
    deck_data = mesh(HARBOR / 'meshes/deck_collision.obj')
    deck = translate(generator.roof_footprint(deck_data[3], deck_data[2]),
                     generator.HARBOR_POSE[0] - pose()[0], generator.HARBOR_POSE[1] - pose()[1])
    assert deck.symmetric_difference(foundation).area < 0.001
    assert foundation_vertices[:, 2].max() == generator.FOUNDATION_TOP < 3.2
    for stem in generator.STEMS[:2]:
        _, records, faces, vertices = mesh(CHART / 'meshes' / (stem + '_without_harbor.obj'))
        retained = surface_projection(vertices, faces, records)
        assert retained.intersection(foundation).area < 0.001
        assert chart.symmetric_difference(retained.union(foundation)).area < 0.1
        assert foundation.area == pytest.approx(5199.830594, abs=0.001)


def test_original_chart_and_user_supplied_harbor_assets_have_not_been_overwritten():
    for name, expected in REPORT['source_chart_sha256'].items():
        assert generator.sha256(CHART / name) == expected
    for name, expected in REPORT['source_harbor_sha256'].items():
        assert generator.sha256(HARBOR / name) == expected
    model = ET.parse(HARBOR / 'model.sdf').getroot().find('model')
    assert model.findtext('static') == 'true'
    assert len(model.findall('link/collision')) == 12
    assert not model.findall('.//plugin')
    for uri in model.findall('.//uri'):
        assert (HARBOR / uri.text).is_file()
    for tag in ('albedo_map', 'metalness_map', 'roughness_map'):
        assert (HARBOR / model.findtext('.//' + tag)).is_file()


def test_runtime_harbor_preserves_original_physics_and_adds_land_and_contact_proxies():
    supplied = ET.parse(HARBOR / 'model.sdf').getroot()
    runtime = ET.parse(CHART / generator.HARBOR_VARIANT).getroot()
    for tag in ('uri', 'albedo_map', 'metalness_map', 'roughness_map'):
        for element in supplied.findall('.//' + tag):
            element.text = generator.HARBOR_URI + element.text
    supplied.find(".//collision[@name='chart_matched_deck']/geometry/mesh/uri").text = (
        'meshes/gaogongdao_deck_collision.obj')
    original_collisions = supplied.findall('model/link/collision')
    runtime_link = runtime.find('model/link')
    for collision in original_collisions:
        assert canonical(runtime_link.find(f"collision[@name='{collision.get('name')}']")) == canonical(collision)
    contacts = json.loads((CHART/'gaogongdao_contacts.json').read_text())
    assert len(runtime_link.findall('collision')) == (
        len(original_collisions)+3+contacts['counts']['added_collision_objects'])
    assert len(runtime_link.findall('visual')) == 11
    assert not runtime.findall('.//plugin')
    for name in ('gaogongdao_foundation', 'gaogongdao_deck_collision'):
        attributes, _, faces, vertices = mesh(CHART / 'meshes' / (name + '.obj'))
        assert attributes['vn']
        for face in faces:
            for ref in face:
                assert len(ref) == 3 and ref[2] > 0
        normals = np.array([list(map(float, n.split())) for n in attributes['vn']])
        np.testing.assert_allclose(np.linalg.norm(normals, axis=1), 1, atol=1e-7)
    _, _, faces, vertices = mesh(CHART / 'meshes/gaogongdao_deck_collision.obj')
    old = mesh(HARBOR / 'meshes/deck_collision.obj')
    np.testing.assert_array_equal(vertices, old[3])
    assert generator.roof_footprint(vertices, faces).equals(
        generator.roof_footprint(old[3], old[2]))


def test_causeway_connects_the_quay_to_mainland_and_has_no_zero_width_neck():
    from shapely.geometry import shape
    c = REPORT['connection']
    line = shapely.LineString(c['centerline_enu_m'])
    road = shape(c['road_footprint_enu_geojson'])
    footprint = shape(c['footprint_enu_geojson'])
    _, _, f, v = mesh(CHART/'meshes/chart_obstacle_001.obj')
    chart = translate(generator.roof_footprint(v, f), *pose()[:2])
    mainland = max(chart.geoms, key=lambda p: p.area)
    _, _, f, v = mesh(HARBOR/'meshes/deck_collision.obj')
    quay = translate(generator.roof_footprint(v, f), *generator.HARBOR_POSE[:2])
    assert quay.union(road).union(mainland).geom_type == 'Polygon'
    assert road.intersection(quay).area > 20
    assert road.intersection(mainland).area > 100
    assert footprint.is_valid and road.is_valid
    assert road.difference(footprint).area < 1e-7
    assert 210 < line.length < 225  # Includes the old-quay overlap and landward landing.
    assert c['road_width_m'] == [3, 9]
    assert 2500 < c['new_fill_area_m2'] < 4000
    assert 0 <= c['landing_max_raise_m'] <= 3.1
    assert c['source_stones'] == 267
    assert len(c['material_groups']) == 8


@pytest.mark.parametrize('name', ['gaogongdao_causeway_fill', 'gaogongdao_landing'])
def test_new_filled_geometry_is_closed_normal_complete_and_has_positive_volume(name):
    attrs, _, faces, vertices = mesh(CHART/'meshes'/f'{name}.obj')
    edges = Counter(tuple(sorted((face[i][0], face[(i+1) % len(face)][0])))
                    for face in faces for i in range(len(face)))
    assert set(edges.values()) == {2}
    triangles = np.array([vertices[[ref[0]-1 for ref in face]] for face in faces])
    volume = np.einsum('ij,ij->i', triangles[:, 0], np.cross(triangles[:, 1], triangles[:, 2])).sum()/6
    assert volume > 100
    directed = Counter((a[0], b[0]) for face in faces
                       for a, b in zip(face, face[1:]+face[:1]))
    assert all(directed[edge[::-1]] == count for edge, count in directed.items())
    assert vertices[:, 2].min() == -5
    assert attrs['vn'] and all(ref[2] > 0 for face in faces for ref in face)


@pytest.mark.parametrize('stem', ['green_island_001', 'green_island_001_collision'])
def test_landing_grading_is_local_and_keeps_all_horizontal_coordinates_and_topology(stem):
    _, before_records, before_faces, before = mesh(CHART/'meshes'/f'{stem}_without_harbor.obj')
    _, after_records, after_faces, after = mesh(CHART/'meshes'/f'{stem}_connected.obj')
    np.testing.assert_array_equal(before[:, :2], after[:, :2])
    assert before_faces == after_faces
    assert before_records == after_records
    change = np.abs(before[:, 2]-after[:, 2]) > 1e-6
    assert 0 < change.sum() == REPORT['connection']['graded_vertices'][stem] < 100
    world = after[change]+pose()
    line = shapely.LineString(REPORT['connection']['centerline_enu_m'])
    assert np.all(shapely.distance(shapely.points(world[:, :2]), line) < 45)
    assert world[:, 1].min() > 540 and world[:, 1].max() < 635
    np.testing.assert_array_equal(before[before[:, 2] < 0], after[before[:, 2] < 0])
    edges = Counter(tuple(sorted((face[i][0], face[(i+1) % len(face)][0])))
                    for face in after_faces for i in range(len(face)))
    assert set(edges.values()) == {2}


def test_local_shadow_split_keeps_every_graded_triangle_without_duplicate_surfaces():
    def triangles(name):
        _, _, faces, vertices = mesh(CHART/'meshes'/name)
        result = np.array([vertices[[ref[0]-1 for ref in face]] for face in faces])
        return result.reshape(len(result), 9)
    original = triangles('green_island_001_connected.obj')
    main = triangles('green_island_001_connected_main.obj')
    local = triangles('green_island_001_landing_terrain.obj')
    combined = np.vstack((main, local))
    assert 0 < len(local) < 150
    assert len(combined) == len(original)
    key_type = np.dtype((np.void, original.dtype.itemsize*9))
    np.testing.assert_array_equal(np.sort(original.view(key_type).ravel()),
                                  np.sort(combined.view(key_type).ravel()))
    root = ET.parse(CHART/generator.VARIANT).getroot()
    assert root.findtext("model/link[@name='chart_obstacle_001']/visual[@name='landing_terrain']/cast_shadows") == 'false'


def test_styled_materials_use_tiled_maps_and_nonmetal_rough_surfaces():
    import cv2
    root = ET.parse(CHART/generator.HARBOR_VARIANT).getroot()
    for visual in root.findall('model/link/visual'):
        if visual.get('name') == 'landing_visual':
            assert visual.find('material') is None  # Match the original terrain's MTL lighting.
            continue
        metal = visual.find('material/pbr/metal')
        assert float(metal.findtext('metalness')) == 0
        assert .75 <= float(metal.findtext('roughness')) <= 1
        for key in ('albedo_map', 'normal_map', 'roughness_map'):
            path = CHART/metal.findtext(key).removeprefix('model://ccs_chart_obstacles_enc_5km/')
            assert path.is_file()
            image = cv2.imread(str(path))
            assert image is not None and max(image.shape[:2]) <= 1024
    old = cv2.imread(str(CHART/'textures/green_island_001_albedo.jpg')).astype(float)
    new = cv2.imread(str(CHART/'textures/green_island_001_harbor_albedo.jpg')).astype(float)
    assert old.shape == new.shape
    assert new.mean() < old.mean()
    assert np.mean(np.ptp(new, axis=2)) < np.mean(np.ptp(old, axis=2))


def test_generator_refuses_to_slice_a_face_connecting_to_the_main_island():
    vertices = np.array([[-170, 200, 1], [-170, 500, 1], [-180, 500, 1]])
    with pytest.raises(ValueError, match='crosses a face'):
        generator.quay_faces(vertices, [[(1,), (2,), (3,)]], np.zeros(3))
    # No vertex lies in the crop, but a large triangle can still cross it.
    vertices = np.array([[-500, 100, 1], [100, 100, 1], [-170, 700, 1]])
    with pytest.raises(ValueError, match='crosses a face'):
        generator.quay_faces(vertices, [[(1,), (2,), (3,)]], np.zeros(3))


def test_preparation_is_reproducible_and_does_not_need_to_rebuild_green_islands(tmp_path):
    generator.prepare(output_dir=tmp_path)
    names = [generator.VARIANT, generator.HARBOR_VARIANT, 'gaogongdao_variant.json',
             'meshes/gaogongdao_foundation.obj', 'meshes/gaogongdao_foundation.mtl',
             'meshes/gaogongdao_deck_collision.obj']
    names += ['meshes/' + stem + '_without_harbor.obj' for stem in generator.STEMS]
    names += [str(p.relative_to(tmp_path)) for p in (tmp_path/'meshes').glob('gaogongdao_styled_*')]
    names += [str(p.relative_to(tmp_path)) for p in (tmp_path/'textures').glob('*')]
    names += [str(p.relative_to(tmp_path)) for p in (tmp_path/'meshes').glob('gaogongdao_causeway_*')]
    names += ['meshes/gaogongdao_landing.obj', 'meshes/gaogongdao_foundation_visual.obj']
    names += ['meshes/'+stem+'_connected.obj' for stem in generator.STEMS]
    names += ['meshes/green_island_001_connected_main.obj', 'meshes/green_island_001_landing_terrain.obj']
    names += ['bridge_landing.json', 'meshes/green_island_001_bridge_landing.mtl',
              'meshes/green_island_001_bridge_landing.obj',
              'meshes/green_island_001_collision_bridge_landing.obj']
    names += ['gaogongdao_contacts.json', 'meshes/gaogongdao_riprap_collision.obj']
    for name in names:
        assert generator.sha256(tmp_path / name) == generator.sha256(CHART / name)
