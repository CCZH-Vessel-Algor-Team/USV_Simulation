"""Harbor contact proxies must cover their targets without closing the marina."""
from collections import Counter
import importlib.util
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

shapely = pytest.importorskip('shapely', minversion='2.1')
from shapely.geometry import shape

PACKAGE = Path(__file__).resolve().parents[1]
CHART = PACKAGE/'worlds/models/ccs_chart_obstacles_enc_5km'
CONTACTS = json.loads((CHART/'gaogongdao_contacts.json').read_text())


def load(name):
    spec = importlib.util.spec_from_file_location(name, PACKAGE/'tools'/f'{name}.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


GENERATOR = load('gaogongdao_collision_assets')
HARBOR = load('prepare_gaogongdao_harbor')


def test_actual_world_loads_contacts_and_leaves_visual_meshes_untouched():
    world = ET.parse(PACKAGE/'worlds/ccs_open_water_crop.sdf').find('world')
    assert world.findtext("include[name='gaogongdao_local_harbor_v1']/uri") == (
        'model://ccs_chart_obstacles_enc_5km/gaogongdao_harbor.sdf')
    link = ET.parse(CHART/'gaogongdao_harbor.sdf').find('model/link')
    assert link.findtext("collision[@name='riprap_band_contact']/geometry/mesh/uri") == CONTACTS['riprap']['mesh']
    assert CONTACTS['counts'] == {
        'stones_in_band': 399, 'riprap_closed_shells': 3, 'moored_boat': 14,
        'pontoon_main': 3, 'pontoon_finger': 9, 'added_collision_objects': 27,
        'total_harbor_collision_objects': 42}
    assert len(link.findall('collision')) == 42 and len(link.findall('visual')) == 11
    for name, digest in CONTACTS['visual_meshes_sha256'].items():
        assert HARBOR.sha256(CHART/'meshes'/name) == digest
    for spec in CONTACTS['boxes']:
        collision = link.find(f"collision[@name='{spec['name']}']")
        np.testing.assert_allclose(np.fromstring(collision.findtext('pose'), sep=' '),
                                   spec['pose_m_rad'], atol=1e-8)
        np.testing.assert_allclose(np.fromstring(collision.findtext('geometry/box/size'), sep=' '),
                                   spec['size_m'], atol=1e-8)


def test_pontoon_boxes_match_the_boards_not_whole_berth_bounding_boxes():
    rectangles = []
    for material in ('concrete', 'roof'):
        rectangles.extend(GENERATOR.footprint(points) for points in GENERATOR.components(
            CHART/'meshes'/f'gaogongdao_styled_{material}.obj', HARBOR.read_obj)
                          if points[:, 2].max() < 2)
    visual = shapely.union_all(rectangles)
    contact = shapely.union_all([shape(s['footprint_local_xy']) for s in CONTACTS['boxes']
                                if s['family'].startswith('pontoon')])
    assert contact.symmetric_difference(visual).area < .005  # six-decimal source OBJ rounding
    assert contact.convex_hull.area > 10*contact.area
    assert len(rectangles) == 12
    # Open lanes between fingers are not filled by one enclosing collision.
    for xy in ((-36, -44), (-26, -48), (0, 17), (30, 99)):
        assert contact.distance(shapely.Point(xy)) > .25


def _berth_metrics():
    """Measure the installed contact boxes in each walkway frame."""
    boxes = CONTACTS['boxes']
    mains = sorted([s for s in boxes if s['family'] == 'pontoon_main'],
                   key=lambda s: s['pose_m_rad'][1])
    fingers = [s for s in boxes if s['family'] == 'pontoon_finger']

    def corners(spec):
        x, y, _, _, _, yaw = spec['pose_m_rad']
        length, width, _ = spec['size_m']
        c, sn = math.cos(yaw), math.sin(yaw)
        return np.array([[x+c*u-sn*v, y+sn*u+c*v]
                         for u, v in ((-length/2, -width/2), (length/2, -width/2),
                                      (length/2, width/2), (-length/2, width/2))])

    result = []
    for main in mains:
        c, sn = math.cos(main['pose_m_rad'][5]), math.sin(main['pose_m_rad'][5])
        center = np.array(main['pose_m_rad'][:2])
        axis = np.array([c, sn])
        normal = np.array([-sn, c])
        selected = sorted([f for f in fingers
                           if np.linalg.norm(corners(f).mean(0)-center) < 30],
                          key=lambda f: np.dot(corners(f).mean(0)-center, axis))
        polygons = [shapely.Polygon(corners(f)) for f in selected]
        widths = [polygons[i].distance(polygons[i+1]) for i in range(len(polygons)-1)]
        v = [(corners(f)-center)@normal for f in selected]
        half = main['size_m'][1]/2
        side = np.mean([item.mean() for item in v])
        depth = (max(item.max() for item in v)-half if side > 0
                 else -min(item.min() for item in v)-half)
        result.append({'widths': widths, 'depth': depth})
    return result


def test_middle_berths_are_4_06_m_wide_and_6_m_deep():
    metrics = _berth_metrics()
    assert metrics[0]['widths'] == pytest.approx([9.158, 9.158], abs=1e-3)
    assert metrics[0]['depth'] == pytest.approx(6.628, abs=1e-3)
    assert metrics[1]['widths'] == pytest.approx([4.06, 4.06], abs=1e-3)
    assert metrics[1]['depth'] == pytest.approx(6.0, abs=1e-3)
    assert metrics[2]['widths'] == pytest.approx([9.185, 9.185], abs=1e-3)
    assert metrics[2]['depth'] == pytest.approx(6.627, abs=2e-3)
    lengths = {s['name']: s['size_m'][0] for s in CONTACTS['boxes']
               if s['family'] == 'pontoon_finger'}
    for name in ('pontoon_finger_03', 'pontoon_finger_04', 'pontoon_finger_05'):
        assert lengths[name] == pytest.approx(6.988, abs=.001)
    for name in ('pontoon_finger_00', 'pontoon_finger_06'):
        assert lengths[name] == pytest.approx(7.616, abs=.001)


def test_all_fourteen_hulls_get_one_local_oriented_contact_box_each():
    boats = [s for s in CONTACTS['boxes'] if s['family'] == 'moored_boat']
    assert len(boats) == 14
    for color in ('blue', 'red'):
        points = GENERATOR.components(CHART/'meshes'/f'gaogongdao_styled_{color}.obj', HARBOR.read_obj)
        fitted = [s for s in boats if f'_{color}_' in s['name']]
        assert len(fitted) == len(points)
        for vertices, spec in zip(points, fitted):
            x, y, z, _, _, yaw = spec['pose_m_rad']
            length, width, height = spec['size_m']
            center = np.array([x, y, z])
            rotation = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                                 [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
            local = (vertices-center)@rotation
            assert np.all(np.abs(local) <= np.array([length, width, height])/2+1e-5)
            # Boat proxies follow the vessel heading instead of a big world AABB.
            assert 8 <= length <= 11.1 and 2 <= width <= 3.1 and height == pytest.approx(1.5)


def test_riprap_shells_are_closed_low_poly_and_do_not_enclose_open_water():
    path = CHART/CONTACTS['riprap']['mesh']
    assert HARBOR.sha256(path) == CONTACTS['riprap']['sha256']
    attrs, _, faces, vertices = HARBOR.read_obj(path)
    edges = Counter(tuple(sorted((a[0], b[0]))) for f in faces for a, b in zip(f, f[1:]+f[:1]))
    assert set(edges.values()) == {2}
    directed = Counter((a[0], b[0]) for f in faces for a, b in zip(f, f[1:]+f[:1]))
    assert all(directed[e[::-1]] == count for e, count in directed.items())
    assert len(faces) == CONTACTS['riprap']['triangles'] < 3000
    assert attrs['vn'] and all(ref[2] > 0 for f in faces for ref in f)
    normals = np.array([list(map(float, v.split())) for v in attrs['vn']])
    np.testing.assert_allclose(np.linalg.norm(normals, axis=1), 1, atol=1e-7)
    assert vertices[:, 2].min() == -1

    outlines = [shape(p['footprint']) for p in CONTACTS['riprap']['outlines_local_xy']]
    envelope = shapely.union_all(outlines)
    original = shapely.union_all([GENERATOR.footprint(points) for points in GENERATOR.components(
        CHART/'meshes/gaogongdao_styled_stone.obj', HARBOR.read_obj)])
    assert original.difference(envelope).area < .001
    assert envelope.difference(original.buffer(1.5)).area < .2
    # Includes the sheltered basin and the water alongside the floating docks.
    for xy in ((-35, 10), (-25, -40), (0, 30), (26, 110), (-70, 65), (20, 160)):
        assert envelope.distance(shapely.Point(xy)) > 1
