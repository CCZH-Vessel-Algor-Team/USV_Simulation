"""Chart-derived Tianwan bridge geometry, collision and world contract."""
from collections import Counter
import importlib.util
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest


PACKAGE = Path(__file__).resolve().parents[1]
MODEL = PACKAGE/'worlds/models/ccs_tianwan_bridge'
LAYOUT = json.loads((MODEL/'layout.json').read_text())
SPEC = importlib.util.spec_from_file_location('bridge_tool', PACKAGE/'tools/prepare_ccs_bridge.py')
TOOL = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(TOOL)
BRIDGE = TOOL.bridge


def test_source_and_alignment_are_the_chart_bridge_inside_the_map():
    assert LAYOUT['source']['objnam'] == 'Tianwankuahaidaqiao'
    assert LAYOUT['source']['nobjnm'] == '田湾跨海大桥'
    assert LAYOUT['source']['feature_lnam'] == '0046010AF46D57E5'
    assert LAYOUT['source']['sha256'] == 'cd394a68b907e8b86039cf04f0a654900ab7b8674b8618acf5f2de373b7042ae'
    assert LAYOUT['datum_wgs84_lon_lat_alt'][:2] == [119.481403, 34.692120]
    line = LAYOUT['centerline_enu']
    assert line[0] == pytest.approx([-653.2931, 831.0], abs=1e-3)
    assert line[-1] == pytest.approx([-1097.5817, -2501.0], abs=1e-3)
    assert LAYOUT['clip']['length_m'] == pytest.approx(3367.9, abs=.2)
    assert all(-2501 <= x <= 2501 and -2501 <= y <= 2501 for x, y in line)


def test_published_span_layout_and_clearance_are_preserved():
    plan, dims = LAYOUT['plan'], LAYOUT['dimensions']
    assert dims['main_span_m'] == 125 and dims['side_span_m'] == 70 and dims['aux_span_m'] == 70
    assert dims['published_main_clear_width_m'] == 100
    assert dims['published_main_clear_height_m'] == 21
    assert dims['deck_width_m'] == 34 and dims['platform_width_m'] == 38
    assert [round(s['length_m'], 3) for s in plan['spans'] if s['kind'] == 'main'] == [125.0]
    assert [round(s['length_m'], 3) for s in plan['spans'] if s['kind'] == 'side'] == [70.0, 70.0]
    assert [round(s['length_m'], 3) for s in plan['spans'] if s['kind'] == 'aux'] == [70.0]
    assert BRIDGE.profile(plan['main_center_m'], plan['main_center_m'],
                          plan['aux_center_m'], None)[1] == pytest.approx(21.0)
    main = [s for s in plan['spans'] if s['kind'] == 'main'][0]
    assert main['end_m']-main['start_m'] == pytest.approx(125.0)
    # Main span sits between the two charted pier-warning marks, as on the chart.
    No7 = next(b for b in LAYOUT['beacons'] if 'No 7' in b['name'])
    No16 = next(b for b in LAYOUT['beacons'] if 'No 16' in b['name'])
    assert (No7['station_m']+No16['station_m'])/2 == pytest.approx(
        plan['main_center_m'], abs=.1)
    assert all(s['length_m'] >= 30 for s in plan['spans'] if s['kind'] == 'approach')
    assert all(s['length_m'] <= 45 for s in plan['spans'] if s['kind'] == 'approach')


def test_caps_piers_platform_and_beacons_are_built():
    dims = LAYOUT['dimensions']
    sections = dims['pier_sections_m']
    assert set(sections) == {'approach', 'main', 'aux'}
    for section in sections.values():
        length, width, chamfer = section['shaft_length_width_chamfer']
        cap_l, cap_w, cap_h = section['cap_length_width_height']
        assert cap_l > length and cap_w > width and cap_h > 0
        assert 0 < chamfer < min(length, width)/2
    assert sections['main']['shaft_length_width_chamfer'][0] > sections['approach'][
        'shaft_length_width_chamfer'][0]
    assert dims['pier_offset_m'] == 8.5
    widths = {round(sample[1], 1) for sample in LAYOUT['profile_samples']}
    assert 34.0 in widths and 38.0 in widths
    beacons = LAYOUT['beacons']
    assert len(beacons) == 4
    assert {b['name'].split()[-1] for b in beacons} == {'3', '7', '11', '16'}
    assert {b['height_m'] for b in beacons if b['warning']} == {10.0}
    assert {b['height_m'] for b in beacons if not b['warning']} == {3.0}


def test_road_has_no_pier_humps_and_the_haunches_only_affect_the_soffit():
    plan = LAYOUT['plan']
    main, aux = plan['main_center_m'], plan['aux_center_m']
    center = BRIDGE.Centerline(LAYOUT['centerline_enu'])
    rows = BRIDGE._road_sections(center, LAYOUT)
    slopes = [(b[-1]-a[-1])/(s1-s0) for (s0, a), (s1, b) in zip(rows, rows[1:])]
    assert max(map(abs, slopes)) < .005  # less than 0.5% on every mesh segment
    depths = []
    for station in (main-BRIDGE.MAIN_SPAN/2, main, main+BRIDGE.MAIN_SPAN/2):
        _, bottom, top = BRIDGE.profile(station, main, aux, None)
        depths.append(top-bottom)
    assert depths == pytest.approx([6, 3.5, 6])
    for s in (main-62.5, main, main+62.5, aux-35, aux, aux+35):
        a, b, c = (BRIDGE.profile(s+ds, main, aux, None) for ds in (-.02, 0, .02))
        assert abs(c[-1]-2*b[-1]+a[-1]) < 1e-6
        assert abs(c[-2]-2*b[-2]+a[-2]) < 2e-5


def _obj_geometry(obj):
    vertices, faces = [], []
    for line in obj.read_text().splitlines():
        if line.startswith('v '):
            vertices.append(tuple(map(float, line.split()[1:])))
        elif line.startswith('f '):
            faces.append(tuple(int(i.split('//')[0])-1 for i in line.split()[1:]))
    return vertices, faces


def _waterline_pier_gaps():
    """Project the solid mesh waterline onto each opening's tangent."""
    center = BRIDGE.Centerline(LAYOUT['centerline_enu'])
    plan = LAYOUT['plan']
    points = []
    for obj in (MODEL/'meshes').glob('*_solid.obj'):
        vertices, faces = _obj_geometry(obj)
        for a, b, c in faces:
            for i, j in ((a, b), (b, c), (c, a)):
                p, q = vertices[i], vertices[j]
                if (p[2] < 0) != (q[2] < 0):
                    points.append(BRIDGE._lerp(p, q, -p[2]/(q[2]-p[2])))
    result = {}
    for name, span in (('main', BRIDGE.MAIN_SPAN), ('aux', BRIDGE.AUX_SPAN)):
        x, y, (tx, ty), _ = center.sample(plan[f'{name}_center_m'])
        projected = [(px-x)*tx+(py-y)*ty for px, py, _ in points]
        north = max(s for s in projected if abs(s+span/2) < 12)
        south = min(s for s in projected if abs(s-span/2) < 12)
        assert not any(north < s < south for s in projected)
        result[name] = south-north
    return result


def test_waterline_has_real_pier_gaps_without_a_solid_cross_span():
    gaps = _waterline_pier_gaps()
    assert 110 < gaps['main'] < 125
    assert 60 < gaps['aux'] < 70


def test_model_has_closed_collision_chunks_railings_markings_and_lights():
    root = ET.parse(MODEL/'model.sdf').getroot()
    model = root.find('model')
    assert model.get('name') == 'ccs_tianwan_bridge'
    assert model.findtext('static') == 'true'
    link = model.find('link')
    names = {c.get('name') for c in link.findall('collision')}
    solids = link.findall('collision/geometry/mesh/uri')
    assert len(solids) == LAYOUT['model']['segment_count'] == 4
    assert len(LAYOUT['plan']['piers']) == len(LAYOUT['plan']['spans']) == 80
    assert sum(name.startswith('beacon_') for name in names) == 4
    visuals = {v.get('name') for v in link.findall('visual')}
    assert any(name.startswith('railing_') for name in visuals)
    assert any(name.startswith('markings_white_') for name in visuals)
    assert any(name.startswith('markings_yellow_') for name in visuals)
    assert sum(name.endswith('_light') for name in visuals) == 4
    assert sum(v.find('material/emissive') is not None for v in link.findall('visual')) == 4
    for uri in solids:
        _, faces = _obj_geometry(MODEL/'meshes'/Path(uri.text).name)
        edges = Counter()
        for a, b, c in faces:
            assert len({a, b, c}) == 3
            edges.update(tuple(sorted(edge)) for edge in ((a, b), (b, c), (c, a)))
        assert set(edges.values()) == {2}
    for obj in (MODEL/'meshes').glob('*.obj'):
        vertices, _ = _obj_geometry(obj)
        assert min(y for _, y, _ in vertices) >= BRIDGE.SOUTH_CLIP_Y
    assert LAYOUT['model']['triangles']['visual'] < 65000


def test_world_loads_the_bridge_alongside_the_other_assets():
    world = ET.parse(PACKAGE/'worlds/ccs_open_water_crop.sdf').find('world')
    uris = [i.findtext('uri') for i in world.findall('include')]
    assert 'model://ccs_tianwan_bridge' in uris
    assert 'model://ccs_offshore_pv_farm' in uris
    assert 'model://ccs_hill_wind_farm' in uris


def test_generator_reproduces_all_assets_without_touching_source(tmp_path):
    assert TOOL.prepare(output=tmp_path, layout_input=MODEL/'layout.json') == LAYOUT
    generated = {p.relative_to(tmp_path) for p in tmp_path.rglob('*') if p.is_file()}
    assert generated == {p.relative_to(MODEL) for p in MODEL.rglob('*') if p.is_file()}
    for path in generated:
        assert (tmp_path/path).read_bytes() == (MODEL/path).read_bytes(), path


def test_bridge_does_not_overlap_existing_scene_assets():
    line = LAYOUT['centerline_enu']
    def distance(x, y):
        return min(math.hypot(x-a, y-b) for a, b in line)
    assert distance(1270.8, -1347.8) > 2000   # PV farm
    assert distance(-170, 270) > 500          # harbor
    for hill in ((-1228.4, 1507.2), (-608.2, 2062.6), (-2191.4, -489.4), (-1803.9, 373.0)):
        assert distance(*hill) > 700
