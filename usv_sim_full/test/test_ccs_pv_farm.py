"""Resource provenance, whole-field geometry and retained wind-turbine contract."""
from collections import Counter
import importlib.util
import json
from pathlib import Path
import xml.etree.ElementTree as ET

from PIL import Image
import pytest


shapely = pytest.importorskip('shapely', minversion='2.1')
PACKAGE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('pv_farm', PACKAGE/'tools/prepare_ccs_pv_farm.py')
pv = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(pv)
REPORT = json.loads((pv.MODELS/pv.FARM/'layout.json').read_text())


def test_wgs84_center_square_and_symmetric_native_scale_array():
    assert REPORT['center_wgs84_lon_lat_deg'] == [119.49527, 34.67997]
    assert REPORT['center_enu_m'] == pytest.approx([1270.801922, -1347.770737], abs=1e-5)
    cx, cy = REPORT['center_enu_m']
    square = shapely.geometry.shape(REPORT['region_enu_geojson'])
    assert square.bounds == pytest.approx([cx-500, cy-500, cx+500, cy+500])
    assert square.area == pytest.approx(1_000_000)
    farm = ET.parse(pv.MODELS/pv.FARM/'model.sdf').find('model')
    assert farm.findtext('static') == 'true'
    includes = farm.findall('include')
    assert len(includes) == 1
    assert len(REPORT['placements']) == REPORT['platform_count'] == 294
    assert len({s['name'] for s in REPORT['placements']}) == 294
    for i, site in enumerate(REPORT['placements']):
        row, col = divmod(i, 14)
        x, y = site['center_enu_m']
        assert (x, y) == pytest.approx((cx+(col-6.5)*70, cy+(row-10)*47))
        assert square.covers(shapely.box(x-30, y-17.5, x+30, y+17.5))
    site, = [s for s in REPORT['placements'] if s['model'] == pv.SOURCE]
    include, = includes
    assert include.findtext('name') == site['name']
    assert include.findtext('uri') == f'model://{pv.SOURCE}'
    assert tuple(map(float, include.findtext('pose').split())) == pytest.approx(
        (*site['center_enu_m'], 0., 0., 0., 0.))
    assert include.find('scale') is None
    assert REPORT['pitch_xy_m'] == [70., 47.]
    assert REPORT['waterline_piles'] == 1176
    assert len([s for s in REPORT['placements'] if s['model'] == pv.SOURCE]) == 1


def test_only_offshore_wind_instances_removed_and_other_scene_assets_intact():
    world = ET.parse(pv.WORLD).find('world')
    uris = [i.findtext('uri') for i in world.findall('include')]
    assert f'model://{pv.HILLS}' in uris and f'model://{pv.FARM}' in uris
    assert 'model://ccs_wind_farm' not in uris
    assert 'model://ccs_water' in uris
    assert 'model://ccs_chart_obstacles_enc_5km/gaogongdao_harbor.sdf' in uris
    old = ET.parse(pv.MODELS/'ccs_wind_farm/model.sdf').find('model')
    retained = ET.parse(pv.MODELS/pv.HILLS/'model.sdf').find('model')
    assert [ET.tostring(i).strip() for i in retained.findall('include')] == [
        ET.tostring(i).strip() for i in old.findall('include') if i.findtext('name').startswith('wind_hill_')]
    assert ET.tostring(retained.find('model')) == ET.tostring(old.find('model'))
    assert len(retained.findall('include')) == 4


def test_array_variant_keeps_exact_piles_and_batches_overhead_collision():
    source = pv.MODELS/pv.SOURCE
    for name, digest in REPORT['source_sha256'].items():
        assert pv.sha256((source/name).read_bytes()) == digest
    original = ET.parse(source/'model.sdf').find('model/link')
    derived = ET.parse(pv.MODELS/pv.LIGHT/'model.sdf').find('model/link')
    original_by_name = {c.get('name'): c for c in original.findall('collision')}
    selected = {c.get('name'): c for c in derived.findall('collision')}
    assert len(selected) == 5 < len(original_by_name) == 404
    for name, element in selected.items():
        if name != 'overhead_structure':
            assert ET.tostring(element).strip() == ET.tostring(original_by_name[name]).strip()
    assert {f'pile_{n}' for n in range(1, 5)} <= selected.keys()
    uri = selected['overhead_structure'].findtext('geometry/mesh/uri')
    collision_mesh = pv.MODELS/uri.removeprefix('model://')
    rows = collision_mesh.read_text().splitlines()
    assert sum(row.startswith('f ') for row in rows) == 38*32+16*12
    assert min(float(row.split()[3]) for row in rows if row.startswith('v ')) > 4.6
    assert derived.findall('visual') == []


def test_collision_tiles_preserve_every_analytic_pile_and_disconnected_panel_geometry():
    farm = ET.parse(pv.MODELS/pv.FARM/'model.sdf').find('model')
    source = ET.parse(pv.MODELS/pv.SOURCE/'model.sdf').find('model/link')
    piles = {c.get('name'): c for c in source.findall('collision')
             if c.get('name') in {f'pile_{i}' for i in range(1, 5)}}
    expected = Counter()
    for site in REPORT['placements']:
        if site['model'] == pv.SOURCE:
            continue
        x, y = site['center_enu_m']
        for name, c in piles.items():
            dx, dy, *_ = map(float, c.findtext('pose').split())
            expected[name, round(x+dx, 5), round(y+dy, 5)] += 1
    actual, cached = Counter(), {}
    assert REPORT['collision_tiles'] == {'standalone_platforms': 1, 'active_tile_links': 14,
                                         'analytic_piles_on_tiles': 1172,
                                         'overhead_mesh_collisions': 14}
    for tile in farm.findall('link'):
        x, y, *_ = map(float, tile.findtext('pose').split())
        grouped = Counter()
        for part in tile.findall('collision'):
            if part.get('name') == 'overhead_structure':
                continue
            _, _, name = part.get('name').split('_', 2)
            template = piles[name]
            px, py, *pose = map(float, part.findtext('pose').split())
            dx, dy, *original_pose = map(float, template.findtext('pose').split())
            assert pose == original_pose
            assert part.findtext('geometry/cylinder/radius') == template.findtext('geometry/cylinder/radius')
            assert part.findtext('geometry/cylinder/length') == template.findtext('geometry/cylinder/length')
            actual[name, round(x+px, 5), round(y+py, 5)] += 1
            grouped[round((px-dx)/70), round((py-dy)/47)] += 1
        assert set(grouped.values()) == {4}
        overhead, = tile.findall("collision[@name='overhead_structure']")
        uri = overhead.findtext('geometry/mesh/uri')
        if uri not in cached:
            meshes = (pv.MODELS/uri.removeprefix('model://')).read_text().splitlines()
            xyz = [tuple(map(float, line.split()[1:])) for line in meshes if line.startswith('v ')]
            assert min(p[2] for p in xyz) > 4.6
            count = Counter()
            for line in meshes:
                if line.startswith('f '):
                    vertices = [xyz[int(v.split('/')[0])-1] for v in line.split()[1:]]
                    (i, j), = {(round(p[0]/70), round(p[1]/47)) for p in vertices}
                    assert all(abs(p[0]-i*70) < 30.1 and abs(p[1]-j*47) < 17.6 for p in vertices)
                    count[i, j] += 1
            cached[uri] = count
        assert cached[uri] == {key: 38*32+16*12 for key in grouped}
    assert actual == expected
    assert len(cached) == 2


def test_visual_tiles_cover_each_repeat_once_without_filling_the_waterways():
    farm = ET.parse(pv.MODELS/pv.FARM/'model.sdf').find('model')
    tiles = farm.findall('link')
    stats = REPORT['render_tiles']
    assert stats['triangles_per_repeat_platform'] < 2200
    assert len(tiles) == stats['tiles'] == 14
    assert sum(len(tile.findall('visual')) for tile in tiles) == 70
    expected = [(x, y) for site in REPORT['placements'] if site['model'] == pv.LIGHT
                for x, y in [site['center_enu_m']]]
    # Read the actual tiled meshes, not just SDF placement metadata. The same
    # twelve strip rectangles must exist at each repeat but not at the detail platform.
    expected_counts = Counter((round(x, 4), round(y, 4)) for x, y in expected)
    resolved = {}
    for face in ('pv_front', 'pv_back'):
        actual_counts = Counter()
        for tile in tiles:
            x0, y0, *_ = map(float, tile.findtext('pose').split())
            visual = tile.find(f"visual[@name='{face}']")
            assert visual.findtext('cast_shadows') == 'false'
            uri = visual.findtext('geometry/mesh/uri')
            if uri not in resolved:
                rows = (pv.MODELS/uri.removeprefix('model://')).read_text().splitlines()
                vertices = [tuple(map(float, line.split()[1:])) for line in rows if line.startswith('v ')]
                quads = [vertices[i:i+4] for i in range(0, len(vertices), 4)]
                assert all(len(q) == 4 for q in quads)
                assert sum(line.startswith('f ') for line in rows) == len(quads)*2
                assert all(6.6 <= v[2] <= 7.85 for v in vertices)
                resolved[uri] = quads
            for quad in resolved[uri]:
                x = sum(v[0] for v in quad)/4
                y = sum(v[1] for v in quad)/4
                row_y = round(y/47)*47
                assert abs(y-row_y) < 17.5
                assert max(v[1] for v in quad)-min(v[1] for v in quad) == pytest.approx(2.1714, abs=.001)
                actual_counts[(round(x0+x, 4), round(y0+row_y, 4))] += 1
        assert actual_counts == {site: 12*count for site, count in expected_counts.items()}
        atlas = visual.findtext('material/pbr/metal/albedo_map')
        with Image.open(pv.MODELS/atlas.removeprefix('model://')) as image:
            assert image.size == (4096, 160)
    assert len(resolved) == 4  # two shared geometry variants per panel face


def test_generation_reproduces_installed_sources_without_external_archive(tmp_path):
    files, _ = pv.artifacts(pv.MODELS)
    for name, data in files.items():
        assert (pv.MODELS/name).read_bytes() == data
    target = tmp_path/pv.FARM/'layout.json'
    target.parent.mkdir()
    target.write_text('unfamiliar edit')
    with pytest.raises(ValueError, match='Refusing to overwrite'):
        pv.generate(output=tmp_path)
    assert target.read_text() == 'unfamiliar edit'
