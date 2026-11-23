"""Wind-farm layout, anchoring, clearance, and unmodified source-asset contracts."""
import importlib.util
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

shapely = pytest.importorskip('shapely', minversion='2.1')
PACKAGE = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('wind_farm', PACKAGE/'tools/prepare_ccs_wind_farm.py')
generator = importlib.util.module_from_spec(spec)
spec.loader.exec_module(generator)
REPORT = json.loads((generator.OUTPUT/'layout.json').read_text())


def test_real_ccs_world_selects_only_hill_turbines_without_removing_harbor_or_water():
    world = ET.parse(PACKAGE/'worlds/ccs_open_water_crop.sdf').getroot().find('world')
    assert not any(i.findtext('uri') == 'model://ccs_wind_farm' for i in world.findall('include'))
    farms = [i for i in world.findall('include') if i.findtext('uri') == 'model://ccs_hill_wind_farm']
    assert len(farms) == 1
    assert farms[0].findtext('pose') == '0 0 0 0 0 0'
    assert world.findtext("include[name='gaogongdao_local_harbor_v1']/pose") == '-170 270 0 0 0 0'
    assert world.findtext("include[name='CCS Crop Waves']/uri") == 'model://ccs_water'


def test_imported_source_geometry_normals_anchors_and_rotor_controller_are_preserved():
    for name, expected in REPORT['asset_sha256'].items():
        assert generator.sha256(generator.ASSET/name) == expected
    root = ET.parse(generator.ASSET/'model.sdf').getroot()
    assert root.findtext('model/static') == 'false'
    assert root.findtext("model/joint[@name='world_anchor']/parent") == 'world'
    assert root.findtext("model/joint[@name='world_anchor']/child") == 'support'
    assert root.findtext("model/joint[@name='rotor_shaft']/axis/xyz") == '1 0 0'
    assert float(root.findtext('model/plugin/initial_velocity')) == pytest.approx(.7906)
    assert float(root.findtext('model/link/collision/geometry/cylinder/radius')) == 5
    for name in ('tower_nacelle.obj', 'rotor.obj'):
        rows = (generator.ASSET/'meshes'/name).read_text().splitlines()
        assert any(row.startswith('vn ') for row in rows)
        assert all('//' in row for row in rows if row.startswith('f '))
    assert not any('/home/' in uri.text for uri in root.findall('.//uri'))


def test_offshore_array_is_southeast_and_keeps_rotor_and_coast_clearance():
    sea = [p for p in REPORT['placements'] if p['kind'] == 'offshore']
    assert len(sea) == REPORT['offshore_count'] == 12
    land, _ = generator.land_footprint()
    xy = np.array([p['pose_enu_m_rad'][:2] for p in sea])
    assert np.all(xy[:, 0] > 650) and np.all(xy[:, 0] < 2200)
    assert np.all(xy[:, 1] < -600) and np.all(xy[:, 1] > -2400)
    distances = np.linalg.norm(xy[:, None]-xy[None, :], axis=2)
    np.fill_diagonal(distances, np.inf)
    assert distances.min() > 480 > 2*REPORT['rotor_radius_m']
    assert np.all(shapely.distance(shapely.points(xy), land) > 220)
    assert all(p['pose_enu_m_rad'][2] == 0 for p in sea)


def test_hill_sites_use_existing_terrain_and_nonfloating_footings():
    hills = [p for p in REPORT['placements'] if p['kind'] == 'hill']
    assert len(hills) == REPORT['hill_count'] == 4
    _, land = generator.land_footprint()
    xy = np.array([p['pose_enu_m_rad'][:2] for p in hills])
    distances = np.linalg.norm(xy[:, None]-xy[None, :], axis=2)
    np.fill_diagonal(distances, np.inf)
    assert distances.min() >= 500
    farm = ET.parse(generator.OUTPUT/'model.sdf').getroot()
    assert len(farm.findall('model/include')) == 16
    for item in hills:
        x, y, z, *_ = item['pose_enu_m_rad']
        assert land.covers(shapely.Point(x, y).buffer(8.1))
        lo, hi = item['sampled_terrain_z_range_m']
        assert lo > 65 and hi-lo <= 5
        assert z == pytest.approx(hi+.4)
        assert item['foundation_bottom_z_m'] < lo
        link = farm.find(f"model/model/link[@name='{item['name']}_footing']")
        assert link is not None
        assert float(link.findtext('collision/geometry/cylinder/radius')) == 8
    for name, expected in REPORT['terrain_sha256'].items():
        assert generator.sha256(generator.CHART/name) == expected


def test_hill_rotor_tip_sweeps_clear_both_terrain_meshes():
    link = ET.parse(generator.CHART/'model_gaogongdao.sdf').getroot().find("model/link[@name='chart_obstacle_001']")
    pose = np.fromstring(link.findtext('pose'), sep=' ')[:3]
    visual = generator.Surface(generator.CHART/'meshes/green_island_001_connected.obj', pose)
    collision = generator.Surface(generator.CHART/'meshes/green_island_001_collision_connected.obj', pose)
    angle = np.linspace(0, 2*math.pi, 361)
    tip = np.column_stack((-120*np.sin(.10472)*np.sin(angle)+11.288,
                           120*np.cos(angle), 120*np.cos(.10472)*np.sin(angle)+150))
    for item in REPORT['placements']:
        if item['kind'] != 'hill':
            continue
        x, y, z, _, _, yaw = item['pose_enu_m_rad']
        points = tip.copy()
        rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
        points[:, :2] = points[:, :2]@rotation.T
        points += [x, y, z]
        a, _ = visual.sample(points[:, :2])
        b, _ = collision.sample(points[:, :2])
        clearance = points[:, 2]-np.maximum(a, b)
        assert np.all(np.isfinite(clearance)) and clearance.min() > 15


def test_layout_can_be_regenerated_identically_without_external_documents_directory(tmp_path):
    generator.generate(tmp_path)
    # GeoJSON mappings use tuples in memory and lists after JSON decoding.
    # Compare serialized artifacts, not those two different Python schemas.
    for name in ('model.sdf', 'layout.json'):
        assert generator.sha256(tmp_path/name) == generator.sha256(generator.OUTPUT/name)
