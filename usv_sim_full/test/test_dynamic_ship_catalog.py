"""No ROS graph or running simulator: approved resources and native SDF contract."""
import copy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS
import xml.etree.ElementTree as ET

import pytest

from usv_sim_full.dynamic_ship_catalog import (
    ShipCatalog, check_water_footprint, rectangles_overlap,
)

PACKAGE = Path(__file__).parents[1]
CATALOG = PACKAGE/'description/models/target_fleet/catalog.json'


@pytest.fixture(scope='module')
def catalog():
    return ShipCatalog(CATALOG)


def test_catalog_has_legacy_debug_and_ten_distinct_native_models(catalog):
    assert len(catalog.models) == 13
    assert catalog.get('legacy_10m').length == 10.
    assert catalog.get('debug_box').backend == 'box'
    assert catalog.get('debug_cylinder').beam == 8.
    native = [m for m in catalog.models.values() if m.backend == 'fleet_sdf']
    assert len(native) == 10
    assert min(m.length for m in native) > 20
    assert max(m.length for m in native) == pytest.approx(399.004)
    assert all(m.radius == math.hypot(m.length, m.beam)/2 for m in native)
    with pytest.raises(ValueError, match='Unknown'):
        catalog.get('not_a_model')


def test_catalog_display_names_are_chinese_without_changing_source_identity(catalog):
    entries = json.loads(CATALOG.read_text())['models']
    for entry in entries:
        model = catalog.get(entry['model_id'])
        assert model.display_name == entry['display_name']
        assert any('\u4e00' <= c <= '\u9fff' for c in model.display_name)
        assert not any(c.isascii() and c.isalpha() for c in model.display_name)
        assert entry['source_display_name']
        assert ET.parse(CATALOG.parent/model.sdf).find('model').get('name') == model.model_id


@pytest.mark.parametrize('model_id', [m['model_id'] for m in json.loads(CATALOG.read_text())['models']])
def test_native_instances_preserve_source_physics_materials_and_resources(catalog, model_id):
    model = catalog.get(model_id)
    source = ET.parse(CATALOG.parent/model.sdf).getroot()
    instances = [ET.fromstring(catalog.sdf(model_id, name)) for name in ('fleet_a', 'fleet_b')]
    for root, name in zip(instances, ('fleet_a', 'fleet_b')):
        entity = root.find('model')
        assert entity.get('name') == name
        assert entity.findtext('link/inertial/mass') == source.findtext('model/link/inertial/mass')
        assert entity.findtext("plugin[@filename='libTargetShipController.so']/topic") == f'/model/{name}/cmd_vel'
        added = entity.find("plugin[@filename='libFleetBuoyancy.so']")
        assert added.findtext('link_name') == 'base_link'
        assert not root.findall(".//plugin[@filename='libSurface.so']")
        assert len(root.findall('.//plugin')) == 3
        entity.remove(added)
        entity.set('name', source.find('model').get('name'))
        entity.find("plugin[@filename='libTargetShipController.so']/topic").text = source.findtext(
            "model/plugin[@filename='libTargetShipController.so']/topic")
        original_resources = list(source.iter('uri')) + list(source.iter('albedo_map'))
        actual_resources = list(root.iter('uri')) + list(root.iter('albedo_map'))
        for original, actual in zip(original_resources, actual_resources):
            assert actual.text == (CATALOG.parent/model_id/original.text).resolve().as_uri()
            actual.text = original.text
        # Covers complete link offset, all collisions, inertia, drag, controller,
        # scale and material, rather than checking a handful of selected values.
        assert ET.tostring(root) == ET.tostring(source)


def test_catalog_rejects_changed_resources_and_unsafe_instance_names(catalog, tmp_path):
    for name in ('bad/name', 'bad<name', '', '42'):
        with pytest.raises(ValueError, match='instance name'):
            catalog.sdf('vb_rumba', name)
    data = json.loads(CATALOG.read_text())
    data['source_sha256'] = {'changed': '0'*64}
    (tmp_path/'changed').write_text('different')
    path = tmp_path/'catalog.json'
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError, match='hash differs'):
        ShipCatalog(path)
    data['source_sha256'] = {'../escape': '0'*64}
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError, match='Invalid installed'):
        ShipCatalog(path)


def test_installed_payload_matches_recorded_source_hashes():
    data = json.loads(CATALOG.read_text())
    assert len(data['source_sha256']) == 60
    for name, digest in data['source_sha256'].items():
        assert hashlib.sha256((CATALOG.parent/name).read_bytes()).hexdigest() == digest
    assert all(not p.name.endswith('.sh') for p in CATALOG.parent.rglob('*'))


def test_importer_is_reproducible_and_refuses_overwriting_existing_work(tmp_path):
    spec = importlib.util.spec_from_file_location('fleet_import', PACKAGE/'tools/import_dynamic_fleet.py')
    importer = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(importer)
    # Minimal source using one copied, approved resource folder; no external path
    # is needed for the regression suite and no supplied executable is run.
    source = tmp_path/'source'
    source.mkdir()
    import shutil
    shutil.copytree(CATALOG.parent/'vb_rumba', source/'models/vb_rumba')
    boat = {'id': 'vb_rumba', 'name': 'VB RUMBA', 'kind': 'harbour_tug',
            'length': 27.59, 'beam': 12.93, 'deck': 9.98, 'draft': 5.95, 'start_z_m': .2}
    (source/'fleet-manifest.json').write_text(json.dumps({'boats': [boat]}))
    output = tmp_path/'output'
    first = importer.import_fleet(source, output)
    assert first['models'][0]['display_name'] == '伦巴号（港作拖轮）'
    assert first['models'][0]['source_display_name'] == 'VB RUMBA'
    assert first['models'][0]['display_name'] == next(
        entry['display_name'] for entry in json.loads(CATALOG.read_text())['models']
        if entry['model_id'] == 'vb_rumba')
    assert importer.import_fleet(source, output) == first
    (output/'vb_rumba/model.sdf').write_text('user work')
    with pytest.raises(ValueError, match='Refusing to overwrite'):
        importer.import_fleet(source, output)
    assert (output/'vb_rumba/model.sdf').read_text() == 'user work'


def water_map():
    info = NS(resolution=1., width=100, height=100,
              origin=NS(position=NS(x=0., y=0.), orientation=NS(x=0., y=0., z=0., w=1.)))
    return info, [0]*10000


def test_rotated_whole_hull_checks_cells_not_just_center():
    info, data = water_map()
    check_water_footprint(info, data, 50., 50., math.pi/2, 40., 6.)
    data[68*100+50] = 100  # far from center, inside the rotated bow
    with pytest.raises(ValueError, match='occupied/unknown'):
        check_water_footprint(info, data, 50., 50., math.pi/2, 40., 6.)
    data[68*100+50] = -1
    with pytest.raises(ValueError, match='occupied/unknown'):
        check_water_footprint(info, data, 50., 50., math.pi/2, 40., 6.)
    # The same non-square footprint now points away from that occupied cell.
    check_water_footprint(info, data, 50., 50., 0., 40., 6.)


def test_whole_hull_must_fit_map_and_map_must_be_valid():
    info, data = water_map()
    with pytest.raises(ValueError, match='inside the map'):
        check_water_footprint(info, data, 10., 50., 0., 40., 6.)
    for change in (lambda i: setattr(i, 'resolution', 0.),
                   lambda i: setattr(i.origin.orientation, 'x', .1),
                   lambda i: setattr(i.origin.position, 'x', float('nan'))):
        invalid = copy.deepcopy(info)
        change(invalid)
        with pytest.raises(ValueError, match='valid, unrotated'):
            check_water_footprint(invalid, data, 50., 50., 0., 40., 6.)


def test_rectangle_overlap_handles_long_rotated_hulls_and_margin():
    a = (0., 0., math.pi/2, 400., 58.)
    assert rectangles_overlap(a, (0., 199., 0., 10., 4.))
    assert not rectangles_overlap(a, (100., 0., 0., 10., 4.))
    assert not rectangles_overlap((0., 0., 0., 10., 4.), (11., 0., 0., 10., 4.))
    assert rectangles_overlap((0., 0., 0., 10., 4.), (11., 0., 0., 10., 4.), margin=1.)
