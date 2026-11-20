"""Runtime meshes are self-contained, metric and independent of CAD conversion tools."""
import hashlib
import json
from pathlib import Path
import struct

import numpy as np
import pytest

DIRECTORY = Path(__file__).resolve().parents[1] / 'description/meshes'
PROFILE = json.loads((DIRECTORY / 'cad_assets.json').read_text())
MESHES = {**PROFILE['mast']['meshes'], **PROFILE['navigation_lamp']['meshes']}


@pytest.mark.parametrize('name', sorted(MESHES))
def test_binary_mesh_matches_profile_and_uses_metres(name):
    data = (DIRECTORY / name).read_bytes()
    count = struct.unpack_from('<I', data, 80)[0]
    assert len(data) == 84 + 50 * count
    assert count == MESHES[name]['triangles']
    assert 0 < count < 100000
    assert hashlib.sha256(data).hexdigest() == MESHES[name]['sha256']
    records = np.frombuffer(data, dtype=np.dtype([
        ('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2'),
    ]), offset=84)
    vertices = records['vertices'].reshape(-1, 3)
    assert np.isfinite(vertices).all()
    actual = np.concatenate([vertices.min(axis=0), vertices.max(axis=0)])
    assert actual == pytest.approx(MESHES[name]['bounds_m'], abs=1e-7)
    assert max(abs(actual)) < 1.0  # A millimetre/metre error must fail here.


def test_fixed_base_and_moving_mesh_meet_at_virtual_hinge():
    mast = PROFILE['mast']
    base, moving = MESHES['mast_base.stl']['bounds_m'], MESHES['mast_body.stl']['bounds_m']
    assert base[5] == pytest.approx(mast['hinge_height_m'])
    assert moving[2] == pytest.approx(0)
    assert moving[5] + mast['hinge_height_m'] == pytest.approx(mast['native_height_m'])
    assert mast['native_height_m'] == pytest.approx(0.5978691, abs=1e-6)
    lens_bounds = MESHES['mast_lens.stl']['bounds_m']
    lens_top = mast['lens_origin_from_hinge_m'][2] + lens_bounds[5]
    assert lens_top < moving[5]


def test_navigation_lamp_omits_the_second_open_lid_display():
    assert PROFILE['navigation_lamp']['selected_solids'] == 6
    bbox = MESHES['navigation_lamp_housing.stl']['bounds_m']
    assert bbox[1] > -0.08 and bbox[4] < 0.08
    assert bbox[5] == pytest.approx(0.200034, abs=1e-6)


def test_replaced_crossbar_lamps_are_removed_from_mast_body():
    mast = PROFILE['mast']
    assert mast['removed_crossbar_lamps'] == {'solids': 24, 'open_shells': 4, 'lamps': 4}
    data = (DIRECTORY / 'mast_body.stl').read_bytes()
    records = np.frombuffer(data, dtype=np.dtype([
        ('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2'),
    ]), offset=84)
    vertices = records['vertices'].reshape(-1, 3)
    for anchor in mast['navigation_light_mounts_from_hinge_m'].values():
        local = vertices - np.array(anchor)
        old_bulb = ((abs(local[:, 0]) < 0.0068) & (abs(local[:, 1]) < 0.0068)
                    & (local[:, 2] > 0.0002) & (local[:, 2] < 0.0158))
        assert not old_bulb.any(), 'An old decorative lens/cap would overlap the new lamp'


def test_all_seven_extra_decorative_lamps_removed_but_mounts_preserved():
    mast = PROFILE['mast']
    assert mast['removed_extra_lamps'] == {'solids': 36, 'open_shells': 3, 'lamps': 7}
    assert len(mast['extra_light_order']) == 7
    data = (DIRECTORY / 'mast_body.stl').read_bytes()
    records = np.frombuffer(data, dtype=np.dtype([
        ('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2'),
    ]), offset=84)
    vertices = records['vertices'].reshape(-1, 3)
    for mount in mast['extra_light_mounts_from_hinge_m'].values():
        low, high = np.array(mount['source_selection_bounds_m']).reshape(2, 3)
        # Ignore mounting tray at z=0 and source precision at selection boundaries.
        low[2] += 0.0002
        old_lamp = np.all((vertices > low + 1e-6) & (vertices < high - 1e-6), axis=1)
        assert not old_lamp.any()
        local = vertices - np.array(mount['xyz'])
        tray = ((abs(local[:, 0]) < 0.012) & (abs(local[:, 1]) < 0.012)
                & (local[:, 2] < 0.0001) & (local[:, 2] > -0.01))
        assert tray.any(), 'Replacement must retain its CAD mounting support'
