"""Appearance assets must not change the physical model or navigation envelope."""
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from usv_sim_full.wind_turbine_appearance import (
    OFFSHORE, HILL, generate, physical_signature, source_triangles,
)


MODELS = Path(__file__).parents[1]/'worlds/models'
SOURCE = MODELS/'iea15_offshore_turbine'
REPORT = json.loads((MODELS/OFFSHORE/'appearance.json').read_text())


@pytest.mark.parametrize('variant', [OFFSHORE, HILL])
def test_visual_variant_retains_all_source_physics_and_controllers(variant):
    original = ET.parse(SOURCE/'model.sdf').getroot()
    styled = ET.parse(MODELS/variant/'model.sdf').getroot()
    assert physical_signature(styled) == physical_signature(original)
    assert styled.findtext("model/link[@name='rotor']/pose") == '11.288 0 150 0 -0.104720 0'
    assert len(styled.findall('model/link')) == 2
    assert len(styled.findall('model/joint')) == 2
    assert not styled.findall('.//light') and not styled.findall('.//sensor')
    assert not styled.findall(".//link[@name='rotor']/collision")


def test_original_files_and_generated_resources_match_provenance():
    for name, expected in REPORT['source_sha256'].items():
        assert hashlib.sha256((SOURCE/name).read_bytes()).hexdigest() == expected
    for name, expected in REPORT['generated_sha256'].items():
        assert hashlib.sha256((MODELS/name).read_bytes()).hexdigest() == expected


def test_material_batched_meshes_are_finite_have_valid_normals_and_bounded_cost():
    for variant in (OFFSHORE, HILL):
        stats = REPORT['statistics'][variant]
        assert stats['support_triangles']+stats['rotor_triangles'] < 15000
        assert stats['support_visuals'] <= 7 and stats['rotor_visuals'] == 2
    for path in (MODELS/OFFSHORE/'meshes').glob('*.obj'):
        vertices, normals, faces = [], [], []
        for line in path.read_text().splitlines():
            p = line.split()
            if p and p[0] == 'v':
                vertices.append(list(map(float, p[1:4])))
            elif p and p[0] == 'vn':
                normals.append(list(map(float, p[1:4])))
            elif p and p[0] == 'f':
                assert len(p) == 4 and all('//' in s for s in p[1:])
                faces.append([int(s.split('//')[0])-1 for s in p[1:]])
        vertices, normals = np.array(vertices), np.array(normals)
        assert len(vertices) == len(normals) == 3*len(faces)
        assert np.isfinite(vertices).all() and np.isfinite(normals).all()
        assert np.allclose(np.linalg.norm(normals, axis=1), 1., atol=1e-6)
        tri = vertices[np.array(faces)]
        cross = np.cross(tri[:, 1]-tri[:, 0], tri[:, 2]-tri[:, 0])
        assert np.all(np.linalg.norm(cross, axis=1) > 1e-10)
        assert np.all(np.einsum('ij,ij->i', cross, normals[::3]) > 0), path.name


def test_source_blade_surfaces_and_swept_radius_are_not_rescaled():
    before = source_triangles(SOURCE/'meshes/rotor.obj')
    blades = source_triangles(MODELS/OFFSHORE/'meshes/blades.obj')
    assert len(blades) == 270
    source_counts = Counter(before)
    assert all(source_counts[tri] >= count for tri, count in Counter(blades).items())
    radius = lambda triangles: max(math.hypot(p[1], p[2]) for tri in triangles for p in tri)
    assert radius(blades) == pytest.approx(radius(before), abs=1e-7)
    assert radius(blades)*2 == pytest.approx(233.4434278144707)


def test_variants_use_installed_model_uris_and_distinguish_sea_from_hill():
    for variant in (OFFSHORE, HILL):
        root = ET.parse(MODELS/variant/'model.sdf').getroot()
        visuals = root.findall('.//visual')
        for visual in visuals:
            uri = visual.findtext('geometry/mesh/uri')
            assert uri.startswith('model://') and '/home/' not in uri
            assert (MODELS/uri.removeprefix('model://')).is_file()
            assert visual.findtext('geometry/mesh/scale') == '1 1 1'
            assert 0 <= float(visual.findtext('material/pbr/metal/roughness')) <= 1
        color = root.findtext("model/link/visual[@name='appearance_foundation']/material/diffuse")
        if variant == OFFSHORE:
            assert color.startswith('0.9 0.65 0.09')
            assert root.find("model/link/visual[@name='appearance_patina']") is not None
        else:
            assert color.startswith('0.84 0.87 0.88')
            assert root.find("model/link/visual[@name='appearance_patina']") is None


def test_generator_is_reproducible_and_protects_unfamiliar_edits(tmp_path):
    first = generate(SOURCE, tmp_path)
    assert first == REPORT
    assert generate(SOURCE, tmp_path) == first
    target = tmp_path/OFFSHORE/'meshes/sea_shell.obj'
    target.write_text('user edit')
    with pytest.raises(ValueError, match='modified outside'):
        generate(SOURCE, tmp_path)
    assert target.read_text() == 'user edit'


def test_farm_selects_only_appearance_variants_with_the_original_placement_names():
    root = ET.parse(MODELS/'ccs_wind_farm/model.sdf').getroot()
    includes = root.findall('model/include')
    assert Counter(i.findtext('uri') for i in includes) == {
        'model://'+OFFSHORE: 12, 'model://'+HILL: 4,
    }
    assert len({i.findtext('name') for i in includes}) == 16
