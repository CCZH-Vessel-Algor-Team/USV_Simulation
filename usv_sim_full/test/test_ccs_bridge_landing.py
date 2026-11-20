"""Geometry contracts for the Tianwan bridge's north shore tie-in."""
from collections import Counter
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest


PACKAGE = Path(__file__).resolve().parents[1]
MODELS = PACKAGE/'worlds/models'
CHART = MODELS/'ccs_chart_obstacles_enc_5km'
BRIDGE = MODELS/'ccs_tianwan_bridge'
LAYOUT = json.loads((BRIDGE/'layout.json').read_text())
sys.path.insert(0, str(PACKAGE))
from usv_sim_full import ccs_bridge as bridge
from usv_sim_full import ccs_bridge_landing as landing


def _center_section():
    center = bridge.Centerline(LAYOUT['centerline_enu'])
    return bridge._road_sections(center, LAYOUT)[0][1]


def _obj(path, roof_only=False):
    vertices, faces = [], []
    groups = ()
    for line in path.read_text().splitlines():
        if line.startswith('v '):
            vertices.append(tuple(map(float, line.split()[1:])))
        elif line.startswith('g '):
            groups = line.split()[1:]
        elif line.startswith('f ') and (not roof_only or 'terrain_surface' in groups):
            faces.append(tuple(int(i.split('/')[0])-1 for i in line.split()[1:]))
    return np.asarray(vertices), np.asarray(faces)


def _height_function(triangles):
    o, a, b = triangles[:, 0], triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0]
    denom = a[:, 0]*b[:, 1]-a[:, 1]*b[:, 0]
    keep = abs(denom) > 1e-10
    o, a, b, denom = o[keep], a[keep], b[keep], denom[keep]

    def height(x, y):
        diff = (x, y)-o[:, :2]
        u = (diff[:, 0]*b[:, 1]-diff[:, 1]*b[:, 0])/denom
        v = (a[:, 0]*diff[:, 1]-a[:, 1]*diff[:, 0])/denom
        hit = (u >= -1e-8)&(v >= -1e-8)&(u+v <= 1+1e-8)
        assert hit.any(), (x, y)
        return np.max((o[:, 2]+u*a[:, 2]+v*b[:, 2])[hit])
    return height


def test_approach_matches_road_profile_and_has_real_continuous_collision():
    first = _center_section()
    rows = landing.sections(first)
    assert rows[-1] == (0.0, first)
    assert LAYOUT['model']['north_approach']['length_m'] == pytest.approx(-rows[0][0], abs=.001)
    assert LAYOUT['model']['north_approach']['extra_pier_lines'] == 2
    assert abs(landing.section(first[1]+.1, first)[-1]-first[-1]) < .001
    assert max(abs((b[-1]-a[-1])/(s1-s0)) for (s0, a), (s1, b) in zip(rows, rows[1:])) < .09

    vertices, faces = _obj(BRIDGE/'meshes/seg_00_solid.obj')
    tris = vertices[faces]
    top = _height_function(tris[tris[:, :, 1].max(axis=1) > 811])
    for y in (821, 831, 833, 844, 868, 892, 894, 942, 987, 1018):
        if y < first[1]:
            x, _, n, width, _, z = first
            assert top(x+(first[1]-y)*n[1]/n[0], y) == pytest.approx(z, abs=.02)
            continue
        x, _, (nx, ny), width, _, z = landing.section(y, first)
        for off in (-width*.4, 0, width*.4):
            assert top(x+nx*off, y+ny*off) == pytest.approx(z, abs=.10)


@pytest.mark.parametrize('kind', ['visual', 'collision'])
def test_cut_and_fill_supports_the_land_road_and_leaves_the_far_island_alone(kind):
    variant = ET.parse(CHART/'model_gaogongdao.sdf')
    link = variant.find("model/link[@name='chart_obstacle_001']")
    pose = np.fromstring(link.findtext('pose'), sep=' ')[:3]
    uri = link.findtext(f"{kind}[@name='{kind}']/geometry/mesh/uri")
    report = json.loads((CHART/'bridge_landing.json').read_text())
    assert uri.endswith('_bridge_landing.obj')
    assert bridge.sha256(CHART/uri) == report['terrain'][kind]['output_sha256']
    verts, faces = _obj(CHART/uri, roof_only=True)
    tri = verts[faces]+pose
    xs, ys = tri[:, :, 0], tri[:, :, 1]
    nearby = (xs.max(axis=1) > -740)&(xs.min(axis=1) < -380)&(ys.max(axis=1) > 820)&(ys.min(axis=1) < 1090)
    height = _height_function(tri[nearby])
    first = _center_section()
    for y in range(int(landing.JOINT_Y), int(landing.END_Y)+1, 2):
        x, _, (nx, ny), width, _, z = landing.section(y, first)
        for off in np.linspace(-width/2, width/2, 9):
            assert -.65 < height(x+nx*off, y+ny*off)-z < -.14
    old_name = ('green_island_001_connected_main.obj' if kind == 'visual' else
                'green_island_001_collision_connected.obj')
    old_verts, old_faces = _obj(CHART/'meshes'/old_name, roof_only=True)

    def unchanged(vertices, indices):
        terrain = vertices[indices]+pose
        y = terrain[:, :, 1]
        outside = (y.max(axis=1) < 800)|(y.min(axis=1) > 1120)
        return Counter(tuple(map(tuple, t)) for t in terrain[outside])

    assert unchanged(verts, faces) == unchanged(old_verts, old_faces)
