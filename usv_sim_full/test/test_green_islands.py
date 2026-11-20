"""The terrain must preserve the chart footprint and stay reproducible."""
from collections import Counter
from functools import lru_cache
import importlib.util
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest


MODEL = (Path(__file__).resolve().parents[1] / 'worlds/models'
         / 'ccs_chart_obstacles_enc_5km')


@lru_cache(maxsize=16)
def read_obj(path, roof_only):
    vertices, faces = [], []
    group = None
    for line in path.read_text().splitlines():
        parts = line.split()
        if not parts:
            continue
        if parts[0] == 'v':
            vertices.append(tuple(map(float, parts[1:4])))
        elif parts[0] == 'g':
            group = parts[1]
        elif parts[0] == 'f' and len(parts) == 4:
            if roof_only and group != 'terrain_surface':
                continue
            face = tuple(int(part.split('/')[0]) - 1 for part in parts[1:])
            faces.append(face)
    return vertices, faces


def area_xy(vertices, faces):
    total = 0.0
    for face in faces:
        a, b, c = (vertices[i] for i in face)
        cross = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
        assert cross > 0
        total += cross / 2
    return total


@pytest.mark.parametrize('index', range(1, 5))
def test_original_chart_area_and_coordinates_preserved(index):
    mesh = MODEL / 'meshes'
    original, original_faces = read_obj(mesh / f'chart_obstacle_{index:03d}.obj', False)
    source_roof = [face for face in original_faces if all(original[i][2] == 3 for i in face)]
    expected = area_xy(original, source_roof)
    for suffix in ('', '_collision'):
        vertices, roof = read_obj(mesh / f'green_island_{index:03d}{suffix}.obj', True)
        assert roof
        assert area_xy(vertices, roof) == pytest.approx(expected, rel=1e-6)
        assert max(v[2] for v in vertices) > 3
        assert min(v[2] for v in vertices) == -5
        assert min(v[0] for v in vertices) == pytest.approx(min(v[0] for v in original))
        assert max(v[0] for v in vertices) == pytest.approx(max(v[0] for v in original))
        assert min(v[1] for v in vertices) == pytest.approx(min(v[1] for v in original))
        assert max(v[1] for v in vertices) == pytest.approx(max(v[1] for v in original))


def test_sdf_uses_islands_for_only_the_four_chart_links():
    root = ET.parse(MODEL / 'model.sdf').getroot()
    for index in range(1, 5):
        link = root.find(f".//link[@name='chart_obstacle_{index:03d}']")
        assert link.findtext('visual/geometry/mesh/uri') == f'meshes/green_island_{index:03d}.obj'
        assert link.findtext('collision/geometry/mesh/uri') == f'meshes/green_island_{index:03d}_collision.obj'
        assert link.find('visual/material') is None
        assert link.findtext("visual[@name='vegetation']/geometry/mesh/uri") == f'meshes/green_island_{index:03d}_vegetation.obj'
        assert link.findtext("visual[@name='vegetation']/cast_shadows") == 'false'
    assert root.find(".//link[@name='chart_obstacle_005']/visual/geometry/mesh/uri").text.startswith('model://mb_marker_buoy_red')


@pytest.mark.parametrize('index', range(1, 5))
def test_terrain_is_closed_without_unmatched_edges(index):
    for suffix in ('', '_collision'):
        vertices, faces = read_obj(MODEL / 'meshes' / f'green_island_{index:03d}{suffix}.obj', False)
        assert all(0 <= i < len(vertices) for face in faces for i in face)
        edges = Counter(tuple(sorted(edge)) for a, b, c in faces
                        for edge in ((a, b), (b, c), (c, a)))
        assert set(edges.values()) == {2}, Counter(edges.values())


@pytest.mark.parametrize('index', range(1, 5))
def test_bundled_textures_and_bounded_decoration_budget(index):
    import cv2

    stem = f'green_island_{index:03d}'
    metadata = json.loads((MODEL / 'meshes' / f'{stem}_details.json').read_text())
    assert metadata['generator_version'] == 2
    assert 0 < len(metadata['trees']) <= {1: 1400, 2: 240, 3: 180, 4: 14}[index]
    assert metadata['decoration_faces'] < 400000
    for name in (stem, stem + '_vegetation'):
        mtl = MODEL / 'meshes' / f'{name}.mtl'
        for line in mtl.read_text().splitlines():
            if line.startswith('map_Kd '):
                texture = (mtl.parent / line.split(maxsplit=1)[1]).resolve()
                assert texture.is_file()
                image = cv2.imread(str(texture))
                assert image is not None and max(image.shape[:2]) <= 4096
            if line.startswith(('Kd ', 'Ka ', 'Ks ')):
                assert all(0 <= float(value) <= 1 for value in line.split()[1:])


@pytest.mark.parametrize('index', range(1, 5))
def test_coastline_holes_and_decorations_remain_inside_chart(index):
    shapely = pytest.importorskip('shapely', minversion='2.1')
    mesh = MODEL / 'meshes'
    old, old_faces = read_obj(mesh / f'chart_obstacle_{index:03d}.obj', False)
    old = np.asarray(old)
    roofs = [face for face in old_faces if np.all(old[list(face), 2] == 3)]
    footprint = shapely.union_all(shapely.polygons(old[roofs, :2]))
    for suffix in ('', '_collision'):
        points, faces = read_obj(mesh / f'green_island_{index:03d}{suffix}.obj', True)
        projected = shapely.union_all(shapely.polygons(np.asarray(points)[faces, :2]))
        assert footprint.symmetric_difference(projected).area < max(0.01, footprint.area * 1e-8)
    details = json.loads((mesh / f'green_island_{index:03d}_details.json').read_text())
    for item in details['trees'] + details['rocks']:
        pos = shapely.Point(*item['position'][:2])
        assert footprint.contains(pos)
        assert pos.distance(footprint.boundary) > item['radius'] * 1.2


def test_generator_preserves_a_concave_coast_and_hole_deterministically():
    shapely = pytest.importorskip('shapely', minversion='2.1')
    path = MODEL.parents[2] / 'tools' / 'generate_green_islands.py'
    spec = importlib.util.spec_from_file_location('island_generator', path)
    generator = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(generator)
    polygon = shapely.Polygon([(0, 0), (90, 0), (90, 70), (50, 70), (50, 90), (0, 90)],
                              [[(10, 10), (20, 10), (20, 30), (10, 30)]])
    one = generator.triangulated_surface(polygon, 8, 73)
    two = generator.triangulated_surface(polygon, 8, 73)
    np.testing.assert_array_equal(one[0], two[0])
    np.testing.assert_array_equal(one[1], two[1])
    projected = shapely.union_all(shapely.polygons(one[0][one[1]]))
    assert polygon.symmetric_difference(projected).area < 0.001


def test_chart_writer_supports_natural_islands_and_block_fallback(tmp_path):
    pytest.importorskip('shapely', minversion='2.1')
    path = MODEL.parents[2] / 'tools' / 'generate_chart_obstacles.py'
    spec = importlib.util.spec_from_file_location('chart_generator', path)
    generator = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(generator)
    components = [('chart_obstacle_001', (12.5, -8)), ('chart_obstacle_005', (20, 30))]
    for natural in (True, False):
        generator.write_model_sdf(tmp_path, 'test', components, natural_islands=natural)
        root = ET.parse(tmp_path / 'model.sdf').getroot()
        link = root.find(".//link[@name='chart_obstacle_001']")
        assert link.findtext('pose') == '12.500000 -8.000000 0 0 0 0'
        if natural:
            assert link.findtext('visual/geometry/mesh/uri') == 'meshes/green_island_001.obj'
            assert link.find("visual[@name='vegetation']") is not None
        else:
            assert link.findtext('visual/geometry/mesh/uri') == 'meshes/chart_obstacle_001.obj'
            assert link.find("visual[@name='vegetation']") is None
        buoy = root.find(".//link[@name='chart_obstacle_005']")
        assert buoy.findtext('visual/geometry/mesh/uri').startswith('model://mb_marker_buoy_red')
        assert buoy.findtext('collision/geometry/mesh/uri') == 'meshes/chart_obstacle_005.obj'
