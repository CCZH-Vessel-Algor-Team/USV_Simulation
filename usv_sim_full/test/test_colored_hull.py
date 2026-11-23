"""Contract tests for the painted M5 visual asset."""
from collections import Counter
from pathlib import Path
import xml.etree.ElementTree as ET


MESH_DIR = Path(__file__).resolve().parents[1] / 'description/models/body-Base/mesh'
NS = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}


def load_mesh(name):
    root = ET.parse(MESH_DIR / name).getroot()
    mesh = root.find('.//c:geometry/c:mesh', NS)
    return root, mesh


def faces(mesh):
    result = Counter()
    for part in mesh.findall('c:triangles', NS):
        values = list(map(int, part.find('c:p', NS).text.split()))
        stride = 1 + max(int(i.get('offset')) for i in part.findall('c:input', NS))
        width = 3 * stride
        assert len(values) == int(part.get('count')) * width
        result.update(tuple(values[i:i + width]) for i in range(0, len(values), width))
    return result


def test_paint_preserves_all_geometry_and_assigns_materials():
    _, original = load_mesh('M5_body.dae')
    root, painted = load_mesh('M5_body_colored.dae')
    assert faces(original) == faces(painted)
    for old, new in zip(original.findall('c:source', NS), painted.findall('c:source', NS)):
        assert old.find('c:float_array', NS).text == new.find('c:float_array', NS).text
    parts = painted.findall('c:triangles', NS)
    assert {part.get('material') for part in parts} == {'hull_red', 'deck_blue', 'cabin_white'}
    assert all(int(part.get('count')) > 0 for part in parts)
    bindings = root.findall('.//c:instance_geometry//c:instance_material', NS)
    assert {item.get('symbol') for item in bindings} == {'hull_red', 'deck_blue', 'cabin_white'}
