#!/usr/bin/env python3
"""Generate the painted M5 mesh without changing its geometry or normals.

The original CAD export has one material and no part labels.  Its local Y axis
is vertical; the sloping sheer line below was fitted to its side projection.
Run this script after changing the palette or the partition, not at runtime.
"""
import copy
from pathlib import Path
import xml.etree.ElementTree as ET


MESH_DIR = Path(__file__).resolve().parents[1] / 'description/models/body-Base/mesh'
SOURCE = MESH_DIR / 'M5_body.dae'
OUTPUT = MESH_DIR / 'M5_body_colored.dae'
COLORS = {
    'hull_red': '0.56 0.24 0.25 1',
    'deck_blue': '0.015 0.22 0.58 1',
    'cabin_white': '0.82 0.82 0.78 1',
}


def classify(x, y):
    # The deck rises towards the bow (positive local X).
    sheer = 1.54 + min(max(x - 4.45, 0.0) * 0.105, 0.235)
    if y < sheer:
        return 'hull_red'
    if y < sheer + 0.20:
        return 'deck_blue'
    return 'cabin_white'


def main():
    tree = ET.parse(SOURCE)
    root = tree.getroot()
    ns = root.tag.split('}')[0][1:]
    ET.register_namespace('', ns)
    q = lambda name: f'{{{ns}}}{name}'
    mesh = root.find('.//' + q('geometry') + '/' + q('mesh'))
    positions = list(map(float, mesh.find(q('source') + '/' + q('float_array')).text.split()))
    triangles = mesh.find(q('triangles'))
    inputs = triangles.findall(q('input'))
    stride = max(int(i.get('offset')) for i in inputs) + 1
    vertex_offset = next(int(i.get('offset')) for i in inputs if i.get('semantic') == 'VERTEX')
    indices = list(map(int, triangles.find(q('p')).text.split()))
    grouped = {name: [] for name in COLORS}
    for start in range(0, len(indices), 3 * stride):
        face = indices[start:start + 3 * stride]
        vertices = [face[i * stride + vertex_offset] * 3 for i in range(3)]
        x = sum(positions[i] for i in vertices) / 3
        y = sum(positions[i + 1] for i in vertices) / 3
        grouped[classify(x, y)].extend(face)

    mesh.remove(triangles)
    for name, values in grouped.items():
        part = copy.deepcopy(triangles)
        part.set('material', name)
        part.set('count', str(len(values) // (3 * stride)))
        part.find(q('p')).text = ' '.join(map(str, values))
        mesh.append(part)

    effects = root.find(q('library_effects'))
    materials = root.find(q('library_materials'))
    binding = root.find('.//' + q('instance_geometry') + '/' + q('bind_material') + '/' + q('technique_common'))
    for child in list(binding):
        binding.remove(child)
    for name, rgba in COLORS.items():
        effect = ET.SubElement(effects, q('effect'), id=name + '_effect')
        profile = ET.SubElement(effect, q('profile_COMMON'))
        common = ET.SubElement(profile, q('technique'), sid='common')
        phong = ET.SubElement(common, q('phong'))
        for channel in ('ambient', 'diffuse'):
            ET.SubElement(ET.SubElement(phong, q(channel)), q('color')).text = rgba
        ET.SubElement(ET.SubElement(phong, q('specular')), q('color')).text = '0.1 0.1 0.1 1'
        ET.SubElement(ET.SubElement(phong, q('shininess')), q('float')).text = '20'
        material = ET.SubElement(materials, q('material'), id=name, name=name)
        ET.SubElement(material, q('instance_effect'), url='#' + name + '_effect')
        ET.SubElement(binding, q('instance_material'), symbol=name, target='#' + name)
    tree.write(OUTPUT, encoding='utf-8', xml_declaration=True)
    print(f'{OUTPUT}: ' + ', '.join(f'{k}={len(v) // (3 * stride)} faces' for k, v in grouped.items()))


if __name__ == '__main__':
    main()
