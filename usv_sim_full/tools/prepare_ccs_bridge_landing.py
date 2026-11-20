#!/usr/bin/env python3
"""Bake a narrow cut/fill corridor into the existing Gaogongdao terrain OBJ."""
import argparse
from collections import defaultdict
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import shapely

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
from usv_sim_full import ccs_bridge as bridge
from usv_sim_full import ccs_bridge_landing as landing

MODELS = PACKAGE/'worlds/models'
CHART = MODELS/'ccs_chart_obstacles_enc_5km'
BRIDGE = MODELS/'ccs_tianwan_bridge'
CHART_FILE = 'model_gaogongdao.sdf'
SOURCE_MESHES = {
    'visual': 'green_island_001_connected_main.obj',
    'collision': 'green_island_001_collision_connected.obj',
}
OUTPUT_MESHES = {
    'visual': 'green_island_001_bridge_landing.obj',
    'collision': 'green_island_001_collision_bridge_landing.obj',
}
LOCAL_MATERIAL = 'green_island_001_bridge_landing.mtl'
DIVISIONS = 7


def reference_section(layout):
    center = bridge.Centerline(layout['centerline_enu'])
    return bridge._road_sections(center, layout)[0][1]


def _edge(a, b):
    return tuple(sorted((a, b)))


def _grade_mesh(source, target, pose, first, read_obj, matte=False):
    attributes, records, faces, vertices = read_obj(source)
    aligned = vertices+pose
    uv = np.array([list(map(float, t.split())) for t in attributes['vt']])
    normals = np.array([list(map(float, n.split())) for n in attributes['vn']])
    road = shapely.LineString([landing.section(y, first)[:2] for y in range(862, 1053, 3)])
    patch = road.buffer(62).intersection(shapely.box(-730, 838, -395, 1080))
    shapely.prepare(patch)
    lo_x, lo_y, hi_x, hi_y = patch.bounds

    # Match by position instead of by OBJ index where UVs split an edge.
    def edge(a, b):
        return _edge(tuple(vertices[a]), tuple(vertices[b]))

    selected, split_edges = set(), set()
    group = ''
    for kind, value in records:
        if kind == 'directive' and value.startswith('g '):
            group = value.split()[1]
        elif kind == 'f' and group == 'terrain_surface':
            ids = [ref[0]-1 for ref in faces[value]]
            xy = aligned[ids, :2]
            if (xy[:, 0].max() < lo_x or xy[:, 0].min() > hi_x or
                    xy[:, 1].max() < lo_y or xy[:, 1].min() > hi_y):
                continue
            if patch.intersects(shapely.Polygon(xy)):
                selected.add(value)
                split_edges.update(edge(a, b) for a, b in zip(ids, ids[1:]+ids[:1]))

    replacements = {}
    points, sums = {}, defaultdict(lambda: np.zeros(3))
    old_normals = {}
    moved = {}

    def point(refs, weights):
        weights = np.asarray(weights)
        indices = np.asarray(refs)-1
        source_xyz = weights@vertices[indices[:, 0]]
        key = tuple(round(v, 6)+0.0 for v in source_xyz)
        if key not in points:
            xyz = np.asarray(key)
            x, y, z = xyz+pose
            dz = landing.grade(x, y, z, first)-z
            points[key] = xyz + (0, 0, dz)
            if abs(dz) > 1e-6:
                moved[key] = dz
        old_normals.setdefault(key, weights@normals[indices[:, 2]])
        tex = tuple(round(v, 8)+0.0 for v in weights@uv[indices[:, 1]])
        return key, tex

    def remember(tri):
        a, b, c = (points[k] for k, _ in tri)
        n = np.cross(b-a, c-a)
        for k, _ in tri:
            sums[k] += n
        return tri

    n = DIVISIONS
    for index, refs in enumerate(faces):
        if index in selected:
            grid = {(i, j): point(refs, ((n-i-j)/n, i/n, j/n))
                    for i in range(n+1) for j in range(n+1-i)}
            sub = []
            for i in range(n):
                for j in range(n-i):
                    sub.append(remember((grid[i, j], grid[i+1, j], grid[i, j+1])))
                    if i+j < n-1:
                        sub.append(remember((grid[i+1, j], grid[i+1, j+1], grid[i, j+1])))
            replacements[index] = sub
        else:
            ids = [ref[0]-1 for ref in refs]
            refine = [edge(a, b) in split_edges for a, b in zip(ids, ids[1:]+ids[:1])]
            if any(refine):
                # Keep the original planar neighbor but match every new edge
                # vertex, including the shoreline walls of the solid island.
                boundary = []
                for i, divide in enumerate(refine):
                    for j in range(n if divide else 1):
                        weights = [0.0]*3
                        weights[i], weights[(i+1) % 3] = 1-j/n, j/n
                        boundary.append(point(refs, weights))
                middle = point(refs, (1/3, 1/3, 1/3))
                replacements[index] = [remember((a, b, middle)) for a, b in
                                       zip(boundary, boundary[1:]+boundary[:1])]

    # Retain every unused attribute reference exactly. Reuse positional
    # indices where the original mesh meets the refined triangles.
    key_indices = {tuple(v): i+1 for i, v in enumerate(vertices)}
    reuse = set()
    for index, refs in enumerate(faces):
        if index not in replacements:
            for i, _, _ in refs:
                k = tuple(vertices[i-1])
                if k in moved:
                    raise ValueError(f'{source}: grade crossed an unrefined triangle')
                reuse.add(i)
    new_indices, texture_indices = {}, {}
    for k, v in points.items():
        original = key_indices.get(k)
        if original in reuse:
            vi = original
        else:
            attributes['v'].append(' '.join(f'{a:.6f}' for a in v))
            vi = len(attributes['v'])
        normal = sums[k] if k in moved else old_normals[k]
        normal = normal/max(float(np.linalg.norm(normal)), 1e-12)
        attributes['vn'].append(' '.join(f'{a:.8f}' for a in normal))
        new_indices[k] = vi, len(attributes['vn'])
    with target.open('w') as out:
        out.write('# CCS Tianwan bridge approach; local resurfacing of Gaogongdao.\n')
        for sub in replacements.values():
            for tri in sub:
                for _, t in tri:
                    if t not in texture_indices:
                        attributes['vt'].append(' '.join(f'{a:.8f}' for a in t))
                        texture_indices[t] = len(attributes['vt'])
        for name, values in attributes.items():
            out.writelines(f'{name} {v}\n' for v in values)
        source_material = output_material = ''
        for kind, value in records:
            if kind == 'directive':
                if matte and value.startswith('mtllib '):
                    out.write(f'mtllib {LOCAL_MATERIAL}\n')
                else:
                    out.write(value+'\n')
                if value.startswith('usemtl '):
                    source_material = output_material = value.split()[1]
            else:
                material = 'terrain_cut_fill' if matte and value in replacements else source_material
                if material != output_material:
                    out.write(f'usemtl {material}\n')
                    output_material = material
                if value not in replacements:
                    out.write('f '+' '.join('/'.join(map(str, r)) for r in faces[value])+'\n')
                else:
                    for tri in replacements[value]:
                        out.write('f '+' '.join(f'{new_indices[k][0]}/{texture_indices[t]}/{new_indices[k][1]}'
                                                for k, t in tri)+'\n')
    return {'source_sha256': bridge.sha256(source), 'output_sha256': bridge.sha256(target),
            'refined_source_faces': len(replacements), 'adjusted_vertices': len(moved),
            'height_delta_m': [round(min(moved.values()), 3), round(max(moved.values()), 3)]}


def prepare(chart=CHART, layout_path=BRIDGE/'layout.json', output=CHART, read_obj=None):
    if read_obj is None:
        from prepare_gaogongdao_harbor import read_obj
    chart, output = Path(chart), Path(output)
    layout = json.loads(Path(layout_path).read_text())
    first = reference_section(layout)
    document = ET.parse(chart/CHART_FILE)
    link = document.find("model/link[@name='chart_obstacle_001']")
    pose = np.fromstring(link.findtext('pose'), sep=' ')
    if not np.allclose(pose[3:], 0):
        raise ValueError('Island pose must be a translation in CCS ENU')
    meshes = output/'meshes'
    meshes.mkdir(parents=True, exist_ok=True)
    old_material = (chart/'meshes/green_island_001_harbor.mtl').read_text()
    (meshes/LOCAL_MATERIAL).write_text(old_material + '\n'
                                     'newmtl terrain_cut_fill\nKa 1 1 1\nKd 1 1 1\n'
                                     'Ks 0 0 0\nNs 1\nillum 1\nd 1\n'
                                     'map_Kd ../textures/green_island_001_harbor_albedo.jpg\n')
    report = {}
    for kind, name in OUTPUT_MESHES.items():
        report[kind] = _grade_mesh(chart/'meshes'/SOURCE_MESHES[kind], meshes/name, pose[:3], first, read_obj,
                                   matte=kind == 'visual')
        link.find(f"{kind}[@name='{kind}']/geometry/mesh/uri").text = 'meshes/'+name
    (output/CHART_FILE).write_bytes(ET.tostring(document.getroot(), encoding='utf-8',
                                               xml_declaration=True)+b'\n')
    result = {'bridge_join_enu_m': [*first[:2], first[-1]],
              'land_road_end_enu_m': [*landing.section(landing.END_Y, first)[:2],
                                     landing.section(landing.END_Y, first)[-1]],
              'terrain': report}
    (output/'bridge_landing.json').write_text(json.dumps(result, indent=2)+'\n')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--chart', type=Path, default=CHART)
    parser.add_argument('--layout', type=Path, default=BRIDGE/'layout.json')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(prepare(args.chart, args.layout, args.output), indent=2))
