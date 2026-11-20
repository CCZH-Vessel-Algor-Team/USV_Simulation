#!/usr/bin/env python3
"""Prepare a LOCAL chart variant for the supplied Gaogongdao harbor model.

Never rewrite the supplied harbor, original chart, or green-island assets.
The old quay comprises whole disconnected components. The connected variant
also grades a small landing strip and bakes matching harbor/land materials.
Refuse an accidental partial cut of the old quay. Offline generation needs
NumPy, OpenCV and Shapely >= 2.1; Gazebo loads only the baked assets.
"""
import argparse
from collections import Counter
import hashlib
import importlib.util
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import shapely
from shapely.affinity import translate


PACKAGE = Path(__file__).resolve().parents[1]
CHART = PACKAGE / 'worlds/models/ccs_chart_obstacles_enc_5km'
HARBOR = (PACKAGE / 'worlds/models/gaogongdao_harbor/models'
          / 'gaogongdao_local_harbor_v1')
# ENU crop documented in the supplied construction-manifest.json.
QUAY_BOUNDS = (-262.0, 145.0, -80.0, 435.0)
HARBOR_POSE = (-170.0, 270.0, 0.0)
# Slightly below the supplied 3.2 m deck: no coplanar collision/visual surfaces.
FOUNDATION_TOP = 3.19
FOUNDATION_BOTTOM = -5.0
VARIANT = 'model_gaogongdao.sdf'
HARBOR_VARIANT = 'gaogongdao_harbor.sdf'
HARBOR_URI = 'model://gaogongdao_harbor/models/gaogongdao_local_harbor_v1/'
STEMS = ('green_island_001', 'green_island_001_collision',
         'green_island_001_vegetation')


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_obj(path):
    """Preserve attribute text and face/material order when compacting an OBJ."""
    attributes = {name: [] for name in ('v', 'vt', 'vn')}
    records, faces = [], []
    for line in path.read_text().splitlines():
        parts = line.split()
        if not parts or parts[0].startswith('#'):
            continue
        if parts[0] in attributes:
            attributes[parts[0]].append(' '.join(parts[1:]))
        elif parts[0] == 'f':
            refs = [tuple(int(value) if value else 0 for value in token.split('/'))
                    for token in parts[1:]]
            if any(ref[0] <= 0 or any(index < 0 for index in ref) for ref in refs):
                raise ValueError(f'{path}: expected positive OBJ vertex references')
            records.append(('f', len(faces)))
            faces.append(refs)
        else:
            records.append(('directive', line))
    vertices = np.array([list(map(float, line.split()[:3]))
                         for line in attributes['v']])
    return attributes, records, faces, vertices


def quay_faces(vertices, faces, offset):
    """Select whole components; reject any face which straddles the crop."""
    points = vertices[:, :2] + np.asarray(offset)[:2]
    lower, upper = np.array(QUAY_BOUNDS[:2]), np.array(QUAY_BOUNDS[2:])
    inside = np.all((points >= lower) & (points <= upper), axis=1)
    crop = shapely.box(*QUAY_BOUNDS)
    selected = []
    for face in faces:
        indices = [ref[0] - 1 for ref in face]
        contained = inside[indices]
        xy = points[indices]
        if not contained.all() and np.all(xy.max(axis=0) >= lower) and np.all(xy.min(axis=0) <= upper):
            hull = shapely.MultiPoint(xy).convex_hull
            if hull.intersects(crop):
                raise ValueError('Quay crop crosses a face; refusing to cut the main island')
        selected.append(bool(contained.all()))
    return np.array(selected)


def write_subset(output, attributes, records, faces, keep):
    used = [set(), set(), set()]
    for face, include in zip(faces, keep):
        if include:
            for ref in face:
                for column, index in enumerate(ref):
                    if index:
                        used[column].add(index)
    maps = [{old: new for new, old in enumerate(sorted(indices), start=1)} for indices in used]
    with output.open('w') as stream:
        stream.write('# CCS harbor variant; original attribute values retained.\n')
        for column, name in enumerate(('v', 'vt', 'vn')):
            for old in sorted(used[column]):
                stream.write(f'{name} {attributes[name][old - 1]}\n')
        for kind, value in records:
            if kind == 'directive':
                stream.write(value + '\n')
            elif keep[value]:
                refs = ['/'.join(str(maps[column][index]) if index else ''
                                 for column, index in enumerate(ref)) for ref in faces[value]]
                stream.write('f ' + ' '.join(refs) + '\n')


def roof_footprint(vertices, faces):
    top = vertices[:, 2].max()
    roofs = [vertices[[ref[0] - 1 for ref in face], :2] for face in faces
             if np.allclose(vertices[[ref[0] - 1 for ref in face], 2], top)]
    return shapely.union_all([shapely.Polygon(roof) for roof in roofs])


def write_collision_obj(output, vertices, faces):
    """DART requires a normal for every loaded collision-mesh vertex."""
    vertices = np.asarray(vertices)
    normals = []
    for a, b, c in faces:
        normal = np.cross(vertices[b] - vertices[a], vertices[c] - vertices[a])
        length = np.linalg.norm(normal)
        if length < 1e-12:
            raise ValueError('Degenerate collision triangle')
        normals.append(normal / length)
    with output.open('w') as stream:
        stream.write('# CCS harbor collision mesh; metres; explicit flat normals for DART.\n'
                     'mtllib gaogongdao_foundation.mtl\no harbor\n')
        for vertex in vertices:
            stream.write('v {:.6f} {:.6f} {:.6f}\n'.format(*vertex))
        for normal in normals:
            stream.write('vn {:.8f} {:.8f} {:.8f}\n'.format(*normal))
        stream.write('usemtl foundation\ns off\n')
        for normal_index, face in enumerate(faces, start=1):
            stream.write('f ' + ' '.join(f'{index + 1}//{normal_index}' for index in face) + '\n')


def write_foundation(output, footprint):
    """A closed solid quay base, not a bounding box across navigable water."""
    vertices, faces = [], []
    parts = [footprint] if footprint.geom_type == 'Polygon' else list(footprint.geoms)
    for polygon in parts:
        # Keep point-touching polygons topologically separate, as in the terrain.
        lookup, xy, roofs = {}, [], []
        for triangle in shapely.constrained_delaunay_triangles(polygon).geoms:
            face = []
            for point in list(triangle.exterior.coords)[:3]:
                key = tuple(point)
                if key not in lookup:
                    lookup[key] = len(xy)
                    xy.append(key)
                face.append(lookup[key])
            a, b, c = np.asarray(xy)[face]
            if np.cross(b - a, c - a) < 0:
                face.reverse()
            roofs.append(face)
        edges = Counter(tuple(sorted(edge)) for a, b, c in roofs
                        for edge in ((a, b), (b, c), (c, a)))
        size, start = len(xy), len(vertices)
        vertices.extend((x, y, FOUNDATION_TOP) for x, y in xy)
        vertices.extend((x, y, FOUNDATION_BOTTOM) for x, y in xy)
        for a, b, c in roofs:
            faces.extend(((start + a, start + b, start + c),
                          (start + c + size, start + b + size, start + a + size)))
            for u, v in ((a, b), (b, c), (c, a)):
                if edges[tuple(sorted((u, v)))] == 1:
                    faces.extend(((start + u, start + u + size, start + v + size),
                                  (start + u, start + v + size, start + v)))
    write_collision_obj(output, vertices, faces)


def prepare(chart_dir=CHART, harbor_dir=HARBOR, output_dir=None):
    chart_dir, harbor_dir = Path(chart_dir), Path(harbor_dir)
    output_dir = Path(output_dir) if output_dir else chart_dir
    root = ET.parse(chart_dir / 'model.sdf').getroot()
    link = root.find("model/link[@name='chart_obstacle_001']")
    pose = np.fromstring(link.findtext('pose'), sep=' ')
    if len(pose) != 6 or np.any(pose[2:] != 0):
        raise ValueError('Expected an unrotated, zero-height chart link')
    manifest_path = harbor_dir.parent.parent / 'construction-manifest.json'
    manifest = json.loads(manifest_path.read_text())
    if tuple(manifest['origin_enu']) != HARBOR_POSE:
        raise ValueError('Supplied harbor origin changed; recheck world placement')

    source_path = chart_dir / 'meshes/chart_obstacle_001.obj'
    _, _, chart_faces, chart_vertices = read_obj(source_path)
    chart = translate(roof_footprint(chart_vertices, chart_faces), *pose[:2])
    deck_path = harbor_dir / 'meshes/deck_collision.obj'
    _, _, deck_faces, deck_vertices = read_obj(deck_path)
    deck = translate(roof_footprint(deck_vertices, deck_faces), *HARBOR_POSE[:2])
    if deck.symmetric_difference(chart.intersection(shapely.box(*QUAY_BOUNDS))).area > 0.001:
        raise ValueError('Harbor deck no longer matches the original ENC quay footprint')
    if not np.allclose(deck_vertices[:, 2], 3.2):
        raise ValueError('Deck height changed; recheck foundation clearance')

    inputs = {'model.sdf': sha256(chart_dir / 'model.sdf'),
              'meshes/chart_obstacle_001.obj': sha256(source_path)}
    statistics = {}
    (output_dir / 'meshes').mkdir(parents=True, exist_ok=True)
    for stem in STEMS:
        source = chart_dir / 'meshes' / (stem + '.obj')
        attributes, records, faces, vertices = read_obj(source)
        removed = quay_faces(vertices, faces, pose[:3])
        if not removed.any() or removed.all():
            raise ValueError(f'{stem}: expected both quay and retained island geometry')
        target = output_dir / 'meshes' / (stem + '_without_harbor.obj')
        write_subset(target, attributes, records, faces, ~removed)
        inputs['meshes/' + source.name] = sha256(source)
        statistics[stem] = {'removed_faces': int(removed.sum()),
                            'retained_faces': int((~removed).sum())}

    foundation = output_dir / 'meshes/gaogongdao_foundation.obj'
    write_foundation(foundation, translate(deck, -pose[0], -pose[1]))
    (output_dir / 'meshes/gaogongdao_foundation.mtl').write_text(
        'newmtl foundation\nKa 0.56 0.57 0.55\nKd 0.56 0.57 0.55\nKs 0.02 0.02 0.02\nNs 4\nd 1\n')

    # The supplied deck OBJ has no normals: Gazebo's DART collision loader
    # rejects it (and ODE can crash on the resulting empty submesh). Use a
    # normal-complete copy, never overwrite the user's original model/mesh.
    deck_triangles = []
    for face in deck_faces:
        indices = [ref[0] - 1 for ref in face]
        if len(indices) != 3:
            raise ValueError('Expected a triangulated harbor deck')
        a, b, c = deck_vertices[indices]
        if np.cross(b - a, c - a)[2] < 0:
            indices.reverse()
        deck_triangles.append(indices)
    write_collision_obj(output_dir / 'meshes/gaogongdao_deck_collision.obj',
                        deck_vertices, deck_triangles)
    harbor_root = ET.parse(harbor_dir / 'model.sdf').getroot()
    for tag in ('uri', 'albedo_map', 'metalness_map', 'roughness_map'):
        for element in harbor_root.findall('.//' + tag):
            if '://' in element.text or element.text.startswith('/'):
                raise ValueError('Expected local, relocatable supplied harbor resources')
            element.text = HARBOR_URI + element.text
    harbor_root.find(".//collision[@name='chart_matched_deck']/geometry/mesh/uri").text = (
        'meshes/gaogongdao_deck_collision.obj')
    for element in link.findall('.//mesh/uri'):
        uri = element.text
        if uri.startswith('meshes/green_island_001'):
            element.text = uri.replace('.obj', '_without_harbor.obj')
    for kind in ('visual', 'collision'):
        element = ET.SubElement(link, kind, name='harbor_foundation_' + kind)
        mesh = ET.SubElement(ET.SubElement(element, 'geometry'), 'mesh')
        ET.SubElement(mesh, 'uri').text = 'meshes/gaogongdao_foundation.obj'
        if kind == 'visual':
            material = ET.SubElement(element, 'material')
            for tag in ('ambient', 'diffuse'):
                ET.SubElement(material, tag).text = '0.56 0.57 0.55 1'
    spec = importlib.util.spec_from_file_location('gaogongdao_connected_assets',
                                                Path(__file__).with_name('gaogongdao_connected_assets.py'))
    connected = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(connected)
    connection = connected.build(chart_dir, harbor_dir, output_dir, root, harbor_root,
                                 pose[:3], read_obj, write_subset)
    # Keep the legacy land report byte-identical (the navigation map records its
    # hash). Berth metadata belongs to the contact report instead.
    middle_berth = connection.pop('middle_berth')
    spec = importlib.util.spec_from_file_location('gaogongdao_collision_assets',
                                                Path(__file__).with_name('gaogongdao_collision_assets.py'))
    contacts = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(contacts)
    contacts.build(output_dir, harbor_root, read_obj, write_collision_obj,
                   connection['source_stones'], middle_berth)
    ET.indent(harbor_root, space='  ')
    ET.ElementTree(harbor_root).write(output_dir / HARBOR_VARIANT,
                                     encoding='utf-8', xml_declaration=True)
    ET.indent(root, space='  ')
    ET.ElementTree(root).write(output_dir / VARIANT, encoding='utf-8', xml_declaration=True)
    bridge_layout = chart_dir.parent / 'ccs_tianwan_bridge/layout.json'
    if bridge_layout.is_file():
        # Grade the north bridge approach over the finished harbor variant,
        # using the already-built source meshes, never the raw harbor data.
        spec = importlib.util.spec_from_file_location('prepare_ccs_bridge_landing',
                                                     Path(__file__).with_name('prepare_ccs_bridge_landing.py'))
        bridge_landing = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(bridge_landing)
        bridge_landing.prepare(chart=output_dir, layout_path=bridge_layout,
                               output=output_dir, read_obj=read_obj)

    # This is the historical land-generation report whose digest is embedded
    # in the existing navigation map. Keep it unchanged for contact-only edits;
    # the current contact policy and geometry live in gaogongdao_contacts.json.
    report = {
        'integration_version': 2,
        'world_crop_enu_m': QUAY_BOUNDS, 'harbor_pose_enu_m': HARBOR_POSE,
        'quay_area_m2': deck.area,
        'foundation_z_m': [FOUNDATION_BOTTOM, FOUNDATION_TOP],
        'connection': connection,
        'source_chart_sha256': inputs,
        'source_harbor_sha256': {str(p.relative_to(harbor_dir)): sha256(p)
                                  for p in sorted(harbor_dir.rglob('*')) if p.is_file()},
        'meshes': statistics,
        'policy': 'Only whole old quay components removed. Original source files unchanged. '
                  'Landing grading is limited to the documented local vertices; the main island is retained. '
                  'A new filled causeway changes the local shoreline; '
                  'use the generated Gaogongdao navigation map, not the original ENC map. '
                  'Runtime harbor wrapper adds missing deck collision normals for DART. '
                   'Pontoons, decorative boats and armor stones remain visual-only.',
    }
    (output_dir / 'gaogongdao_variant.json').write_text(json.dumps(report, indent=2) + '\n')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--chart-dir', type=Path, default=CHART)
    parser.add_argument('--harbor-dir', type=Path, default=HARBOR)
    args = parser.parse_args()
    print(json.dumps(prepare(args.chart_dir, args.harbor_dir), indent=2))
