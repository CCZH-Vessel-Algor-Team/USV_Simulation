"""Bake low-cost harbor contact geometry without changing any visual mesh."""
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import shapely


REPORT = 'gaogongdao_contacts.json'
RIPRAP_MESH = 'gaogongdao_riprap_collision.obj'
RIPRAP_BOTTOM = -1.0
RIPRAP_PADDING = .4
CLOSING_RADIUS = 2.0
OUTLINE_TOLERANCE = .3


def components(path, read_obj):
    """Split disconnected solids by shared positions, not UV/material seams."""
    _, _, faces, vertices = read_obj(path)
    parent = list(range(len(faces)))
    owners = {}

    def root(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for i, face in enumerate(faces):
        for ref in face:
            point = tuple(vertices[ref[0]-1])
            if point in owners:
                parent[root(i)] = root(owners[point])
            else:
                owners[point] = i
    groups = defaultdict(set)
    for i, face in enumerate(faces):
        groups[root(i)].update(ref[0]-1 for ref in face)
    # Preserve source component order so original and added causeway stones can
    # be treated separately, without joining across the harbor's entrance.
    return [vertices[sorted(ids)] for ids in sorted(groups.values(), key=min)]


def footprint(points):
    return shapely.MultiPoint(points[:, :2]).convex_hull


def contact_box(points, name, family):
    rectangle = footprint(points).minimum_rotated_rectangle
    corners = np.asarray(rectangle.exterior.coords)[:4]
    vectors = np.roll(corners, -1, axis=0)-corners
    lengths = np.linalg.norm(vectors, axis=1)
    direction = vectors[int(np.argmax(lengths))]
    yaw = (math.atan2(direction[1], direction[0])+math.pi/2) % math.pi-math.pi/2
    center = corners.mean(axis=0)
    low, high = points[:, 2].min(), points[:, 2].max()
    return {'name': name, 'family': family,
            'pose_m_rad': [*map(float, center), float((low+high)/2), 0.0, 0.0, yaw],
            'size_m': [float(lengths.max()), float(lengths.min()), float(high-low)],
            'footprint_local_xy': shapely.geometry.mapping(rectangle)}


def _riprap_parts(stones):
    """Close small between-stone slots, never take a convex hull of the harbor."""
    individual = [footprint(points) for points in stones]
    original = shapely.union_all(individual)
    band = original.buffer(CLOSING_RADIUS, quad_segs=3).buffer(
        -CLOSING_RADIUS+RIPRAP_PADDING, quad_segs=3).simplify(OUTLINE_TOLERANCE)
    if original.difference(band).area > .001:
        raise ValueError('Simplified riprap envelope leaves visible stones exposed')
    # Refuse a change that could seal a berth or connect opposite banks.
    if band.difference(original.buffer(1.5)).area > .1:
        raise ValueError('Riprap envelope reaches too far into open water')
    parts = sorted(shapely.get_parts(band), key=lambda p: (*p.bounds, p.area))
    result = []
    for part in parts:
        relevant = [points for points, poly in zip(stones, individual) if part.intersects(poly)]
        if not relevant:
            raise ValueError('Empty riprap component')
        top = math.ceil((max(p[:, 2].max() for p in relevant)+.03)*20)/20
        result.append((part, top, len(relevant)))
    return result


def _extrude(parts):
    vertices, faces = [], []
    for polygon, top, _ in parts:
        lookup, xy, roofs = {}, [], []
        for triangle in shapely.constrained_delaunay_triangles(polygon).geoms:
            face = []
            for point in list(triangle.exterior.coords)[:3]:
                if point not in lookup:
                    lookup[point] = len(xy)
                    xy.append(point)
                face.append(lookup[point])
            a, b, c = np.asarray(xy)[face]
            if np.cross(b-a, c-a) < 0:
                face.reverse()
            roofs.append(face)
        edges = Counter(tuple(sorted(edge)) for a, b, c in roofs
                        for edge in ((a, b), (b, c), (c, a)))
        size, start = len(xy), len(vertices)
        vertices.extend((x, y, top) for x, y in xy)
        vertices.extend((x, y, RIPRAP_BOTTOM) for x, y in xy)
        for a, b, c in roofs:
            faces.extend(((start+a, start+b, start+c),
                          (start+c+size, start+b+size, start+a+size)))
            for u, v in ((a, b), (b, c), (c, a)):
                if edges[tuple(sorted((u, v)))] == 1:
                    faces.extend(((start+u, start+u+size, start+v+size),
                                  (start+u, start+v+size, start+v)))
    return np.asarray(vertices), faces


def build(output_dir, harbor_root, read_obj, write_collision_obj, source_stones, middle_berth=None):
    output_dir = Path(output_dir)
    folder = output_dir/'meshes'
    link = harbor_root.find('model/link')
    inputs = {}

    def load(material):
        path = folder/f'gaogongdao_styled_{material}.obj'
        inputs[path.name] = hashlib.sha256(path.read_bytes()).hexdigest()
        return components(path, read_obj)

    stone = load('stone')
    parts = _riprap_parts(stone[:source_stones])+_riprap_parts(stone[source_stones:])
    vertices, faces = _extrude(parts)
    write_collision_obj(folder/RIPRAP_MESH, vertices, faces)
    collision = ET.SubElement(link, 'collision', name='riprap_band_contact')
    mesh = ET.SubElement(ET.SubElement(collision, 'geometry'), 'mesh')
    ET.SubElement(mesh, 'uri').text = 'meshes/'+RIPRAP_MESH

    boxes = []
    # These colors designate actual hulls in the supplied atlas, not pontoons.
    for color in ('blue', 'red'):
        for i, points in enumerate(load(color)):
            if points[:, 2].max() > 2.0 or points[:, 2].min() >= 0:
                raise ValueError('Unexpected boat mesh: review its contact fit')
            boxes.append(contact_box(points, f'moored_boat_{color}_{i:02d}', 'moored_boat'))
    for material, family in (('roof', 'pontoon_main'), ('concrete', 'pontoon_finger')):
        low = [points for points in load(material) if points[:, 2].max() < 2.0]
        for i, points in enumerate(low):
            spec = contact_box(points, f'{family}_{i:02d}', family)
            if spec['size_m'][1] > 2.1 or spec['size_m'][2] > .65:
                raise ValueError('Unexpected floating walkway: review its contact fit')
            boxes.append(spec)
    counts = Counter(spec['family'] for spec in boxes)
    if counts != {'moored_boat': 14, 'pontoon_main': 3, 'pontoon_finger': 9}:
        raise ValueError(f'Unexpected harbor components: {counts}')
    for spec in boxes:
        collision = ET.SubElement(link, 'collision', name=spec['name'])
        ET.SubElement(collision, 'pose').text = ' '.join(f'{v:.9f}' for v in spec['pose_m_rad'])
        box = ET.SubElement(ET.SubElement(collision, 'geometry'), 'box')
        ET.SubElement(box, 'size').text = ' '.join(f'{v:.9f}' for v in spec['size_m'])
    names = [element.get('name') for element in link.findall('collision')]
    if len(names) != len(set(names)):
        raise ValueError('Duplicate harbor contact geometry')

    report = {
        'harbor_pose_enu_m_rad': [-170, 270, 0, 0, 0, 0],
        'counts': {'stones_in_band': len(stone), 'riprap_closed_shells': len(parts),
                   **counts, 'added_collision_objects': 1+len(boxes),
                   'total_harbor_collision_objects': len(names)},
        'riprap': {'mesh': 'meshes/'+RIPRAP_MESH, 'triangles': len(faces),
                   'sha256': hashlib.sha256((folder/RIPRAP_MESH).read_bytes()).hexdigest(),
                   'bottom_m': RIPRAP_BOTTOM, 'outward_padding_m': RIPRAP_PADDING,
                   'outlines_local_xy': [{'footprint': shapely.geometry.mapping(poly),
                                          'top_m': top, 'stone_count': count}
                                         for poly, top, count in parts]},
        'boxes': boxes, 'visual_meshes_sha256': inputs,
        'middle_berth': middle_berth,
        'policy': 'Contact-only static geometry. No visuals, source harbor assets or navigation maps '
                  'modified. Riprap is a conservative solid envelope, not individual rock contact. '
                  'Pontoons and moored decorative boats remain fixed, without buoyancy or motion.',
    }
    (output_dir/REPORT).write_text(json.dumps(report, indent=2)+'\n')
    return report['counts']
