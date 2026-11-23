"""Offline connected-harbor geometry/material baking; no Gazebo runtime code."""
from collections import defaultdict
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import shapely
from shapely.affinity import translate


ORIGIN = np.array([-170., 270., 0.])
# Start inside the old quay, bridging its point-touching narrow northern neck.
# About 40 m is overlap with the old quay and 40 m is on the landing, not all sea fill.
CONTROL_POINTS = np.array([[-99., 390.], [-99., 458.], [-61., 528.], [-43., 600.]])
LANDING_CENTER = np.array([-43., 584.])
LANDING_RADIUS = 36.
ROAD_Z = 3.2
COLLISION_BOTTOM = -5.
MIDDLE_BERTH_CLEAR_WIDTH = 4.06
MIDDLE_BERTH_DEPTH = 6.0
MATERIALS = {
    'concrete': ((151, 149, 136), 8., .9, 1024),
    'stone': ((121, 119, 104), 3.5, .95, 1024),
    'roof': ((62, 99, 115), 4., .8, 512),
    'blue': ((51, 92, 105), 3., .82, 512),
    'red': ((135, 75, 59), 3., .87, 512),
    'green': ((82, 106, 91), 3., .88, 512),
    'dark': ((60, 67, 65), 3., .85, 512),
    'sand': ((147, 139, 117), 4., .95, 512),
}
CHART_URI = 'model://ccs_chart_obstacles_enc_5km/'


def smoothstep(a, b, value):
    t = np.clip((value - a) / (b - a), 0, 1)
    return t * t * (3 - 2 * t)


def road_height(y):
    """Flat quay, gentle 8 m transition, then a 3% landward service-road grade."""
    y = np.asarray(y)
    return np.where(y <= 550., ROAD_Z,
                    np.where(y < 558., ROAD_Z+.001875*(y-550.)**2,
                             ROAD_Z+.12+.03*(y-558.)))


def centerline():
    t = np.linspace(0, 1, 81)[:, None]
    p = CONTROL_POINTS
    xy = (1-t)**3*p[0] + 3*(1-t)**2*t*p[1] + 3*(1-t)*t*t*p[2] + t**3*p[3]
    tangent = np.gradient(xy, axis=0)
    tangent /= np.linalg.norm(tangent, axis=1)[:, None]
    normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
    # A smooth flare from the existing narrow quay to a 9 m service road.
    width = 3. + 6. * smoothstep(0., .16, t[:, 0])
    return xy, normal, width


def strip_polygon(xy, normal, half_width):
    return shapely.Polygon(np.vstack((xy + normal * half_width[:, None],
                                      (xy - normal * half_width[:, None])[::-1])))


def triangles_of(polygon):
    parts = [polygon] if polygon.geom_type == 'Polygon' else list(polygon.geoms)
    for part in parts:
        if part.geom_type != 'Polygon' or part.area < 1e-8:
            continue
        for triangle in shapely.constrained_delaunay_triangles(part).geoms:
            xy = np.asarray(triangle.exterior.coords)[:3]
            if np.cross(xy[1] - xy[0], xy[2] - xy[0]) < 0:
                xy = xy[::-1]
            yield xy


def face_uv(points, scale):
    n = np.cross(points[1] - points[0], points[2] - points[0])
    axis = np.abs(n).argmax()
    columns = [(1, 2), (0, 2), (0, 1)][axis]
    return points[:, columns] / scale


def write_mesh(path, triangles, material='stone', custom_uv=None):
    """Shared position indices, explicit per-face normals, and metre-scaled UVs."""
    path = Path(path)
    triangles = np.asarray(triangles, dtype=float)
    cross = np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0])
    length = np.linalg.norm(cross, axis=1)
    if np.any(length < 1e-10):
        raise ValueError(f'Degenerate triangle in {path.name}')
    normals = cross / length[:, None]
    vertices, indices, lookup = [], [], {}
    for triangle in triangles:
        face = []
        for vertex in triangle:
            key = tuple(np.round(vertex, 6))
            if key not in lookup:
                lookup[key] = len(vertices)+1
                vertices.append(key)
            face.append(lookup[key])
        indices.append(face)
    if custom_uv is None:
        custom_uv = [face_uv(t + ORIGIN, MATERIALS[material][1]) for t in triangles]
    with path.open('w') as f:
        f.write('# Connected CCS harbor; metres; normal-complete, tiled UVs.\n'
                f'mtllib {path.stem}.mtl\no harbor\n')
        for v in vertices:
            f.write('v {:.6f} {:.6f} {:.6f}\n'.format(*v))
        for uv in custom_uv:
            for value in uv:
                f.write('vt {:.8f} {:.8f}\n'.format(*value))
        for n in normals:
            f.write('vn {:.8f} {:.8f} {:.8f}\n'.format(*n))
        f.write('usemtl surface\ns off\n')
        for i, face in enumerate(indices):
            f.write('f '+' '.join(f'{v}/{i*3+j+1}/{i+1}' for j, v in enumerate(face))+'\n')
    path.with_suffix('.mtl').write_text(
        'newmtl surface\nKa 1 1 1\nKd 1 1 1\nKs 0.02 0.02 0.02\nNs 4\nd 1\n'
        f'map_Kd ../textures/harbor_{material}_albedo.png\n')


def add_material(element, name, albedo=None):
    old = element.find('material')
    if old is not None:
        element.remove(old)
    material = ET.SubElement(element, 'material')
    ET.SubElement(material, 'ambient').text = '1 1 1 1'
    ET.SubElement(material, 'diffuse').text = '1 1 1 1'
    metal = ET.SubElement(ET.SubElement(material, 'pbr'), 'metal')
    ET.SubElement(metal, 'metalness').text = '0'
    ET.SubElement(metal, 'roughness').text = str(MATERIALS[name][2])
    ET.SubElement(metal, 'albedo_map').text = CHART_URI + (
        albedo or f'textures/harbor_{name}_albedo.png')
    ET.SubElement(metal, 'normal_map').text = CHART_URI+'textures/harbor_'+name+'_normal.png'
    ET.SubElement(metal, 'roughness_map').text = CHART_URI+'textures/harbor_'+name+'_roughness.png'


def add_geometry(link, kind, name, filename, material=None, albedo=None):
    element = ET.SubElement(link, kind, name=name)
    mesh = ET.SubElement(ET.SubElement(element, 'geometry'), 'mesh')
    ET.SubElement(mesh, 'uri').text = 'meshes/'+filename
    if material:
        add_material(element, material, albedo)
        if material == 'stone':
            ET.SubElement(element, 'cast_shadows').text = 'false'
    return element


def texture_noise(size, seed):
    rng = np.random.default_rng(seed)
    noise = np.zeros((size, size), dtype=np.float32)
    for grid, weight in ((16, .5), (64, .28), (256, .14), (size, .08)):
        tile = rng.random((grid, grid)).astype(np.float32)
        # Central period of a repeated tile keeps the filtered border periodic.
        expanded = cv2.resize(np.tile(tile, (3, 3)), (size*3, size*3), interpolation=cv2.INTER_LINEAR)
        noise += weight * expanded[size:size*2, size:size*2]
    return noise


def bake_materials(chart_dir, output_dir):
    folder = output_dir/'textures'
    folder.mkdir(exist_ok=True)
    rock = cv2.cvtColor(cv2.imread(str(chart_dir/'textures/rock_face_03_diff_1k.jpg')), cv2.COLOR_BGR2RGB)
    for index, (name, (color, _, roughness, size)) in enumerate(MATERIALS.items()):
        noise = texture_noise(size, 702+index)
        rgb = np.array(color)[None, None, :] * (.85+.30*noise[..., None])
        height = noise.copy()
        if name == 'stone':
            photo = cv2.resize(rock, (size, size)).astype(float)
            gray = photo.mean(axis=2, keepdims=True)
            photo = photo*.38+gray*.62
            photo *= np.array([1.03, 1.00, .92])
            photo *= 116/max(photo.mean(), 1)
            rgb = photo*.83+rgb*.17
            height = cv2.cvtColor(np.uint8(np.clip(photo, 0, 255)), cv2.COLOR_RGB2GRAY)/255
        elif name == 'concrete':
            x, y = np.meshgrid(np.arange(size)/size*8, np.arange(size)/size*8)
            seam = (np.minimum(x % 4, 4-x % 4) < .018) | (np.minimum(y % 4, 4-y % 4) < .018)
            stains = smoothstep(.57, .72, noise)
            rgb *= (1 - .10*stains - .16*seam)[..., None]
            height -= seam*.12
        elif name == 'roof':
            rib = .5+.5*np.cos(np.arange(size)/size*math.pi*64)
            rgb *= (.91+.09*rib)[None, :, None]
            height += rib[None, :]*.18
        dx = (np.roll(height, -1, 1)-np.roll(height, 1, 1))*.9
        dy = (np.roll(height, -1, 0)-np.roll(height, 1, 0))*.9
        normal = np.dstack((-dx, dy, np.ones_like(height)))
        normal /= np.linalg.norm(normal, axis=2)[..., None]
        for suffix, image in (
                ('albedo', np.uint8(np.clip(rgb, 0, 255))),
                ('normal', np.uint8(np.clip((normal*.5+.5)*255, 0, 255))),
                ('roughness', np.uint8(np.clip((roughness+(noise-.5)*.10)*255, 0, 255)))):
            if image.ndim == 3:
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            cv2.imwrite(str(folder/f'harbor_{name}_{suffix}.png'), image)
    return rock


def load_harbor_triangles(path):
    ns = {'d': 'http://www.collada.org/2005/11/COLLADASchema'}
    mesh = ET.parse(path).getroot().find('.//d:mesh', ns)
    def source(name, stride):
        return np.fromstring(mesh.find(f"d:source[@id='{name}']/d:float_array", ns).text,
                             sep=' ').reshape(-1, stride)
    positions, uv = source('positions', 3), source('uv', 2)
    indices = np.fromstring(mesh.findtext('d:triangles/d:p', namespaces=ns), sep=' ', dtype=int)
    indices = indices.reshape(-1, 3, 3)
    groups = defaultdict(list)
    # Read source atlas slots directly, not a second flipped texture lookup.
    # In the supplied COLLADA the V orientation made blue roofs appear white in Ogre.
    names = {(0, 1): 'green', (0, 2): 'red', (0, 3): 'stone',
             (1, 1): 'roof', (1, 2): 'concrete', (1, 3): 'dark',
             (2, 1): 'sand', (2, 3): 'dark', (3, 1): 'concrete',
             (3, 2): 'concrete', (3, 3): 'blue'}
    for face in indices:
        slot = tuple(np.floor(uv[face[0, 2]]*4).astype(int))
        groups[names[slot]].append(positions[face[:, 0]])
    return {name: np.array(faces) for name, faces in groups.items()}


def vary_stones(triangles):
    # 267 independent 12-triangle source blocks, still one batched visual.
    parent = list(range(len(triangles)))
    def root(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i
    owners = {}
    for i, triangle in enumerate(triangles):
        for v in triangle:
            key = tuple(v)
            if key in owners:
                parent[root(i)] = root(owners[key])
            else:
                owners[key] = i
    components = defaultdict(list)
    for i in range(len(triangles)):
        components[root(i)].append(i)
    rng = np.random.default_rng(9014)
    result = triangles.copy()
    for faces in components.values():
        points = triangles[faces].reshape(-1, 3)
        unique, inverse = np.unique(points, axis=0, return_inverse=True)
        center = (unique.min(0)+unique.max(0))/2
        offset = unique-center
        offset *= rng.uniform(.85, 1.13, 3)
        offset += rng.uniform(-.11, .11, offset.shape) * np.ptp(unique, axis=0)
        angle = rng.uniform(-.23, .23)
        rotation = np.array([[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]])
        offset[:, :2] = offset[:, :2] @ rotation.T
        result[faces] = (offset+center)[inverse].reshape(-1, 3, 3)
    return result, len(components)


def tube_geometry(xy, normal, width):
    rings = []
    for point, n, w in zip(xy, normal, width):
        half, toe = w/2+.75, w/2+4.
        level = float(road_height(point[1]))-.01
        profile = [(-half, level), (half, level), (toe, -.8), (toe, -5), (-toe, -5), (-toe, -.8)]
        rings.append([(* (point+q*n), z) for q, z in profile])
    rings = np.array(rings)
    faces = []
    for a, b in zip(rings, rings[1:]):
        for j in range(6):
            k = (j+1) % 6
            faces.extend(([a[j], b[j], b[k]], [a[j], b[k], a[k]]))
    for ring, reverse in ((rings[0], False), (rings[-1], True)):
        for j in range(1, 5):
            tri = np.array([ring[0], ring[j], ring[j+1]])
            faces.append(tri[::-1] if reverse else tri)
    faces = np.array(faces)
    volume = np.einsum('ij,ij->i', faces[:, 0], np.cross(faces[:, 1], faces[:, 2])).sum()/6
    if volume < 0:
        faces = faces[:, ::-1]
    return faces


def _triangle_components(triangles):
    """Connected triangle groups over shared positions (ignoring UV/material seams)."""
    parent = list(range(len(triangles)))
    owners = {}

    def root(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for i, triangle in enumerate(triangles):
        for vertex in triangle:
            key = tuple(np.round(vertex, 6))
            if key in owners:
                parent[root(i)] = root(owners[key])
            else:
                owners[key] = i
    groups = defaultdict(list)
    for i in range(len(triangles)):
        groups[root(i)].append(i)
    return list(groups.values())


def _rect_frame(points):
    rectangle = shapely.MultiPoint(points[:, :2]).minimum_rotated_rectangle
    corners = np.asarray(rectangle.exterior.coords)[:4]
    vectors = np.roll(corners, -1, axis=0)-corners
    lengths = np.linalg.norm(vectors, axis=1)
    long_side = int(np.argmax(lengths))
    return (corners.mean(axis=0), vectors[long_side]/lengths[long_side],
            float(lengths[long_side]), float(lengths.min()))


def resize_middle_berths(concrete, roof):
    """Set the middle group to the requested 4.06 m clear width and 6.0 m depth.

    The other groups are untouched. Scaling is about each finger's root, so all
    fingers remain attached to their walkway; gaps are closed numerically
    against the actual rectangles, not an axis-aligned bounding box.
    """
    concrete = np.asarray(concrete)
    roof = np.asarray(roof)
    components = _triangle_components(concrete)
    walkways = [c for c in _triangle_components(roof)
                if roof[c][:, :, 2].max() < 2 and np.ptp(roof[c][:, :, 0]) > 20]
    if len(walkways) != 3:
        raise ValueError(f'Expected three pontoon walkways, found {len(walkways)}')
    walkways.sort(key=lambda c: roof[c][:, :, 1].mean())
    main_points = roof[walkways[1]].reshape(-1, 3)
    center, e_u, _, main_width = _rect_frame(main_points)
    e_v = np.array([-e_u[1], e_u[0]])
    if np.dot(e_v, np.array([0., 1.])) < 0:
        e_v = -e_v
    half = main_width/2
    fingers = [c for c in components
               if concrete[c][:, :, 2].max() < 2
               and np.linalg.norm(concrete[c].reshape(-1, 3)[:, :2].mean(0)-center) < 15]
    if len(fingers) != 3:
        raise ValueError(f'Expected three middle berth fingers, found {len(fingers)}')
    fingers.sort(key=lambda c: np.dot(concrete[c].reshape(-1, 3)[:, :2].mean(0)-center, e_u))
    result = concrete.copy()

    def points_of(component):
        return result[component].reshape(-1, 3)

    def gap(a, b):
        return shapely.MultiPoint(points_of(a)[:, :2]).convex_hull.distance(
            shapely.MultiPoint(points_of(b)[:, :2]).convex_hull)

    for component in fingers:
        points = points_of(component)
        _, axis, length, _ = _rect_frame(points)
        centroid = points[:, :2].mean(0)
        ends = (centroid+axis*length/2, centroid-axis*length/2)
        root = max(ends, key=lambda p: np.dot(p-center, e_v))
        tip = min(ends, key=lambda p: np.dot(p-center, e_v))
        target_tip = -(MIDDLE_BERTH_DEPTH+half)
        root_v = np.dot(root-center, e_v)
        scale = ((target_tip-root_v) /
                 (np.dot(tip-center, e_v)-root_v))
        offset = points[:, :2]-root
        points[:, :2] += np.outer(offset@axis, axis)*(scale-1)
        for _ in range(3):
            v = (points[:, :2]-center)@e_v
            if abs(v.min()-target_tip) < 1e-4:
                break
            ratio = (target_tip-root_v)/(v.min()-root_v)
            offset = points[:, :2]-root
            points[:, :2] += np.outer(offset@axis, axis)*(ratio-1)
        result[component] = points.reshape(-1, 3, 3)

    for _ in range(6):
        gaps = [gap(fingers[0], fingers[1]), gap(fingers[1], fingers[2])]
        if all(abs(g-MIDDLE_BERTH_CLEAR_WIDTH) < 1e-4 for g in gaps):
            break
        for index, value in ((0, gaps[0]), (2, gaps[1])):
            inward = (1 if np.dot(points_of(fingers[index])[:, :2].mean(0)-center, e_u) < 0
                      else -1)
            cells = result[fingers[index]].copy()
            cells[:, :, :2] += e_u*inward*(value-MIDDLE_BERTH_CLEAR_WIDTH)
            result[fingers[index]] = cells
    final = [gap(fingers[0], fingers[1]), gap(fingers[1], fingers[2])]
    if any(abs(g-MIDDLE_BERTH_CLEAR_WIDTH) > 1e-3 for g in final):
        raise ValueError(f'Unable to set middle berth widths: {final}')
    depths = []
    for component in fingers:
        v = (points_of(component)[:, :2]-center)@e_v
        depths.append(float(-v.min()-half))
    if any(abs(d-MIDDLE_BERTH_DEPTH) > 1e-3 for d in depths):
        raise ValueError(f'Unable to set middle berth depth: {depths}')
    return result, {'clear_width_m': MIDDLE_BERTH_CLEAR_WIDTH,
                    'depth_m': MIDDLE_BERTH_DEPTH,
                    'measured_widths_m': [round(g, 4) for g in final],
                    'measured_depths_m': [round(d, 4) for d in depths],
                    'finger_length_m': round(_rect_frame(points_of(fingers[0]))[2], 3)}


def style_land(chart_dir, output_dir):
    source = chart_dir/'textures/green_island_001_albedo.jpg'
    rgb = cv2.cvtColor(cv2.imread(str(source)), cv2.COLOR_BGR2RGB).astype(np.float32)
    gray = (rgb*np.array([.2126, .7152, .0722])).sum(2, keepdims=True)
    rgb = (gray*.20+rgb*.80)*.95
    rgb *= np.array([1.015, .99, .98])
    styled = np.uint8(np.clip(rgb, 0, 255))
    cv2.imwrite(str(output_dir/'textures/green_island_001_harbor_albedo.jpg'),
                cv2.cvtColor(styled, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 92])
    for stem in ('green_island_001', 'green_island_001_vegetation'):
        obj = output_dir/'meshes'/f'{stem}_without_harbor.obj'
        mtl_name = stem+'_harbor.mtl'
        text = obj.read_text().replace('mtllib '+stem+'.mtl', 'mtllib '+mtl_name)
        obj.write_text(text)
        material = (chart_dir/'meshes'/f'{stem}.mtl').read_text()
        if stem == 'green_island_001':
            material = material.replace('green_island_001_albedo.jpg', 'green_island_001_harbor_albedo.jpg')
        else:
            material = material.replace('foliage.jpg', 'harbor_foliage.jpg')
        (output_dir/'meshes'/mtl_name).write_text(material)
    foliage = cv2.imread(str(chart_dir/'textures/foliage.jpg')).astype(float)
    gray = foliage.mean(axis=2, keepdims=True)
    foliage = foliage*.84+gray*.16
    cv2.imwrite(str(output_dir/'textures/harbor_foliage.jpg'), np.uint8(np.clip(foliage*.95, 0, 255)))
    return styled


def grade_landing(chart_dir, output_dir, chart_root, pose, apron, read_obj, write_subset):
    """Grade only nearby source vertices, keeping XY, bottom and distant geometry."""
    xy, _, _ = centerline()
    line = shapely.LineString(xy)
    counts, affected_surfaces, deltas = {}, [], {}
    for stem, kind in (('green_island_001', 'visual'), ('green_island_001_collision', 'collision')):
        source = output_dir/'meshes'/f'{stem}_without_harbor.obj'
        attributes, records, faces, vertices = read_obj(source)
        world = vertices+pose
        distance = shapely.distance(shapely.points(world[:, :2]), line)
        weight = (1-smoothstep(11., 26., distance))*smoothstep(540., 552., world[:, 1])
        weight *= 1-smoothstep(607., 635., world[:, 1])
        weight[vertices[:, 2] < 0] = 0
        group = ''
        # Make the source triangles below the landing planar too. This avoids
        # an old coarse collision roof sticking through the new service road.
        for record_kind, value in records:
            if record_kind == 'directive' and value.startswith('g '):
                group = value.split()[1]
            elif record_kind == 'f' and group == 'terrain_surface':
                indices = [ref[0]-1 for ref in faces[value]]
                if shapely.Polygon(world[indices, :2]).intersection(apron).area > 1e-7:
                    weight[indices] = 1
        grade = ROAD_Z+.03*(world[:, 1]-558.)
        z = vertices[:, 2]*(1-weight)+grade*weight
        changed = np.flatnonzero(np.abs(z-vertices[:, 2]) > 1e-7)
        vertices[changed, 2] = z[changed]
        affected = set(changed+1)
        deltas[stem] = [float((z-world[:, 2]).min()), float((z-world[:, 2]).max())]
        normals = np.zeros((len(attributes['vn']), 3))
        changed_normals = set()
        for face in faces:
            indices = [ref[0]-1 for ref in face]
            a, b, c = vertices[indices[:3]]
            normal = np.cross(b-a, c-a)
            for ref in face:
                normals[ref[2]-1] += normal
                if any(i in affected for i, *_ in face):
                    changed_normals.add(ref[2]-1)
            if any(i in affected for i, *_ in face) and abs(normal[2]) > 1e-8:
                affected_surfaces.append(shapely.Polygon(world[indices, :2]))
        lengths = np.linalg.norm(normals, axis=1)
        normals /= np.maximum(lengths[:, None], 1e-12)
        output, vi, ni = [], 0, 0
        for row in source.read_text().splitlines():
            if row.startswith('v '):
                if vi+1 in affected:
                    row = 'v {:.6f} {:.6f} {:.6f}'.format(*vertices[vi])
                vi += 1
            elif row.startswith('vn '):
                if ni in changed_normals:
                    row = 'vn {:.8f} {:.8f} {:.8f}'.format(*normals[ni])
                ni += 1
            output.append(row)
        target = output_dir/'meshes'/f'{stem}_connected.obj'
        target.write_text('\n'.join(output)+'\n')
        chart_root.find(f"model/link[@name='chart_obstacle_001']/{kind}[@name='{kind}']/geometry/mesh/uri").text = 'meshes/'+target.name
        if kind == 'visual':
            # Keep all original geometry, but stop only the local graded faces
            # casting grazing-angle self-shadow acne. The rest of the island
            # still casts its normal mountain shadows; global lighting is unchanged.
            attrs, recs, new_faces, _ = read_obj(target)
            local = np.array([any(ref[0] in affected for ref in face) for face in new_faces])
            main = target.with_name(stem+'_connected_main.obj')
            patch = target.with_name(stem+'_landing_terrain.obj')
            write_subset(main, attrs, recs, new_faces, ~local)
            write_subset(patch, attrs, recs, new_faces, local)
            chart_link = chart_root.find("model/link[@name='chart_obstacle_001']")
            chart_link.find("visual[@name='visual']/geometry/mesh/uri").text = 'meshes/'+main.name
            visual = add_geometry(chart_link, 'visual', 'landing_terrain', patch.name)
            ET.SubElement(visual, 'cast_shadows').text = 'false'
            # The original terrain albedo was baked for its old slopes. Give
            # newly exposed steep cut banks the same warm rock palette instead
            # of leaving a vertical wall painted bright green. Blend locally.
            image_path = output_dir/'textures/green_island_001_harbor_albedo.jpg'
            image = cv2.imread(str(image_path)).astype(float)
            mask = np.zeros(image.shape[:2], dtype=np.float32)
            lo, hi = world[:, :2].min(0), world[:, :2].max(0)
            group = ''
            for record_kind, value in records:
                if record_kind == 'directive' and value.startswith('g '):
                    group = value.split()[1]
                elif record_kind == 'f' and group == 'terrain_surface':
                    face = faces[value]
                    if not any(ref[0] in affected for ref in face):
                        continue
                    tri = vertices[[ref[0]-1 for ref in face]]
                    n = np.cross(tri[1]-tri[0], tri[2]-tri[0])
                    slope = np.linalg.norm(n[:2])/max(abs(n[2]), 1e-9)
                    strength = float(smoothstep(.35, 1., slope)*.75)
                    pixel = (tri[:, :2]+pose[:2]-lo)/(hi-lo)
                    pixel[:, 1] = 1-pixel[:, 1]
                    pixel *= [image.shape[1]-1, image.shape[0]-1]
                    cv2.fillPoly(mask, [np.rint(pixel).astype(np.int32)], strength)
            mask = cv2.GaussianBlur(mask, (0, 0), 1.3)
            ys, xs = np.nonzero(mask > .001)
            if len(xs):
                rock = cv2.imread(str(chart_dir/'textures/rock_face_03_diff_1k.jpg')).astype(float)
                rock = rock*.38+rock.mean(axis=2, keepdims=True)*.62
                rock *= np.array([.92, 1., 1.03])
                rock *= 116/max(rock.mean(), 1)
                terrain_xy = lo+np.column_stack((xs/(image.shape[1]-1), 1-ys/(image.shape[0]-1)))*(hi-lo)
                samples = rock[(terrain_xy[:, 1]/9 % 1*rock.shape[0]).astype(int),
                               (terrain_xy[:, 0]/9 % 1*rock.shape[1]).astype(int)]
                blend = mask[ys, xs, None]
                image[ys, xs] = image[ys, xs]*(1-blend)+samples*blend
            cv2.imwrite(str(image_path), np.uint8(np.clip(image, 0, 255)), [cv2.IMWRITE_JPEG_QUALITY, 92])
        counts[stem] = len(changed)
    # Remove complete tree/rock envelopes whose roots would float over the graded strip.
    details = json.loads((chart_dir/'meshes/green_island_001_details.json').read_text())
    source = output_dir/'meshes/green_island_001_vegetation_without_harbor.obj'
    _, _, faces, vertices = read_obj(source)
    world = vertices+pose
    selected = np.zeros(len(vertices), dtype=bool)
    roots = 0
    clearance = shapely.union_all(affected_surfaces).buffer(3.)
    for item in details['trees']+details['rocks']:
        position = np.array(item['position'])+pose
        radius = item['radius']*1.8
        if not shapely.Point(position[:2]).buffer(radius).intersects(clearance):
            continue
        lower = position-np.array([radius, radius, 2*radius+1])
        upper = position+np.array([radius, radius, item.get('height', 2*radius)+radius])
        selected |= np.all((world >= lower) & (world <= upper), axis=1)
        roots += 1
    keep = np.array([not selected[[ref[0]-1 for ref in face]].all() for face in faces])
    output, face_index = [], 0
    for row in source.read_text().splitlines():
        if row.startswith('f '):
            if keep[face_index]:
                output.append(row)
            face_index += 1
        else:
            output.append(row)
    target = output_dir/'meshes/green_island_001_vegetation_connected.obj'
    target.write_text('\n'.join(output)+'\n')
    chart_root.find("model/link[@name='chart_obstacle_001']/visual[@name='vegetation']/geometry/mesh/uri").text = 'meshes/'+target.name
    counts['removed_decoration_roots'] = roots
    counts['removed_decoration_faces'] = int((~keep).sum())
    counts['vertical_change_m'] = deltas
    return counts


def build(chart_dir, harbor_dir, output_dir, chart_root, harbor_root, pose, read_obj, write_subset):
    """Mutate only generated SDF roots; all source files are read-only inputs."""
    bake_materials(chart_dir, output_dir)
    land_image = style_land(chart_dir, output_dir)
    xy, normal, width = centerline()
    line = shapely.LineString(xy)
    road = strip_polygon(xy, normal, width/2)
    toe = strip_polygon(xy, normal, width/2+4)
    _, _, source_faces, source_vertices = read_obj(chart_dir/'meshes/chart_obstacle_001.obj')
    roof = [source_vertices[[r[0]-1 for r in f], :2] for f in source_faces
            if all(source_vertices[r[0]-1, 2] == 3 for r in f)]
    chart = translate(shapely.union_all([shapely.Polygon(p) for p in roof]), *pose[:2])
    mainland = max(chart.geoms, key=lambda p: p.area)
    apron = mainland.intersection(shapely.Point(LANDING_CENTER).buffer(LANDING_RADIUS, quad_segs=32))
    apron = apron.intersection(road.buffer(1.5, quad_segs=8))
    # The ramp is a plane, so an exact constrained polygon triangulation is
    # enough. Avoid sampling near-collinear buffer edges into microscopic,
    # overlapping sliver triangles that can invert a closed collision shell.
    apron = shapely.set_precision(apron.simplify(.02, preserve_topology=True), .0001)
    apron = apron.intersection(mainland)
    grading = grade_landing(chart_dir, output_dir, chart_root, pose, apron, read_obj, write_subset)
    land_image = cv2.cvtColor(cv2.imread(str(output_dir/'textures/green_island_001_harbor_albedo.jpg')),
                             cv2.COLOR_BGR2RGB)
    points, faces, lookup = [], [], {}
    for triangle in triangles_of(apron):
        face = []
        for point in triangle:
            key = tuple(point)
            if key not in lookup:
                lookup[key] = len(points)
                points.append(point)
            face.append(lookup[key])
        faces.append(face)
    points, faces = np.array(points), np.array(faces)
    edge_count = defaultdict(int)
    for a, b, c in faces:
        for edge in ((a,b), (b,c), (c,a)):
            edge_count[tuple(sorted(edge))] += 1
    edges = [(a,b) for a,b,c in faces for a,b in ((a,b),(b,c),(c,a))
             if edge_count[tuple(sorted((a,b)))] == 1]
    height = road_height(points[:, 1])
    source_z = height-.12
    top = np.column_stack((points, height))
    bottom = np.column_stack((points, np.full(len(points), COLLISION_BOTTOM)))
    all_vertices = np.vstack((top, bottom))
    solid = [all_vertices[f] for f in faces]
    solid += [all_vertices[f[::-1]+len(points)] for f in faces]
    for a, b in edges:
        solid.extend((all_vertices[[a, a+len(points), b+len(points)]],
                      all_vertices[[a, b+len(points), b]]))
    local = np.array(solid)-ORIGIN
    apron_bounds = np.array(apron.bounds)
    uv = [(t[:, :2]-apron_bounds[:2])/(apron_bounds[2:]-apron_bounds[:2]) for t in solid]
    write_mesh(output_dir/'meshes/gaogongdao_landing.obj', local, 'sand', uv)
    (output_dir/'meshes/gaogongdao_landing.mtl').write_text(
        'newmtl surface\nKa 1 1 1\nKd 1 1 1\nKs 0.015 0.015 0.015\nNs 4\nd 1\n'
        'map_Kd ../textures/harbor_landing_albedo.png\n')
    # Bake the same world-space land, concrete and bare-earth palette across the landing.
    lo, hi = apron_bounds[:2], apron_bounds[2:]
    xx, yy = np.meshgrid(np.linspace(lo[0], hi[0], 1024), np.linspace(hi[1], lo[1], 1024))
    sample = np.column_stack((xx.ravel(), yy.ravel()))
    full_lo, full_hi = (source_vertices[:, :2]+pose[:2]).min(0), (source_vertices[:, :2]+pose[:2]).max(0)
    u = (xx-full_lo[0])/(full_hi[0]-full_lo[0])*(land_image.shape[1]-1)
    v = (full_hi[1]-yy)/(full_hi[1]-full_lo[1])*(land_image.shape[0]-1)
    base = cv2.remap(land_image, u.astype(np.float32), v.astype(np.float32), cv2.INTER_LINEAR).astype(float)
    dist = shapely.distance(shapely.points(sample), line).reshape(xx.shape)
    fade = 1-smoothstep(28., LANDING_RADIUS, np.hypot(xx-LANDING_CENTER[0], yy-LANDING_CENTER[1]))
    road_weight = (1-smoothstep(4.15, 4.65, dist))*fade
    soil_weight = (1-smoothstep(6., 18., dist))*fade*(1-road_weight)
    for name, blend, scale in (('sand', soil_weight, 4.), ('concrete', road_weight, 8.)):
        texture = cv2.cvtColor(cv2.imread(str(output_dir/f'textures/harbor_{name}_albedo.png')), cv2.COLOR_BGR2RGB)
        tex = cv2.remap(texture, (xx/scale % 1*texture.shape[1]).astype(np.float32),
                        ((1-yy/scale % 1)*texture.shape[0]).astype(np.float32),
                        cv2.INTER_LINEAR, borderMode=cv2.BORDER_WRAP)
        base = base*(1-blend[..., None])+tex*blend[..., None]
    cv2.imwrite(str(output_dir/'textures/harbor_landing_albedo.png'),
                cv2.cvtColor(np.uint8(np.clip(base, 0, 255)), cv2.COLOR_RGB2BGR))

    groups = load_harbor_triangles(harbor_dir/'meshes/harbor_no_bridge.dae')
    groups['stone'], source_stone_count = vary_stones(groups['stone'])
    # Clip ONLY old horizontal deck visuals under the new road; retain buildings/boats.
    concrete = []
    for t in groups['concrete']:
        if np.allclose(t[:, 2], ROAD_Z):
            polygon = shapely.Polygon((t+ORIGIN)[:, :2]).difference(road)
            for part in triangles_of(polygon):
                concrete.append(np.column_stack((part, np.full(3, ROAD_Z)))-ORIGIN)
        else:
            concrete.append(t)
    groups['concrete'] = np.array(concrete)
    groups['concrete'], middle_berth = resize_middle_berths(groups['concrete'], groups['roof'])
    # Small irregular riprap pieces along the extension, batched with existing stones.
    rng = np.random.default_rng(1004)
    cube = np.array([[-1,-1,-1],[1,-1,-1],[1,1,-1],[-1,1,-1],[-1,-1,1],[1,-1,1],[1,1,1],[-1,1,1]])
    cube_faces = np.array([[0,2,1],[0,3,2],[4,5,6],[4,6,7],[0,1,5],[0,5,4],
                           [1,2,6],[1,6,5],[2,3,7],[2,7,6],[3,0,4],[3,4,7]])
    stones = []
    for i in range(5, len(xy)-10):
        if mainland.contains(shapely.Point(xy[i])):
            continue
        for side in (-1, 1):
            center = np.array([*(xy[i]+normal[i]*side*(width[i]/2+2.3)), 1.05])
            points3 = cube*rng.uniform([.8,1.,.8], [1.35,1.65,1.3])
            points3 += rng.uniform(-.25, .25, points3.shape)
            angle = rng.uniform(-math.pi, math.pi)
            rot = np.array([[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]])
            points3[:, :2] = points3[:, :2]@rot.T
            center[:2] += (xy[i+1]-xy[i-1])/np.linalg.norm(xy[i+1]-xy[i-1])*rng.uniform(-.5, .5)
            stones.extend((points3+center-ORIGIN)[cube_faces])
    groups['stone'] = np.concatenate((groups['stone'], np.array(stones)))
    harbor_link = harbor_root.find('model/link')
    for visual in list(harbor_link.findall('visual')):
        harbor_link.remove(visual)
    for name, triangles in sorted(groups.items()):
        filename = f'gaogongdao_styled_{name}.obj'
        write_mesh(output_dir/'meshes'/filename, triangles, name)
        add_geometry(harbor_link, 'visual', 'harbor_'+name, filename, name)
    fill = tube_geometry(xy, normal, width)-ORIGIN
    write_mesh(output_dir/'meshes/gaogongdao_causeway_fill.obj', fill)
    _, _, deck_faces, deck_vertices = read_obj(harbor_dir/'meshes/deck_collision.obj')
    old_deck = translate(shapely.union_all([shapely.Polygon(deck_vertices[[r[0]-1 for r in f], :2])
                                          for f in deck_faces]), *ORIGIN[:2])
    exposed_road = road.difference(old_deck.union(apron))
    road_faces = []
    # Insert the two grade break lines before triangulating so a long triangle
    # cannot interpolate a ramp across the entire otherwise-flat sea segment.
    for ymin, ymax in ((-1e5, 550.), (550., 558.), (558., 1e5)):
        for t in triangles_of(exposed_road.intersection(shapely.box(-1e5, ymin, 1e5, ymax))):
            road_faces.append(np.column_stack((t, road_height(t[:, 1]))))
    road_triangles = np.array(road_faces)-ORIGIN
    write_mesh(output_dir/'meshes/gaogongdao_causeway_road.obj', road_triangles, 'concrete')
    for kind in ('visual', 'collision'):
        for suffix, mat in (('fill', 'stone'), ('road', 'concrete')):
            add_geometry(harbor_link, kind, 'causeway_'+suffix+'_'+kind,
                         'gaogongdao_causeway_'+suffix+'.obj', mat if kind == 'visual' else None)
        element = add_geometry(harbor_link, kind, 'landing_'+kind, 'gaogongdao_landing.obj')
        if kind == 'visual':
            ET.SubElement(element, 'cast_shadows').text = 'false'
    chart_link = chart_root.find("model/link[@name='chart_obstacle_001']")
    foundation = chart_link.find("visual[@name='harbor_foundation_visual']")
    _, _, f, verts = read_obj(output_dir/'meshes/gaogongdao_foundation.obj')
    triangles = np.array([verts[[r[0]-1 for r in face]] for face in f])+pose-ORIGIN
    # Foundation stays in the chart link's local frame, but uses the same rock palette.
    target = output_dir/'meshes/gaogongdao_foundation_visual.obj'
    write_mesh(target, triangles+ORIGIN-pose, 'stone',
               [face_uv(t+ORIGIN, MATERIALS['stone'][1]) for t in triangles])
    foundation.find('geometry/mesh/uri').text = 'meshes/'+target.name
    add_material(foundation, 'stone')
    (output_dir/'textures/GAOGONGDAO_SOURCES.md').write_text(
        '# Connected harbor textures\n\nRock textures derive from the bundled CC0 Poly Haven '
        '`rock_face_03_diff_1k.jpg`; land and foliage are color-adjusted copies of existing '
        'CCS assets. See SOURCES.md for the original CC0 licenses. Concrete, roof, paint, '
        'roughness and tangent-space normals are deterministic local procedural bakes. '
        'The satellite screenshot is a layout reference only; no satellite imagery is redistributed.\n')
    return {
        'centerline_enu_m': xy.tolist(), 'control_points_enu_m': CONTROL_POINTS.tolist(),
        'centerline_length_m': line.length, 'road_width_m': [float(width.min()), float(width.max())],
        'road_z_m': ROAD_Z, 'landing_radius_m': LANDING_RADIUS,
        'landing_enu_m': LANDING_CENTER.tolist(), 'landing_max_z_m': float(height.max()),
        'landing_max_raise_m': float((height-source_z).max()),
        'footprint_enu_geojson': shapely.geometry.mapping(toe.union(apron)),
        'road_footprint_enu_geojson': shapely.geometry.mapping(road),
        'landing_footprint_enu_geojson': shapely.geometry.mapping(apron),
        'graded_vertices': grading,
        'new_fill_area_m2': toe.difference(chart).area,
        'water_gap_m': shapely.Point(-91, 432.414214).distance(mainland),
        'source_stones': source_stone_count, 'new_riprap_triangles': len(stones),
        'middle_berth': middle_berth,
        'material_groups': sorted(groups),
        'style': 'Muted weathered concrete, warm gray rocks, blue-gray roofs; no global lighting/water changes.',
    }
