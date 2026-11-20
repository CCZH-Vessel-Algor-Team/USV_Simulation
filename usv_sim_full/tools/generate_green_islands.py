#!/usr/bin/env python3
"""Bake fictional CCS islands, blended CC0 textures and batched vegetation.

Only an OFFLINE asset generator; no terrain Python dependencies are needed by
Gazebo. Original chart meshes, link poses, holes and waterway footprints stay
unchanged. Requires NumPy, OpenCV, SciPy and Shapely >= 2.1 / GEOS >= 3.10.
"""
import argparse
from collections import Counter, defaultdict
import json
import math
from pathlib import Path
import shutil

import cv2
import numpy as np
from scipy.interpolate import LinearNDInterpolator
from scipy.spatial import Delaunay
import shapely


MODEL_DIR = (Path(__file__).resolve().parents[1] / 'worlds/models'
             / 'ccs_chart_obstacles_enc_5km')
# index: nominal mesh spacing (m), height scale (m), texture size, tree budget.
ISLANDS = {1: (22, 160, 4096, 1400), 2: (9, 65, 2048, 240),
           3: (10, 45, 2048, 180), 4: (3, 16, 1024, 14)}


def smoothstep(low, high, value):
    value = np.clip((value - low) / (high - low), 0, 1)
    return value * value * (3 - 2 * value)


def noise(x, y, seed=0):
    """Continuous, deterministic value noise, with no sinusoidal terrain bands."""
    ix, iy = np.floor(x), np.floor(y)
    fx, fy = smoothstep(0, 1, x - ix), smoothstep(0, 1, y - iy)

    def hashed(dx, dy):
        value = ((ix + dx).astype(np.int64).astype(np.uint32) * np.uint32(374761393)
                 + (iy + dy).astype(np.int64).astype(np.uint32) * np.uint32(668265263)
                 + np.uint32(seed * 1447))
        value = (value ^ (value >> 13)) * np.uint32(1274126177)
        return ((value ^ (value >> 16)) & 0xffffff).astype(np.float32) / 16777215

    a = hashed(0, 0) * (1 - fx) + hashed(1, 0) * fx
    b = hashed(0, 1) * (1 - fx) + hashed(1, 1) * fx
    return a * (1 - fy) + b * fy


def fbm(x, y, scale, seed):
    return sum(weight * noise(x / (scale / 2**octave), y / (scale / 2**octave),
                              seed + octave * 19)
               for octave, weight in enumerate((0.55, 0.26, 0.13, 0.06)))


def load_source(path):
    vertices, faces = [], []
    for line in path.read_text().splitlines():
        fields = line.split()
        if fields and fields[0] == 'v':
            vertices.append(tuple(map(float, fields[1:4])))
        elif fields and fields[0] == 'f' and len(fields) == 4:
            faces.append(tuple(int(part.split('/')[0]) - 1 for part in fields[1:]))
    vertices = np.asarray(vertices)
    top, bottom = vertices[:, 2].max(), vertices[:, 2].min()
    roofs = np.asarray([face for face in faces if np.allclose(vertices[list(face), 2], top)])
    if not len(roofs):
        raise ValueError(f'{path}: missing triangulated chart roof')
    footprint = shapely.union_all(shapely.polygons(vertices[roofs, :2]))
    if not footprint.is_valid:
        raise ValueError(f'{path}: invalid chart footprint')
    return footprint, float(bottom)


def polygon_parts(geometry):
    if geometry.geom_type == 'Polygon':
        yield geometry
    elif hasattr(geometry, 'geoms'):
        for part in geometry.geoms:
            yield from polygon_parts(part)


def triangulated_surface(footprint, spacing, seed):
    """Delaunay interior, clipped exactly to all chart rings and holes.

    Refining the old cap triangles retained very skinny CAD triangles. Start
    with well-distributed samples instead; constrained triangulation of clipped
    boundary cells prevents new triangles crossing a harbour or coastline.
    """
    boundary = []
    for polygon in polygon_parts(footprint):
        for ring in (polygon.exterior, *polygon.interiors):
            coords = np.asarray(ring.coords)
            for a, b in zip(coords, coords[1:]):
                count = max(1, math.ceil(np.linalg.norm(b - a) / spacing))
                boundary.extend(a + (b - a) * (i / count) for i in range(count))
    lo, hi = np.array(footprint.bounds[:2]), np.array(footprint.bounds[2:])
    x, y = np.meshgrid(np.arange(lo[0], hi[0], spacing),
                       np.arange(lo[1], hi[1], spacing * 0.866))
    rng = np.random.default_rng(seed)
    points = np.column_stack((x.ravel(), y.ravel()))
    points += rng.uniform(-0.23, 0.23, points.shape) * spacing
    points = points[shapely.contains_xy(footprint, points[:, 0], points[:, 1])]
    points = np.unique(np.round(np.vstack((boundary, points)), 6), axis=0)
    candidates = Delaunay(points).simplices
    lookup = {tuple(point): i for i, point in enumerate(points)}
    all_points, faces = points.tolist(), []
    for start in range(0, len(candidates), 8192):
        batch = candidates[start:start + 8192]
        polygons = shapely.polygons(points[batch])
        inside = shapely.covers(footprint, polygons)
        faces.extend(batch[inside].tolist())
        for clipped in shapely.intersection(polygons[~inside], footprint):
            for polygon in polygon_parts(clipped):
                if polygon.area < 1e-9:
                    continue
                for triangle in shapely.constrained_delaunay_triangles(polygon).geoms:
                    face = []
                    for point in np.round(np.asarray(triangle.exterior.coords)[:3], 6):
                        key = tuple(point)
                        if key not in lookup:
                            lookup[key] = len(all_points)
                            all_points.append(point.tolist())
                        face.append(lookup[key])
                    faces.append(face)
    points, faces = np.asarray(all_points), np.asarray(faces, dtype=int)
    used, inverse = np.unique(faces, return_inverse=True)
    points, faces = points[used], inverse.reshape(-1, 3)
    a, b, c = points[faces[:, 0]], points[faces[:, 1]], points[faces[:, 2]]
    cross = (b[:, 0] - a[:, 0]) * (c[:, 1] - a[:, 1]) - (b[:, 1] - a[:, 1]) * (c[:, 0] - a[:, 0])
    faces[cross < 0] = faces[cross < 0][:, [0, 2, 1]]
    faces = faces[np.abs(cross) > 1e-9]
    edges = Counter(tuple(sorted(edge)) for a, b, c in faces
                    for edge in ((a, b), (b, c), (c, a)))
    # Raster chart contours can touch at a single XY point. They must not share
    # one vertical skirt edge (four incident faces). Split each separate roof
    # fan at such a point, without moving the coastline even a micrometre.
    degree = Counter(i for edge, count in edges.items() if count == 1 for i in edge)
    points_list = points.tolist()
    for vertex, count in degree.items():
        if count <= 2:
            continue
        incident = set(np.flatnonzero(np.any(faces == vertex, axis=1)))
        neighbours = defaultdict(list)
        for face_id in incident:
            for other in faces[face_id]:
                if other != vertex:
                    neighbours[other].append(face_id)
        first = True
        while incident:
            queue, fan = [incident.pop()], []
            while queue:
                face_id = queue.pop()
                fan.append(face_id)
                for other in faces[face_id]:
                    for adjacent in neighbours.get(other, ()):
                        if adjacent in incident:
                            incident.remove(adjacent)
                            queue.append(adjacent)
            if not first:
                new_vertex = len(points_list)
                points_list.append(points[vertex].tolist())
                for face_id in fan:
                    faces[face_id, faces[face_id] == vertex] = new_vertex
            first = False
    points = np.asarray(points_list)
    edges = Counter(tuple(sorted(edge)) for a, b, c in faces
                    for edge in ((a, b), (b, c), (c, a)))
    boundary_edges = [(a, b) for a, b, c in faces for a, b in ((a, b), (b, c), (c, a))
                      if edges[tuple(sorted((a, b)))] == 1]
    return points, faces, boundary_edges


class TerrainField:
    """One continuous field shared by all levels of mesh and the texture bake."""

    def __init__(self, footprint, height, seed):
        self.footprint, self.height, self.seed = footprint, height, seed
        self.lo, self.hi = np.array(footprint.bounds[:2]), np.array(footprint.bounds[2:])
        self.extent = self.hi - self.lo
        self.cell = max(0.25, self.extent.max() / 2048)
        self.scale = max(12, min(330, self.extent.min() * 0.65))
        self.shore_scale = max(4, min(85, self.extent.min() * 0.14))
        size = np.ceil(self.extent / self.cell).astype(int) + 7
        mask = np.zeros((size[1], size[0]), dtype=np.uint8)
        for polygon in polygon_parts(footprint):
            for i, ring in enumerate((polygon.exterior, *polygon.interiors)):
                coords = np.rint((np.asarray(ring.coords) - self.lo) / self.cell + 3).astype(np.int32)
                cv2.fillPoly(mask, [coords], 1 if i == 0 else 0)
        self.distances = cv2.GaussianBlur(cv2.distanceTransform(mask, cv2.DIST_L2, 5),
                                        (0, 0), 0.7) * self.cell

    def distance(self, x, y):
        shape = np.broadcast_shapes(np.shape(x), np.shape(y))
        x = np.broadcast_to(x, shape).ravel()
        y = np.broadcast_to(y, shape).ravel()
        u, v = (x - self.lo[0]) / self.cell + 3, (y - self.lo[1]) / self.cell + 3
        # Chunked remap avoids OpenCV's 32767-pixel dimension limit.
        result = np.empty(len(x), dtype=np.float32)
        for start in range(0, len(x), 16000):
            result[start:start + 16000] = cv2.remap(
                self.distances, u[start:start + 16000].astype(np.float32)[None, :],
                v[start:start + 16000].astype(np.float32)[None, :], cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_CONSTANT).ravel()
        return result.reshape(shape)

    def elevation(self, x, y):
        d, scale = self.distance(x, y), self.scale
        wx = x + (noise(x / scale, y / scale, self.seed) - 0.5) * scale * 0.65
        wy = y + (noise(x / scale, y / scale, self.seed + 7) - 0.5) * scale * 0.65
        broad = fbm(wx, wy, scale * 2.8, self.seed + 9)
        ridge = (1 - np.abs(2 * noise(wx / scale, wy / scale, self.seed + 31) - 1))**2.7
        detail = fbm(wx, wy, scale * 0.48, self.seed + 63)
        shape = 0.07 + 0.40 * broad + 0.62 * ridge + 0.12 * detail
        shore = 1 - np.exp(-(np.maximum(d - self.cell * 0.65, 0) / self.shore_scale)**1.3)
        return 0.25 + self.height * shape * shore

    def slope(self, x, y):
        step = max(0.5, self.cell)
        dx = (self.elevation(x + step, y) - self.elevation(x - step, y)) / (2 * step)
        dy = (self.elevation(x, y + step) - self.elevation(x, y - step)) / (2 * step)
        return np.hypot(dx, dy)


def write_surface(output, xy, faces, boundary_edges, field, bottom, textured):
    z = field.elevation(xy[:, 0], xy[:, 1])
    z[np.unique(boundary_edges)] = 0.25
    top = np.column_stack((xy, z))
    size = len(top)
    vertices = np.vstack((top, np.column_stack((xy, np.full(size, bottom)))))
    normals = np.zeros_like(top)
    face_normals = np.cross(top[faces[:, 1]] - top[faces[:, 0]], top[faces[:, 2]] - top[faces[:, 0]])
    for corner in range(3):
        np.add.at(normals, faces[:, corner], face_normals)
    normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)
    walls = [(a, a + size, b + size) for a, b in boundary_edges]
    walls += [(a, b + size, b) for a, b in boundary_edges]
    wall_normals = np.cross(vertices[np.array(walls)[:, 1]] - vertices[np.array(walls)[:, 0]],
                            vertices[np.array(walls)[:, 2]] - vertices[np.array(walls)[:, 0]])
    wall_normals /= np.maximum(np.linalg.norm(wall_normals, axis=1, keepdims=True), 1e-12)
    material = output.with_suffix('.mtl')
    material.write_text('newmtl terrain\nKa 1 1 1\nKd 1 1 1\nKs 0.015 0.015 0.015\nNs 4\nd 1\n'
                        + (f'map_Kd ../textures/{output.stem}_albedo.jpg\n' if textured else ''))
    with output.open('w') as f:
        f.write(f'# Fictional terrain; original chart footprint; metres\nmtllib {material.name}\no island\n')
        for v in vertices:
            f.write('v {:.6f} {:.6f} {:.6f}\n'.format(*v))
        uv = (vertices[:, :2] - field.lo) / field.extent
        for u, v in uv:
            f.write(f'vt {u:.8f} {v:.8f}\n')
        for n in np.vstack((normals, wall_normals, [[0, 0, -1]])):
            f.write('vn {:.7f} {:.7f} {:.7f}\n'.format(*n))
        f.write('usemtl terrain\ns 1\ng terrain_surface\n')
        for face in faces:
            f.write('f ' + ' '.join(f'{i+1}/{i+1}/{i+1}' for i in face) + '\n')
        f.write('s off\ng shoreline_skirt\n')
        for ni, face in enumerate(walls, start=size + 1):
            f.write('f ' + ' '.join(f'{i+1}/{i+1}/{ni}' for i in face) + '\n')
        f.write('g seabed_cap\n')
        ni = size + len(walls) + 1
        for face in faces[:, ::-1] + size:
            f.write('f ' + ' '.join(f'{i+1}/{i+1}/{ni}' for i in face) + '\n')
    return top


def tiled_image(image, x, y, tile_size, pixel_size):
    size = int(np.clip(tile_size / pixel_size * 1.5, 16, image.shape[0]))
    image = cv2.resize(image, (size, size), interpolation=cv2.INTER_AREA)
    u = np.mod(x / tile_size, 1).astype(np.float32) * size
    v = np.mod(y / tile_size, 1).astype(np.float32) * size
    return cv2.remap(image, u, v, cv2.INTER_LINEAR, borderMode=cv2.BORDER_WRAP) / 255.0


def bake_albedo(field, textures, stem, size):
    grass = cv2.imread(str(textures / 'rocky_terrain_02_diff_1k.jpg'))
    rock = cv2.imread(str(textures / 'rock_face_03_diff_1k.jpg'))
    gravel = cv2.imread(str(textures / 'grass_path_2_diff_1k.jpg'))
    if grass is None or rock is None or gravel is None:
        raise FileNotFoundError(f'Missing bundled CC0 source textures in {textures}')
    # Row zero is maximum world Y; OBJ V=0 is the bottom of the image.
    xs = np.linspace(field.lo[0], field.hi[0], size, dtype=np.float32)
    ys = np.linspace(field.hi[1], field.lo[1], max(256, round(size * field.extent[1] / field.extent[0])),
                     dtype=np.float32)
    # Cap both dimensions (long thin components must not allocate giant maps).
    ys = np.linspace(field.hi[1], field.lo[1], min(size, len(ys)), dtype=np.float32)
    image = np.empty((len(ys), size, 3), dtype=np.uint8)
    for row in range(0, len(ys), 128):
        x, y = np.meshgrid(xs, ys[row:row + 128])
        d, h = field.distance(x, y), field.elevation(x, y)
        patch = fbm(x, y, max(15, field.scale * 0.38), field.seed + 90)
        slope = field.slope(x, y)
        rock_weight = smoothstep(0.40, 1.05, slope + (patch - 0.5) * 0.40)
        rock_weight = np.maximum(rock_weight, smoothstep(0.78, 1.08, h / field.height) * 0.65)
        grass_rgb = tiled_image(grass, x, y, max(14, field.scale * 0.12), field.extent[0] / size)
        rock_rgb = tiled_image(rock, x, y, max(9, field.scale * 0.09), field.extent[0] / size)
        # OpenCV BGR: retain natural ochre/green rather than the old neon paint.
        grass_rgb = grass_rgb * np.array([0.92, 1.18, 0.80]) + np.array([0.035, 0.045, 0.025])
        grass_rgb *= (0.81 + 0.40 * patch)[..., None]
        gray = rock_rgb.mean(axis=2, keepdims=True)
        rock_rgb = rock_rgb * 0.38 + gray * np.array([0.66, 0.65, 0.63])
        rgb = grass_rgb * (1 - rock_weight[..., None]) + rock_rgb * rock_weight[..., None]
        shore = 1 - smoothstep(2.5, 7.5 + patch * 4, d)
        sand = tiled_image(gravel, x, y, 12, field.extent[0] / size) * 0.88
        rgb = rgb * (1 - shore[..., None]) + sand * shore[..., None]
        rgb *= (0.68 + 0.32 * smoothstep(0.0, 3.2, d))[..., None]
        image[row:row + len(x)] = np.clip(rgb * 255, 0, 255).astype(np.uint8)
    path = textures / f'{stem}_albedo.jpg'
    if not cv2.imwrite(str(path), image, [cv2.IMWRITE_JPEG_QUALITY, 92]):
        raise OSError(f'Cannot write {path}')


def shared_vegetation_textures(textures):
    rng = np.random.default_rng(71027)
    x, y = np.meshgrid(np.arange(256, dtype=np.float32), np.arange(256, dtype=np.float32))
    leaves = fbm(x, y, 26, 92) * 0.9 + rng.random(x.shape) * 0.25
    rgb = np.array([56, 91, 67])[None, None, :] * (0.50 + leaves[..., None])
    cv2.imwrite(str(textures / 'foliage.jpg'), np.clip(rgb, 0, 255).astype(np.uint8))
    ridges = 0.65 + 0.42 * noise(x / 6, y / 70, 78) + rng.random(x.shape) * 0.17
    cv2.imwrite(str(textures / 'bark.jpg'), np.clip(ridges[..., None] * [48, 65, 78], 0, 255).astype(np.uint8))


class Decorations:
    """Batch trees/boulders into one OBJ per island, not thousands of entities."""

    def __init__(self):
        self.vertices, self.uv, self.normals = [], [], []
        self.faces = defaultdict(list)

    def ellipsoid(self, center, radius, rng, material):
        segments, rings = 8, 5
        first = len(self.vertices)
        for row in range(rings + 1):
            latitude = math.pi * row / rings
            for col in range(segments + 1):
                longitude = 2 * math.pi * col / segments
                n = np.array([math.sin(latitude) * math.cos(longitude),
                              math.sin(latitude) * math.sin(longitude), math.cos(latitude)])
                wobble = 1 if row in (0, rings) else 0.86 + rng.random() * 0.26
                self.vertices.append(np.asarray(center) + n * radius * wobble)
                normal = n / radius
                self.normals.append(normal / max(np.linalg.norm(normal), 1e-12))
                self.uv.append((col / segments, row / rings))
        for row in range(rings):
            for col in range(segments):
                a = first + row * (segments + 1) + col
                b, c, d = a + 1, a + segments + 1, a + segments + 2
                if row != 0:
                    self.faces[material].append((a, c, b))
                if row != rings - 1:
                    self.faces[material].append((b, c, d))

    def tree(self, x, y, z, height, radius, rng):
        # A tapered six-sided trunk is both more natural and much cheaper than
        # a stretched sphere for every tree. Most detail belongs in the crown.
        first = len(self.vertices)
        lean = rng.uniform(-0.22, 0.22, 2) * radius
        for row in range(2):
            r = height * (0.023 if row == 0 else 0.010)
            for col in range(7):
                angle = col * math.pi / 3
                self.vertices.append((x + lean[0] * row + r * math.cos(angle),
                                      y + lean[1] * row + r * math.sin(angle), z + height * 0.73 * row))
                self.normals.append((math.cos(angle), math.sin(angle), 0))
                self.uv.append((col / 6, row))
        for col in range(6):
            a = first + col
            self.faces['bark'].extend(((a, a + 1, a + 7), (a + 1, a + 8, a + 7)))
        material = 'leaves_' + str(rng.integers(0, 3))
        if rng.random() < 0.38:
            for level in range(4):
                base = z + height * (0.20 + level * 0.17)
                tier_radius = radius * (1 - level * 0.19)
                first = len(self.vertices)
                for row, (rise, width) in enumerate(((0, 1), (0.38, 0.71), (1, 0.015))):
                    for col in range(9):
                        angle = 2 * math.pi * col / 8
                        r = tier_radius * width * (0.85 + rng.random() * 0.28)
                        self.vertices.append((x + r * math.cos(angle), y + r * math.sin(angle),
                                              base + rise * height * 0.34))
                        normal = np.array([math.cos(angle), math.sin(angle), tier_radius / (height * 0.34)])
                        self.normals.append(normal / np.linalg.norm(normal))
                        self.uv.append((col / 8, rise))
                for row in range(2):
                    for col in range(8):
                        a = first + row * 9 + col
                        self.faces[material].extend(((a, a + 1, a + 9), (a + 1, a + 10, a + 9)))
            return
        angle = rng.uniform(0, 2 * math.pi)
        for dx, dy, dz, factor in ((0, 0, 0.72, 1), (0.36, 0.12, 0.56, 0.72), (-0.30, -0.25, 0.62, 0.78)):
            cx = x + radius * (dx * math.cos(angle) - dy * math.sin(angle))
            cy = y + radius * (dx * math.sin(angle) + dy * math.cos(angle))
            self.ellipsoid((cx, cy, z + height * dz),
                           np.array([radius * factor, radius * 0.88 * factor, height * 0.30 * factor]), rng, material)

    def write(self, output):
        material = output.with_suffix('.mtl')
        with material.open('w') as f:
            for name, color, texture in (
                    ('bark', (1, 1, 1), 'bark.jpg'),
                    ('leaves_0', (0.82, 0.92, 0.78), 'foliage.jpg'),
                    ('leaves_1', (0.98, 1.0, 0.87), 'foliage.jpg'),
                    ('leaves_2', (0.73, 0.86, 0.70), 'foliage.jpg'),
                    ('boulder', (0.86, 0.87, 0.83), 'rock_face_03_diff_1k.jpg')):
                f.write(f'newmtl {name}\nKa 1 1 1\nKd {color[0]} {color[1]} {color[2]}\n'
                        f'Ks 0.01 0.01 0.01\nNs 3\nd 1\nmap_Kd ../textures/{texture}\n\n')
        with output.open('w') as f:
            f.write(f'# Decorative vegetation and rocks, no individual collision\nmtllib {material.name}\no vegetation\ns 1\n')
            for v in self.vertices:
                f.write('v {:.5f} {:.5f} {:.5f}\n'.format(*v))
            for uv in self.uv:
                f.write('vt {:.6f} {:.6f}\n'.format(*uv))
            for n in self.normals:
                f.write('vn {:.6f} {:.6f} {:.6f}\n'.format(*n))
            for name, faces in self.faces.items():
                f.write(f'usemtl {name}\n')
                for face in faces:
                    f.write('f ' + ' '.join(f'{i+1}/{i+1}/{i+1}' for i in face) + '\n')


def plant_decorations(field, top, budget, output):
    rng = np.random.default_rng(field.seed + 101)
    candidates = rng.uniform(field.lo, field.hi, (max(400, budget * 85), 2))
    candidates = candidates[shapely.contains_xy(field.footprint, candidates[:, 0], candidates[:, 1])]
    x, y = candidates.T
    depth = field.distance(x, y)
    slope = field.slope(x, y)
    stand = fbm(x, y, max(18, field.scale * 0.7), field.seed + 177)
    habitat = (depth > 6) & (slope < 0.85)
    forest_cut = float(np.quantile(stand[habitat], 0.68)) if habitat.any() else 1.0
    acceptable = habitat & (stand > forest_cut)
    acceptable &= rng.random(len(x)) < np.clip(0.25 + (stand - forest_cut) * 6, 0, 1)
    sample_height = LinearNDInterpolator(top[:, :2], top[:, 2])
    decoration, trees, rocks, cells = Decorations(), [], [], {}
    tree_scale = min(1.0, max(0.28, field.height / 65))
    for pos in candidates[acceptable]:
        x, y = pos
        height = rng.uniform(7, 15) * tree_scale
        radius = height * rng.uniform(0.27, 0.38)
        # Exact geometric distance bounds the entire canopy inside chart land.
        if shapely.distance(shapely.Point(x, y), field.footprint.boundary) < radius * 1.5 + 1:
            continue
        cell = (int(x // 5), int(y // 5))
        if any(np.linalg.norm(pos - previous) < radius * 1.15
               for dx in range(-2, 3) for dy in range(-2, 3)
               for previous in cells.get((cell[0] + dx, cell[1] + dy), [])):
            continue
        z = float(sample_height(pos))
        if not math.isfinite(z):
            continue
        decoration.tree(x, y, z - 0.2, height, radius, rng)
        trees.append({'position': [float(x), float(y), z], 'height': height, 'radius': radius})
        cells.setdefault(cell, []).append(pos)
        if len(trees) >= budget:
            break
    for pos in candidates[(depth > 2) & (depth < 24)][::3]:
        x, y = pos
        radius = rng.uniform(0.65, 2.2) * max(0.45, tree_scale)
        if shapely.distance(shapely.Point(x, y), field.footprint.boundary) < radius * 1.25:
            continue
        z = float(sample_height(pos))
        if not math.isfinite(z):
            continue
        decoration.ellipsoid((x, y, z + radius * 0.18), np.array([radius, radius * 0.8, radius * 0.66]), rng, 'boulder')
        rocks.append({'position': [float(x), float(y), z], 'radius': radius})
        if len(rocks) >= max(6, budget // 7):
            break
    decoration.write(output)
    return trees, rocks, sum(len(faces) for faces in decoration.faces.values())


def generate_all(model_dir=MODEL_DIR, only=None):
    if not hasattr(shapely, 'constrained_delaunay_triangles'):
        raise RuntimeError('Offline generation needs Shapely >= 2.1 / GEOS >= 3.10')
    model_dir = Path(model_dir)
    textures = model_dir / 'textures'
    textures.mkdir(parents=True, exist_ok=True)
    # A new output model also needs the bundled CC0 inputs, not a network fetch.
    for name in ('rocky_terrain_02_diff_1k.jpg', 'rock_face_03_diff_1k.jpg',
                 'grass_path_2_diff_1k.jpg', 'SOURCES.md'):
        target = textures / name
        if not target.exists():
            shutil.copy2(MODEL_DIR / 'textures' / name, target)
    shared_vegetation_textures(textures)
    for index, (spacing, height, texture_size, budget) in ISLANDS.items():
        if only is not None and index not in only:
            continue
        source = model_dir / 'meshes' / f'chart_obstacle_{index:03d}.obj'
        if not source.exists():
            continue
        footprint, bottom = load_source(source)
        field = TerrainField(footprint, height, 421 + index * 79)
        stem = f'green_island_{index:03d}'
        xy, faces, edges = triangulated_surface(footprint, spacing, field.seed)
        top = write_surface(source.with_name(stem + '.obj'), xy, faces, edges, field, bottom, True)
        bake_albedo(field, textures, stem, texture_size)
        trees, rocks, decoration_faces = plant_decorations(field, top, budget, source.with_name(stem + '_vegetation.obj'))
        collision_xy, collision_faces, collision_edges = triangulated_surface(footprint, spacing * 1.55, field.seed)
        write_surface(source.with_name(stem + '_collision.obj'), collision_xy, collision_faces, collision_edges,
                      field, bottom, False)
        metadata = {'generator_version': 2, 'source': source.name, 'seed': field.seed,
                    'footprint_area_m2': footprint.area, 'roof_faces': len(faces),
                    'collision_roof_faces': len(collision_faces), 'decoration_faces': decoration_faces,
                    'height_range_m': [float(top[:, 2].min()), float(top[:, 2].max())],
                    'trees': trees, 'rocks': rocks}
        source.with_name(stem + '_details.json').write_text(json.dumps(metadata, indent=2) + '\n')
        print(f'{stem}: {len(faces)} terrain / {len(collision_faces)} collision roof triangles, '
              f'{len(trees)} trees, {len(rocks)} rocks, {decoration_faces} decorative triangles', flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model-dir', type=Path, default=MODEL_DIR)
    parser.add_argument('--only', type=int, nargs='+', choices=range(1, 5))
    args = parser.parse_args()
    generate_all(args.model_dir, args.only)
