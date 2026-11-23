#!/usr/bin/env python3
"""Bake a reproducible fictional wind farm without editing terrain or source models.

Requires NumPy and Shapely >= 2.1 offline. Runtime uses installed SDF/OBJ only.
"""
import argparse
from functools import lru_cache
import hashlib
import json
import math
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import numpy as np
import shapely
from shapely.affinity import translate
from usv_sim_full.wind_turbine_appearance import (
    OFFSHORE as OFFSHORE_APPEARANCE, HILL as HILL_APPEARANCE, generate as prepare_appearance,
)


PACKAGE = Path(__file__).resolve().parents[1]
MODELS = PACKAGE/'worlds/models'
CHART = MODELS/'ccs_chart_obstacles_enc_5km'
ASSET = MODELS/'iea15_offshore_turbine'
OUTPUT = MODELS/'ccs_wind_farm'
SEED = 150026
PILE_RADIUS = 5.0
LAND_RADIUS = 8.0
ROTOR_RADIUS = 120.0


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def mesh_triangles(path, surface_only=False):
    vertices, faces, group = [], [], ''
    for line in path.read_text().splitlines():
        p = line.split()
        if not p:
            continue
        if p[0] == 'v':
            vertices.append(list(map(float, p[1:4])))
        elif p[0] == 'g':
            group = p[1]
        elif p[0] == 'f' and (not surface_only or group == 'terrain_surface'):
            indices = [int(ref.split('/')[0])-1 for ref in p[1:]]
            faces.extend([indices[0], indices[i], indices[i+1]] for i in range(1, len(indices)-1))
    return np.asarray(vertices)[np.asarray(faces)]


class Surface:
    def __init__(self, path, pose):
        self.triangles = mesh_triangles(path, surface_only=True)+pose
        self.tree = shapely.STRtree(shapely.polygons(self.triangles[:, :, :2]))

    def sample(self, xy):
        xy = np.asarray(xy)
        height, slope = np.full(len(xy), np.nan), np.full(len(xy), np.nan)
        pairs = self.tree.query(shapely.points(xy), predicate='intersects')
        _, first = np.unique(pairs[0], return_index=True)
        points, faces = pairs[:, first]
        tri = self.triangles[faces]
        # Explicit 2x2 barycentric inverse avoids firing a large BLAS thread
        # pool for thousands of tiny systems in an offline geometry operation.
        a = tri[:, 1, :2]-tri[:, 0, :2]
        b = tri[:, 2, :2]-tri[:, 0, :2]
        delta = xy[points]-tri[:, 0, :2]
        determinant = a[:, 0]*b[:, 1]-a[:, 1]*b[:, 0]
        u = (delta[:, 0]*b[:, 1]-delta[:, 1]*b[:, 0])/determinant
        v = (a[:, 0]*delta[:, 1]-a[:, 1]*delta[:, 0])/determinant
        height[points] = tri[:, 0, 2]+u*(tri[:, 1, 2]-tri[:, 0, 2])+v*(tri[:, 2, 2]-tri[:, 0, 2])
        normal = np.cross(tri[:, 1]-tri[:, 0], tri[:, 2]-tri[:, 0])
        slope[points] = np.linalg.norm(normal[:, :2], axis=1)/np.maximum(abs(normal[:, 2]), 1e-9)
        return height, slope


def import_asset(source):
    """Copy once; never silently replace an existing workspace or external model."""
    source = Path(source).resolve()
    expected = ('model.config', 'model.sdf', 'meshes/tower_nacelle.obj', 'meshes/rotor.obj')
    for name in expected:
        src, dst = source/name, ASSET/name
        if not src.is_file():
            raise ValueError(f'Missing turbine input: {src}')
        if dst.exists() and sha256(src) != sha256(dst):
            raise ValueError(f'Refusing to overwrite a different turbine asset: {dst}')
    for name in expected:
        src, dst = source/name, ASSET/name
        if not dst.exists():
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src, dst)


@lru_cache(maxsize=1)
def land_footprint():
    chart = ET.parse(CHART/'model.sdf').getroot()
    parts = []
    for index in range(1, 5):
        link = chart.find(f"model/link[@name='chart_obstacle_{index:03d}']")
        pose = np.fromstring(link.findtext('pose'), sep=' ')[:3]
        triangles = mesh_triangles(CHART/f'meshes/chart_obstacle_{index:03d}.obj')
        roof = triangles[np.all(np.isclose(triangles[:, :, 2], triangles[:, :, 2].max()), axis=1)]
        parts.append(translate(shapely.union_all(shapely.polygons(roof[:, :, :2])), *pose[:2]))
    harbor = json.loads((CHART/'gaogongdao_variant.json').read_text())
    return shapely.union_all(parts+[shapely.geometry.shape(harbor['connection']['footprint_enu_geojson'])]), parts[0]


def choose_layout():
    rng = np.random.default_rng(SEED)
    land, island = land_footprint()
    placements = []
    for row in range(4):
        for column in range(3):
            x, y = np.array([750+650*column, -650-550*row])+rng.uniform(-18, 18, 2)
            if shapely.Point(x, y).distance(land) < ROTOR_RADIUS+100:
                raise ValueError('Offshore rotor envelope too close to existing land')
            placements.append({'name': f'wind_offshore_{len(placements)+1:02d}', 'kind': 'offshore',
                               'pose_enu_m_rad': [float(x), float(y), 0., 0., 0., -.35],
                               'collision_radius_m': PILE_RADIUS})
    link = ET.parse(CHART/'model_gaogongdao.sdf').getroot().find("model/link[@name='chart_obstacle_001']")
    pose = np.fromstring(link.findtext('pose'), sep=' ')[:3]
    visual = Surface(CHART/'meshes/green_island_001_connected.obj', pose)
    collision = Surface(CHART/link.findtext('collision/geometry/mesh/uri'), pose)
    lo, hi = np.array(island.bounds[:2]), np.array(island.bounds[2:])
    candidates = rng.uniform(lo, hi, (24000, 2))
    inside = shapely.contains_xy(island, candidates[:, 0], candidates[:, 1])
    candidates = candidates[inside]
    candidates = candidates[shapely.distance(shapely.points(candidates), island.boundary) > 180]
    heights, slopes = visual.sample(candidates)
    other, other_slopes = collision.sample(candidates)
    candidates = candidates[(heights > 70) & (other > 70) & (slopes < .18) & (other_slopes < .18)]
    details = json.loads((CHART/'meshes/green_island_001_details.json').read_text())
    roots = np.array([item['position'][:2] for item in details['trees']+details['rocks']])+pose[:2]
    ring = LAND_RADIUS*np.column_stack((np.cos(np.arange(16)*math.pi/8), np.sin(np.arange(16)*math.pi/8)))
    hills = []
    for xy in candidates:
        if hills and min(np.linalg.norm(xy-np.array(p['pose_enu_m_rad'][:2])) for p in hills) < 500:
            continue
        if np.linalg.norm(roots-xy, axis=1).min() < 13:
            continue
        sample = np.vstack((xy, xy+ring))
        a, _ = visual.sample(sample)
        b, _ = collision.sample(sample)
        values = np.r_[a, b]
        if not np.all(np.isfinite(values)) or np.ptp(values) > 5:
            continue
        top = float(values.max()+.4)
        hills.append({'name': f'wind_hill_{len(hills)+1:02d}', 'kind': 'hill',
                      'pose_enu_m_rad': [float(xy[0]), float(xy[1]), top, 0., 0., -.35],
                      'collision_radius_m': PILE_RADIUS, 'foundation_radius_m': LAND_RADIUS,
                      'foundation_bottom_z_m': float(values.min()-1),
                      'sampled_terrain_z_range_m': [float(values.min()), float(values.max())]})
        if len(hills) == 4:
            break
    if len(hills) != 4:
        raise ValueError(f'Only {len(hills)} suitable, separated hill sites; refusing unsafe placement')
    return placements+hills


def generate(output=OUTPUT):
    output = Path(output)
    for name in ('model.sdf', 'model.config', 'meshes/tower_nacelle.obj', 'meshes/rotor.obj'):
        if not (ASSET/name).is_file():
            raise ValueError('Import the turbine first with --source /path/to/iea15_offshore_turbine')
    # Test/custom destinations keep every derived asset inside that destination.
    # Model URIs remain relocatable; the installed default uses worlds/models.
    appearance_root = MODELS if output.resolve() == OUTPUT.resolve() else output/'appearance_models'
    appearance = prepare_appearance(ASSET, appearance_root)
    placements = choose_layout()
    root = ET.Element('sdf', version='1.9')
    farm = ET.SubElement(root, 'model', name='ccs_wind_farm')
    ET.SubElement(farm, 'static').text = 'false'
    footings = ET.SubElement(farm, 'model', name='hill_footings')
    ET.SubElement(footings, 'static').text = 'true'
    footprints = []
    for item in placements:
        include = ET.SubElement(farm, 'include')
        variant = OFFSHORE_APPEARANCE if item['kind'] == 'offshore' else HILL_APPEARANCE
        ET.SubElement(include, 'uri').text = 'model://'+variant
        ET.SubElement(include, 'name').text = item['name']
        ET.SubElement(include, 'pose').text = ' '.join(f'{v:.6f}' for v in item['pose_enu_m_rad'])
        x, y, z, *_ = item['pose_enu_m_rad']
        radius = item.get('foundation_radius_m', PILE_RADIUS)
        footprints.append(shapely.Point(x, y).buffer(radius+.05, quad_segs=32))
        if item['kind'] != 'hill':
            continue
        bottom = item['foundation_bottom_z_m']
        length = z-bottom
        link = ET.SubElement(footings, 'link', name=item['name']+'_footing')
        ET.SubElement(link, 'pose').text = f'{x:.6f} {y:.6f} {(z+bottom)/2:.6f} 0 0 0'
        for kind in ('visual', 'collision'):
            element = ET.SubElement(link, kind, name=kind)
            cylinder = ET.SubElement(ET.SubElement(element, 'geometry'), 'cylinder')
            ET.SubElement(cylinder, 'radius').text = str(LAND_RADIUS)
            ET.SubElement(cylinder, 'length').text = f'{length:.6f}'
            if kind == 'visual':
                material = ET.SubElement(element, 'material')
                for tag in ('ambient', 'diffuse'):
                    ET.SubElement(material, tag).text = '.48 .47 .43 1'
                ET.SubElement(material, 'specular').text = '.02 .02 .02 1'
    output.mkdir(parents=True, exist_ok=True)
    ET.indent(root, space='  ')
    ET.ElementTree(root).write(output/'model.sdf', encoding='utf-8', xml_declaration=True)
    inputs = ['model_gaogongdao.sdf', 'gaogongdao_variant.json',
              'meshes/green_island_001_connected.obj', 'meshes/green_island_001_collision_connected.obj',
              'meshes/green_island_001_details.json']
    report = {'seed': SEED, 'offshore_count': 12, 'hill_count': 4,
              'orientation': 'ENU north-up map: offshore farm in southeast (+X, -Y)',
              'rotor_radius_m': ROTOR_RADIUS, 'rotor_speed_rad_s': .7906,
              'placements': placements, 'model_sdf_sha256': sha256(output/'model.sdf'),
               'asset_sha256': {str(p.relative_to(ASSET)): sha256(p) for p in sorted(ASSET.rglob('*')) if p.is_file()},
               'appearance_asset_sha256': appearance['generated_sha256'],
              'terrain_sha256': {name: sha256(CHART/name) for name in inputs},
              'navigation_footprint_enu_geojson': shapely.geometry.mapping(shapely.union_all(footprints)),
              'policy': 'Fictional layout, not a surveyed wind farm or engineering spacing design. '
                        'Original asset and terrain unchanged. Fixed world anchors and original rotor controllers retained. '
                        'Only pile/footing disks are navigation obstacles; channels between turbines remain open. '
                        'Rotor blades are visual-only, as in the supplied model.'}
    (output/'layout.json').write_text(json.dumps(report, indent=2)+'\n')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path)
    args = parser.parse_args()
    if args.source:
        import_asset(args.source)
    result = generate()
    print(json.dumps({key: value for key, value in result.items() if key not in ('placements', 'navigation_footprint_enu_geojson')}, indent=2))
