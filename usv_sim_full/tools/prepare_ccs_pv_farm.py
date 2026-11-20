#!/usr/bin/env python3
"""Install the supplied static platform and generate the CCS offshore PV array."""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import re
import stat
import xml.etree.ElementTree as ET
import zipfile

import shapely
from shapely.geometry import Point, box, mapping

from usv_sim_full.ccs_pv_visuals import add_tiles, cylinder_axis, rotated_offset, tile_layout, tile_obj
from usv_sim_full.wind_turbine_appearance import Mesh, add


MODELS = Path(__file__).resolve().parents[1]/'worlds/models'
WORLD = MODELS.parent/'ccs_open_water_crop.sdf'
SOURCE = 'offshore_pv_platform'
LIGHT = 'offshore_pv_platform_array'
FARM = 'ccs_offshore_pv_farm'
HILLS = 'ccs_hill_wind_farm'
LON, LAT = 119.49527, 34.67997
SIDE_M = 1000.
COLS, ROWS, DX, DY = 14, 21, 70., 47.
PAYLOAD = {'model.config', 'model.sdf',
           *('meshes/'+name+'.dae' for name in
             ('coating', 'frames', 'hardware', 'piles', 'pv_back', 'pv_front', 'truss')),
           'materials/textures/module_back.png', 'materials/textures/module_front.png'}


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def install_archive(archive, models):
    """Read only approved entries and never overwrite differing source files."""
    with zipfile.ZipFile(archive) as src:
        expected = {SOURCE+'/'+name for name in PAYLOAD} | {'README.md'}
        entries = [item for item in src.infolist() if not item.is_dir()]
        if ({item.filename for item in entries} != expected or len(entries) != len(expected)
                or any(stat.S_ISLNK(item.external_attr >> 16) or item.file_size > 8_000_000
                       for item in entries)):
            raise ValueError('Unexpected archive contents')
        assets = {name: src.read(SOURCE+'/'+name) for name in PAYLOAD}
        assets['SOURCE_README.md'] = src.read('README.md')
    for name, data in assets.items():
        dest = models/SOURCE/name
        if dest.is_symlink() or (dest.exists() and dest.read_bytes() != data):
            raise ValueError(f'Refusing to overwrite source asset: {dest}')
    for name, data in assets.items():
        dest = models/SOURCE/name
        if not dest.exists():
            dest.parent.mkdir(parents=True, exist_ok=True)
            dest.write_bytes(data)


def local_enu(lon, lat, lon0, lat0, alt0=0.):
    """WGS84 ECEF to the same tangent plane as Gazebo/LocalCartesian."""
    def ecef(lon, lat, alt):
        a = 6378137.
        e2 = 6.6943799901413165e-3
        n = a/math.sqrt(1-e2*math.sin(lat)**2)
        return ((n+alt)*math.cos(lat)*math.cos(lon), (n+alt)*math.cos(lat)*math.sin(lon),
                (n*(1-e2)+alt)*math.sin(lat))

    lon, lat, lon0, lat0 = map(math.radians, (lon, lat, lon0, lat0))
    dx, dy, dz = (a-b for a, b in zip(ecef(lon, lat, 0.), ecef(lon0, lat0, alt0)))
    x = -math.sin(lon0)*dx+math.cos(lon0)*dy
    y = -math.sin(lat0)*(math.cos(lon0)*dx+math.sin(lon0)*dy)+math.cos(lat0)*dz
    return x, y


def xml_bytes(root):
    ET.indent(root, space='  ')
    return ET.tostring(root, encoding='utf-8', xml_declaration=True)+b'\n'


def config_bytes(name, description):
    root = ET.Element('model')
    ET.SubElement(root, 'name').text = name
    ET.SubElement(root, 'version').text = '1.0'
    ET.SubElement(root, 'sdf', version='1.9').text = 'model.sdf'
    ET.SubElement(root, 'description').text = description
    return xml_bytes(root)


def add_include(model, uri, name, pose):
    include = ET.SubElement(model, 'include')
    ET.SubElement(include, 'uri').text = 'model://'+uri
    ET.SubElement(include, 'name').text = name
    ET.SubElement(include, 'pose').text = ' '.join(f'{x:.6f}' for x in pose)


def simplified_platform(source_root):
    """Retain the single-platform collision asset as a reference for the tiled array."""
    root = copy.deepcopy(source_root)
    model = root.find('model')
    model.set('name', LIGHT)
    link = model.find('link')
    for visual in link.findall('visual'):
        link.remove(visual)
    overhead = Mesh()
    for collision in link.findall('collision'):
        name = collision.get('name')
        if re.fullmatch(r'pile_[1-4]', name):
            continue  # Retain exact analytic cylinders at the navigation waterline.
        link.remove(collision)
        pose = tuple(map(float, collision.findtext('pose').split()))
        cylinder = collision.find('geometry/cylinder')
        primary = name.startswith(('pile_collar_', 'bearing_', 'panel_strip_', 'lower_chord_', 'upper_chord_'))
        if primary and cylinder is not None:
            start, end, radius = cylinder_axis(cylinder, pose)
            overhead.beam(start, end, radius, segments=8)
        elif primary:
            primitive = Mesh()
            primitive.box((0., 0., 0.), tuple(map(float, collision.findtext('geometry/box/size').split())))
            for vertices, normals in primitive.triangles:
                overhead.triangle(*(add(pose[:3], rotated_offset(p, pose[3:])) for p in vertices),
                                  normals=tuple(rotated_offset(n, pose[3:]) for n in normals))
    # These are the individual overhead surfaces, not their hull or a filled sheet.
    # One static mesh per repeat instead of dozens of DART collision objects.
    collision = ET.SubElement(link, 'collision', name='overhead_structure')
    geometry = ET.SubElement(ET.SubElement(collision, 'geometry'), 'mesh')
    ET.SubElement(geometry, 'uri').text = f'model://{LIGHT}/meshes/overhead_collision.obj'
    return root, overhead


def add_tile_collisions(model, tiles, template_link, overhead):
    """Keep exact pile cylinders; batch disjoint overhead surfaces by tile."""
    piles = [c for c in template_link.findall('collision') if re.fullmatch(r'pile_[1-4]', c.get('name'))]
    faces = [(p, n, (None,)*3) for p, n in overhead.triangles]
    files = {}
    for link_name, variant, _, offsets in tiles:
        tile = model.find(f"link[@name='{link_name}']")
        for index, offset in enumerate(offsets):
            for pile in piles:
                c = copy.deepcopy(pile)
                c.set('name', f"site_{index+1:02d}_{pile.get('name')}")
                x, y, z, roll, pitch, yaw = map(float, c.findtext('pose').split())
                coords = (x+offset[0], y+offset[1], z, roll, pitch, yaw)
                c.find('pose').text = ' '.join(format(v, '.15g') for v in coords)
                tile.append(c)
        # Do not form a hull across platforms: send the same panel bands and
        # members as disjoint triangles, leaving both passages and clearance open.
        path = f'{FARM}/meshes/{variant}_overhead_collision.obj'
        if path not in files:
            files[path] = tile_obj(faces, offsets)
        collision = ET.SubElement(tile, 'collision', name='overhead_structure')
        geom = ET.SubElement(ET.SubElement(collision, 'geometry'), 'mesh')
        ET.SubElement(geom, 'uri').text = 'model://'+path
    stats = {'standalone_platforms': 1, 'active_tile_links': len(tiles),
             'analytic_piles_on_tiles': sum(len(tile[3])*len(piles) for tile in tiles),
             'overhead_mesh_collisions': len(tiles)}
    return files, stats


def artifacts(models, world=WORLD):
    """Return deterministic SDF/manifests without reading the external ZIP again."""
    datum = ET.parse(world).find('world/spherical_coordinates')
    if datum.findtext('surface_model') != 'EARTH_WGS84' or datum.findtext(
            'world_frame_orientation') != 'ENU' or float(datum.findtext('heading_deg')) != 0:
        raise ValueError('Expected the unrotated CCS WGS84/ENU world')
    center_ref = (float(datum.findtext('longitude_deg')), float(datum.findtext('latitude_deg')),
                  float(datum.findtext('elevation')))
    cx, cy = local_enu(LON, LAT, *center_ref)
    square = box(cx-SIDE_M/2, cy-SIDE_M/2, cx+SIDE_M/2, cy+SIDE_M/2)
    source = models/SOURCE
    original = ET.parse(source/'model.sdf').getroot()
    template = original.find('model')
    if template.findtext('static') != 'true' or list(template.iter('plugin')):
        raise ValueError('Expected a static platform without plugins')
    original_link = template.find('link')
    piles = []
    for c in original_link.findall('collision'):
        if re.fullmatch(r'pile_[1-4]', c.get('name')):
            x, y, z, roll, pitch, _ = map(float, c.findtext('pose').split())
            cylinder = c.find('geometry/cylinder')
            half = float(cylinder.findtext('length'))/2
            if roll != 0 or pitch != 0 or not z-half < 0 < z+half:
                raise ValueError('Expected vertical piles crossing the design waterline')
            piles.append((x, y, float(cylinder.findtext('radius'))))
    if len(piles) != 4:
        raise ValueError('Expected four platform piles')
    light, overhead = simplified_platform(original)
    farm = ET.Element('sdf', version='1.9')
    model = ET.SubElement(farm, 'model', name=FARM)
    ET.SubElement(model, 'static').text = 'true'
    positions, footprints, centers = [], [], []
    detailed = (ROWS-1, 0)
    for row in range(ROWS):
        centers.append([])
        for col in range(COLS):
            x, y = cx+(col-(COLS-1)/2)*DX, cy+(row-(ROWS-1)/2)*DY
            if not square.covers(box(x-30, y-17.5, x+30, y+17.5)):
                raise ValueError('Platform outside the 1 km boundary')
            variant = SOURCE if (row, col) == detailed else LIGHT
            centers[row].append((x, y))
            name = f'pv_r{row+1:02d}_c{col+1:02d}'
            if variant == SOURCE:
                add_include(model, variant, name, (x, y, 0., 0., 0., 0.))
            positions.append({'name': name, 'center_enu_m': [x, y], 'model': variant})
            footprints.extend(Point(x+px, y+py).buffer(r+.05, quad_segs=8) for px, py, r in piles)
    tiles = list(tile_layout(centers, detailed, DX, DY))
    visual_files, visual_stats = add_tiles(model, source, original_link, FARM, tiles)
    collision_files, collision_stats = add_tile_collisions(model, tiles, original_link, overhead)
    # Do not change the historical wind asset or rerun its terrain placement.
    old_farm = ET.parse(models/'ccs_wind_farm/model.sdf').getroot()
    hills = old_farm.find('model')
    hills.set('name', HILLS)
    for i in hills.findall('include'):
        if not i.findtext('name').startswith('wind_hill_'):
            hills.remove(i)
    hill_layout = json.loads((models/'ccs_wind_farm/layout.json').read_text())
    hill_sites = [p for p in hill_layout['placements'] if p['kind'] == 'hill']
    if len(hill_sites) != 4 or len(hills.findall('include')) != 4:
        raise ValueError('Expected four existing hill turbines')
    for hill in hill_sites:
        x, y, *_ = hill['pose_enu_m_rad']
        footprints.append(Point(x, y).buffer(hill['foundation_radius_m']+.05, quad_segs=32))
    files = {
        f'{LIGHT}/model.sdf': xml_bytes(light),
        f'{LIGHT}/model.config': config_bytes('Offshore PV platform (array)',
                                              'Static collisions; render tiles are supplied by CCS Offshore PV Farm.'),
        f'{LIGHT}/meshes/overhead_collision.obj': overhead.obj().encode(),
        f'{FARM}/model.sdf': xml_bytes(farm),
        f'{FARM}/model.config': config_bytes('CCS Offshore PV Farm', '1 km square, 294 fixed platforms.'),
        f'{HILLS}/model.sdf': xml_bytes(old_farm),
        f'{HILLS}/model.config': config_bytes('CCS Hill Wind Farm', 'Four original hill turbines and bases.'),
        **visual_files,
        **collision_files,
    }
    layout = {
        'center_wgs84_lon_lat_deg': [LON, LAT], 'datum_wgs84_lon_lat_alt': list(center_ref),
        'center_enu_m': [cx, cy], 'side_m': SIDE_M, 'region_enu_geojson': mapping(square),
        'grid_columns_rows': [COLS, ROWS], 'pitch_xy_m': [DX, DY],
        'platform_nominal_extent_xy_m': [60., 35.], 'platform_count': len(positions),
        'placements': positions, 'hill_turbines': hill_sites, 'waterline_piles': len(positions)*len(piles),
        'model_sdf_sha256': sha256(files[f'{FARM}/model.sdf']),
        'source_sha256': {name: sha256((source/name).read_bytes()) for name in sorted(PAYLOAD)},
        'render_tiles': visual_stats,
        'collision_tiles': collision_stats,
        'navigation_footprint_enu_geojson': mapping(shapely.union_all(footprints)),
        'policy': 'Fixed-pile simulation model. Navigation marks only piles and preserved hill footings; '
                  'under-platform clearance and vessel air draft must be evaluated separately.',
    }
    files[f'{FARM}/layout.json'] = (json.dumps(layout, indent=2)+'\n').encode()
    return files, layout


def generate(models=MODELS, output=None, world=WORLD):
    models, output = Path(models), Path(output) if output else Path(models)
    files, layout = artifacts(models, world)
    for name, data in files.items():
        path = output/name
        if path.exists() and path.read_bytes() != data:
            raise ValueError(f'Refusing to overwrite different generated output: {path}; use a clean output directory')
    for name, data in files.items():
        path = output/name
        if not path.exists():
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(data)
    return layout


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--archive', type=Path, help='Import the original package once (read-only)')
    args = parser.parse_args()
    if args.archive:
        install_archive(args.archive, MODELS)
    result = generate()
    print(f"ENU center: {result['center_enu_m']}; platforms: {result['platform_count']}; "
          f"collision tiles: {result['collision_tiles']['active_tile_links']}")
