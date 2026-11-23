"""Bake reusable, spatially bounded render tiles for the static CCS PV array."""
import copy
import io
import math
import xml.etree.ElementTree as ET

from PIL import Image

from usv_sim_full.wind_turbine_appearance import Mesh, add, cross, mul, sub, unit


TILE_COLS, TILE_ROWS = 7, 3
MATERIALS = ('piles', 'coating', 'truss', 'pv_front', 'pv_back')
PANELS_PER_STRIP = 52
STRIPS = 12


def rotated_offset(p, rpy):
    a, b, c = p
    (cr, sr), (cp, sp), (cy, sy) = [(math.cos(t), math.sin(t)) for t in rpy]
    b, c = cr*b-sr*c, sr*b+cr*c
    a, c = cp*a+sp*c, -sp*a+cp*c
    return cy*a-sy*b, sy*a+cy*b, c


def cylinder_axis(cylinder, pose):
    half = float(cylinder.findtext('length'))/2
    a, b = (add(pose[:3], rotated_offset((0., 0., z), pose[3:])) for z in (-half, half))
    return a, b, float(cylinder.findtext('radius'))


def chord(mesh, a, b, radius):
    """Open square prism; its tiny end caps are hidden by adjoining steelwork."""
    axis = unit(sub(b, a))
    u = unit(cross(axis, (1., 0., 0.) if abs(axis[0]) < .9 else (0., 1., 0.)))
    v = cross(axis, u)
    offsets = [mul(p, radius) for p in (u, v, mul(u, -1), mul(v, -1))]
    for i, p in enumerate(offsets):
        q = offsets[(i+1) % 4]
        mesh.quad(add(a, p), add(a, q), add(b, q), add(b, p))


def platform_faces(source, link):
    """Build each material's local triangles, preserving pile and panel elevations."""
    structures = {name: Mesh() for name in MATERIALS[:3]}
    for collision in link.findall('collision'):
        name = collision.get('name')
        cylinder = collision.find('geometry/cylinder')
        if cylinder is None:
            if name.startswith('bearing_'):
                pose = tuple(map(float, collision.findtext('pose').split()))
                size = tuple(map(float, collision.findtext('geometry/box/size').split()))
                if any(pose[3:]):
                    raise ValueError(f'Expected an axis-aligned PV bearing: {name}')
                structures['truss'].box(pose[:3], size)
            continue
        pose = tuple(map(float, collision.findtext('pose').split()))
        a, b, radius = cylinder_axis(cylinder, pose)
        if name in {f'pile_{i}' for i in range(1, 5)}:
            structures['piles'].beam(a, (*a[:2], -1.), radius, segments=12)
            structures['piles'].beam((*b[:2], 4.553), b, radius, segments=12)
            structures['coating'].beam((*a[:2], -1.), (*b[:2], 4.553), radius+.001, segments=12)
        elif name.startswith('pile_collar_'):
            structures['piles'].beam(a, b, radius, segments=12)
        elif (name.startswith(('lower_chord_', 'upper_chord_', 'post_'))
              or (name.startswith('web_') and (abs(pose[0]) >= 29.7 or abs(pose[1]) >= 17.4))):
            chord(structures['truss'], a, b, radius)
    faces = {name: [(p, n, (None,)*3) for p, n in mesh.triangles]
             for name, mesh in structures.items()}
    for name in MATERIALS[3:]:
        mesh = ET.parse(source/'meshes'/f'{name}.dae').find('.//{*}mesh')
        sources = {el.get('id'): tuple(map(float, el[0].text.split())) for el in mesh.findall('{*}source')}
        positions = sources['position']
        if len(positions) != STRIPS*PANELS_PER_STRIP*4*3:
            raise ValueError(f'Unexpected {name} source topology')
        vertices = tuple(zip(positions[::3], positions[1::3], positions[2::3]))
        faces[name] = []
        for offset in range(0, len(vertices), PANELS_PER_STRIP*4):
            first = vertices[offset:offset+4]
            last = vertices[offset+(PANELS_PER_STRIP-1)*4:offset+PANELS_PER_STRIP*4]
            # Join only within a row of modules, never across the 12 distinct strips.
            # Keep the original back-face winding and V orientation.
            quad = (first[0], last[1], last[2], first[3])
            uv = ((0., 0.), (1., 0.), (1., 1.), (0., 1.)) if name == 'pv_front' else (
                  (0., 1.), (1., 1.), (1., 0.), (0., 0.))
            normal = unit(cross(sub(quad[1], quad[0]), sub(quad[2], quad[0])))
            for face in ((0, 1, 2), (0, 2, 3)):
                faces[name].append((tuple(quad[i] for i in face), (normal,)*3, tuple(uv[i] for i in face)))
    return faces


def panel_atlas(source):
    """Put 52 copies and their small seams in one UV span (no sampler wrapping)."""
    width, height = 4096, 160
    atlas = Image.new('RGB', (width, height), '#444b53')
    with Image.open(source) as original:
        rgb = original.convert('RGB')
        resized = {}
        for i in range(PANELS_PER_STRIP):
            lo = round(i*width/PANELS_PER_STRIP)
            hi = round((i+1)*width/PANELS_PER_STRIP)
            gap = 0 if i == PANELS_PER_STRIP-1 else 3
            size = (hi-lo-gap, height)
            if size not in resized:
                resized[size] = rgb.resize(size, getattr(Image, 'Resampling', Image).LANCZOS)
            atlas.paste(resized[size], (lo, 0))
    data = io.BytesIO()
    atlas.save(data, format='PNG')
    return data.getvalue()


def tile_obj(faces, offsets):
    """Compact indexed OBJ. Each file has one material and one renderable submesh."""
    verts, normals, uv = {}, {}, {}

    def index(table, point):
        key = tuple(round(v, 6) or 0. for v in point)
        return table.setdefault(key, len(table)+1)

    indices = []
    for offset in offsets:
        for tri, ns, uvs in faces:
            row = []
            for p, n, tex in zip(tri, ns, uvs):
                v, normal = index(verts, add(p, offset)), index(normals, n)
                texture = '' if tex is None else index(uv, tex)
                row.append(f'{v}/{texture}/{normal}')
            indices.append('f '+' '.join(row))
    rows = ['# Generated tiled CCS PV array; metre units.', 'mtllib tile.mtl', 'usemtl tile']
    for prefix, table in (('v', verts), ('vn', normals), ('vt', uv)):
        rows.extend(prefix+' '+' '.join(f'{v:.6f}' for v in p) for p in table)
    return ('\n'.join(rows+indices)+'\n').encode()


def tile_layout(centers, detailed, dx, dy):
    """Use the same origins and omissions for render geometry and collision."""
    rows, cols = len(centers), len(centers[0])
    if rows % TILE_ROWS or cols % TILE_COLS:
        raise ValueError('PV grid must divide into regular tiles')
    for row in range(0, rows, TILE_ROWS):
        for col in range(0, cols, TILE_COLS):
            skipped = (detailed[0]-row, detailed[1]-col)
            corner = 0 <= skipped[0] < TILE_ROWS and 0 <= skipped[1] < TILE_COLS
            variant = 'detail_corner' if corner else 'repeat'
            name = f'array_tile_{row+1:02d}_{col+1:02d}'
            offsets = [(c*dx, r*dy, 0.) for r in range(TILE_ROWS) for c in range(TILE_COLS)
                       if (r, c) != skipped]
            yield name, variant, centers[row][col], offsets


def add_tiles(farm, source, original_link, farm_name, tiles):
    """Add one static link per tile and pre-bake shared render meshes."""
    mesh_dir = f'{farm_name}/meshes'
    tex_dir = f'{farm_name}/materials/textures'
    files = {f'{mesh_dir}/tile.mtl': b'newmtl tile\nKd 1 1 1\n'}
    for face in ('front', 'back'):
        files[f'{tex_dir}/{face}_strip.png'] = panel_atlas(source/f'materials/textures/module_{face}.png')
    faces = platform_faces(source, original_link)
    for link_name, variant, (x, y), offsets in tiles:
        tile = ET.SubElement(farm, 'link', name=link_name)
        ET.SubElement(tile, 'pose').text = f'{x:.6f} {y:.6f} 0 0 0 0'
        for name in MATERIALS:
            mesh_path = f'{mesh_dir}/{variant}_{name}.obj'
            if mesh_path not in files:
                files[mesh_path] = tile_obj(faces[name], offsets)
            visual = copy.deepcopy(original_link.find(f"visual[@name='{name}']"))
            visual.find('geometry/mesh/uri').text = 'model://'+mesh_path
            ET.SubElement(visual, 'cast_shadows').text = 'false'
            albedo = visual.find('material/pbr/metal/albedo_map')
            if albedo is not None:
                albedo.text = f"model://{tex_dir}/{name.removeprefix('pv_')}_strip.png"
            tile.append(visual)
    stats = {'tile_columns_rows': [TILE_COLS, TILE_ROWS],
             'tiles': len(tiles), 'visuals_per_tile': len(MATERIALS),
             'triangles_per_repeat_platform': sum(map(len, faces.values()))}
    return files, stats
