"""Bake visual-only turbine variants; original SDF physics and blade size survive.

No runtime Python, textures, new links, lights, sensors or controllers are needed.
Small service details are batched by material, not individual Gazebo entities.
"""
from collections import defaultdict
import copy
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


OFFSHORE = 'iea15_offshore_turbine_detailed'
HILL = 'iea15_hill_turbine_detailed'
TAU = 2*math.pi


def add(a, b):
    return tuple(x+y for x, y in zip(a, b))


def sub(a, b):
    return tuple(x-y for x, y in zip(a, b))


def mul(a, k):
    return tuple(x*k for x in a)


def dot(a, b):
    return sum(x*y for x, y in zip(a, b))


def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def unit(a):
    length = math.sqrt(dot(a, a))
    return mul(a, 1/length) if length > 1e-14 else (0., 0., 1.)


class Mesh:
    def __init__(self):
        self.triangles = []

    def triangle(self, a, b, c, normals=None):
        normal = cross(sub(b, a), sub(c, a))
        if dot(normal, normal) < 1e-20:
            return
        normals = normals or (unit(normal),)*3
        self.triangles.append(((a, b, c), tuple(unit(n) for n in normals)))

    def quad(self, a, b, c, d, normals=None):
        self.triangle(a, b, c, None if normals is None else normals[:3])
        self.triangle(a, c, d, None if normals is None else (normals[0], normals[2], normals[3]))

    def box(self, center, size):
        x, y, z = (s/2 for s in size)
        v = [add(center, p) for p in ((-x, -y, -z), (x, -y, -z), (x, y, -z), (-x, y, -z),
                                     (-x, -y, z), (x, -y, z), (x, y, z), (-x, y, z))]
        for face in ((0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4),
                     (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)):
            self.quad(*(v[i] for i in face))

    def lathe(self, origin, axis, rings, segments=64):
        """Axial positions/radii, with smooth side normals and hard end caps."""
        axis = unit(axis)
        u = unit(cross(axis, (1., 0., 0.) if abs(axis[0]) < .9 else (0., 1., 0.)))
        v = cross(axis, u)
        normals, points = [], []
        for j, (distance, radius) in enumerate(rings):
            lo, hi = max(0, j-1), min(len(rings)-1, j+1)
            slope = (rings[lo][1]-rings[hi][1])/(rings[hi][0]-rings[lo][0])
            pn, nn = [], []
            for i in range(segments):
                radial = add(mul(u, math.cos(TAU*i/segments)), mul(v, math.sin(TAU*i/segments)))
                pn.append(add(origin, add(mul(axis, distance), mul(radial, radius))))
                nn.append(unit(add(radial, mul(axis, slope))))
            points.append(pn)
            normals.append(nn)
        for j in range(len(points)-1):
            for i in range(segments):
                k = (i+1) % segments
                self.quad(points[j][i], points[j][k], points[j+1][k], points[j+1][i],
                          (normals[j][i], normals[j][k], normals[j+1][k], normals[j+1][i]))
        for row, sign in ((0, -1), (-1, 1)):
            center = add(origin, mul(axis, rings[row][0]))
            for i in range(segments):
                a, b = points[row][i], points[row][(i+1) % segments]
                if sign < 0:
                    a, b = b, a
                self.triangle(center, a, b, (mul(axis, sign),)*3)

    def beam(self, start, end, radius=.04, segments=6):
        length = math.sqrt(dot(sub(end, start), sub(end, start)))
        self.lathe(start, sub(end, start), ((0., radius), (length, radius)), segments)

    def annulus(self, inner, outer, bottom, top, segments=64):
        for i in range(segments):
            a, b = TAU*i/segments, TAU*(i+1)/segments
            point = lambda r, t, z: (r*math.cos(t), r*math.sin(t), z)
            self.quad(point(inner, a, top), point(outer, a, top),
                      point(outer, b, top), point(inner, b, top))
            self.quad(point(inner, b, bottom), point(outer, b, bottom),
                      point(outer, a, bottom), point(inner, a, bottom))
            self.quad(point(outer, a, bottom), point(outer, b, bottom),
                      point(outer, b, top), point(outer, a, top))
            self.quad(point(inner, b, bottom), point(inner, a, bottom),
                      point(inner, a, top), point(inner, b, top))

    def torus(self, center, radius, tube=.035, segments=48, sides=6):
        def point(a, b):
            normal = (math.cos(a)*math.cos(b), math.sin(a)*math.cos(b), math.sin(b))
            p = add(center, ((radius+tube*math.cos(b))*math.cos(a),
                             (radius+tube*math.cos(b))*math.sin(a), tube*math.sin(b)))
            return p, normal
        for i in range(segments):
            for j in range(sides):
                pairs = [point(TAU*a/segments, TAU*b/sides) for a, b in
                         ((i, j), (i+1, j), (i+1, j+1), (i, j+1))]
                self.quad(*(p[0] for p in pairs), normals=tuple(p[1] for p in pairs))

    def patch(self, radius, angle, width, bottom, top, segments=12):
        for i in range(segments):
            a, b = angle-width/2+width*i/segments, angle-width/2+width*(i+1)/segments
            point = lambda t, z: (radius(z)*math.cos(t), radius(z)*math.sin(t), z)
            self.quad(point(a, bottom), point(b, bottom), point(b, top), point(a, top))

    def obj(self):
        rows = ['# Generated visual-only CCS turbine; metre units; explicit normals.']
        vertices = [p for triangle, _ in self.triangles for p in triangle]
        normals = [n for _, triple in self.triangles for n in triple]
        rows += ['v '+' '.join(f'{c:.8f}' for c in p) for p in vertices]
        rows += ['vn '+' '.join(f'{c:.8f}' for c in n) for n in normals]
        rows += [f'f {i}//{i} {i+1}//{i+1} {i+2}//{i+2}' for i in range(1, len(vertices)+1, 3)]
        return '\n'.join(rows)+'\n'


def source_triangles(path):
    vertices, triangles = [], []
    for line in Path(path).read_text().splitlines():
        p = line.split()
        if p and p[0] == 'v':
            vertices.append(tuple(map(float, p[1:4])))
        elif p and p[0] == 'f':
            ids = [int(s.split('/')[0])-1 for s in p[1:]]
            triangles.extend(tuple(vertices[j] for j in (ids[0], ids[i], ids[i+1]))
                             for i in range(1, len(ids)-1))
    return triangles


def smooth_blades(path):
    """Keep all three source blade surfaces, replacing only the faceted hub."""
    triangles = source_triangles(path)
    parent = {}

    def find(p):
        parent.setdefault(p, p)
        while parent[p] != p:
            parent[p] = parent[parent[p]]
            p = parent[p]
        return p

    for tri in triangles:
        for p in tri[1:]:
            parent[find(p)] = find(tri[0])
    components = defaultdict(list)
    for tri in triangles:
        components[find(tri[0])].append(tri)
    blades = [group for group in components.values()
              if max(math.hypot(p[1], p[2]) for tri in group for p in tri) > 10.]
    if len(blades) != 3:
        raise ValueError('Expected three separate source blade surfaces')
    mesh = Mesh()
    for group in blades:
        adjacency = defaultdict(list)
        for a, b, c in group:
            n = cross(sub(b, a), sub(c, a))
            for p in (a, b, c):
                adjacency[p].append(n)
        for a, b, c in group:
            face = unit(cross(sub(b, a), sub(c, a)))
            normals = []
            for p in (a, b, c):
                neighbors = [n for n in adjacency[p] if dot(unit(n), face) > math.cos(math.radians(65))]
                normals.append(unit(tuple(sum(n[j] for n in neighbors) for j in range(3))))
            mesh.triangle(a, b, c, normals)
    return mesh


def nacelle(shell, steel, dark, red):
    # Chamfered housing stays within the original 16 x 7.6 x 7.2 m body envelope.
    profile = ((3.2, -3.6), (3.8, -3.), (3.8, 3.), (3.2, 3.6),
               (-3.2, 3.6), (-3.8, 3.), (-3.8, -3.), (-3.2, -3.6))
    rings = [[(x, y*scale, 148.6+z*scale) for y, z in profile]
             for x, scale in ((-10., .8), (-9.3, 1.), (5.25, 1.), (6., .78))]
    for left, right in zip(rings, rings[1:]):
        for i in range(8):
            k = (i+1) % 8
            shell.quad(left[i], left[k], right[k], right[i])
    for ring, x, sign in ((rings[0], -10., -1), (rings[-1], 6., 1)):
        for i in range(8):
            a, b = ring[i], ring[(i+1) % 8]
            shell.triangle((x, 0., 148.6), *( (b, a) if sign < 0 else (a, b)))
    # Visible shaft fairing closes the old housing-to-hub gap at the unchanged 6-degree axis.
    tilt = .104720
    steel.lathe((11.288, 0., 150.), (math.cos(tilt), 0., math.sin(tilt)),
                ((-5.5, 1.8), (-3., 2.35)), 32)
    for side in (-1, 1):
        dark.box((-4., side*3.807, 148.3), (4.8, .055, 2.5))
        for i in range(13):
            shell.box((-6.2+i*.36, side*3.85, 148.3), (.065, .075, 2.35))
    for x in (-5.5, -1.8):
        steel.box((x, 0., 152.225), (2.3, 2.7, .08))
        shell.box((x, 0., 152.28), (2.15, 2.55, .04))
    # Roof service rail and passive beacon/sensor silhouettes: no extra light systems.
    for side in (-1, 1):
        steel.beam((-7.5, side*2.4, 153.1), (-.5, side*2.4, 153.1), .045)
        for x in (-7.5, -4., -.5):
            steel.beam((x, side*2.4, 152.2), (x, side*2.4, 153.1), .045)
    steel.beam((-8.1, 0., 152.2), (-8.1, 0., 154.2), .055)
    dark.beam((-8.6, 0., 154.1), (-7.6, 0., 154.1), .03)
    dark.beam((-8.1, -.45, 154.15), (-8.1, .45, 154.15), .03)
    steel.lathe((-6.7, 0., 152.2), (0., 0., 1.), ((0., .23), (.3, .23)), 12)
    red.lathe((-6.7, 0., 152.5), (0., 0., 1.), ((0., .19), (.25, .19), (.38, .06)), 16)


def fixed_meshes(offshore):
    meshes = {name: Mesh() for name in ('shell', 'foundation', 'steel', 'dark', 'door', 'red', 'patina')}
    shell, foundation, steel, dark, door, red, patina = (meshes[n] for n in meshes)
    foundation.lathe((0., 0., 0.), (0., 0., 1.), ((-8., 5.), (15., 5.)), 64)
    shell.lathe((0., 0., 0.), (0., 0., 1.), ((15., 5.), (145., 3.25)), 64)
    radius = lambda z: 5.-max(0., z-15.)*1.75/130.
    for z in (15.3, 47., 79., 111., 144.6):
        steel.torus((0., 0., z), radius(z)+.012, .024, 64, 4)
    nacelle(shell, steel, dark, red)
    z = 17.5 if offshore else 2.0
    dark.patch(lambda h: radius(h)+.018, 0., .265, z-1.2, z+1.2)
    door.patch(lambda h: radius(h)+.027, 0., .225, z-1.08, z+1.08)
    steel.beam((radius(z)+.07, -.35, z), (radius(z)+.07, -.35, z+.24), .025)
    if offshore:
        steel.annulus(4.88, 6.15, 14.7, 14.94)
        for h in (15.47, 16.05):
            steel.torus((0., 0., h), 6.03, .04, 48, 6)
        for i in range(24):
            a = TAU*i/24
            x, y = 6.03*math.cos(a), 6.03*math.sin(a)
            steel.beam((x, y, 14.94), (x, y, 16.05), .04)
        for i in range(8):
            a = TAU*i/8
            steel.beam((4.98*math.cos(a), 4.98*math.sin(a), 11.8),
                       (5.95*math.cos(a), 5.95*math.sin(a), 14.7), .07)
        for y in (-.43, .43):
            steel.beam((5.23, y, 3.5), (5.23, y, 15.4), .05)
        for i in range(35):
            h = 3.65+i*.325
            steel.beam((5.23, -.43, h), (5.23, .43, h), .028)
        for h in (6., 8., 10., 12., 14.):
            steel.torus((5.26, 0., h), .58, .025, 20, 5)
        for a in (-math.pi/2, 0., math.pi/2):
            x, y = 5.26+.58*math.cos(a), .58*math.sin(a)
            steel.beam((x, y, 6.), (x, y, 14.), .023)
        # The waterline coating remains inside the existing map's 5.05 m margin.
        patina.lathe((0., 0., 0.), (0., 0., 1.), ((-.45, 5.008), (.45, 5.008)), 64)
        for i in range(7):
            a = .55+i*.77
            patina.patch(lambda h: 5.012, a, .009, .5, 1.15+(i % 3)*.26, 2)
    else:
        steel.box((5.4, 0., .34), (1., 1.7, .25))
        steel.box((5.85, 0., .12), (.8, 1.9, .2))
    return meshes


MATERIALS = {
    'shell': ((.84, .87, .88), .02, .58),
    'foundation': ((.9, .65, .09), .03, .74),
    'steel': ((.36, .4, .43), .65, .48),
    'dark': ((.095, .125, .14), .1, .83),
    'door': ((.25, .33, .4), .18, .65),
    'red': ((.64, .025, .012), .03, .35),
    'patina': ((.22, .28, .20), 0., .98),
    'blades': ((.88, .9, .9), .015, .6),
    'spinner': ((.83, .86, .87), .025, .5),
}


def mesh_visual(link, name, uri, material_name, color=None):
    visual = ET.SubElement(link, 'visual', name=name)
    ET.SubElement(visual, 'cast_shadows').text = 'true'
    mesh = ET.SubElement(ET.SubElement(visual, 'geometry'), 'mesh')
    ET.SubElement(mesh, 'uri').text = uri
    ET.SubElement(mesh, 'scale').text = '1 1 1'
    material = ET.SubElement(visual, 'material')
    rgb, metalness, roughness = MATERIALS[material_name]
    rgb = color or rgb
    for tag in ('ambient', 'diffuse'):
        ET.SubElement(material, tag).text = ' '.join(str(c) for c in (*rgb, 1.))
    ET.SubElement(material, 'specular').text = '.08 .08 .08 1'
    metal = ET.SubElement(ET.SubElement(material, 'pbr'), 'metal')
    ET.SubElement(metal, 'metalness').text = str(metalness)
    ET.SubElement(metal, 'roughness').text = str(roughness)
    if material_name == 'red':
        ET.SubElement(material, 'emissive').text = '.12 .003 .001 1'


def physical_signature(root):
    """Exclude only visuals and cosmetic model name, retaining all physical XML."""
    root = copy.deepcopy(root)
    root.find('model').set('name', 'source_turbine')
    for link in root.findall('model/link'):
        for visual in link.findall('visual'):
            link.remove(visual)
    for element in root.iter():
        if element.text is not None:
            element.text = element.text.strip() or None
        element.tail = None
    return ET.tostring(root)


def artifacts(source):
    source = Path(source)
    original = ET.parse(source/'model.sdf').getroot()
    shared = f'model://{OFFSHORE}/meshes/'
    result, statistics = {}, {}
    blades = smooth_blades(source/'meshes/rotor.obj')
    spinner = Mesh()
    spinner.lathe((0., 0., 0.), (1., 0., 0.),
                  ((-3.2, 2.8), (-2.9, 3.97), (1.7, 3.75), (2.8, 2.6), (3.4, .1)), 48)
    for name, mesh in (('blades', blades), ('spinner', spinner)):
        result[f'{OFFSHORE}/meshes/{name}.obj'] = mesh.obj()
        statistics[name] = len(mesh.triangles)
    for variant, offshore in ((OFFSHORE, True), (HILL, False)):
        root = copy.deepcopy(original)
        model = root.find('model')
        model.set('name', variant)
        for link in model.findall('link'):
            for visual in link.findall('visual'):
                link.remove(visual)
        support, rotor = model.find("link[@name='support']"), model.find("link[@name='rotor']")
        meshes = fixed_meshes(offshore)
        for name, mesh in meshes.items():
            if not mesh.triangles:
                continue
            filename = f'{"sea" if offshore else "hill"}_{name}.obj'
            result[f'{OFFSHORE}/meshes/{filename}'] = mesh.obj()
            mesh_visual(support, 'appearance_'+name, shared+filename, name,
                        (.84, .87, .88) if name == 'foundation' and not offshore else None)
        for name in ('blades', 'spinner'):
            mesh_visual(rotor, 'appearance_'+name, shared+name+'.obj', name)
        if physical_signature(root) != physical_signature(original):
            raise ValueError('Appearance adapter changed non-visual turbine state')
        ET.indent(root, space='  ')
        result[f'{variant}/model.sdf'] = ET.tostring(root, encoding='unicode', xml_declaration=True)+'\n'
        config = ET.Element('model')
        ET.SubElement(config, 'name').text = 'CCS detailed turbine: '+('offshore' if offshore else 'hill')
        ET.SubElement(config, 'version').text = '1.0'
        ET.SubElement(config, 'sdf', version='1.9').text = 'model.sdf'
        ET.SubElement(config, 'description').text = (
            'Visual-only derivative. Original anchors, collision, inertia, joint and controller retained. '
            'Service details are illustrative, not a surveyed or certified engineering design.')
        ET.indent(config, space='  ')
        result[f'{variant}/model.config'] = ET.tostring(config, encoding='unicode', xml_declaration=True)+'\n'
        statistics[variant] = {
            'support_triangles': sum(len(m.triangles) for m in meshes.values()),
            'rotor_triangles': len(blades.triangles)+len(spinner.triangles),
            'support_visuals': len(support.findall('visual')),
            'rotor_visuals': len(rotor.findall('visual')),
        }
    report = {
        'version': 1, 'source_model': 'iea15_offshore_turbine',
        'source_sha256': {str(p.relative_to(source)): hashlib.sha256(p.read_bytes()).hexdigest()
                          for p in sorted(source.rglob('*')) if p.is_file()},
        'generated_sha256': {p: hashlib.sha256(text.encode()).hexdigest() for p, text in sorted(result.items())},
        'statistics': statistics,
        'policy': 'Appearance only. Source assets, physical parameters, blade surfaces, rotor envelope and '
                  'speed unchanged. New platforms, railings, ladder, coatings and service fixtures are visual-only.',
    }
    result[f'{OFFSHORE}/appearance.json'] = json.dumps(report, indent=2)+'\n'
    return result, report


def generate(source, output_root):
    output_root = Path(output_root)
    result, report = artifacts(source)
    manifest = output_root/OFFSHORE/'appearance.json'
    managed = json.loads(manifest.read_text())['generated_sha256'] if manifest.is_file() else {}
    for name, expected in managed.items():
        path = output_root/name
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise ValueError(f'Generated appearance was modified outside this generator: {path}')
    for name, text in result.items():
        path = output_root/name
        if path.exists() and name not in managed and path != manifest and path.read_text() != text:
            raise ValueError(f'Refusing to overwrite an unfamiliar turbine asset: {path}')
    for name, text in result.items():
        path = output_root/name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text)
    return report
