#!/usr/bin/env python3
"""Offline conversion of the two user-provided STEP archives, never needed at runtime.

Requires cadquery-ocp==7.7.2. All exported vertices are in metres. The original
archives stay external; source hashes and geometric decisions accompany the meshes.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import struct
import tempfile
from zipfile import ZipFile

from OCP.BRep import BRep_Tool
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.Bnd import Bnd_Box
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPControl import STEPControl_Reader
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED, TopAbs_SHELL, TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS


def subshapes(shape, kind, avoid=None):
    explorer = TopExp_Explorer(shape, kind) if avoid is None else TopExp_Explorer(shape, kind, avoid)
    while explorer.More():
        yield explorer.Current()
        explorer.Next()


def bounds(shape):
    box = Bnd_Box()
    BRepBndLib.AddOptimal_s(shape, box, False, False)
    return box.Get()


def load_archive(path, member):
    with ZipFile(path) as archive:
        data = archive.read(member)
    if b'SI_UNIT(.MILLI.,.METRE.)' not in data:
        raise ValueError('Expected STEP coordinates in millimetres')
    with tempfile.TemporaryDirectory(prefix='vessel-cad-') as directory:
        source = Path(directory) / 'source.stp'
        source.write_bytes(data)
        reader = STEPControl_Reader()
        if reader.ReadFile(str(source)) != IFSelect_RetDone:
            raise RuntimeError(f'Cannot read {member}')
        if reader.TransferRoots() == 0:
            raise RuntimeError(f'No transferable geometry in {member}')
        shape = reader.OneShape()
    mesher = BRepMesh_IncrementalMesh(shape, 0.2, False, 0.4, True)
    if not mesher.IsDone():
        raise RuntimeError('OpenCASCADE tessellation failed')
    provenance = {
        'archive': Path(path).name,
        'archive_sha256': hashlib.sha256(Path(path).read_bytes()).hexdigest(),
        'step_member': member,
        'step_sha256': hashlib.sha256(data).hexdigest(),
        'cad_bounds_mm': list(bounds(shape)),
    }
    return shape, provenance


def face_triangles(shape):
    for raw_face in subshapes(shape, TopAbs_FACE):
        face = TopoDS.Face_s(raw_face)
        location = TopLoc_Location()
        mesh = BRep_Tool.Triangulation_s(face, location)
        if mesh is None:
            raise RuntimeError('A CAD face has no triangulation; refusing an incomplete export')
        transform = location.Transformation()
        points = {}
        for i in range(1, mesh.NbNodes() + 1):
            p = mesh.Node(i).Transformed(transform)
            points[i] = (p.X(), p.Y(), p.Z())
        for i in range(1, mesh.NbTriangles() + 1):
            indices = list(mesh.Triangle(i).Get())
            if face.Orientation() == TopAbs_REVERSED:
                indices[1], indices[2] = indices[2], indices[1]
            yield face, [points[j] for j in indices]


def clip(triangle, height, above):
    """Clip rendered geometry at the virtual hinge; keep both halves without overlap."""
    polygon = []
    previous = triangle[-1]
    for current in triangle:
        current_inside = current[2] >= height if above else current[2] <= height
        previous_inside = previous[2] >= height if above else previous[2] <= height
        if current_inside != previous_inside:
            t = (height - previous[2]) / (current[2] - previous[2])
            polygon.append(tuple(previous[k] + t * (current[k] - previous[k]) for k in range(3)))
        if current_inside:
            polygon.append(current)
        previous = current
    return [[polygon[0], polygon[i], polygon[i + 1]] for i in range(1, len(polygon) - 1)]


def metres(triangle, origin):
    return [tuple((point[k] - origin[k]) * 0.001 for k in range(3)) for point in triangle]


def write_stl(path, triangles):
    records = []
    lo, hi = [math.inf] * 3, [-math.inf] * 3
    for a, b, c in triangles:
        u, v = [b[k] - a[k] for k in range(3)], [c[k] - a[k] for k in range(3)]
        normal = [u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0]]
        length = math.sqrt(sum(n*n for n in normal))
        if length < 1e-18:
            continue
        normal = [n / length for n in normal]
        records.append(struct.pack('<12fH', *normal, *a, *b, *c, 0))
        for point in [a, b, c]:
            for k in range(3):
                lo[k], hi[k] = min(lo[k], point[k]), max(hi[k], point[k])
    if not records:
        raise RuntimeError(f'Empty mesh: {path}')
    header = b'USV CAD visual mesh; coordinates in metres; see cad_assets.json'
    path.write_bytes(header.ljust(80, b'\0') + struct.pack('<I', len(records)) + b''.join(records))
    return {'triangles': len(records), 'bounds_m': lo + hi,
            'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}


def navigation_mounts(shape, origin, hinge):
    """Find the four small lamps on the two paired crossbars, not other fittings.

    Keep the mounting pads and beams; remove the old bulb/lens/cap assemblies so
    the new independently controlled lamps do not overlap decorative CAD lamps.
    """
    solids = list(subshapes(shape, TopAbs_SOLID))
    shells = list(subshapes(shape, TopAbs_SHELL, TopAbs_SOLID))
    bases = []
    for solid in solids:
        b = bounds(solid)
        if (10 < b[3] - b[0] < 15 and 10 < b[4] - b[1] < 15
                and 1 < b[5] - b[2] < 1.3
                and 90 < abs((b[1] + b[4]) / 2) < 110 and b[2] > 200):
            bases.append(b)
    tiers = sorted({round(b[2], 3) for b in bases})
    if len(bases) != 4 or len(tiers) != 2:
        raise ValueError('Cannot uniquely identify the four paired crossbar lights')
    anchors = {}
    removed_solids, removed_shells = [], []
    for base in bases:
        centre = ((base[0] + base[3]) / 2, (base[1] + base[4]) / 2, base[2])
        side = 'left' if centre[1] > origin[1] else 'right'
        tier = 'front' if round(centre[2], 3) == tiers[-1] else 'rear'
        name = side + '_' + tier
        if name in anchors:
            raise ValueError('Crossbar light layout is not two left/right pairs')
        anchors[name] = [(centre[k] - (origin[0], origin[1], hinge)[k]) * 0.001 for k in range(3)]

        def belongs(component):
            b = bounds(component)
            return (b[0] >= centre[0] - 7 and b[3] <= centre[0] + 7
                    and b[1] >= centre[1] - 7 and b[4] <= centre[1] + 7
                    and b[2] >= centre[2] - 0.001 and b[5] <= centre[2] + 16)

        removed_solids.extend(s for s in solids if belongs(s))
        removed_shells.extend(s for s in shells if belongs(s))
    if len(removed_solids) != 24 or len(removed_shells) != 4:
        raise ValueError('Unexpected old crossbar-lamp geometry; inspect CAD before filtering')
    excluded = {}
    for component in removed_solids + removed_shells:
        for face in subshapes(component, TopAbs_FACE):
            excluded.setdefault(face.HashCode(2147483647), []).append(face)
    return anchors, excluded


def extra_navigation_mounts(shape, origin, hinge):
    """Identify every remaining lamp below the preserved white top beacon."""
    solids = list(subshapes(shape, TopAbs_SOLID))
    shells = list(subshapes(shape, TopAbs_SHELL, TopAbs_SOLID))
    aft, forward, side = [], [], []
    for solid in solids:
        b = bounds(solid)
        x, y = (b[0] + b[3]) / 2, (b[1] + b[4]) / 2
        if (10 < b[3] - b[0] < 15 and 10 < b[4] - b[1] < 15
                and 1 < b[5] - b[2] < 1.3 and 150 < b[2] < 400):
            if abs(y) < 1 and x < origin[0] - 10:
                aft.append(b)
            elif y < -90 and b[2] < 200:
                side.append(b)
        if (15 < b[3] - b[0] < 20 and 15 < b[4] - b[1] < 20
                and 2.5 < b[5] - b[2] < 3.5 and abs(y) < 1
                and x > origin[0] + 10 and 300 < b[2] < 530):
            forward.append(b)
    if len(aft) != 3 or len(forward) != 3 or len(side) != 1:
        raise ValueError('Expected one lower side light and two three-light centreline stacks')
    groups = [('right_lower', side[0], 7.0, 16.0, 1.0)]
    for prefix, bases in [('aft', aft), ('forward', forward)]:
        for tier, base in zip(['lower', 'middle', 'upper'], sorted(bases, key=lambda b: b[2])):
            groups.append((prefix + '_' + tier, base,
                           7.0 if prefix == 'aft' else 10.0,
                           16.0 if prefix == 'aft' else 26.0, 0.8))
    anchors, excluded = {}, {}
    total_solids, total_shells = 0, 0
    for name, base, half_width, lamp_height, size_factor in groups:
        centre = ((base[0] + base[3]) / 2, (base[1] + base[4]) / 2, base[2])
        low = (centre[0] - half_width, centre[1] - half_width, centre[2] - 0.001)
        high = (centre[0] + half_width, centre[1] + half_width, centre[2] + lamp_height)

        def belongs(component):
            b = bounds(component)
            return all(b[k] >= low[k] and b[k + 3] <= high[k] for k in range(3))

        removed_solids = [s for s in solids if belongs(s)]
        removed_shells = [s for s in shells if belongs(s)]
        if len(removed_solids) != (6 if name == 'right_lower' else 5):
            raise ValueError(f'Unexpected decorative lamp geometry for {name}')
        total_solids += len(removed_solids)
        total_shells += len(removed_shells)
        pivot = (origin[0], origin[1], hinge)
        anchors[name] = {
            'xyz': [(centre[k] - pivot[k]) * 0.001 for k in range(3)],
            'size_factor': size_factor,
            'source_selection_bounds_m': [(low[k] - pivot[k]) * 0.001 for k in range(3)]
                                         + [(high[k] - pivot[k]) * 0.001 for k in range(3)],
            'removed_solids': len(removed_solids),
            'removed_open_shells': len(removed_shells),
        }
        for component in removed_solids + removed_shells:
            for face in subshapes(component, TopAbs_FACE):
                excluded.setdefault(face.HashCode(2147483647), []).append(face)
    if total_solids != 36 or total_shells != 3:
        raise ValueError('Remaining decorative-lamp surface selection changed')
    return anchors, excluded


def convert_mast(archive, output):
    shape, source = load_archive(archive, 'MastLight/MastLight.stp')
    height = bounds(shape)[5]
    shells = list(subshapes(shape, TopAbs_SHELL, TopAbs_SOLID))
    shaft = [s for s in shells if bounds(s)[5] - bounds(s)[2] > 0.8 * height]
    lens = [s for s in shells if bounds(s)[2] > 0.95 * height
            and bounds(s)[5] < height - 1 and bounds(s)[5] - bounds(s)[2] > 10]
    if len(shaft) != 1 or len(lens) != 1:
        raise ValueError('Cannot uniquely identify mast shaft/top lens; inspect the new CAD first')
    shaft_box = bounds(shaft[0])
    origin = ((shaft_box[0] + shaft_box[3]) / 2, (shaft_box[1] + shaft_box[4]) / 2, 0.0)
    feet = [bounds(s) for s in subshapes(shape, TopAbs_SOLID)
            if bounds(s)[2] < 1.0 and 100 < bounds(s)[5] < 150]
    if not feet:
        raise ValueError('Cannot identify the fixed lower mounting bracket')
    hinge = max(b[5] for b in feet)
    lens_box = bounds(lens[0])
    lens_origin = tuple((lens_box[k] + lens_box[k + 3]) / 2 for k in range(3))
    lens_faces = list(subshapes(lens[0], TopAbs_FACE))
    mounts, removed_faces = navigation_mounts(shape, origin, hinge)
    extra_mounts, extra_faces = extra_navigation_mounts(shape, origin, hinge)
    for key, faces in extra_faces.items():
        removed_faces.setdefault(key, []).extend(faces)
    groups = {'mast_base': [], 'mast_body': [], 'mast_lens': []}
    for face, triangle in face_triangles(shape):
        if any(face.IsSame(lens_face) for lens_face in lens_faces):
            groups['mast_lens'].append(metres(triangle, lens_origin))
            continue
        if any(face.IsSame(old_face) for old_face in removed_faces.get(face.HashCode(2147483647), [])):
            continue
        for triangle_part in clip(triangle, hinge, above=False):
            groups['mast_base'].append(metres(triangle_part, origin))
        for triangle_part in clip(triangle, hinge, above=True):
            groups['mast_body'].append(metres(triangle_part, (origin[0], origin[1], hinge)))
    result = {
        'source': source,
        'native_height_m': height * 0.001,
        'hinge_height_m': hinge * 0.001,
        'mount_origin_cad_mm': list(origin),
        'lens_origin_from_hinge_m': [(lens_origin[k] - (origin[0], origin[1], hinge)[k]) * 0.001
                                     for k in range(3)],
        'hinge_note': 'Virtual folding joint at top of fixed clamp; not an authored CAD mechanism.',
        'navigation_light_mounts_from_hinge_m': mounts,
        'removed_crossbar_lamps': {'solids': 24, 'open_shells': 4, 'lamps': 4},
        'extra_light_order': list(extra_mounts),
        'extra_light_mounts_from_hinge_m': extra_mounts,
        'removed_extra_lamps': {'solids': 36, 'open_shells': 3, 'lamps': 7},
        'meshes': {},
    }
    for name, triangles in groups.items():
        result['meshes'][name + '.stl'] = write_stl(output / (name + '.stl'), triangles)
    return result


def convert_lamp(archive, output):
    shape, source = load_archive(archive, 'Marine navigation light.stp')
    # The STEP includes a closed lamp at y=0 and an open-lid demonstration at
    # y=205.76 mm. Import ONLY the closed state, not both display examples.
    closed = [s for s in subshapes(shape, TopAbs_SOLID)
              if bounds(s)[1] > -90 and bounds(s)[4] < 90]
    lens = [s for s in closed if abs(bounds(s)[2] - 40) < 0.01
            and abs(bounds(s)[5] - 150) < 0.01 and abs(bounds(s)[3] - 60) < 0.01]
    if len(closed) != 6 or len(lens) != 1:
        raise ValueError('Closed navigation lamp/lens selection changed; inspect CAD first')
    lens_faces = list(subshapes(lens[0], TopAbs_FACE))
    groups = {'navigation_lamp_housing': [], 'navigation_lamp_lens': []}
    for solid in closed:
        for face, triangle in face_triangles(solid):
            name = ('navigation_lamp_lens' if any(face.IsSame(f) for f in lens_faces)
                    else 'navigation_lamp_housing')
            groups[name].append(metres(triangle, (0, 0, 0)))
    result = {'source': source, 'selected_state': 'closed lamp centred at CAD y=0',
              'selected_solids': len(closed), 'meshes': {}}
    for name, triangles in groups.items():
        result['meshes'][name + '.stl'] = write_stl(output / (name + '.stl'), triangles)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mast-archive', required=True, type=Path)
    parser.add_argument('--lamp-archive', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    profile = {
        'coordinate_units': 'metre',
        'converter': 'cadquery-ocp 7.7.2 / OpenCASCADE; linear deflection 0.2 mm, angular 0.4 rad',
        'rights': 'User-provided CAD; no licence identified in source archives. Not relicensed by this package.',
        'mast': convert_mast(args.mast_archive, args.output),
        'navigation_lamp': convert_lamp(args.lamp_archive, args.output),
    }
    (args.output / 'cad_assets.json').write_text(json.dumps(profile, indent=2) + '\n')
    print(json.dumps(profile, indent=2))


if __name__ == '__main__':
    main()
