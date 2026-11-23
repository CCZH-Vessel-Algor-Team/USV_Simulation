"""Bake the chart-derived Tianwan bridge into a static Gazebo scene.

The chart supplies the plan alignment and four bridge-opening marks. The
published spans and widths constrain the model; pier geometry and the road
profile are simulation approximations informed by photographs.
"""
from __future__ import annotations

import bisect
import hashlib
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from . import ccs_bridge_landing as landing


MODEL = 'ccs_tianwan_bridge'
DEFAULT_ENC = Path('/home/cc-liu/Documents/xwechat_files/wxid_q4i5islkwi7f22_399f/msg/file/2026-11/CN441122.000')
DATUM_LON, DATUM_LAT = 119.481403, 34.692120
NORTH_CLIP_Y = 831.0
SOUTH_CLIP_Y = -2501.0
DECK_WIDTH = 34.0
PLATFORM_WIDTH = 38.0
MAIN_SPAN = 125.0
SIDE_SPAN = 70.0
AUX_SPAN = 70.0
APPROACH_SPAN = 40.0
# Shaft length, width, corner chamfer; cap length, width, height.
PIER_SECTIONS = {
    'approach': ((3.8, 4.0, .22), (7.4, 9.6, 1.65)),
    'main': ((7.6, 5.8, .42), (8.8, 9.2, 1.60)),
    'aux': ((5.4, 4.8, .32), (7.8, 9.2, 1.50)),
}
PIER_OFFSET = 8.5
CLEAR_HEIGHT = 21.0
PLATFORM_Y = -1195.0
VISUAL_BATCH_M = 850.0
PIER_BASE_Z = -5.0
END_PIER_INSET = 5.5


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def ecef(lon: float, lat: float, altitude: float = 0.0):
    a = 6378137.0
    e2 = 6.6943799901413165e-3
    lon, lat = math.radians(lon), math.radians(lat)
    n = a/math.sqrt(1-e2*math.sin(lat)**2)
    return ((n+altitude)*math.cos(lat)*math.cos(lon),
            (n+altitude)*math.cos(lat)*math.sin(lon),
            (n*(1-e2)+altitude)*math.sin(lat))


_BASE = ecef(DATUM_LON, DATUM_LAT)


def local_enu(lon: float, lat: float):
    """WGS84 to the same local ENU as the CCS world and raster map."""
    x, y, z = ecef(lon, lat)
    dx, dy, dz = x-_BASE[0], y-_BASE[1], z-_BASE[2]
    lon0, lat0 = math.radians(DATUM_LON), math.radians(DATUM_LAT)
    return (-math.sin(lon0)*dx+math.cos(lon0)*dy,
            -math.sin(lat0)*(math.cos(lon0)*dx+math.sin(lon0)*dy)+math.cos(lat0)*dz)


def _segments(geometry):
    if geometry.is_empty:
        return []
    if geometry.geom_type in ('LineString', 'Polygon'):
        return [geometry]
    return [g for g in getattr(geometry, 'geoms', []) if not g.is_empty]


def extract_chart(enc: Path = DEFAULT_ENC) -> dict:
    """Read only the bridge and its four opening marks from the source cell."""
    import fiona
    from shapely.geometry import LineString, Polygon

    enc = Path(enc).resolve()
    with fiona.open(enc, layer='BRIDGE') as layer:
        features = [f for f in layer if 'Tianwan' in (f['properties'].get('OBJNAM') or '')]
    if len(features) != 1:
        raise ValueError(f'Expected one Tianwan bridge feature, found {len(features)}')
    feature = features[0]
    ring = [local_enu(*p[:2]) for p in feature['geometry']['coordinates'][0]]
    poly = Polygon(ring)
    if not poly.is_valid or poly.area < 100_000:
        raise ValueError('Unexpected bridge polygon')
    min_x, min_y, max_x, max_y = poly.bounds

    # A horizontal cross-section follows the gentle chart curve and gives a
    # stable midpoint while staying inside the narrow bridge footprint.
    points, widths, last_x = [], [], ring[0][0]
    y = max_y-2.0
    while y > min_y+2.0:
        cut = poly.intersection(LineString([(min_x-100, y), (max_x+100, y)]))
        candidates = [g for g in _segments(cut) if g.geom_type == 'LineString' and g.length > 1]
        if not candidates:
            y -= 5.0
            continue
        line = min(candidates, key=lambda g: abs(sum(p[0] for p in g.coords)/len(g.coords)-last_x))
        coords = list(line.coords)
        mid_x = (min(p[0] for p in coords)+max(p[0] for p in coords))/2
        last_x = mid_x
        points.append((mid_x, y))
        widths.append(max(p[0] for p in coords)-min(p[0] for p in coords))
        y -= 5.0

    xs = np.array([p[0] for p in points])
    smoothed = xs.copy()
    for i in range(1, len(xs)-1):
        lo, hi = max(0, i-4), min(len(xs), i+5)
        smoothed[i] = float(np.mean(xs[lo:hi]))
    smoothed[0], smoothed[-1] = xs[0], xs[-1]

    beacons = []
    with fiona.open(enc, layer='BCNSPP') as layer:
        for f in layer:
            name = f['properties'].get('OBJNAM') or ''
            if 'Bridge' not in name:
                continue
            x, y = local_enu(*f['geometry']['coordinates'][:2])
            warning = 'Qiaodun' in (f['properties'].get('INFORM') or '')
            colours = [int(v) for v in (f['properties'].get('COLOUR') or [])]
            beacons.append({'name': name, 'enu': [x, y], 'warning': warning, 'colours': colours,
                            'height_m': 10.0 if warning else 3.0})
    if len(beacons) != 4:
        raise ValueError(f'Expected four charted opening marks, found {len(beacons)}')
    return {
        'source': {'file': enc.name, 'sha256': sha256(enc),
                   'feature_lnam': feature['properties'].get('LNAM'),
                   'objnam': feature['properties'].get('OBJNAM'),
                   'nobjnm': feature['properties'].get('NOBJNM'),
                   'catbrg': feature['properties'].get('CATBRG')},
        'datum_wgs84_lon_lat_alt': [DATUM_LON, DATUM_LAT, 0.0],
        'polygon_enu': [[round(x, 4), round(y, 4)] for x, y in zip(smoothed, [p[1] for p in points])],
        'width_samples_m': [round(w, 2) for w in widths],
        'beacons': beacons,
    }


def _cumulative(points):
    s = [0.0]
    for a, b in zip(points, points[1:]):
        s.append(s[-1]+math.hypot(b[0]-a[0], b[1]-a[1]))
    return s


class Centerline:
    def __init__(self, points):
        self.points = [tuple(map(float, p)) for p in points]
        self.s = _cumulative(self.points)
        self.length = self.s[-1]

    def sample(self, station):
        station = min(max(station, 0.0), self.length)
        i = min(max(bisect.bisect_right(self.s, station)-1, 0), len(self.points)-2)
        span = self.s[i+1]-self.s[i]
        t = 0.0 if span == 0 else (station-self.s[i])/span
        ax, ay = self.points[i]
        bx, by = self.points[i+1]
        x, y = ax+(bx-ax)*t, ay+(by-ay)*t
        dx, dy = bx-ax, by-ay
        length = math.hypot(dx, dy) or 1.0
        tangent = (dx/length, dy/length)
        return x, y, tangent, (-tangent[1], tangent[0])

    def project(self, x, y):
        best = (1e30, 0.0, 0.0, 0.0)
        for i in range(len(self.points)-1):
            ax, ay = self.points[i]
            bx, by = self.points[i+1]
            dx, dy = bx-ax, by-ay
            length2 = dx*dx+dy*dy
            t = 0.0 if length2 == 0 else max(0.0, min(1.0, ((x-ax)*dx+(y-ay)*dy)/length2))
            px, py = ax+dx*t, ay+dy*t
            distance = math.hypot(x-px, y-py)
            if distance < best[0]:
                best = (distance, self.s[i]+math.sqrt(length2)*t, px, py)
        return best


def clip_centerline(chart, north_y=NORTH_CLIP_Y, south_y=SOUTH_CLIP_Y):
    points, widths = chart['polygon_enu'], chart['width_samples_m']
    selected = [(p, w) for p, w in zip(points, widths) if south_y <= p[1] <= north_y]
    if len(selected) < 10:
        raise ValueError('Bridge does not cross the CCS map')
    for edge in (north_y, south_y):
        for (x0, y0), (x1, y1) in zip(points, points[1:]):
            if (y0-edge)*(y1-edge) <= 0 and y0 != y1:
                t = (edge-y0)/(y1-y0)
                selected.append(((x0+(x1-x0)*t, edge), 0.0))
                break
    selected.sort(key=lambda item: -item[0][1])
    if abs(selected[0][0][1]-north_y) > 1e-6:
        raise ValueError('North clipping point not found')
    center = Centerline([p for p, _ in selected])
    beacons = []
    for item in chart['beacons']:
        distance, station, px, py = center.project(*item['enu'])
        beacons.append({**item, 'station_m': station, 'offset_m': distance, 'snapped_enu': [px, py]})
    main_marks = [b for b in beacons if 'No 7' in b['name'] or 'No 16' in b['name']]
    aux_marks = [b for b in beacons if 'No 3' in b['name']]
    if len(main_marks) != 2 or len(aux_marks) != 1:
        raise ValueError('Missing main or auxiliary opening marks')
    return center, beacons, sum(b['station_m'] for b in main_marks)/2, aux_marks[0]['station_m']


def profile(station, main_station, aux_station, platform_station):
    """Deck width, soffit and road-surface elevation at a station."""
    def smoothstep(t):
        t = min(1.0, max(0.0, t))
        return t*t*(3-2*t)

    width = DECK_WIDTH
    if platform_station is not None:
        width += (PLATFORM_WIDTH-DECK_WIDTH)*smoothstep((60-abs(station-platform_station))/20)

    # Road rises gently over a much longer distance than the beam haunches.
    main_d, aux_d = abs(station-main_station), abs(station-aux_station)
    top = 24.0+.5*smoothstep(1-main_d/260)+.35*smoothstep(1-aux_d/120)

    def haunch(d, half_span, transition, mid, pier):
        if d <= half_span:
            return mid+(pier-mid)*smoothstep(d/half_span)
        return 2.2+(pier-2.2)*smoothstep(1-(d-half_span)/transition)

    depth = max(haunch(main_d, MAIN_SPAN/2, SIDE_SPAN, 24.5-CLEAR_HEIGHT, 6.0),
                haunch(aux_d, AUX_SPAN/2, 35.0, 2.55, 4.1))
    return width, top-depth, top


def plan_spans(center, main_station, aux_station):
    main_start = main_station-MAIN_SPAN/2-SIDE_SPAN
    main_end = main_station+MAIN_SPAN/2+SIDE_SPAN
    aux_start = aux_station-AUX_SPAN/2
    aux_end = aux_station+AUX_SPAN/2
    if not (0 < main_start < main_end < aux_start < aux_end < center.length):
        raise ValueError('Opening stations do not fit inside the clipped bridge')

    def fill(start, end, target=APPROACH_SPAN):
        length = end-start
        if length < 25:
            return [start, end]
        count = max(1, round(length/target))
        while length/count > 45:
            count += 1
        while count > 1 and length/count < 30:
            count -= 1
        return [start+i*length/count for i in range(count+1)]

    boundaries = fill(0.0, main_start)
    boundaries += [main_start+SIDE_SPAN, main_end-SIDE_SPAN, main_end]
    boundaries += fill(main_end, aux_start)[1:]
    boundaries += [aux_end]
    boundaries += fill(aux_end, center.length)[1:]
    boundaries = sorted({round(v, 6) for v in boundaries})
    piers = []
    for station in boundaries:
        if station < 1e-6:
            continue
        main = min(abs(station-(main_start+SIDE_SPAN)), abs(station-(main_end-SIDE_SPAN))) < 1e-4
        aux = abs(station-aux_start) < 1e-4 or abs(station-aux_end) < 1e-4
        # The final section is a map cut, not an abutment. Keep its support
        # inside the map rather than extending half the cap across the boundary.
        pier_station = center.length-END_PIER_INSET if abs(center.length-station) < 1e-4 else station
        piers.append({'station_m': round(pier_station, 6),
                      'kind': 'main' if main else 'aux' if aux else 'approach'})
    spans = []
    for a, b in zip(boundaries, boundaries[1:]):
        if b-a < 1e-6:
            continue
        kind = ('main' if a >= main_start+SIDE_SPAN-1e-4 and b <= main_end-SIDE_SPAN+1e-4
                else 'side' if a >= main_start-1e-4 and b <= main_end+1e-4
                else 'aux' if a >= aux_start-1e-4 and b <= aux_end+1e-4 else 'approach')
        spans.append({'start_m': a, 'end_m': b, 'length_m': b-a, 'kind': kind})
    return {'main_center_m': main_station, 'aux_center_m': aux_station,
            'main_start_m': main_start, 'main_end_m': main_end,
            'aux_start_m': aux_start, 'aux_end_m': aux_end,
            'boundaries_m': boundaries, 'piers': piers, 'spans': spans,
            'north_abutment_m': 0.0, 'south_cut_m': center.length}


def build_layout(chart, center, beacons, main_station, aux_station):
    platform_station = center.project(center.sample(center.length/2)[0], PLATFORM_Y)[1]
    plan = plan_spans(center, main_station, aux_station)
    samples = []
    stations = set(np.linspace(0, center.length, max(2, int(center.length//10)+1)))
    stations.update((main_station, aux_station, *(p['station_m'] for p in plan['piers'])))
    for station in sorted(stations):
        width, soffit, top = profile(station, main_station, aux_station, platform_station)
        samples.append([round(float(station), 3), round(width, 3), round(soffit, 3), round(top, 3)])
    return {
        'source': chart['source'],
        'datum_wgs84_lon_lat_alt': chart['datum_wgs84_lon_lat_alt'],
        'clip': {'north_enu_y_m': NORTH_CLIP_Y, 'south_enu_y_m': SOUTH_CLIP_Y,
                 'length_m': center.length},
        'centerline_enu': [[round(x, 4), round(y, 4)] for x, y in center.points],
        'beacons': beacons,
        'platform_station_m': platform_station,
        'plan': plan,
        'profile_samples': samples,
        'dimensions': {
            'deck_width_m': DECK_WIDTH, 'platform_width_m': PLATFORM_WIDTH,
            'main_span_m': MAIN_SPAN, 'side_span_m': SIDE_SPAN, 'aux_span_m': AUX_SPAN,
            'published_main_clear_width_m': 100.0, 'published_main_clear_height_m': CLEAR_HEIGHT,
            'approach_span_m': [30.0, 40.0],
            'pier_sections_m': {
                kind: {'shaft_length_width_chamfer': list(shaft), 'cap_length_width_height': list(cap)}
                for kind, (shaft, cap) in PIER_SECTIONS.items()
            },
            'pier_offset_m': PIER_OFFSET,
        },
        'assumptions': [
            'The S-57 cell supplies the bridge outline and four opening marks only.',
            'Main span 70+125+70 m, 100 m clear width and 21 m clear height are published design values.',
            f'Approach spans are fitted to approximately 40 m; the south cut support is inset by {END_PIER_INSET} m.',
            'Pier profiles and road grading are photo-informed simulation fits, not surveyed dimensions.',
            'The published 100 x 21 m navigation envelope is not reproduced in full; the main soffit is 21 m at midspan.',
            'The reported 7.5 x 6.5 x 2.5 m structure is a buried foundation cap at south approach pier 108, not a typical pier head.',
            'The chart is 1:90,000; horizontal alignment is generalized by roughly 10-20 m.',
            'The chart-fitted sea profile ends at the north shore datum; a separate local approach joins the graded island.',
            'The south end is cut at the 5 km map edge.',
            'The model is static: no traffic, wind, ship impact or structural deformation.',
        ],
    }


class ObjBuilder:
    """Small indexed OBJ writer; one material per file for predictable Gazebo loads."""
    def __init__(self):
        self.vertices = {}
        self.normals = {}
        self.faces = []

    def vertex(self, point):
        key = tuple(round(v, 5)+0.0 for v in point)
        return self.vertices.setdefault(key, len(self.vertices)+1)

    def normal(self, vector):
        length = math.sqrt(sum(v*v for v in vector))
        if length < 1e-12:
            vector, length = (0.0, 0.0, 1.0), 1.0
        return self.normals.setdefault(tuple(round(v/length, 6)+0.0 for v in vector),
                                       len(self.normals)+1)

    def triangle(self, a, b, c, normal):
        cross = ((b[1]-a[1])*(c[2]-a[2])-(b[2]-a[2])*(c[1]-a[1]),
                 (b[2]-a[2])*(c[0]-a[0])-(b[0]-a[0])*(c[2]-a[2]),
                 (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]))
        if sum(cross[i]*normal[i] for i in range(3)) < 0:
            b, c = c, b
        index = self.normal(normal)
        self.faces.append((self.vertex(a), self.vertex(b), self.vertex(c), index))

    def quad(self, a, b, c, d, normal):
        # Orient both triangles independently; a slightly twisted quad would
        # otherwise leave one half back-facing and appear as a hole.
        self.triangle(a, b, c, normal)
        self.triangle(a, c, d, normal)

    def write(self, path: Path, material: str):
        path.parent.mkdir(parents=True, exist_ok=True)
        rows = [f'# Generated CCS Tianwan bridge; metre units.\nmtllib bridge.mtl\nusemtl {material}']
        rows += ['v '+' '.join(f'{v:.5f}' for v in p) for p in self.vertices]
        rows += ['vn '+' '.join(f'{v:.6f}' for v in n) for n in self.normals]
        rows += [f'f {a}//{n} {b}//{n} {c}//{n}' for a, b, c, n in self.faces]
        path.write_text('\n'.join(rows)+'\n')


def _box(builder, center, size, yaw, bottom=True):
    cx, cy, cz = center
    hx, hy, hz = (v/2 for v in size)
    c, s = math.cos(yaw), math.sin(yaw)
    def world(x, y, z):
        return (cx+c*x-s*y, cy+s*x+c*y, cz+z)
    corners = [world(x, y, z) for z in (-hz, hz) for y in (-hy, hy) for x in (-hx, hx)]
    faces = [((0, 3, 2, 1), (0, 0, -1)), ((4, 5, 6, 7), (0, 0, 1)),
             ((0, 1, 5, 4), (0, -1, 0)), ((1, 2, 6, 5), (1, 0, 0)),
             ((2, 3, 7, 6), (0, 1, 0)), ((3, 0, 4, 7), (-1, 0, 0))]
    for ids, local_normal in faces[0 if bottom else 1:]:
        nx, ny, nz = local_normal
        builder.quad(*(corners[i] for i in ids), (c*nx-s*ny, s*nx+c*ny, nz))


def _road_sections(center, layout):
    """Share sections between the road, soffit, lines and railing."""
    plan = layout['plan']

    def section(station):
        x, y, _, _ = center.sample(station)
        before = center.sample(station-10)
        after = center.sample(station+10)
        dx, dy = after[0]-before[0], after[1]-before[1]
        length = math.hypot(dx, dy)
        normal = (-dy/length, dx/length)
        if station == center.length:
            # Intersect the extruded deck with the ENU map boundary, not a
            # plane perpendicular to the local centerline.
            normal = (1/normal[0], 0.0)
        return x, y, normal, *profile(station, plan['main_center_m'], plan['aux_center_m'],
                                      layout['platform_station_m'])

    def edges(row):
        x, y, (nx, ny), width, soffit, top = row
        return x-nx*width/2, y-ny*width/2, x+nx*width/2, y+ny*width/2, soffit, top

    def refine(a, b, ra, rb):
        # Bound edge/vertical chord error while keeping straight approach
        # spans coarse. A short limit avoids chasing minor ENC jitter.
        if b-a <= 3:
            return [(b, rb)]
        ea, eb = edges(ra), edges(rb)
        cuts = [(a+(b-a)*t, t) for t in (.25, .5, .75)]
        samples = [(s, t, section(s)) for s, t in cuts]
        if b-a <= 24 and all(abs(q-(p+(r-p)*t)) <= .045
                              for _, t, row in samples
                              for p, q, r in zip(ea, edges(row), eb)):
            return [(b, rb)]
        mid, _, rm = samples[1]
        return refine(a, mid, ra, rm)+refine(mid, b, rm, rb)

    breaks = {0.0, center.length, plan['main_center_m'], plan['aux_center_m']}
    breaks.update(plan['boundaries_m'][1:-1])
    breaks.update(np.arange(VISUAL_BATCH_M, center.length, VISUAL_BATCH_M))
    stations = sorted(breaks)
    result = [(0.0, section(0.0))]
    for a, b in zip(stations, stations[1:]):
        result.extend(refine(a, b, result[-1][1], section(b)))
    return result


def _section_point(row, offset, z):
    x, y, (nx, ny), _, _, _ = row
    return x+nx*offset, y+ny*offset, z


def _beam_ring(row):
    """CCW box-girder section (viewed in offset/elevation coordinates)."""
    _, _, _, width, bottom, top = row
    half = width/2
    return [_section_point(row, offset, z) for offset, z in (
        (-half, top), (-half, top-.55), (-half+3, bottom),
        (half-3, bottom), (half, top-.55), (half, top))]


def _sweep(builder, a, b, edge_count=None, reverse=False):
    """Join equal rings, optionally leaving the final edge open."""
    for i in range(len(a) if edge_count is None else edge_count):
        j = (i+1) % len(a)
        p, q, r = a[i], b[i], b[j]
        u, v = (tuple(y-x for x, y in zip(p, end)) for end in (q, r))
        normal = (u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0])
        if reverse:
            normal = tuple(-n for n in normal)
        builder.quad(p, q, r, a[j], normal)


def _close_ring(builder, ring, normal):
    for i in range(1, len(ring)-1):
        builder.triangle(ring[0], ring[i], ring[i+1], normal)


def generate_model(layout: dict, output: Path, center: Centerline) -> dict:
    output = Path(output)
    mesh_dir = output/'meshes'
    mesh_dir.mkdir(parents=True, exist_ok=True)
    stores = {k: {} for k in ('concrete', 'solid', 'asphalt', 'railing',
                             'markings_white', 'markings_yellow')}
    segment_count = math.ceil(center.length/VISUAL_BATCH_M)

    def batch(station):
        return max(0, min(segment_count-1, int(station//VISUAL_BATCH_M)))

    def get(kind, station):
        store, index = stores[kind], batch(station)
        if index not in store:
            store[index] = ObjBuilder()
        return store[index]

    sea_sections = _road_sections(center, layout)
    approach = landing.sections(sea_sections[0][1])
    sections = approach[:-1]+sea_sections
    north_start = sections[0][0]
    rail_start = next(s for s, row in approach if row[1] <= 1000)
    for (s0, a), (s1, b) in zip(sections, sections[1:]):
        r0, r1 = _beam_ring(a), _beam_ring(b)
        concrete, solid = get('concrete', s0), get('solid', s0)
        _sweep(concrete, r0, r1, edge_count=5, reverse=True)  # pavement covers the top
        _sweep(solid, r0, r1, reverse=True)
        start = s0 == north_start or s0 > 0 and s0 % VISUAL_BATCH_M == 0
        end = s1 == center.length or s1 > 0 and s1 % VISUAL_BATCH_M == 0
        for ring, row, close, outside, sign in (
                (r0, a, start, s0 == north_start, -1), (r1, b, end, s1 == center.length, 1)):
            if close:
                nx, ny = row[2]
                normal = (sign*ny, -sign*nx, 0)
                _close_ring(solid, ring, normal)
                if outside:
                    _close_ring(concrete, ring, normal)
        _pavement(get, s0, s1, a, b)
        if s0 >= rail_start:
            _railing(get('railing', s0), s0, s1, a, b, center.length, rail_start)

    _piers(get, layout, center)
    for station, (x, y, (nx, ny), width, soffit, _) in approach:
        tangent = ny, -nx
        if y in landing.SUPPORT_Y:
            for side in (-1, 1):
                rings = _pier_shape(x, y, tangent, (nx, ny), soffit, 'approach', side)
                _add_pier(get, station, rings)
        elif y == landing.JOINT_Y:
            # A narrow abutment buried behind the new bridge seat. Unlike the
            # old freestanding block, its wings sit inside the graded banks.
            position = (x-1.5*tangent[0], y-1.5*tangent[1], (soffit+5)/2)
            for kind in ('concrete', 'solid'):
                _box(get(kind, station), position, (3.4, width-4, soffit-5),
                     math.atan2(tangent[1], tangent[0]))

    (mesh_dir/'bridge.mtl').write_text(''.join(f'newmtl {kind}\nKd 1 1 1\n' for kind in stores))
    triangles = {'visual': 0, 'collision': 0}
    for kind, batches in stores.items():
        for index, builder in sorted(batches.items()):
            builder.write(mesh_dir/f'seg_{index:02d}_{kind}.obj', kind)
            triangles['collision' if kind == 'solid' else 'visual'] += len(builder.faces)
    return {**_write_sdf(output, layout, stores, segment_count), 'triangles': triangles,
            'north_approach': {'length_m': round(-north_start, 3),
                               'end_enu_xy_m': list(approach[0][1][:2]),
                               'abutment_enu_y_m': landing.JOINT_Y, 'extra_pier_lines': len(landing.SUPPORT_Y)}}


def _lerp(a, b, t):
    return tuple(x+(y-x)*t for x, y in zip(a, b))


def _pavement(get, s0, s1, a, b):
    wa, wb = a[3]/2, b[3]/2

    def strip(builder, aa, ab, ba, bb, lift=0.0, start=0.0, end=1.0):
        left = (_section_point(a, aa, a[-1]+lift), _section_point(b, ba, b[-1]+lift))
        right = (_section_point(a, ab, a[-1]+lift), _section_point(b, bb, b[-1]+lift))
        builder.quad(_lerp(*left, start), _lerp(*right, start),
                     _lerp(*right, end), _lerp(*left, end), (0, 0, 1))

    strip(get('asphalt', s0), -wa+.45, wa-.45, -wb+.45, wb-.45)
    strip(get('concrete', s0), -wa, -wa+.45, -wb, -wb+.45)
    strip(get('concrete', s0), wa-.45, wa, wb-.45, wb)

    def line(kind, offset, width, start=0.0, end=1.0):
        low, high = offset-width/2, offset+width/2
        strip(get(kind, s0), low, high, low, high, .025, start, end)

    for off in (-.15, .15):
        line('markings_yellow', off, .12)
    for off in (-10.5, 10.5):
        line('markings_white', off, .15)
    for i in range(int(s0//10), math.ceil(s1/10)):
        start, end = max(s0, i*10), min(s1, i*10+4)
        if start < end:
            for off in (-7, -3.5, 3.5, 7):
                line('markings_white', off, .12, (start-s0)/(s1-s0), (end-s0)/(s1-s0))


def _railing(builder, s0, s1, a, b, length, start=0.0):
    for side in (-1, 1):
        def rail_ring(row, z):
            offset = side*(row[3]/2-.30)
            return [_section_point(row, offset+dx, row[-1]+z+dz)
                    for dx, dz in ((-.045, .045), (-.045, -.045), (.045, -.045), (.045, .045))]

        for z in (.35, .70, 1.05):
            r0, r1 = rail_ring(a, z), rail_ring(b, z)
            _sweep(builder, r0, r1, reverse=True)
            if s0 == start:
                _close_ring(builder, r0, (-a[2][1], a[2][0], 0))
            if s1 == length:
                _close_ring(builder, r1, (b[2][1], -b[2][0], 0))
        p = _section_point(a, side*(a[3]/2-.30), a[-1]+.55)
        q = _section_point(b, side*(b[3]/2-.30), b[-1]+.55)
        yaw = math.atan2(q[1]-p[1], q[0]-p[0])
        for i in range(math.ceil(s0/3), math.ceil(s1/3)):
            # Move the last post inward if it would straddle the map edge.
            station = min(3*i, length-.3)
            _box(builder, _lerp(p, q, (station-s0)/(s1-s0)), (.12, .12, 1.1), yaw, bottom=False)


def _pier_rings(center, layout, pier, side):
    station = pier['station_m']
    plan = layout['plan']
    x, y, tangent, normal = center.sample(station)
    _, soffit, _ = profile(station, plan['main_center_m'], plan['aux_center_m'],
                            layout['platform_station_m'])
    return _pier_shape(x, y, tangent, normal, soffit, pier['kind'], side)


def _pier_shape(x, y, tangent, normal, soffit, kind, side):
    (shaft_l, shaft_w, bevel), (cap_l, cap_w, cap_h) = PIER_SECTIONS[kind]
    x, y = x+side*PIER_OFFSET*normal[0], y+side*PIER_OFFSET*normal[1]

    def ring(length, width, z):
        l, w = length/2, width/2
        # Clockwise in XY for the bottom-to-top sweep.
        return [(x+tangent[0]*u+normal[0]*v, y+tangent[1]*u+normal[1]*v, z)
                for u, v in ((l, -w+bevel), (l-bevel, -w), (-l+bevel, -w), (-l, -w+bevel),
                             (-l, w-bevel), (-l+bevel, w), (l-bevel, w), (l, w-bevel))]

    return (ring(shaft_l, shaft_w, PIER_BASE_Z),
            ring(shaft_l, shaft_w, soffit-cap_h),
            ring(cap_l, cap_w, soffit-.28),
            ring(cap_l, cap_w, soffit+.14))


def _add_pier(get, station, rings):
    for kind in ('concrete', 'solid'):
        builder = get(kind, station)
        for a, b in zip(rings, rings[1:]):
            _sweep(builder, a, b)
        _close_ring(builder, rings[0], (0, 0, -1))
        _close_ring(builder, rings[-1], (0, 0, 1))


def _piers(get, layout, center):
    for pier in layout['plan']['piers']:
        for side in (-1, 1):
            _add_pier(get, pier['station_m'], _pier_rings(center, layout, pier, side))


def _write_sdf(output: Path, layout: dict, stores: dict, segment_count: int) -> dict:
    root = ET.Element('sdf', version='1.9')
    model = ET.SubElement(root, 'model', name=MODEL)
    ET.SubElement(model, 'static').text = 'true'
    ET.SubElement(model, 'self_collide').text = 'false'
    link = ET.SubElement(model, 'link', name='bridge')
    materials = {
        'concrete': ('0.58 0.60 0.62 1', 0.9, 0.0),
        'asphalt': ('0.10 0.10 0.11 1', 1.0, 0.0),
        'railing': ('0.78 0.80 0.82 1', 0.45, 0.3),
        'markings_white': ('0.92 0.92 0.92 1', 0.8, 0.0),
        'markings_yellow': ('0.90 0.72 0.08 1', 0.8, 0.0),
    }
    for index in range(segment_count):
        for suffix, material in materials.items():
            if index not in stores[suffix]:
                continue
            visual = ET.SubElement(link, 'visual', name=f'{suffix}_{index:02d}')
            mesh = ET.SubElement(ET.SubElement(visual, 'geometry'), 'mesh')
            ET.SubElement(mesh, 'uri').text = f'model://{MODEL}/meshes/seg_{index:02d}_{suffix}.obj'
            ET.SubElement(visual, 'cast_shadows').text = 'true' if suffix == 'concrete' else 'false'
            _material(visual, material)
        # DART uses the untransformed static triangle meshes, not a convex
        # hull of the whole chunk. Each pier and beam segment is closed.
        collision = ET.SubElement(link, 'collision', name=f'solid_{index:02d}')
        mesh = ET.SubElement(ET.SubElement(collision, 'geometry'), 'mesh')
        ET.SubElement(mesh, 'uri').text = f'model://{MODEL}/meshes/seg_{index:02d}_solid.obj'
    _beacons(link, layout)
    ET.indent(root, space='  ')
    sdf = ET.tostring(root, encoding='utf-8', xml_declaration=True)+b'\n'
    (output/'model.sdf').write_bytes(sdf)
    config = ET.Element('model')
    ET.SubElement(config, 'name').text = MODEL
    ET.SubElement(config, 'version').text = '2.0'
    ET.SubElement(config, 'sdf', version='1.9').text = 'model.sdf'
    ET.SubElement(config, 'description').text = 'Chart-derived Tianwan bridge, clipped to the CCS map edge.'
    ET.indent(config, space='  ')
    (output/'model.config').write_bytes(ET.tostring(config, encoding='utf-8', xml_declaration=True)+b'\n')
    return {'model_sdf_sha256': hashlib.sha256(sdf).hexdigest(), 'segment_count': segment_count,
            'mesh_files': sum(map(len, stores.values())), 'collisions': len(link.findall('collision')),
            'visuals': len(link.findall('visual'))}


def _material(visual, spec):
    rgb, roughness, metalness = spec
    material = ET.SubElement(visual, 'material')
    ET.SubElement(material, 'ambient').text = rgb
    ET.SubElement(material, 'diffuse').text = rgb
    pbr = ET.SubElement(material, 'pbr')
    metal = ET.SubElement(pbr, 'metal')
    ET.SubElement(metal, 'roughness').text = str(roughness)
    ET.SubElement(metal, 'metalness').text = str(metalness)


def _beacons(link, layout):
    for i, beacon in enumerate(layout['beacons']):
        x, y = beacon['enu']
        height = float(beacon['height_m'])
        warning = bool(beacon['warning'])
        band = 'yellow' if warning else 'white'
        for j, (z0, z1, colour) in enumerate(((0.0, height*0.45, 'red'),
                                              (height*0.45, height*0.8, band),
                                              (height*0.8, height, 'red'))):
            visual = ET.SubElement(link, 'visual', name=f'beacon_{i}_{j}')
            ET.SubElement(visual, 'pose').text = f'{x:.4f} {y:.4f} {(z0+z1)/2:.3f} 0 0 0'
            cyl = ET.SubElement(ET.SubElement(visual, 'geometry'), 'cylinder')
            ET.SubElement(cyl, 'radius').text = '0.55'
            ET.SubElement(cyl, 'length').text = f'{z1-z0:.3f}'
            rgb = '0.8 0.05 0.05' if colour == 'red' else ('0.95 0.75 0.1' if colour == 'yellow' else '0.95 0.95 0.95')
            material = ET.SubElement(visual, 'material')
            ET.SubElement(material, 'ambient').text = f'{rgb} 1'
            ET.SubElement(material, 'diffuse').text = f'{rgb} 1'
        light = ET.SubElement(link, 'visual', name=f'beacon_{i}_light')
        ET.SubElement(light, 'pose').text = f'{x:.4f} {y:.4f} {height+0.25:.3f} 0 0 0'
        ET.SubElement(ET.SubElement(light, 'geometry'), 'sphere')
        sphere = light.find('geometry/sphere')
        ET.SubElement(sphere, 'radius').text = '0.18'
        material = ET.SubElement(light, 'material')
        ET.SubElement(material, 'ambient').text = '1 0.9 0.25 1'
        ET.SubElement(material, 'diffuse').text = '1 0.9 0.25 1'
        ET.SubElement(material, 'emissive').text = '1 0.75 0.1 1'
        coll = ET.SubElement(link, 'collision', name=f'beacon_{i}')
        ET.SubElement(coll, 'pose').text = f'{x:.4f} {y:.4f} {height/2:.3f} 0 0 0'
        cyl = ET.SubElement(ET.SubElement(coll, 'geometry'), 'cylinder')
        ET.SubElement(cyl, 'radius').text = '0.6'
        ET.SubElement(cyl, 'length').text = f'{height:.3f}'
