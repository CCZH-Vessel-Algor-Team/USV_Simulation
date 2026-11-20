"""Pure-Python model catalog and safe native-SDF instance adapter."""
from dataclasses import dataclass
import hashlib
import json
import math
import re
from pathlib import Path
import xml.etree.ElementTree as ET


@dataclass(frozen=True)
class ShipModel:
    model_id: str
    display_name: str
    category: str
    backend: str
    length: float
    beam: float
    height: float
    draft: float = 0.0
    spawn_z: float = 0.5
    recommended_speed: float = 3.0
    sdf: str = ''

    @property
    def radius(self):
        return math.hypot(self.length, self.beam)/2


LEGACY_MODELS = (
    ShipModel('legacy_10m', '默认 10 米目标船', 'legacy', 'mesh_profile', 10., 3.6, 2.),
    ShipModel('debug_box', '调试方块', 'debug', 'box', 3.6, 10., 2.),
    ShipModel('debug_cylinder', '调试圆柱', 'debug', 'cylinder', 8., 8., 5.),
)


class ShipCatalog:
    def __init__(self, path):
        self.path = Path(path).resolve()
        self.root = self.path.parent
        self.models = {model.model_id: model for model in LEGACY_MODELS}
        data = json.loads(self.path.read_text())
        if not isinstance(data, dict) or data.get('version') != 1:
            raise ValueError('Unsupported fleet catalog version')
        if not isinstance(data.get('source_sha256'), dict) or not isinstance(data.get('models'), list):
            raise ValueError('Invalid fleet catalog structure')
        for name, expected in data['source_sha256'].items():
            file = self._resource(name)
            if hashlib.sha256(file.read_bytes()).hexdigest() != expected:
                raise ValueError(f'Fleet resource hash differs: {name}')
        for entry in data['models']:
            fields = ShipModel.__dataclass_fields__
            model = ShipModel(**{key: value for key, value in entry.items() if key in fields})
            if (not re.fullmatch(r'[a-z0-9_]+', model.model_id) or
                    model.model_id in self.models or model.backend != 'fleet_sdf'):
                raise ValueError('Duplicate or unsupported fleet model')
            if not all(math.isfinite(v) and v > 0 for v in (model.length, model.beam, model.height, model.draft)):
                raise ValueError('Invalid vessel dimensions')
            if (not math.isfinite(model.spawn_z) or not math.isfinite(model.recommended_speed) or
                    not 0 < model.recommended_speed <= 20):
                raise ValueError('Invalid spawn height')
            self._native_root(model, 'catalog_validation')
            self.models[model.model_id] = model

    def _resource(self, name):
        path = (self.root/name).resolve()
        if not path.is_relative_to(self.root) or not path.is_file():
            raise ValueError(f'Invalid installed fleet resource: {name}')
        return path

    def get(self, model_id):
        try:
            return self.models[model_id]
        except KeyError:
            raise ValueError(f'Unknown vessel model_id: {model_id}') from None

    def _native_root(self, model, instance_name):
        path = self._resource(model.sdf)
        root = ET.parse(path).getroot()
        entity = root.find('model')
        if entity is None or entity.findtext('static') != 'false' or len(entity.findall('link')) != 1:
            raise ValueError('Expected a single-link dynamic native vessel')
        link = entity.find("link[@name='base_link']")
        if link is None or link.findtext('kinematic', 'false') == 'true':
            raise ValueError('Invalid vessel base_link')
        mass = float(link.findtext('inertial/mass'))
        if not math.isfinite(mass) or mass <= 0:
            raise ValueError('Invalid vessel mass')
        collisions = link.findall('collision')
        if not collisions or any(c.find('geometry/box') is None for c in collisions):
            raise ValueError('Scoped fleet buoyancy currently supports box collisions only')
        entity.set('name', instance_name)
        for element in root.iter():
            if element.tag in ('uri', 'albedo_map', 'normal_map', 'roughness_map', 'metalness_map'):
                uri = element.text or ''
                if '://' in uri or Path(uri).is_absolute():
                    raise ValueError('Only relocatable approved native resources are allowed')
                resource = (path.parent/uri).resolve()
                if not resource.is_relative_to(path.parent) or not resource.is_file():
                    raise ValueError('Missing or escaping native resource')
                element.text = resource.as_uri()
        plugins = entity.findall('plugin')
        allowed = {'gz-sim-hydrodynamics-system', 'libTargetShipController.so'}
        if len(plugins) != 2 or {p.get('filename') for p in plugins} != allowed:
            raise ValueError('Unexpected plugin in approved native vessel')
        controller = next(p for p in plugins if p.get('filename') == 'libTargetShipController.so')
        topic = controller.find('topic')
        if topic is None:
            raise ValueError('Native controller has no relocatable command topic')
        topic.text = f'/model/{instance_name}/cmd_vel'
        buoyancy = ET.SubElement(entity, 'plugin', filename='libFleetBuoyancy.so', name='vrx::FleetBuoyancy')
        ET.SubElement(buoyancy, 'link_name').text = 'base_link'
        ET.SubElement(buoyancy, 'water_density').text = '1025'
        ET.SubElement(buoyancy, 'air_density').text = '1.225'
        ET.SubElement(buoyancy, 'water_level').text = '0'
        return root

    def sdf(self, model_id, instance_name):
        if not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_.-]{0,95}', instance_name):
            raise ValueError('Invalid instance name')
        model = self.get(model_id)
        if model.backend != 'fleet_sdf':
            raise ValueError('Not a native fleet model')
        return ET.tostring(self._native_root(model, instance_name), encoding='unicode')


def rectangles_overlap(a, b, margin=0.):
    """SAT for (x,y,yaw,length,beam); touching expanded hulls are not clear."""
    def axes(p):
        c, s = math.cos(p[2]), math.sin(p[2])
        return ((c, s), (-s, c))
    aa, bb = axes(a), axes(b)
    delta = (b[0]-a[0], b[1]-a[1])
    for u in (*aa, *bb):
        dot = lambda x, y: x[0]*y[0]+x[1]*y[1]
        ra = sum((a[3+i]/2+margin)*abs(dot(axis, u)) for i, axis in enumerate(aa))
        rb = sum((b[3+i]/2+margin)*abs(dot(axis, u)) for i, axis in enumerate(bb))
        if abs(dot(delta, u)) > ra+rb:
            return False
    return True


def check_water_footprint(info, data, x, y, yaw, length, beam, margin=2.):
    """Conservative cell intersection check, without a new geometry dependency."""
    origin = info.origin
    if (not math.isfinite(info.resolution) or info.resolution <= 0 or
            info.width <= 0 or info.height <= 0 or info.width*info.height != len(data) or
            not all(math.isfinite(v) for v in (origin.position.x, origin.position.y,
                origin.orientation.x, origin.orientation.y, origin.orientation.z, origin.orientation.w)) or
            abs(origin.orientation.x) > 1e-9 or abs(origin.orientation.y) > 1e-9 or
            abs(origin.orientation.z) > 1e-9 or
            abs(origin.orientation.w-1) > 1e-9):
        raise ValueError('Need a valid, unrotated ENU occupancy map')
    c, s, r = math.cos(yaw), math.sin(yaw), info.resolution
    a, b = length/2+margin, beam/2+margin
    ex, ey = abs(c)*a+abs(s)*b, abs(s)*a+abs(c)*b
    x0, y0 = origin.position.x, origin.position.y
    left, right = math.floor((x-ex-x0)/r), math.floor((x+ex-x0)/r)
    low, high = math.floor((y-ey-y0)/r), math.floor((y+ey-y0)/r)
    if left < 0 or low < 0 or right >= info.width or high >= info.height:
        raise ValueError('Whole vessel footprint must fit inside the map')
    pad = r*math.sqrt(2)/2
    for row in range(low, high+1):
        dy = y0+(row+.5)*r-y
        for col in range(left, right+1):
            dx = x0+(col+.5)*r-x
            if abs(c*dx+s*dy) <= a+pad and abs(-s*dx+c*dy) <= b+pad:
                value = data[row*info.width+col]
                if value < 0 or value >= 50:
                    raise ValueError('Vessel footprint overlaps occupied/unknown map cells')
