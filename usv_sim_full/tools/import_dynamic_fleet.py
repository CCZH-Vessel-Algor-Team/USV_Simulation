#!/usr/bin/env python3
"""Import the approved fleet once; never run its scripts or overwrite other assets."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET


OUTPUT = Path(__file__).resolve().parents[1]/'description/models/target_fleet'
NS = {'d': 'http://www.collada.org/2005/11/COLLADASchema'}
# UI translations only: keep source names, stable IDs and model payload intact.
DISPLAY_NAMES_ZH = {
    'munich_maersk': '慕尼黑马士基号（集装箱船）',
    'berge_stahl': '伯格施塔尔号（矿砂船）',
    'vassos': '瓦索斯号（散货船）',
    '21_sunny': '阳光二十一号（带吊散货船）',
    'mark': '马克号（拖网渔船）',
    'pohnpei_no_1': '波纳佩一号（围网渔船）',
    'katoa_no_11': '卡托阿十一号（延绳钓渔船）',
    'vb_bremen_fighter': '不来梅斗士号（海工拖轮）',
    'vb_rumba': '伦巴号（港作拖轮）',
    'ocean_myst_ii': '海洋迷雾二号（渔船）',
}


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def mesh_bounds(path):
    root = ET.parse(path).getroot()
    if root.findtext('d:asset/d:up_axis', namespaces=NS) != 'Z_UP':
        raise ValueError('Expected the approved Z-up meshes')
    vertices = []
    for mesh in root.findall('.//d:mesh', NS):
        position = mesh.find('d:vertices/d:input[@semantic="POSITION"]', NS)
        source = mesh.find("d:source[@id='%s']/d:float_array" % position.get('source')[1:], NS)
        values = list(map(float, source.text.split()))
        vertices.extend(zip(values[::3], values[1::3], values[2::3]))
    return [min(v[i] for v in vertices) for i in range(3)], [max(v[i] for v in vertices) for i in range(3)]


def import_fleet(source, output=OUTPUT):
    source, output = Path(source).resolve(), Path(output)
    manifest = json.loads((source/'fleet-manifest.json').read_text())
    inputs, entries = {}, []
    for boat in manifest['boats']:
        model_id = boat['id']
        if not model_id or any(c not in 'abcdefghijklmnopqrstuvwxyz0123456789_' for c in model_id):
            raise ValueError('Unsafe source model id')
        folder = source/'models'/model_id
        for path in sorted(folder.rglob('*')):
            if not path.is_file():
                continue
            if path.is_symlink():
                raise ValueError('Source symlinks are not allowed')
            rel = path.relative_to(source/'models')
            dst = output/rel
            inputs[str(rel)] = sha256(path)
            if dst.exists() and sha256(dst) != inputs[str(rel)]:
                raise ValueError(f'Refusing to overwrite a different resource: {dst}')
        lo, hi = mesh_bounds(folder/'meshes/ship.dae')
        entries.append({'model_id': model_id, 'display_name': DISPLAY_NAMES_ZH.get(model_id, boat['name']),
                        'source_display_name': boat['name'], 'category': boat['kind'],
                        'backend': 'fleet_sdf', 'sdf': f'{model_id}/model.sdf',
                        'length': max(float(boat['length']), 2*max(abs(lo[0]), abs(hi[0]))),
                        'beam': max(float(boat['beam']), 2*max(abs(lo[1]), abs(hi[1]))),
                        'height': max(float(boat['deck']), hi[2]), 'draft': float(boat['draft']),
                        'spawn_z': float(boat['start_z_m']), 'recommended_speed': 1.0,
                        'reference_length': boat['length'], 'reference_beam': boat['beam'],
                        'mesh_bounds_m': [lo, hi]})
    # Validate every destination before copying any file.
    for rel in inputs:
        dst = output/rel
        if not dst.exists():
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source/'models'/rel, dst)
    output.mkdir(parents=True, exist_ok=True)
    result = {'version': 1, 'models': entries, 'source_sha256': inputs,
              'policy': 'User-supplied approved fleet, unchanged source files. Native model parameters are '
                        'calm-water test approximations, not calibrated vessel hydrodynamics. '
                        'Instances use local scoped graded buoyancy; no whole-world buoyancy is added.'}
    (output/'catalog.json').write_text(json.dumps(result, indent=2, ensure_ascii=False)+'\n')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', required=True, type=Path)
    args = parser.parse_args()
    result = import_fleet(args.source)
    print('Imported unchanged resources for', len(result['models']), 'vessel models into', OUTPUT)
