#!/usr/bin/env python3
"""Import the S-57 bridge once and bake the static Gazebo model.

The generated model is independent of the external chart.  Re-running from the
stored ``layout.json`` is deterministic; ``--reimport`` reads the chart again and
records a new source hash.
"""
import argparse
import json
from pathlib import Path
import sys

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))

from usv_sim_full import ccs_bridge as bridge


MODELS = PACKAGE/'worlds/models'
OUTPUT = MODELS/bridge.MODEL


def prepare(enc=None, output=OUTPUT, reimport=False, layout_input=None):
    output = Path(output)
    layout_path = Path(layout_input) if layout_input is not None else output/'layout.json'
    if not reimport and (layout_input is not None or layout_path.is_file()):
        layout = json.loads(layout_path.read_text())
        chart = {key: layout[key] for key in ('source', 'datum_wgs84_lon_lat_alt')}
        beacons = layout['beacons']
        center = bridge.Centerline(layout['centerline_enu'])
        main_station = layout['plan']['main_center_m']
        aux_station = layout['plan']['aux_center_m']
    else:
        enc = Path(enc) if enc else bridge.DEFAULT_ENC
        if not enc.is_file():
            raise FileNotFoundError(f'ENC cell not found: {enc}')
        chart = bridge.extract_chart(enc)
        center, beacons, main_station, aux_station = bridge.clip_centerline(chart)
        # Bake against the same coordinates that will be available to rebuild.
        center = bridge.Centerline([(round(x, 4), round(y, 4)) for x, y in center.points])
    layout = bridge.build_layout(chart, center, beacons, main_station, aux_station)
    result = bridge.generate_model(layout, output, center)
    layout['model'] = {'name': bridge.MODEL, **result}
    (output/'layout.json').write_text(json.dumps(layout, indent=2)+'\n')
    (output/'SOURCE.md').write_text(
        '# 田湾跨海大桥仿真模型来源\n\n'
        f'- 海图：`{layout["source"]["file"]}`，SHA256 `{layout["source"]["sha256"]}`\n'
        f'- 物标：`{layout["source"]["objnam"]}` / `{layout["source"].get("nobjnm")}`，LNAM `{layout["source"].get("feature_lnam")}`\n'
        '- 主通航孔：70 + 125 + 70 m 连续刚构，净宽 100 m、净高 21 m（公开设计资料）\n'
        '- 辅通航孔：70 m 连续刚构；引桥：30–40 m 组合箱梁\n'
        '- 桥面：标准 34 m，观景平台 38 m；桥面纵坡、墩身和盖梁参照照片拟合，尺寸并非实测。\n'
        '- 施工报道中 7.5 × 6.5 × 2.5 m 是南引桥 108# 墩右幅的埋地承台，不是典型盖梁或墩身尺寸。\n'
        '- 21 m 为模型主跨跨中梁底相对仿真水线的高度，不代表整个通航孔均有 21 m 净高。\n'
        '- 海上段位置不变；南端截到 CCS 5 km 地图边缘，北端通过局部引道接入高公岛修坡模型。地图栅格不改。\n')
    return layout


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--enc', type=Path, help='S-57 .000 cell (read-only)')
    parser.add_argument('--output', type=Path, default=OUTPUT)
    source = parser.add_mutually_exclusive_group()
    source.add_argument('--layout', type=Path, help='Rebuild from this layout into --output')
    source.add_argument('--reimport', action='store_true', help='Read the chart again instead of stored layout')
    args = parser.parse_args()
    data = prepare(args.enc, args.output, args.reimport, args.layout)
    print(f"Bridge ENU length: {data['clip']['length_m']:.1f} m; "
          f"spans: {len(data['plan']['spans'])}; piers: {len(data['plan']['piers'])}; "
          f"model: {args.output}")
