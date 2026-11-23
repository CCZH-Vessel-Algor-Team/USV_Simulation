"""Opt-in isolated check of the installed bridge, clearance and pier collisions."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import uuid
import xml.etree.ElementTree as ET

import pytest


pytestmark = pytest.mark.skipif(
    os.getenv('USV_BRIDGE_INTEGRATION') != '1',
    reason='requires installed Gazebo assets and headless rendering',
)


def _sample(centerline, stations, station):
    station = min(max(station, 0.0), stations[-1])
    i = max(0, min(len(centerline)-2, next(
        (j for j, s in enumerate(stations) if s >= station), len(stations)-1)-1))
    t = (station-stations[i])/(stations[i+1]-stations[i])
    ax, ay = centerline[i]
    bx, by = centerline[i+1]
    x, y = ax+(bx-ax)*t, ay+(by-ay)*t
    dx, dy = bx-ax, by-ay
    length = math.hypot(dx, dy)
    tangent = (dx/length, dy/length)
    return x, y, tangent, (-tangent[1], tangent[0])


def test_installed_bridge_passes_vessel_and_blocks_pier_and_deck(tmp_path, monkeypatch):
    import numpy as np
    from PIL import Image
    from ament_index_python.packages import get_package_share_directory
    from gz.transport13 import Node, NodeOptions
    from gz.msgs10.image_pb2 import Image as GzImage
    from gz.msgs10.laserscan_pb2 import LaserScan
    from gz.msgs10.pose_v_pb2 import Pose_V

    package = Path(get_package_share_directory('usv_sim_full'))
    models = Path(os.getenv('USV_BRIDGE_TEST_MODELS', str(package/'worlds/models')))
    layout = json.loads((models/'ccs_tianwan_bridge/layout.json').read_text())
    line = layout['centerline_enu']
    stations = [0.0]
    for a, b in zip(line, line[1:]):
        stations.append(stations[-1]+math.hypot(b[0]-a[0], b[1]-a[1]))
    plan = layout['plan']
    main = plan['main_center_m']
    aux = plan['aux_center_m']
    mx, my = _sample(line, stations, main)[:2]
    approach = next(s for s in plan['spans'] if s['kind'] == 'approach' and s['start_m'] > 1400)
    approach_mid = (approach['start_m']+approach['end_m'])/2
    pier = next(p for p in plan['piers'] if p['kind'] == 'approach'
                and 300 < p['station_m'] < 500)
    px, py, _, (nx, ny) = _sample(line, stations, pier['station_m'])
    offset = layout['dimensions']['pier_offset_m']
    px, py = px+nx*offset, py+ny*offset

    resources = os.pathsep.join(dict.fromkeys((str(models), str(package/'worlds/models'),
                                              str(package/'description/models'),
                                              os.getenv('GZ_SIM_RESOURCE_PATH', ''))))
    partition = 'bridge_validation_'+uuid.uuid4().hex
    monkeypatch.setenv('GZ_PARTITION', partition)
    monkeypatch.setenv('ROS_DOMAIN_ID', '196')
    monkeypatch.setenv('GZ_SIM_RESOURCE_PATH', resources)
    monkeypatch.setenv('SDF_PATH', resources)
    root = ET.parse(package/'worlds/ccs_open_water_crop.sdf').getroot()
    world = root.find('world')
    world.set('name', 'bridge_validation')
    world.remove(world.find('gui'))
    for item in list(world.findall('include')):
        if item.findtext('uri') != 'model://ccs_tianwan_bridge':
            world.remove(item)
    ET.SubElement(world.find("plugin[@name='gz::sim::systems::SceneBroadcaster']"),
                  'dynamic_pose_hertz').text = '20'
    drive = ET.SubElement(world, 'plugin', filename='gz-sim-apply-link-wrench-system',
                          name='gz::sim::systems::ApplyLinkWrench')
    # Best-pass beacons are offset to one side, near the charted centers.
    # Probe a parallel clear line inside each large opening, not the beacon.
    passages = {f'{name}_pass': _sample(line, stations, s)
                for name, s in (('main', main+12), ('aux', aux+12), ('approach', approach_mid))}
    probes = {name: (x-n[0]*32, y-n[1]*32, 1.0, (*n, 0.0), 4.0)
              for name, (x, y, _, n) in passages.items()}
    probes['pier_hit'] = (px+nx*16, py+ny*16, 1.0, (-nx, -ny, 0.0), 4.0)
    probes['deck_hit'] = (mx, my, 14.0, (0.0, 0.0, 1.0), 22.0)
    for name, (x, y, z, axis, force) in probes.items():
        xyz = ' '.join(map(str, axis))
        geometry = '<geometry><sphere><radius>.3</radius></sphere></geometry>'
        if name in passages:
            yaw = math.atan2(axis[1], axis[0])
            geometry = (f'<pose>0 0 0 0 0 {yaw}</pose>'
                        '<geometry><box><size>10 3 1.6</size></box></geometry>')
        world.append(ET.fromstring(f'''<model name="{name}">
          <pose>{x} {y} {z} 0 0 0</pose><link name="body">
          <inertial><mass>1</mass><inertia><ixx>.1</ixx><iyy>.1</iyy><izz>.1</izz></inertia></inertial>
          <collision name="probe">{geometry}</collision>
          </link><joint name="guide" type="prismatic"><parent>world</parent><child>body</child>
          <axis><xyz>{xyz}</xyz><limit><lower>-100</lower><upper>100</upper></limit></axis>
          </joint></model>'''))
        persistent = ET.SubElement(drive, 'persistent')
        ET.SubElement(persistent, 'entity_name').text = name
        ET.SubElement(persistent, 'entity_type').text = 'model'
        ET.SubElement(persistent, 'force').text = ' '.join(str(force*v) for v in axis)

    lidar_topic = '/bridge_validation/pier_scan'
    yaw = math.atan2(-ny, -nx)
    world.append(ET.fromstring(f'''<model name="scan_pier"><static>true</static>
      <pose>{px+nx*22} {py+ny*22} 1 0 0 {yaw}</pose><link name="body">
      <sensor name="scanner" type="gpu_lidar">
        <always_on>true</always_on><update_rate>2</update_rate><topic>{lidar_topic}</topic>
        <lidar><scan><horizontal><samples>5</samples><resolution>1</resolution>
          <min_angle>-.27</min_angle><max_angle>.27</max_angle></horizontal></scan>
          <range><min>.1</min><max>55</max><resolution>.01</resolution></range></lidar>
      </sensor></link></model>'''))
    views = {
        'overview': (300, -650, 420, -850, -650, 20),
        'main': (-660, 210, 75, mx, my, 22),
        'under': (mx-20, my+130, 12, mx, my, 20),
    }
    for name, (x, y, z, tx, ty, tz) in views.items():
        yaw = math.atan2(ty-y, tx-x)
        pitch = math.atan2(z-tz, math.hypot(tx-x, ty-y))
        world.append(ET.fromstring(f'''<model name="camera_{name}"><static>true</static>
          <pose>{x} {y} {z} 0 {pitch} {yaw}</pose><link name="body">
          <sensor name="camera" type="camera"><always_on>true</always_on><update_rate>1</update_rate>
            <topic>/bridge_validation/{name}</topic><camera><horizontal_fov>1.05</horizontal_fov>
            <visibility_mask>268435455</visibility_mask>
            <image><width>1280</width><height>960</height><format>R8G8B8</format></image>
            <clip><near>0.5</near><far>8000</far></clip></camera></sensor></link></model>'''))
    path = tmp_path/'bridge_validation.sdf'
    ET.ElementTree(root).write(str(path))
    options = NodeOptions(); options.partition = partition
    node = Node(options)
    lock = threading.Lock()
    positions, images, times, scans = {}, {}, [], []
    names = set(probes)
    def on_pose(msg):
        stamp = msg.header.stamp.sec+msg.header.stamp.nsec*1e-9
        with lock:
            if not times or stamp > times[-1]+1e-6:
                times.append(stamp)
            for p in msg.pose:
                if p.name in names:
                    positions[p.name] = [p.position.x, p.position.y, p.position.z]
    def on_image(msg, key):
        with lock:
            images[key] = msg
    def on_scan(msg):
        with lock:
            scans[:] = list(msg.ranges)
    node.subscribe(Pose_V, '/world/bridge_validation/dynamic_pose/info', on_pose)
    node.subscribe(LaserScan, lidar_topic, on_scan)
    for key in views:
        node.subscribe(GzImage, '/bridge_validation/'+key, lambda msg, k=key: on_image(msg, k))
    process = None
    log_path = tmp_path/'gazebo.log'
    with log_path.open('w') as log:
        try:
            process = subprocess.Popen(['gz', 'sim', '-s', '-r', '--headless-rendering', '-v', '3', str(path)],
                                       stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic()+180
            while time.monotonic() < deadline:
                assert process.poll() is None, log_path.read_text()[-6000:]
                with lock:
                    if (set(views) == set(images) and scans and names <= positions.keys()
                            and times and times[-1]-times[0] > 8.0):
                        break
                time.sleep(.05)
            with lock:
                assert (set(views) == set(images) and scans and names <= positions.keys()
                        and times and times[-1]-times[0] > 8.0), (
                    positions, images.keys(), scans, times[-1] if times else None,
                    log_path.read_text()[-6000:])
                for name, (x, y, t, n) in passages.items():
                    qx, qy, _ = positions[name]
                    assert (qx-x)*n[0]+(qy-y)*n[1] > 24, (name, positions[name])
                    assert abs((qx-x)*t[0]+(qy-y)*t[1]) < .1
                hit_x, hit_y, _ = positions['pier_hit']
                stop = (hit_x-px)*nx+(hit_y-py)*ny
                deck_z = positions['deck_hit'][2]
                half_width = layout['dimensions']['pier_sections_m']['approach']['shaft_length_width_chamfer'][1]/2
                assert stop == pytest.approx(half_width+.3, abs=.15)
                assert deck_z == pytest.approx(21.0-.3, abs=.15)
                assert scans[2] == pytest.approx(22-half_width, abs=.2)
                assert all(r >= 54.9 for i, r in enumerate(scans) if i != 2), scans
                for key, msg in images.items():
                    image = Image.frombytes('RGB', (msg.width, msg.height), msg.data, 'raw', 'RGB', msg.step)
                    assert np.asarray(image).std() > 5
                    image.save(tmp_path/(key+'.png'))
            result = {'main_center_enu_m': [mx, my], 'passage_probe_enu_m': {
                          name: [x, y] for name, (x, y, _, _) in passages.items()},
                      'pier_center_enu_m': [px, py], 'probe_positions': positions,
                      'passage_probe_box_m': [10, 3, 1.6], 'contact_probe_radius_m': .3,
                      'pier_stop_from_center_m': stop, 'main_soffit_contact_center_z_m': deck_z,
                      'lidar_ranges_m': [r if math.isfinite(r) else None for r in scans], 'views': list(views)}
            (tmp_path/'validation.json').write_text(json.dumps(result, indent=2)+'\n')
            print('PASS: bridge render, main/aux/approach passage, pier/soffit contact and lidar:', tmp_path)
        finally:
            for topic in ['/world/bridge_validation/dynamic_pose/info', lidar_topic,
                          *('/bridge_validation/'+v for v in views)]:
                node.unsubscribe(topic)
            if process is not None and process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
