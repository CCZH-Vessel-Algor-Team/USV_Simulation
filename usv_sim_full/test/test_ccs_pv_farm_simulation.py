"""Opt-in active-CCS PV scene check; launches only its own isolated Gazebo server."""
import json
import math
import os
from pathlib import Path
import signal
import statistics
import subprocess
import threading
import time
import uuid
import xml.etree.ElementTree as ET

import pytest


@pytest.mark.skipif(os.getenv('USV_PV_FARM_INTEGRATION') != '1',
                    reason='requires installed Gazebo assets and headless rendering')
def test_installed_pv_renders_contacts_piles_and_keeps_hill_rotors(tmp_path, monkeypatch):
    import numpy as np
    from PIL import Image
    from ament_index_python.packages import get_package_share_directory

    package = Path(get_package_share_directory('usv_sim_full'))
    resources = os.pathsep.join((str(package/'worlds/models'), str(package/'description/models'),
                                 os.getenv('GZ_SIM_RESOURCE_PATH', '')))
    partition = 'pv_validation_'+uuid.uuid4().hex
    monkeypatch.setenv('GZ_PARTITION', partition)
    monkeypatch.setenv('GZ_SIM_RESOURCE_PATH', resources)
    monkeypatch.setenv('SDF_PATH', resources)
    from gz.transport13 import Node, NodeOptions
    from gz.msgs10.image_pb2 import Image as GzImage
    from gz.msgs10.pose_v_pb2 import Pose_V

    layout = json.loads((package/'worlds/models/ccs_offshore_pv_farm/layout.json').read_text())
    root = ET.parse(package/'worlds/ccs_open_water_crop.sdf').getroot()
    world = root.find('world')
    world.set('name', 'pv_validation')
    world.remove(world.find('gui'))
    ET.SubElement(world.find("plugin[@name='gz::sim::systems::SceneBroadcaster']"),
                  'dynamic_pose_hertz').text = '10'
    cx, cy = layout['center_enu_m']
    sx, sy = next(site['center_enu_m'] for site in layout['placements']
                  if site['model'] == 'offshore_pv_platform')
    views = {
        'overview': (cx+780, cy-890, 920, cx, cy, 0),
        'detail': (sx-76, sy-90, 48, sx, sy, 3),
    }
    for name, (x, y, z, tx, ty, tz) in views.items():
        yaw = math.atan2(ty-y, tx-x)
        pitch = math.atan2(z-tz, math.hypot(tx-x, ty-y))
        world.append(ET.fromstring(f'''<model name="camera_{name}"><static>true</static>
          <pose>{x} {y} {z} 0 {pitch} {yaw}</pose><link name="body">
          <sensor name="camera" type="camera"><always_on>true</always_on><update_rate>1</update_rate>
            <topic>/pv_validation/{name}</topic><camera><horizontal_fov>1.10</horizontal_fov>
            <visibility_mask>268435455</visibility_mask>
            <image><width>1280</width><height>960</height><format>R8G8B8</format></image>
            <clip><near>0.5</near><far>8000</far></clip></camera></sensor></link></model>'''))

    # Test the lightweight variant, not only the retained detailed model.
    x, y = layout['placements'][0]['center_enu_m']
    px, py = x-18, y-10.5
    drive = ET.SubElement(world, 'plugin', filename='gz-sim-apply-link-wrench-system',
                          name='gz::sim::systems::ApplyLinkWrench')
    for name, offset, z in (('pile_probe', 0, 1), ('channel_probe', 4, 1),
                           ('under_panel_probe', 4, 4)):
        world.append(ET.fromstring(f'''<model name="{name}">
           <pose>{px-8} {py+offset} {z} 0 0 0</pose><link name="body">
           <inertial><mass>1</mass><inertia><ixx>.1</ixx><iyy>.1</iyy><izz>.1</izz></inertia></inertial>
           <collision name="probe"><geometry><sphere><radius>.3</radius></sphere></geometry></collision>
           </link><joint name="guide" type="prismatic"><parent>world</parent><child>body</child>
           <axis><xyz>1 0 0</xyz><limit><lower>-10</lower><upper>30</upper></limit></axis>
           </joint></model>'''))
        force = ET.SubElement(drive, 'persistent')
        ET.SubElement(force, 'entity_name').text = name
        ET.SubElement(force, 'entity_type').text = 'model'
        ET.SubElement(force, 'force').text = '4 0 0'
    source = ET.parse(package/'worlds/models/offshore_pv_platform/model.sdf')
    panel_y = float(next(c for c in source.findall('model/link/collision')
                         if c.get('name').startswith('panel_strip_')).findtext('pose').split()[1])
    world.append(ET.fromstring(f'''<model name="panel_contact_probe">
       <pose>{x+2} {y+panel_y} 4 0 0 0</pose><link name="body">
       <inertial><mass>1</mass><inertia><ixx>.1</ixx><iyy>.1</iyy><izz>.1</izz></inertia></inertial>
       <collision name="probe"><geometry><sphere><radius>.3</radius></sphere></geometry></collision>
       </link><joint name="guide" type="prismatic"><parent>world</parent><child>body</child>
       <axis><xyz>0 0 1</xyz><limit><lower>-2</lower><upper>6</upper></limit></axis>
       </joint></model>'''))
    force = ET.SubElement(drive, 'persistent')
    ET.SubElement(force, 'entity_name').text = 'panel_contact_probe'
    ET.SubElement(force, 'entity_type').text = 'model'
    ET.SubElement(force, 'force').text = '0 0 14'
    path = tmp_path/'pv_validation.sdf'
    ET.ElementTree(root).write(path)
    options = NodeOptions()
    options.partition = partition
    node = Node(options)
    lock = threading.Lock()
    images, positions, previous, speeds = {}, {}, {}, {}
    hill_names = {p['name'] for p in layout['hill_turbines']}
    probes = {'pile_probe', 'channel_probe', 'under_panel_probe', 'panel_contact_probe'}
    pose_topic = '/world/pv_validation/dynamic_pose/info'
    observed_names = set()
    times = []

    def on_pose(msg):
        stamp = msg.header.stamp.sec+msg.header.stamp.nsec*1e-9
        with lock:
            if not times or stamp > times[-1][0]+.001:
                times.append((stamp, time.monotonic()))
            for p in msg.pose:
                observed_names.add(p.name)
                if p.name in hill_names | probes or p.name.startswith('wind_offshore_'):
                    positions[p.name] = [p.position.x, p.position.y, p.position.z]
                if p.name.split('::')[-1] == 'rotor':
                    q = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
                    if p.id in previous:
                        t, old = previous[p.id]
                        if .01 < stamp-t < 2.5:
                            speeds.setdefault(p.id, []).append(
                                2*math.acos(float(np.clip(abs(np.dot(q, old)), 0., 1.)))/(stamp-t))
                    previous[p.id] = (stamp, q)

    def on_image(msg, name):
        with lock:
            images[name] = msg

    topics = [pose_topic]
    assert node.subscribe(Pose_V, pose_topic, on_pose)
    for name in views:
        topic = '/pv_validation/'+name
        assert node.subscribe(GzImage, topic, lambda msg, key=name: on_image(msg, key))
        topics.append(topic)
    process = None
    log_path = tmp_path/'gazebo.log'
    with log_path.open('w') as log:
        try:
            process = subprocess.Popen(
                ['gz', 'sim', '-s', '-r', '--headless-rendering', '-v', '3', str(path)],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic()+150
            while time.monotonic() < deadline:
                assert process.poll() is None, log_path.read_text()[-6000:]
                with lock:
                    ready = (set(views) == set(images) and hill_names | probes <= positions.keys()
                             and len(speeds) == 4 and all(len(v) >= 4 for v in speeds.values())
                             and all(positions[k][0] > px+2 for k in ('channel_probe', 'under_panel_probe')))
                    if ready:
                        break
                time.sleep(.05)
            assert ready, (f'images={list(images)}, poses={positions}, rotors={len(speeds)}, '
                           f'last_stamp={times[-1][0] if times else None}, '
                           f'rotor names={[n for n in observed_names if "rotor" in n]}\n'
                           + log_path.read_text()[-6000:])
            with lock:
                assert px-.9 < positions['pile_probe'][0] < px-.6
                assert 5.8 < positions['panel_contact_probe'][2] < 7.55
                assert not any(name.startswith('wind_offshore_') for name in positions)
                measured = [statistics.median(v) for v in speeds.values()]
                assert measured == pytest.approx([.7906]*4, abs=.03)
                for site in layout['hill_turbines']:
                    assert positions[site['name']] == pytest.approx(site['pose_enu_m_rad'][:3], abs=.01)
                for name, msg in images.items():
                    image = Image.frombytes('RGB', (msg.width, msg.height), msg.data, 'raw', 'RGB', msg.step)
                    assert np.asarray(image).std() > 5
                    image.save(tmp_path/(name+'.png'))
                result = {'platform_count': layout['platform_count'], 'rotor_speeds_rad_s': measured,
                          'pile_center_enu_m': [px, py], 'probe_positions_enu_m': {k: positions[k] for k in probes},
                          'views': list(views),
                          'observed_sim_seconds': times[-1][0]-times[0][0],
                          'observation_wall_seconds': times[-1][1]-times[0][1]}
            for line in log_path.read_text().splitlines():
                if any(s in line for s in ('Error Code', 'Unable to find uri', 'Failed to load')):
                    assert not any(s in line for s in ('offshore_pv', 'pv_front', 'pv_back', 'module_front', 'module_back')), line
            (tmp_path/'validation.json').write_text(json.dumps(result, indent=2)+'\n')
            print('PASS: active PV farm rendered; contact, passages and four retained rotors verified:', tmp_path)
        finally:
            for topic in topics:
                node.unsubscribe(topic)
            if process is not None and process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
