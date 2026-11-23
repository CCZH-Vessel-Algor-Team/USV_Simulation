"""Opt-in installed harbor contact checks, never connected to the live world."""
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
    os.getenv('USV_HARBOR_CONTACTS_INTEGRATION') != '1',
    reason='requires installed Gazebo assets; runs in a separate partition and ROS domain')


def test_installed_riprap_pontoons_and_all_moored_boats_have_contact(tmp_path, monkeypatch):
    import numpy as np
    import shapely
    from shapely.geometry import shape
    from ament_index_python.packages import get_package_share_directory

    package = Path(get_package_share_directory('usv_sim_full'))
    models = Path(os.getenv('USV_HARBOR_TEST_MODELS', str(package/'worlds/models')))
    chart = models/'ccs_chart_obstacles_enc_5km'
    report = json.loads((chart/'gaogongdao_contacts.json').read_text())
    origin = np.array(report['harbor_pose_enu_m_rad'][:3])
    partition = 'harbor_contacts_'+uuid.uuid4().hex
    resources = os.pathsep.join(dict.fromkeys((
        str(models), str(package/'worlds/models'), str(package/'description/models'),
        os.getenv('GZ_SIM_RESOURCE_PATH', ''))))
    monkeypatch.setenv('GZ_PARTITION', partition)
    monkeypatch.setenv('ROS_DOMAIN_ID', '203')
    monkeypatch.setenv('GZ_SIM_RESOURCE_PATH', resources)
    monkeypatch.setenv('SDF_PATH', resources)
    from gz.transport13 import Node, NodeOptions
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.msgs10.laserscan_pb2 import LaserScan

    root = ET.parse(package/'worlds/ccs_open_water_crop.sdf').getroot()
    world = root.find('world')
    world.set('name', 'harbor_contacts')
    world.remove(world.find('gui'))
    for item in list(world.findall('include')):
        if item.findtext('name') != 'gaogongdao_local_harbor_v1':
            world.remove(item)
    ET.SubElement(world.find("plugin[@name='gz::sim::systems::SceneBroadcaster']"),
                  'dynamic_pose_hertz').text = '30'
    drive = ET.SubElement(world, 'plugin', filename='gz-sim-apply-link-wrench-system',
                          name='gz::sim::systems::ApplyLinkWrench')
    radius = .15
    expected = {}
    free_names, lateral, scanners = [], {}, {}

    def probe(name, xyz, axis=(0, 0, 1), force=None):
        position = np.asarray(xyz)+origin
        direction = ' '.join(map(str, axis))
        world.append(ET.fromstring(f'''<model name="{name}">
          <pose>{' '.join(map(str, position))} 0 0 0</pose><link name="body">
            <inertial><mass>1</mass><inertia><ixx>.1</ixx><iyy>.1</iyy><izz>.1</izz></inertia></inertial>
            <collision name="ball"><geometry><sphere><radius>{radius}</radius></sphere></geometry></collision>
          </link><joint name="guide" type="prismatic"><parent>world</parent><child>body</child>
            <axis><xyz>{direction}</xyz><limit><lower>-100</lower><upper>100</upper></limit></axis>
          </joint></model>'''))
        if force is not None:
            persistent = ET.SubElement(drive, 'persistent')
            ET.SubElement(persistent, 'entity_name').text = name
            ET.SubElement(persistent, 'entity_type').text = 'model'
            ET.SubElement(persistent, 'force').text = ' '.join(str(force*v) for v in axis)

    for spec in report['boxes']:
        name = 'drop_'+spec['name']
        x, y, z = spec['pose_m_rad'][:3]
        top = z+spec['size_m'][2]/2
        probe(name, (x, y, top+3))
        expected[name] = top+radius
    for i, part in enumerate(report['riprap']['outlines_local_xy']):
        point = shape(part['footprint']).representative_point()
        name = f'drop_riprap_{i}'
        top = part['top_m']
        probe(name, (*point.coords[0], top+3))
        expected[name] = top+radius
    for i, xy in enumerate(((-36, -44), (-26, -48), (3, 21.5), (20, 108.5), (-35, 10))):
        name = f'free_gap_{i}'
        probe(name, (*xy, 5))
        free_names.append(name)

    # Cross the exposed side of a hull and a main pontoon while a steady force
    # keeps pressing. Test at the actual visual heights of the static model.
    for name, target in (('side_boat', 'moored_boat_red_02'), ('side_pontoon', 'pontoon_main_02')):
        spec = next(s for s in report['boxes'] if s['name'] == target)
        x, y, z, _, _, yaw = spec['pose_m_rad']
        normal = np.array([-math.sin(yaw), math.cos(yaw), 0.0])
        half = spec['size_m'][1]/2
        start = np.array([x, y, z])+normal*(half+2)
        probe(name, start, tuple(-normal), .75)
        lateral[name] = (np.array([x, y, z])+origin, normal, half+radius)
        scanners[name] = (np.array([x, y, z])+origin+normal*(half+4), tuple(-normal))

    # Side approach into the new riprap band, clear of the old quay base.
    outlines = [shape(p['footprint']) for p in report['riprap']['outlines_local_xy']]
    band = shapely.union_all(outlines)
    y = -80.0
    slice_ = band.intersection(shapely.LineString([(-100, y), (100, y)]))
    edge = slice_.bounds[2]
    probe('side_riprap', (edge+3, y, .35), (-1, 0, 0), .75)
    lateral['side_riprap'] = (np.array([edge, y, .35])+origin, np.array([1, 0, 0]), radius)
    scanners['side_riprap'] = (np.array([edge+5, y, 1.0])+origin, (-1, 0, 0))

    for name, (pos, axis) in scanners.items():
        yaw = math.atan2(axis[1], axis[0])
        world.append(ET.fromstring(f'''<model name="lidar_{name}"><static>true</static>
          <pose>{' '.join(map(str, pos))} 0 0 {yaw}</pose><link name="body">
            <sensor name="lidar" type="gpu_lidar"><always_on>true</always_on><update_rate>3</update_rate>
              <topic>/harbor_contacts/{name}</topic><lidar>
              <scan><horizontal><samples>3</samples><resolution>1</resolution>
                <min_angle>-.01</min_angle><max_angle>.01</max_angle></horizontal></scan>
              <range><min>.05</min><max>30</max><resolution>.01</resolution></range></lidar>
            </sensor></link></model>'''))
    path = tmp_path/'world.sdf'
    ET.ElementTree(root).write(path)
    options = NodeOptions()
    options.partition = partition
    node = Node(options)
    positions, scans, sim_time = {}, {}, [0.0]
    lock = threading.Lock()

    def on_pose(msg):
        with lock:
            sim_time[0] = msg.header.stamp.sec+msg.header.stamp.nsec*1e-9
            for p in msg.pose:
                if p.name in expected or p.name in free_names or p.name in lateral:
                    positions[p.name] = [p.position.x, p.position.y, p.position.z]

    def on_scan(msg, name):
        with lock:
            scans[name] = list(msg.ranges)

    topics = ['/world/harbor_contacts/dynamic_pose/info']
    node.subscribe(Pose_V, topics[0], on_pose)
    for name in scanners:
        topic = '/harbor_contacts/'+name
        topics.append(topic)
        node.subscribe(LaserScan, topic, lambda msg, key=name: on_scan(msg, key))
    log_path = tmp_path/'gazebo.log'
    process = None
    with log_path.open('w') as log:
        try:
            process = subprocess.Popen(['gz', 'sim', '-s', '-r', '--headless-rendering', '-v', '3', str(path)],
                                       stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic()+100
            while time.monotonic() < deadline:
                assert process.poll() is None, log_path.read_text()[-8000:]
                with lock:
                    ready = (sim_time[0] >= 5 and len(positions) == len(expected)+len(free_names)+len(lateral)
                             and scans.keys() >= scanners.keys())
                if ready:
                    break
                time.sleep(.05)
            with lock:
                assert ready, (sim_time, positions, scans.keys())
                for name, z in expected.items():
                    assert positions[name][2] == pytest.approx(z, abs=.06), (name, positions[name], z)
                for name in free_names:
                    assert positions[name][2] < -15, (name, positions[name])
                stopped = {}
                for name, (center, normal, gap) in lateral.items():
                    distance = float(np.dot(np.asarray(positions[name])-center, normal))
                    assert distance == pytest.approx(gap, abs=.08), (name, positions[name], distance, gap)
                    stopped[name] = distance
                for name, ranges in scans.items():
                    assert len(ranges) == 3 and all(2 < v < 8 for v in ranges), (name, ranges)
                result = {'vertical_contact_count': len(expected), 'free_gap_count': len(free_names),
                          'probe_radius_m': radius, 'contact_expected_center_z_m': expected,
                          'positions_enu_m': positions, 'lateral_contact_distance_m': stopped,
                          'gpu_lidar_ranges_m': scans, 'simulation_time_s': sim_time[0]}
            (tmp_path/'validation.json').write_text(json.dumps(result, indent=2)+'\n')
            text = log_path.read_text()
            assert 'Unable to find uri' not in text and 'CustomMeshShape' not in text
            print('PASS: 14 boats, 12 pontoon pieces, 3 riprap shells, open gaps and side contact/lidar:', tmp_path)
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
