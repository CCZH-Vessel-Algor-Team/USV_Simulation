"""Opt-in regression for the archived 16-turbine layout (not the active PV scene)."""
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


pytestmark = pytest.mark.skipif(os.environ.get('USV_WIND_FARM_INTEGRATION') != '1',
                               reason='requires installed Gazebo world and headless rendering')


def test_installed_wind_farm_rotates_stays_anchored_and_blocks_only_piles(tmp_path, monkeypatch):
    import numpy as np
    from PIL import Image
    from ament_index_python.packages import get_package_share_directory

    package = Path(get_package_share_directory('usv_sim_full'))
    resources = os.pathsep.join([str(package/'worlds/models'), str(package/'description/models'),
                                 os.environ.get('GZ_SIM_RESOURCE_PATH', '')])
    monkeypatch.setenv('GZ_PARTITION', 'wind_validation_'+uuid.uuid4().hex)
    monkeypatch.setenv('GZ_SIM_RESOURCE_PATH', resources)
    monkeypatch.setenv('SDF_PATH', resources)
    from gz.transport13 import Node, NodeOptions
    from gz.msgs10.image_pb2 import Image as ImageMessage
    from gz.msgs10.pose_v_pb2 import Pose_V

    layout = json.loads((package/'worlds/models/ccs_wind_farm/layout.json').read_text())
    expected = {item['name']: item['pose_enu_m_rad'][:3] for item in layout['placements']}
    root = ET.parse(package/'worlds/ccs_open_water_crop.sdf').getroot()
    world = root.find('world')
    world.set('name', 'wind_validation')
    world.remove(world.find('gui'))
    for item in world.findall('include'):
        if item.findtext('uri') in ('model://ccs_hill_wind_farm', 'model://ccs_offshore_pv_farm'):
            world.remove(item)
    world.append(ET.fromstring('<include><uri>model://ccs_wind_farm</uri>'
                               '<name>CCS Wind Farm</name></include>'))
    ET.SubElement(world.find("plugin[@name='gz::sim::systems::SceneBroadcaster']"),
                  'dynamic_pose_hertz').text = '10'
    first = layout['placements'][0]['pose_enu_m_rad']
    hills = np.array([p['pose_enu_m_rad'][:3] for p in layout['placements'] if p['kind'] == 'hill'])
    center = hills.mean(axis=0)
    cameras = {'overview': (0, 0, 6500, 0, 0, 0),
               'offshore': (2750, -3200, 650, 1350, -1400, 100),
               'hills': (-100, -300, 750, *center[:2], center[2]+60),
               'closeup': (first[0]+280, first[1]-230, 180, first[0], first[1], 145)}
    # Close views expose service geometry and material seams hidden in a whole
    # rotor shot. Transform the offsets into the turbine's unchanged local yaw.
    c, s = math.cos(first[5]), math.sin(first[5])
    def local(point):
        x, y, z = point
        return (first[0]+c*x-s*y, first[1]+s*x+c*y, first[2]+z)
    cameras['base_detail'] = (*local((32, -34, 23)), *local((1, 0, 12)))
    cameras['nacelle_detail'] = (*local((-32, -32, 165)), *local((-2, 0, 149)))
    for name, (x, y, z, tx, ty, tz) in cameras.items():
        yaw = math.pi/2 if name == 'overview' else math.atan2(ty-y, tx-x)
        pitch = math.atan2(z-tz, math.hypot(tx-x, ty-y))
        world.append(ET.fromstring(f'''<model name="wind_camera_{name}"><static>true</static>
          <pose>{x} {y} {z} 0 {pitch} {yaw}</pose><link name="body">
            <sensor name="camera" type="camera"><always_on>true</always_on><update_rate>2</update_rate>
              <topic>/wind_validation/{name}</topic><camera><horizontal_fov>1.05</horizontal_fov>
                <visibility_mask>268435455</visibility_mask>
                <image><width>1280</width><height>960</height><format>R8G8B8</format></image>
                <clip><near>1</near><far>15000</far></clip>
              </camera></sensor></link></model>'''))
    drive = ET.SubElement(world, 'plugin', filename='gz-sim-apply-link-wrench-system',
                          name='gz::sim::systems::ApplyLinkWrench')
    ET.SubElement(drive, 'verbose').text = 'false'
    for name, dy in [('wind_pile_probe', 0), ('wind_channel_probe', 50)]:
        world.append(ET.fromstring(f'''<model name="{name}">
          <pose>{first[0]+25} {first[1]+dy} 1 0 0 0</pose><link name="body">
            <inertial><mass>1</mass><inertia><ixx>.1</ixx><iyy>.1</iyy><izz>.1</izz></inertia></inertial>
            <collision name="ball"><geometry><sphere><radius>.5</radius></sphere></geometry></collision>
          </link><joint name="guide" type="prismatic"><parent>world</parent><child>body</child>
            <axis><xyz>1 0 0</xyz><limit><lower>-1000</lower><upper>50</upper></limit></axis>
          </joint></model>'''))
        persistent = ET.SubElement(drive, 'persistent')
        ET.SubElement(persistent, 'entity_name').text = name
        ET.SubElement(persistent, 'entity_type').text = 'model'
        ET.SubElement(persistent, 'force').text = '-2 0 0'
    path = tmp_path/'wind_validation.sdf'
    ET.ElementTree(root).write(path)
    options = NodeOptions()
    options.partition = os.environ['GZ_PARTITION']
    node = Node(options)
    images, positions, rotors, speeds = {}, {}, {}, {}
    lock = threading.Lock()
    topics = []
    server = None
    log_path = tmp_path/'gazebo.log'
    log = log_path.open('w')

    def image_received(message, key):
        with lock:
            images[key] = message

    def poses_received(message):
        stamp = message.header.stamp.sec+message.header.stamp.nsec*1e-9
        with lock:
            for pose in message.pose:
                if pose.name in expected or pose.name in ('wind_pile_probe', 'wind_channel_probe', 'ccs_wind_farm'):
                    positions[pose.name] = [pose.position.x, pose.position.y, pose.position.z]
                if pose.name != 'rotor':
                    continue
                q = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
                if pose.id in rotors:
                    previous_stamp, previous_q = rotors[pose.id]
                    dt = stamp-previous_stamp
                    if .01 < dt < 1:
                        angle = 2*math.acos(float(np.clip(abs(np.dot(q, previous_q)), 0, 1)))
                        speeds.setdefault(pose.id, []).append(angle/dt)
                rotors[pose.id] = (stamp, q)

    def save_images():
        with lock:
            for name, message in images.items():
                assert (message.width, message.height) == (1280, 960)
                image = Image.frombytes('RGB', (message.width, message.height), message.data,
                                        'raw', 'RGB', message.step)
                assert np.asarray(image).std() > 4
                image.save(tmp_path/f'{name}.png')

    try:
        for name in cameras:
            topic = '/wind_validation/'+name
            topics.append(topic)
            assert node.subscribe(ImageMessage, topic, lambda msg, key=name: image_received(msg, key))
        topics.append('/world/wind_validation/dynamic_pose/info')
        assert node.subscribe(Pose_V, topics[-1], poses_received)
        server = subprocess.Popen(['gz', 'sim', '-s', '-r', '--headless-rendering', '-v', '3', str(path)],
                                  stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

        def wait(predicate, timeout=60):
            deadline = time.monotonic()+timeout
            while time.monotonic() < deadline:
                assert server.poll() is None, log_path.read_text()[-10000:]
                with lock:
                    if predicate():
                        return
                time.sleep(.05)
            save_images()
            raise AssertionError(f'Timed out; positions={positions}; rotor_ids={list(rotors)}\n'
                                 +log_path.read_text()[-10000:])

        wait(lambda: len(images) == len(cameras))
        save_images()
        wait(lambda: all(name in positions for name in expected) and len(speeds) == 16
             and all(len(values) > 15 for values in speeds.values()))
        wait(lambda: 'wind_pile_probe' in positions and 'wind_channel_probe' in positions
             and positions['wind_pile_probe'][0] < first[0]+5.8
             and positions['wind_channel_probe'][0] < first[0]-10)
        time.sleep(2)
        with lock:
            errors = {name: float(np.linalg.norm(np.array(positions[name])-value)) for name, value in expected.items()}
            assert max(errors.values()) < .05, (errors, positions.get('ccs_wind_farm'))
            measured = [statistics.median(values) for values in speeds.values()]
            assert all(abs(value-.7906) < .03 for value in measured), measured
            assert 5.2 < positions['wind_pile_probe'][0]-first[0] < 5.8
            assert positions['wind_channel_probe'][0] < first[0]-10
            result = {'model_position_errors_m': errors, 'rotor_speed_rad_s': measured,
                      'probe_positions_m': {name: positions[name] for name in ('wind_pile_probe', 'wind_channel_probe')},
                      'camera_views': list(images), 'offshore_count': 12, 'hill_count': 4,
                      'checks': ['installed resources render', '16 anchored turbines', '16 rotors at supplied speed',
                                  'pile waterline collision', 'open channel control', 'north-up southeast layout',
                                  'appearance-only service platform and nacelle close views']}
        save_images()
        text = log_path.read_text()
        for error in ('Unable to find uri', 'could not be resolved', 'Failed to load a world',
                      'CustomMeshShape', 'This submesh will be', 'Error Code'):
            assert error not in text, text[-10000:]
        (tmp_path/'validation.json').write_text(json.dumps(result, indent=2)+'\n')
        print('PASS: wind farm renders, all 16 rotors turn, anchors stay fixed, pile stops the ball, channel stays open.', tmp_path)
    finally:
        for topic in topics:
            node.unsubscribe(topic)
        if server is not None and server.poll() is None:
            os.killpg(server.pid, signal.SIGINT)
            try:
                server.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(server.pid, signal.SIGKILL)
                server.wait(timeout=5)
        log.close()
