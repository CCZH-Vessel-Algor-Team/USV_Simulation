"""Opt-in installed CCS world render/contact check, isolated from the live world."""
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
    os.environ.get('USV_HARBOR_INTEGRATION') != '1',
    reason='requires built workspace and headless Gazebo rendering; never runs the main launch',
)


def test_installed_ccs_harbor_render_deck_shore_and_open_water(tmp_path, monkeypatch):
    import numpy as np
    import shapely
    from shapely.affinity import translate
    from shapely.geometry import shape
    from PIL import Image
    from ament_index_python.packages import get_package_share_directory

    package = Path(get_package_share_directory('usv_sim_full'))
    resources = os.pathsep.join([str(package / 'worlds/models'),
                                str(package / 'description/models'),
                                os.environ.get('GZ_SIM_RESOURCE_PATH', '')])
    monkeypatch.setenv('GZ_PARTITION', 'harbor_validation_' + uuid.uuid4().hex)
    monkeypatch.setenv('ROS_DOMAIN_ID', '204')
    monkeypatch.setenv('GZ_SIM_RESOURCE_PATH', resources)
    monkeypatch.setenv('SDF_PATH', resources)
    # Transport reads environment defaults at library load. Set them before
    # importing, and use explicit node options even if another test imported it.
    from gz.transport13 import Node, NodeOptions
    from gz.msgs10.image_pb2 import Image as ImageMessage
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.msgs10.laserscan_pb2 import LaserScan
    root = ET.parse(package / 'worlds/ccs_open_water_crop.sdf').getroot()
    world = root.find('world')
    world.set('name', 'harbor_validation')
    assert world.findtext("include[name='gaogongdao_local_harbor_v1']/pose") == '-170 270 0 0 0 0'
    # The normal GUI is neither launched nor contacted.
    world.remove(world.find('gui'))
    broadcaster = world.find("plugin[@name='gz::sim::systems::SceneBroadcaster']")
    assert broadcaster is not None
    ET.SubElement(broadcaster, 'dynamic_pose_hertz').text = '10'

    mesh = package / ('worlds/models/ccs_chart_obstacles_enc_5km/meshes/'
                      'green_island_001_collision_without_harbor.obj')
    vertices = np.array([list(map(float, line.split()[1:4]))
                         for line in mesh.read_text().splitlines() if line.startswith('v ')])
    peak = vertices[vertices[:, 2].argmax()] + [-1444.372646, 1082.602142, 0]
    scene = json.loads((package/'worlds/models/ccs_chart_obstacles_enc_5km/gaogongdao_variant.json').read_text())
    contacts = json.loads((package/'worlds/models/ccs_chart_obstacles_enc_5km/gaogongdao_contacts.json').read_text())
    bands = translate(shapely.union_all([shape(p['footprint'])
                                        for p in contacts['riprap']['outlines_local_xy']]), -170, 270)
    # The new band projects beyond the old flat quay base. Compare sphere
    # centers with the expanded outline rather than the previous bare wall.
    shoreline = bands.buffer(.5).intersection(shapely.LineString([(-160, 245), (-120, 245)]))
    shore_stop_x = shoreline.bounds[2]
    centerline = np.array(scene['connection']['centerline_enu_m'])
    crossing = centerline[42]
    tangent = centerline[43]-centerline[41]
    outward = np.array([-tangent[1], tangent[0]])/np.linalg.norm(tangent)
    side_start = crossing+outward*20
    probes = {'deck': (-145, 245, 20), 'sea_inside': (-205, 280, 20),
              'sea_outside': (-120, 260, 20), 'island': (*peak[:2], peak[2] + 10),
              'shore': (-125, 245, 0.5), 'causeway': (*crossing, 20),
              'landing': (-43, 584, 35), 'causeway_side': (*side_start, .5),
              'lateral_open_water': (-125, 100, .5)}
    horizontal_axes = {'shore': np.array([1., 0., 0.]),
                       'causeway_side': np.r_[outward, 0.],
                       'lateral_open_water': np.array([1., 0., 0.])}
    # This tests contact, not command-transport discovery. Bake a persistent
    # 2 N world force into the test world so delayed/missed cmd_force discovery
    # cannot masquerade as a mesh collision failure. The open-water control
    # must move through the same x range where the shore probe is stopped.
    drive = ET.SubElement(world, 'plugin', filename='gz-sim-apply-link-wrench-system',
                          name='gz::sim::systems::ApplyLinkWrench')
    ET.SubElement(drive, 'verbose').text = 'false'
    for name, (x, y, z) in probes.items():
        direction = horizontal_axes.get(name, np.array([0., 0., 1.]))
        axis = ' '.join(map(str, direction))
        model = ET.fromstring(f'''<model name="probe_{name}">
          <pose>{x} {y} {z} 0 0 0</pose><link name="body">
            <inertial><mass>1</mass><inertia><ixx>0.1</ixx><iyy>0.1</iyy><izz>0.1</izz></inertia></inertial>
            <collision name="ball"><geometry><sphere><radius>0.5</radius></sphere></geometry></collision>
          </link><joint name="guide" type="prismatic"><parent>world</parent><child>body</child>
            <axis><xyz>{axis}</xyz><limit><lower>-1000</lower><upper>50</upper></limit></axis>
          </joint></model>''')
        if name in horizontal_axes:
            persistent = ET.SubElement(drive, 'persistent')
            ET.SubElement(persistent, 'entity_name').text = 'probe_'+name
            ET.SubElement(persistent, 'entity_type').text = 'model'
            ET.SubElement(persistent, 'force').text = ' '.join(map(str, -2*direction))
        world.append(model)

    cameras = {'oblique': (-440, 100, 280, -110, 400, 2),
               'topdown': (-90, 420, 650, -90, 420, 0),
               'context': (-800, -500, 600, -100, 600, 0),
               'landing': (-105, 510, 35, -43, 584, 3.2)}
    for name, (x, y, z, tx, ty, tz) in cameras.items():
        yaw = math.atan2(ty - y, tx - x)
        pitch = math.atan2(z - tz, math.hypot(tx - x, ty - y))
        world.append(ET.fromstring(f'''<model name="camera_{name}"><static>true</static>
          <pose>{x} {y} {z} 0 {pitch} {yaw}</pose><link name="body">
            <sensor name="camera" type="camera"><always_on>true</always_on><update_rate>2</update_rate>
              <topic>/harbor_validation/{name}</topic><camera><horizontal_fov>0.95</horizontal_fov>
                <visibility_mask>268435455</visibility_mask>
                <image><width>1280</width><height>960</height><format>R8G8B8</format></image>
                <clip><near>0.5</near><far>10000</far></clip>
              </camera></sensor></link></model>'''))
    laser_position = crossing+outward*22
    laser_yaw = math.atan2(-outward[1], -outward[0])
    world.append(ET.fromstring(f'''<model name="causeway_lidar"><static>true</static>
      <pose>{laser_position[0]} {laser_position[1]} 1.5 0 0 {laser_yaw}</pose><link name="body">
        <sensor name="lidar" type="gpu_lidar"><always_on>true</always_on><update_rate>2</update_rate>
          <topic>/harbor_validation/causeway_scan</topic><lidar><visibility_mask>268435447</visibility_mask>
            <scan><horizontal><samples>9</samples><resolution>1</resolution>
              <min_angle>-0.02</min_angle><max_angle>0.02</max_angle></horizontal></scan>
            <range><min>0.1</min><max>80</max><resolution>0.01</resolution></range>
          </lidar></sensor></link></model>'''))
    world_file = tmp_path / 'harbor_validation.sdf'
    ET.ElementTree(root).write(world_file)
    log_file = tmp_path / 'gazebo.log'
    log = log_file.open('w')
    server = None
    options = NodeOptions()
    options.partition = os.environ['GZ_PARTITION']
    gz = Node(options)
    images, positions, scans = {}, {}, []
    lock = threading.Lock()
    topics = []

    def image_received(message, key):
        with lock:
            images[key] = message

    probe_names = {'probe_'+key: key for key in probes}

    def pose_received(message):
        # SceneBroadcaster includes every non-static model in each batch,
        # even after contact stops it. A single world stream avoids losing one
        # probe to independent per-model publisher discovery.
        with lock:
            for value in message.pose:
                key = probe_names.get(value.name)
                if key is not None:
                    positions[key] = (value.position.x, value.position.y, value.position.z)

    def save_images():
        with lock:
            for name, message in images.items():
                assert message.width == 1280 and message.height == 960
                pixels = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
                rgb = pixels[:, :message.width * 3].reshape(message.height, message.width, 3)
                assert rgb.std() > 5, 'Camera is blank'
                Image.fromarray(rgb).save(tmp_path / (name + '.png'))

    def scan_received(message):
        with lock:
            scans[:] = list(message.ranges)

    try:
        for name in cameras:
            topic = '/harbor_validation/' + name
            topics.append(topic)
            assert gz.subscribe(ImageMessage, topic,
                                lambda message, key=name: image_received(message, key))
        pose_topic = '/world/harbor_validation/dynamic_pose/info'
        topics.append(pose_topic)
        assert gz.subscribe(Pose_V, pose_topic, pose_received)
        topics.append('/harbor_validation/causeway_scan')
        assert gz.subscribe(LaserScan, topics[-1], scan_received)
        # Register feedback before server discovery, so no probe depends on
        # catching a one-off initial pose announcement.
        server = subprocess.Popen(
            ['gz', 'sim', '-s', '-r', '--headless-rendering', '-v', '3', str(world_file)],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

        def wait(predicate, timeout=60):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                assert server.poll() is None, log_file.read_text()[-10000:]
                with lock:
                    if predicate():
                        return
                time.sleep(0.05)
            raise AssertionError(f'Validation timed out; images={list(images)}; positions={positions}\n'
                                 + log_file.read_text()[-10000:])

        wait(lambda: len(images) == len(cameras))
        save_images()
        wait(lambda: len(positions) == len(probes), timeout=30)
        wait(lambda: positions['sea_inside'][2] < -30 and positions['sea_outside'][2] < -30)
        wait(lambda: abs(positions['deck'][2] - 3.7) < 0.1)
        wait(lambda: abs(positions['island'][2] - peak[2] - 0.5) < 0.15)
        wait(lambda: abs(positions['causeway'][2]-3.7) < .12)
        wait(lambda: 3.6 < positions['landing'][2] < 6)
        wait(lambda: len(scans) == 9 and 8 < min(scans) < 22)
        # Horizontal waterline approach: the old shore collision must not vanish
        # when the old, up-to-10 m quay terrain is removed.
        wait(lambda: abs(positions['shore'][0]-shore_stop_x) < .15 and
             np.dot(np.array(positions['causeway_side'][:2])-crossing, outward) < 9.5 and
             positions['lateral_open_water'][0] < -145, timeout=20)
        time.sleep(2)  # Contact must hold while the baked force is still applied.
        with lock:
            assert positions['shore'][0] == pytest.approx(shore_stop_x, abs=.15), positions
            assert abs(positions['deck'][2] - 3.7) < 0.1, positions
            assert 6.5 < np.dot(np.array(positions['causeway_side'][:2])-crossing, outward) < 9.5
            assert positions['lateral_open_water'][0] < -145, positions
            result = {'probe_positions_m': dict(positions),
                      'original_island_peak_m': peak.tolist(),
                       'max_step_size_s': float(world.findtext('physics/max_step_size')),
                       'causeway_lidar_ranges_m': list(scans),
                       'horizontal_drive': 'SDF persistent world wrench, 2 N per probe',
                       'pose_feedback_topic': pose_topic,
                      'checks': ['installed nested resources', 'Ogre2 camera images',
                                 'deck contact', 'waterline shore contact',
                                 'open water inside/outside harbor', 'retained green island contact',
                                  'causeway deck/landing and lateral contact', 'GPU lidar sees causeway',
                                  'unblocked horizontal open-water force control']}
        save_images()
        text = log_file.read_text()
        assert 'Unable to find uri' not in text
        assert 'could not be resolved' not in text
        assert 'Failed to load a world' not in text
        assert 'CustomMeshShape' not in text
        assert 'This submesh will be' not in text
        (tmp_path / 'validation.json').write_text(json.dumps(result, indent=2) + '\n')
        print('PASS: installed CCS harbor, unchanged island contact, deck and waterline collision, '
              'open water, connected deck/landing, GPU lidar and four rendered views. Artifacts:', tmp_path)
    finally:
        for topic in topics:
            gz.unsubscribe(topic)
        if server is not None and server.poll() is None:
            os.killpg(server.pid, signal.SIGINT)
            try:
                server.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(server.pid, signal.SIGKILL)
                server.wait(timeout=5)
        log.close()
