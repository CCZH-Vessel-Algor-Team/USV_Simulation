"""Opt-in installed fleet test. Never launches the main cleanup/bringup entry."""
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


pytestmark = pytest.mark.skipif(os.environ.get('USV_FLEET_INTEGRATION') != '1',
                               reason='requires installed ROS/Gazebo, isolated domain and rendering')


def test_installed_fleet_spawn_physics_dimensions_roundtrip_and_cleanup(tmp_path, monkeypatch):
    # Set both transports BEFORE importing their libraries. ROS domain is supplied
    # by the caller so it can first check that no existing process uses it.
    assert os.environ.get('ROS_DOMAIN_ID') not in (None, '0', '33', '42')
    partition = 'fleet_validation_'+uuid.uuid4().hex
    monkeypatch.setenv('GZ_PARTITION', partition)
    monkeypatch.setenv('QT_QPA_PLATFORM', 'offscreen')
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    from gz.transport13 import Node as GzNode, NodeOptions
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.msgs10.image_pb2 import Image as ImageMessage
    from nav2_colregs_msgs.msg import TrackedShipList
    from usv_interfaces.msg import DynamicShipSnapshot, GlobalTrackArray
    from usv_interfaces.srv import (ListDynamicShipModels, SpawnDynamicShipV2,
                                    DeleteDynamicShip, ClearDynamicShips)
    from usv_sim_full.dynamic_ship_catalog import ShipCatalog
    from PIL import Image
    import numpy as np

    package = Path(get_package_share_directory('usv_sim_full'))
    catalog = ShipCatalog(package/'description/models/target_fleet/catalog.json')
    plugins = str(Path(get_package_prefix('vrx_gz'))/'lib')
    monkeypatch.setenv('GZ_SIM_SYSTEM_PLUGIN_PATH', plugins+os.pathsep+os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''))
    resources = os.pathsep.join((str(package/'description/models'), str(package/'worlds/models'),
                                 os.environ.get('GZ_SIM_RESOURCE_PATH', '')))
    monkeypatch.setenv('GZ_SIM_RESOURCE_PATH', resources)
    root = ET.fromstring('''<sdf version="1.9"><world name="fleet_validation">
      <gravity>0 0 -9.81</gravity>
      <physics name="physics" type="ignored"><max_step_size>0.005</max_step_size>
        <real_time_factor>2</real_time_factor></physics>
      <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
      <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
      <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster">
        <dynamic_pose_hertz>20</dynamic_pose_hertz></plugin>
      <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
      <scene><ambient>0.6 0.6 0.6 1</ambient><background>0.2 0.35 0.5 1</background></scene>
      <light name="sun" type="directional"><pose>0 0 100 0 0 0</pose>
        <diffuse>0.8 0.8 0.8 1</diffuse><direction>-0.3 0.2 -1</direction></light>
      <model name="ownship_scope_probe"><pose>-1000 -1000 2 0 0 0</pose><link name="base_link">
        <inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial>
        <collision name="box"><geometry><box><size>1 1 1</size></box></geometry></collision>
      </link></model>
    </world></sdf>''')
    world = root.find('world')
    for key, (x, y, z, tx, ty, tz) in {'small': (50, -55, 45, 10, 0, 0),
                                     'large': (1300, -320, 230, 1010, 0, 0)}.items():
        yaw = math.atan2(ty-y, tx-x)
        pitch = math.atan2(z-tz, math.hypot(tx-x, ty-y))
        world.append(ET.fromstring(f'''<model name="camera_{key}"><static>true</static>
          <pose>{x} {y} {z} 0 {pitch} {yaw}</pose><link name="body">
            <sensor name="camera" type="camera"><always_on>true</always_on><update_rate>1</update_rate>
              <topic>/fleet_validation/{key}</topic><camera><horizontal_fov>1.2</horizontal_fov>
                <image><width>960</width><height>720</height><format>R8G8B8</format></image>
                <clip><near>0.2</near><far>5000</far></clip>
              </camera></sensor></link></model>'''))
    world_file = tmp_path/'fleet_validation.sdf'
    ET.ElementTree(root).write(world_file)
    processes, handles = [], []
    lock, poses, history, images = threading.Lock(), {}, {}, {}
    snapshots, tracks, truths = {}, {}, {}
    options = NodeOptions()
    options.partition = partition
    transport = GzNode(options)
    pose_topic = '/world/fleet_validation/dynamic_pose/info'

    def on_pose(message):
        stamp = message.header.stamp.sec+message.header.stamp.nsec*1e-9
        with lock:
            for p in message.pose:
                if '::' in p.name or p.name == 'base_link':
                    continue
                q = p.orientation
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                values = (stamp, p.position.x, p.position.y, p.position.z, yaw,
                          q.x, q.y, q.z, q.w)
                poses[p.name] = values
                history.setdefault(p.name, []).append(values)

    def on_image(message, name):
        with lock:
            images[name] = message

    assert transport.subscribe(Pose_V, pose_topic, on_pose)
    for name in ('small', 'large'):
        assert transport.subscribe(ImageMessage, '/fleet_validation/'+name,
                                   lambda msg, key=name: on_image(msg, key))
    context = Context()
    rclpy.init(context=context)
    client_node = rclpy.create_node('fleet_validation_client', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(client_node)

    def remember(target, message):
        stamp = (message.header.stamp.sec, message.header.stamp.nanosec)
        target[stamp] = message
        if len(target) > 200:
            del target[next(iter(target))]

    client_node.create_subscription(DynamicShipSnapshot, '/dynamic_ship/snapshot', lambda msg: remember(snapshots, msg), 50)
    client_node.create_subscription(TrackedShipList, '/dynamic_ship/tracked_ships', lambda msg: remember(tracks, msg), 50)
    client_node.create_subscription(GlobalTrackArray, '/fleet_validation/truth', lambda msg: remember(truths, msg), 50)
    clients = {key: client_node.create_client(cls, path) for key, cls, path in (
        ('models', ListDynamicShipModels, '/dynamic_ship/models'),
        ('spawn', SpawnDynamicShipV2, '/dynamic_ship/spawn_v2'),
        ('delete', DeleteDynamicShip, '/dynamic_ship/delete'),
        ('clear', ClearDynamicShips, '/dynamic_ship/clear'))}

    def launch(name, command):
        log = (tmp_path/(name+'.log')).open('w')
        handles.append(log)
        proc = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(proc)
        return proc

    def wait(predicate, timeout=60):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            for process in processes:
                assert process.poll() is None, f'Process exited; logs in {tmp_path}'
            executor.spin_once(timeout_sec=.05)
            if predicate():
                return
        raise AssertionError(f'Timed out; poses={poses}; logs in {tmp_path}')

    def call(key, request):
        wait(lambda: clients[key].service_is_ready(), 30)
        future = clients[key].call_async(request)
        wait(future.done, 25)
        result = future.result()
        assert result.success, (key, result.message, tmp_path)
        return result

    expected = {}
    try:
        launch('gazebo', ['gz', 'sim', '-s', '-r', '--headless-rendering', '-v', '3', str(world_file)])
        launch('clock', [str(Path(get_package_prefix('ros_gz_bridge'))/'lib/ros_gz_bridge/parameter_bridge'),
                         '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'])
        launch('manager', [str(Path(get_package_prefix('usv_sim_full'))/'lib/usv_sim_full/dynamic_ship_manager_node'),
                           '--ros-args', '-p', 'use_sim_time:=true', '-p', 'world_name:=fleet_validation',
                           '-p', 'fleet_spawn_check_map:=false', '-p', "ownship_model:=''",
                           '-p', 'default_mesh_profile:='+str(package/'description/models/target_ship/10m_mesh_profile.yaml')])
        launch('truth', [str(Path(get_package_prefix('ground_truth_sensor_sim'))/'lib/ground_truth_sensor_sim/dynamic_ship_to_ground_truth'),
                         '--ros-args', '-p', 'use_sim_time:=true', '-p', 'input_mode:=snapshot',
                         '-p', 'input_topic:=/dynamic_ship/snapshot', '-p', 'output_topic:=/fleet_validation/truth'])
        listed = call('models', ListDynamicShipModels.Request())
        assert len(listed.models) == 13
        # ROS services can be ready before SceneBroadcaster finishes discovery.
        wait(lambda: 'ownship_scope_probe' in poses and len(images) == 2, 60)
        native = [m for m in catalog.models.values() if m.backend == 'fleet_sdf']
        jobs = [('small', 'vb_rumba', 0., 0.), ('large', 'munich_maersk', 1000., 0.),
                ('same_type', 'vb_rumba', 0., 250.)]
        jobs += [('fleet_'+m.model_id, m.model_id, 1000.*(i%3), 1000.+1000.*(i//3))
                 for i, m in enumerate(native) if m.model_id not in ('vb_rumba', 'munich_maersk')]
        jobs.append(('legacy_control', 'legacy_10m', -1000., 0.))
        for name, model_id, x, y in jobs:
            request = SpawnDynamicShipV2.Request(name=name, model_id=model_id, speed=1., half_distance=10.)
            request.pose.position.x, request.pose.position.y, request.pose.orientation.w = x, y, 1.
            call('spawn', request)
            expected[name] = catalog.get(model_id)
            # Scene creation confirmation precedes pose/velocity bootstrap. A
            # following placement correctly fails closed until feedback exists.
            wait(lambda: snapshots and any(s.model_name == name for s in
                 next(reversed(snapshots.values())).ships), 30)
        count = len(expected)

        def complete_chain():
            shared = set(snapshots) & set(tracks) & set(truths)
            return any(len(snapshots[s].ships) == count and len(truths[s].tracks) == count for s in shared)

        wait(complete_chain, 60)

        def roundtrips():
            with lock:
                for name, start in (('small', 0.), ('large', 1000.)):
                    values = history.get(name, [])
                    if not values or max(v[1] for v in values) < start+12.:
                        return False
                    if not any(v[1] < start+10 and abs(v[4]) > 2.8 for v in values):
                        return False
                return True

        wait(roundtrips, 180)
        wait(lambda: len(images) == 2, 30)
        # Demonstrate physical roll/pitch restoration (not a locked pose).
        with lock:
            before = poses['same_type']
        roll, pitch, yaw = math.radians(3), math.radians(2), before[4]
        cr, sr, cp, sp, cy, sy = (math.cos(roll/2), math.sin(roll/2), math.cos(pitch/2),
                                 math.sin(pitch/2), math.cos(yaw/2), math.sin(yaw/2))
        quaternion = (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
                      cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)
        result = subprocess.run(['gz', 'service', '-s', '/world/fleet_validation/set_pose',
            '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean', '--timeout', '5000',
            '--req', f'name: "same_type" position: {{x: {before[1]} y: {before[2]} z: {before[3]}}} '
                     f'orientation: {{x: {quaternion[0]} y: {quaternion[1]} '
                     f'z: {quaternion[2]} w: {quaternion[3]}}}'], capture_output=True, text=True, timeout=10)
        assert result.returncode == 0 and 'data: true' in result.stdout, result

        def recovered():
            with lock:
                values = [v for v in history['same_type'] if v[0] > before[0]]
                tilted = any(math.hypot(v[5], v[6]) > .02 for v in values)
                latest = values[-1] if values else before
                return tilted and latest[0] > before[0]+10 and math.hypot(latest[5], latest[6]) < .003

        wait(recovered, 40)
        # All ten native types must remain stably afloat after settling, not just
        # yield a successful create response. Original dimensions are not scaled.
        stability = {}
        with lock:
            for name, spec in expected.items():
                values = history[name]
                settled = [v for v in values if v[0] > values[0][0]+15]
                assert settled, name
                z = [v[3] for v in settled]
                assert max(abs(v) for v in z) < 3., (name, min(z), max(z))
                assert all(all(math.isfinite(c) for c in v) for v in settled)
                tilt = max(2*math.asin(min(1., math.hypot(v[5], v[6]))) for v in settled)
                assert tilt < .35, (name, tilt)
                stability[name] = {'z_range_m': [min(z), max(z)], 'max_tilt_rad': tilt,
                                   'length_m': spec.length, 'beam_m': spec.beam}
            assert poses['ownship_scope_probe'][3] < -30., 'Unrelated model unexpectedly receives buoyancy'
            for name, msg in images.items():
                rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)[:, :msg.width*3]
                rgb = rgb.reshape(msg.height, msg.width, 3)
                assert rgb.std() > 5
                Image.fromarray(rgb).save(tmp_path/(name+'.png'))

        common = [s for s in snapshots if s in tracks and s in truths and len(snapshots[s].ships) == count]
        stamp = max(common)
        snapshot, nav, truth = snapshots[stamp], tracks[stamp], truths[stamp]
        assert len({bytes(s.target_id.uuid) for s in snapshot.ships}) == count
        for obs, tracked, gt in zip(snapshot.ships, nav.ships, truth.tracks):
            spec = expected[obs.model_name]
            assert obs.radius == tracked.radius == spec.radius
            assert obs.pose == tracked.pose and obs.twist == tracked.twist
            assert (gt.size_l, gt.size_w, gt.size_h) == (spec.length, spec.beam, spec.height)
            assert gt.source_model_name == 'dynamic_ship'
            assert gt.track_id == int.from_bytes(bytes(obs.target_id.uuid)[:4], 'big')
        # A same-type sibling survives deletion, with its original UUID/metadata.
        sibling_id = bytes(next(s for s in snapshot.ships if s.model_name == 'same_type').target_id.uuid)
        call('delete', DeleteDynamicShip.Request(model_name='small'))
        wait(lambda: snapshots and len(next(reversed(snapshots.values())).ships) == count-1, 30)
        remaining = next(reversed(snapshots.values())).ships
        assert bytes(next(s for s in remaining if s.model_name == 'same_type').target_id.uuid) == sibling_id
        call('clear', ClearDynamicShips.Request())
        wait(lambda: snapshots and not next(reversed(snapshots.values())).ships, 30)
        wait(lambda: truths and not next(reversed(truths.values())).tracks, 30)
        # Restore/reuse the name only after authoritative removal confirmation.
        request = SpawnDynamicShipV2.Request(name='small', model_id='vb_rumba', speed=1., half_distance=10.)
        request.pose.orientation.w = 1.
        call('spawn', request)
        call('clear', ClearDynamicShips.Request())
        log = (tmp_path/'gazebo.log').read_text()
        for error in ('Unable to find uri', 'could not be resolved', 'Failed to load', 'Error Code'):
            assert error not in log, log[-10000:]
        (tmp_path/'validation.json').write_text(json.dumps({
            'partition': partition, 'ros_domain_id': os.environ['ROS_DOMAIN_ID'],
            'checks': ['10 native models afloat', 'same-type instances', 'mixed legacy/native',
                       'small/large measured round trip', 'atomic nav/truth dimensions',
                       'delete/clear/name reuse', 'model-local buoyancy scope', 'roll/pitch perturbation recovery',
                       'headless mesh/material rendering'],
            'stability': stability}, indent=2)+'\n')
        print('PASS: fleet physical/transport/geometry/lifecycle validation', tmp_path)
    finally:
        with lock:
            (tmp_path/'observed.json').write_text(json.dumps({
                name: {'latest': values[-1], 'count': len(values),
                       'x_range': [min(v[1] for v in values), max(v[1] for v in values)],
                       'z_range': [min(v[3] for v in values), max(v[3] for v in values)]}
                for name, values in history.items() if values}, indent=2)+'\n')
        executor.shutdown()
        client_node.destroy_node()
        context.shutdown()
        transport.unsubscribe(pose_topic)
        for name in ('small', 'large'):
            transport.unsubscribe('/fleet_validation/'+name)
        # Only processes created by this test; never pkill or main-launch cleanup.
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
        for handle in handles:
            handle.close()
