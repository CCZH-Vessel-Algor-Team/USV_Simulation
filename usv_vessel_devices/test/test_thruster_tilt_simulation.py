"""Opt-in physical test using the actual usv_bringup scene, never the live world."""
import copy
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid
import xml.etree.ElementTree as ET

import pytest
import yaml

pytestmark = pytest.mark.skipif(
    os.environ.get('USV_VESSEL_DEVICES_INTEGRATION') != '1',
    reason='opt-in isolated Gazebo/ROS integration test; requires full workspace',
)


def test_real_outboard_tilt_propulsion_pause_reset_and_isolation(tmp_path, monkeypatch):
    import numpy as np
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64, Float64MultiArray
    from usv_vessel_devices.msg import ThrusterTiltState
    from usv_vessel_devices.srv import SetThrusterTilt
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    from gz.transport13 import Node as GzNode
    from gz.msgs10.double_pb2 import Double
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.world_control_pb2 import WorldControl
    from gz.msgs10.empty_pb2 import Empty
    from gz.msgs10.entity_pb2 import Entity
    from gz.msgs10.serialized_map_pb2 import SerializedStepMap
    from google.protobuf import text_format
    from usv_sim_full.scripts import session_manager as session

    monkeypatch.setenv('GZ_PARTITION', 'outboard_tilt_test_' + uuid.uuid4().hex)
    monkeypatch.setenv('ROS_DOMAIN_ID', str(110 + os.getpid() % 80))
    monkeypatch.setenv('GZ_SIM_SYSTEM_PLUGIN_PATH', os.pathsep.join([
        str(Path(get_package_prefix('usv_vessel_devices')) / 'lib'),
        os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''),
    ]))
    sim_share = Path(get_package_share_directory('usv_sim_full'))
    monkeypatch.setenv('GZ_SIM_RESOURCE_PATH', os.pathsep.join([
        str(sim_share / 'description/models'),
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
    ]))
    bringup = Path(get_package_share_directory('usv_bringup'))
    system_file = bringup / 'config/system/system_sim.yaml'
    system = yaml.safe_load(system_file.read_text())
    scene_file = (system_file.parent / system['launches']['simulation'][
        'arguments']['config_path']['path']).resolve()
    scene = yaml.safe_load(scene_file.read_text())
    assert scene['robot_1']['vessel_devices']['thruster_tilt_enabled'] is True
    scene['environment']['world_name'] = 'tilt_test'
    root = ET.fromstring('''<sdf version="1.9"><world name="tilt_test">
      <physics name="physics" type="dart"><max_step_size>0.004</max_step_size>
        <real_time_factor>1</real_time_factor></physics>
      <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
      <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
      <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    </world></sdf>''')
    bridge_config = []
    for i, ns in enumerate(('usv_1', 'usv_2')):
        boat = copy.deepcopy(scene['robot_1'])
        boat['name'] = ns
        effective = session.build_effective_robot_config(scene, boat)
        path = Path(session.compile_xacro_to_urdf(
            None, effective, str(tmp_path), urdf_filename=ns + '.urdf', multi_vehicle_session=True))
        robot = ET.parse(path).getroot()
        # Fix the real boat to a test stand. Preserve actual outboard meshes,
        # masses, steering, propellers and tilt controls; no hydrodynamic loads.
        allowed = {'usv_vessel_devices::ThrusterTilt', 'gz::sim::systems::Thruster',
                   'gz::sim::systems::JointPositionController',
                   'gz::sim::systems::JointStatePublisher'}
        for gazebo in robot.findall('gazebo'):
            for plugin in gazebo.findall('plugin'):
                if plugin.get('name') not in allowed:
                    gazebo.remove(plugin)
        ET.SubElement(robot, 'link', name='world')
        anchor = ET.SubElement(robot, 'joint', name='anchor', type='fixed')
        ET.SubElement(anchor, 'parent', link='world')
        ET.SubElement(anchor, 'child', link=ns + '/base_link')
        ET.ElementTree(robot).write(path)
        result = subprocess.run(['gz', 'sdf', '-p', str(path)],
                                capture_output=True, text=True, check=True)
        model = ET.fromstring(result.stdout).find('model')
        model.set('name', ns)
        ET.SubElement(model, 'pose').text = f'{i * 20} 0 0 0 0 0'
        root.find('world').append(model)
        bridge_config.extend(b for b in session.generate_bridge_config(effective)
                             if b['ros_type_name'] == 'sensor_msgs/msg/JointState'
                             or '/thrusters/' in b['ros_topic_name'])
    world = tmp_path / 'tilt_test.sdf'
    ET.ElementTree(root).write(world)
    bridge_file = tmp_path / 'bridge.yaml'
    bridge_file.write_text(yaml.safe_dump(bridge_config))
    logs = [(tmp_path / f'{name}.log').open('w') for name in ('gazebo', 'bridge')]
    server = subprocess.Popen(['gz', 'sim', '-r', '-s', str(world)],
                              stdout=logs[0], stderr=subprocess.STDOUT, start_new_session=True)
    bridge = subprocess.Popen(['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                               '--ros-args', '-p', f'config_file:={bridge_file}'],
                              stdout=logs[1], stderr=subprocess.STDOUT, start_new_session=True)
    context = rclpy.context.Context()
    rclpy.init(context=context)
    node = rclpy.create_node('thruster_tilt_integration', context=context)
    executor = rclpy.executors.SingleThreadedExecutor(context=context)
    executor.add_node(node)
    states, joints, native_speed, physics = {}, {}, {}, {}
    max_speed = [0.0]

    def receive_state(message, ns):
        previous = states.get(ns)
        if previous and previous.ready and message.ready:
            dt = (message.header.stamp.sec - previous.header.stamp.sec
                  + (message.header.stamp.nanosec - previous.header.stamp.nanosec) * 1e-9)
            if dt > 0.02:
                max_speed[0] = max(max_speed[0], *(abs(a - b) / dt
                    for a, b in zip(message.position, previous.position)))
        states[ns] = message

    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions = []
    for ns in ('usv_1', 'usv_2'):
        subscriptions.append(node.create_subscription(
            ThrusterTiltState, f'/{ns}/devices/thrusters/state',
            lambda message, key=ns: receive_state(message, key), qos))
        subscriptions.append(node.create_subscription(
            JointState, f'/model/{ns}/joint_state',
            lambda message, key=ns: joints.update({key: dict(zip(message.name, message.position))}), 10))
        subscriptions.append(node.create_subscription(
            Float64MultiArray, f'/{ns}/devices/thrusters/physics',
            lambda message, key=ns: physics.update({key: list(message.data)}), 10))
    gz = GzNode()
    speed_topics = []
    for side in ('left', 'right'):
        topic = f'/usv_1/devices/thrusters/{side}/thrust_applied/ang_vel'
        speed_topics.append(topic)
        gz.subscribe(Double, topic, lambda message, key=side: native_speed.update({key: message.data}))
    thrust = {side: node.create_publisher(Float64, f'/usv_1/thrusters/{side}/thrust', 1)
              for side in ('left', 'right')}
    steering = {side: node.create_publisher(Float64, f'/usv_1/thrusters/{side}/pos', 1)
                for side in ('left', 'right')}

    def wait(predicate, timeout=20):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            assert server.poll() is None and bridge.poll() is None, (
                tmp_path / 'gazebo.log').read_text()[-6000:]
            executor.spin_once(timeout_sec=0.03)
            if predicate():
                return
        raise AssertionError(f'Timed out; states={states}; native={native_speed}; physics={physics}\n'
                             + (tmp_path / 'gazebo.log').read_text()[-4000:])

    def call(side, position):
        client = node.create_client(SetThrusterTilt,
                                   f'/usv_1/devices/thrusters/{side}/tilt/set_position')
        assert client.wait_for_service(timeout_sec=10)
        future = client.call_async(SetThrusterTilt.Request(position=position))
        wait(future.done, 10)
        result = future.result()
        node.destroy_client(client)
        return result

    def request(service, message, response_type):
        result = subprocess.run(['gz', 'service', '-s', service,
                                 '--reqtype', message.DESCRIPTOR.full_name,
                                 '--reptype', response_type.DESCRIPTOR.full_name,
                                 '--timeout', '5000', '--req', str(message)],
                                capture_output=True, text=True, check=True, timeout=10)
        response = response_type()
        text_format.Parse(result.stdout, response)
        return response

    def control(**kwargs):
        assert request('/world/tilt_test/control', WorldControl(**kwargs), Boolean).data

    def component_type(name):
        value = 0xcbf29ce484222325
        for byte in ('gz_sim_components.' + name).encode():
            value = ((value ^ byte) * 0x100000001b3) & ((1 << 64) - 1)
        return value

    def poses():
        snapshot = request('/world/tilt_test/state', Empty(), SerializedStepMap)
        result = {}
        for entity in snapshot.state.entities.values():
            values = {component.type: component.component for component in entity.components.values()}
            if component_type('Name') in values and component_type('Pose') in values:
                result[values[component_type('Name')].decode()] = np.fromstring(
                    values[component_type('Pose')].decode(), sep=' ')
        return result

    def state():
        return states['usv_1']

    try:
        wait(lambda: len(states) == 2 and all(s.ready and all(s.thrust_enabled) for s in states.values())
             and len(native_speed) == 2 and len(joints) == 2 and len(physics) == 2, 40)
        wait(lambda: all(pub.get_subscription_count() for pub in [*thrust.values(), *steering.values()]))
        assert list(state().thruster_names) == ['left', 'right']
        initial = poses()
        thrust['left'].publish(Float64(data=100.0))
        thrust['right'].publish(Float64(data=80.0))
        wait(lambda: list(state().applied_thrust) == [100.0, 80.0]
             and all(value > 0 for value in native_speed.values()))

        assert call('left', math.pi / 3).success
        wait(lambda: state().position[0] > 0.05 and state().applied_thrust[0] == 100
             and native_speed['left'] > 0)
        wait(lambda: state().at_target[0])
        assert state().position[0] == pytest.approx(math.pi / 3, abs=0.005)
        assert state().position[1] == pytest.approx(0, abs=0.005)
        assert state().applied_thrust[1] == 80.0 and native_speed['right'] > 0
        wait(lambda: physics['usv_1'][2] == 0.0 and physics['usv_1'][3] == 0.0)
        assert list(states['usv_2'].target) == [0, 0]
        assert list(states['usv_2'].applied_thrust) == [0, 0]
        raised = poses()
        # This checks real physics/scene geometry, not just command echo.
        before = initial['usv_1/left_propeller_link']
        after = raised['usv_1/left_propeller_link']
        assert after[0] < before[0] - 0.1
        assert after[2] > before[2] + 0.2
        assert raised['usv_1/right_propeller_link'][:3] == pytest.approx(
            initial['usv_1/right_propeller_link'][:3], abs=2e-5)
        for bad in (-0.1, math.radians(75.0) + 0.00001, math.pi / 2,
                    math.pi, float('nan'), float('inf')):
            assert not call('left', bad).success
        thrust['left'].publish(Float64(data=900.0))
        sample = state()
        wait(lambda: state() is not sample)
        assert state().applied_thrust[0] == 900 and native_speed['left'] > 0
        assert physics['usv_1'][2] == 0.0 and physics['usv_1'][3] == 0.0

        assert call('left', 0.0).success
        wait(lambda: state().at_target[0] and state().thrust_enabled[0])
        for _ in range(3):
            sample = state()
            wait(lambda: state() is not sample)
            assert state().applied_thrust[0] == 900 and native_speed['left'] > 0
        thrust['left'].publish(Float64(data=50.0))
        wait(lambda: state().applied_thrust[0] == 50 and native_speed['left'] > 0)
        for command in (-50.0, 50.0, -1000.0, 8000.0):
            print(f'Checking lowered thrust command {command} N', flush=True)
            thrust['left'].publish(Float64(data=command))
            wait(lambda: state().applied_thrust[0] == command
                 and native_speed['left'] * command > 0, 5)
            for _ in range(3):
                sample = state()
                wait(lambda: state() is not sample)
                assert state().applied_thrust[0] == command
                assert state().thrust_enabled[0]
        # Tilt under load without stopping the motor or discarding its command.
        assert call('left', 0.15).success
        wait(lambda: state().target[0] == 0.15 and state().at_target[0]
             and state().applied_thrust[0] == 8000 and native_speed['left'] > 0)
        assert call('left', 0.0).success
        wait(lambda: state().target[0] == 0 and state().at_target[0] and state().thrust_enabled[0])
        # Existing steering still works; it is not commandeered as a tilt axis.
        for side in ('left', 'right'):
            thrust[side].publish(Float64(data=0.0))
            steering[side].publish(Float64(data=0.3))
        wait(lambda: all(abs(joints['usv_1'].get(
            f'usv_1/{side}_chassis_engine_joint', 9) - 0.3) < 0.005 for side in ('left', 'right')))
        assert all(abs(angle) < 0.005 for angle in state().position)

        topic = node.create_publisher(Float64, '/usv_1/devices/thrusters/right/tilt/cmd', 1)
        wait(lambda: topic.get_subscription_count() > 0)
        topic.publish(Float64(data=0.3))
        wait(lambda: state().target[1] == 0.3 and state().at_target[1])
        topic.publish(Float64(data=float('nan')))
        sample = state()
        wait(lambda: state() is not sample)
        assert state().target[1] == 0.3
        assert call('both', 0.4).success
        wait(lambda: list(state().target) == [0.4, 0.4] and all(state().at_target))
        control(pause=True)
        sample = [state()]

        def paused():
            current = state()
            if current is sample[0]:
                return False
            previous, sample[0] = sample[0], current
            return current.header.stamp == previous.header.stamp

        wait(paused, 5)
        frozen = list(state().position)
        assert call('both', 0.8).success
        wait(lambda: list(state().target) == [0.8, 0.8])
        assert list(state().position) == frozen
        control(pause=False)
        wait(lambda: all(state().at_target))
        assert list(state().position) == pytest.approx([0.8, 0.8], abs=0.005)
        assert all(state().thrust_enabled)
        for _ in range(3):
            sample = state()
            wait(lambda: state() is not sample)
            assert list(state().position) == pytest.approx([0.8, 0.8], abs=0.005)
        assert 0.1 < max_speed[0] <= 0.26
        maximum = math.radians(75.0)
        assert call('both', maximum).success
        wait(lambda: list(state().target) == [maximum] * 2 and all(state().at_target))
        assert list(state().position) == pytest.approx([maximum] * 2, abs=0.005)
        control(reset={'all': True}, pause=False)
        wait(lambda: state().ready and list(state().target) == [0, 0] and all(state().at_target))
        assert list(state().applied_thrust) == [0, 0]
        wait(lambda: all(value == 0 for value in native_speed.values()))
        retired = node.create_client(SetThrusterTilt,
                                     '/usv_1/devices/thrusters/left/tilt/set_position')
        assert retired.wait_for_service(timeout_sec=5)
        assert request('/world/tilt_test/remove', Entity(name='usv_1', type=Entity.MODEL), Boolean).data
        wait(lambda: not retired.service_is_ready())
        node.destroy_client(retired)
        other = node.create_client(SetThrusterTilt,
                                   '/usv_2/devices/thrusters/both/tilt/set_position')
        assert other.wait_for_service(timeout_sec=5)
        future = other.call_async(SetThrusterTilt.Request(position=0.2))
        wait(future.done)
        assert future.result().success
        wait(lambda: all(states['usv_2'].at_target)
             and list(states['usv_2'].target) == [0.2, 0.2])
        node.destroy_client(other)
        print('PASS: actual bringup boat, independent/simultaneous tilt, aft/up geometry, '
              'immersion-aware thrust and independent rotation, steering, limits, speed, holding, '
              'pause/reset and two-boat isolation/removal')
    finally:
        for topic in speed_topics:
            gz.unsubscribe(topic)
        executor.shutdown()
        node.destroy_node()
        context.shutdown()
        for process in (bridge, server):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
        for log in logs:
            log.close()
