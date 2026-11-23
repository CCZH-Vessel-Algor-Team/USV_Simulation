"""Opt-in, isolated real Gazebo/ROS test (requires EGL rendering).

USV_VESSEL_DEVICES_INTEGRATION=1 colcon test --packages-select usv_vessel_devices
"""
import math
from collections import deque
import os
import re
from pathlib import Path
import signal
import subprocess
import time
import uuid
import xml.etree.ElementTree as ET

import pytest

PACKAGE = Path(__file__).resolve().parents[1]
pytestmark = pytest.mark.skipif(
    os.environ.get('USV_VESSEL_DEVICES_INTEGRATION') != '1',
    reason='opt-in Gazebo/EGL integration test',
)


def test_real_lights_mast_pause_reset_and_boat_isolation(tmp_path, monkeypatch):
    import numpy as np
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from std_msgs.msg import Bool, Float64
    from std_srvs.srv import SetBool
    from usv_vessel_devices.msg import DeviceState
    from usv_vessel_devices.srv import SetMastPosition
    from gz.transport13 import Node as GzNode
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.world_control_pb2 import WorldControl
    from gz.msgs10.image_pb2 import Image
    from gz.msgs10.empty_pb2 import Empty
    from gz.msgs10.serialized_map_pb2 import SerializedStepMap
    from gz.msgs10.material_pb2 import Material
    from gz.msgs10.visual_pb2 import Visual
    from gz.msgs10.entity_pb2 import Entity
    from google.protobuf import text_format
    from ament_index_python.packages import get_package_prefix
    import xacro

    monkeypatch.setenv('GZ_PARTITION', 'vessel_devices_test_' + uuid.uuid4().hex)
    monkeypatch.setenv('ROS_DOMAIN_ID', str(100 + os.getpid() % 100))
    monkeypatch.setenv('GZ_SIM_SYSTEM_PLUGIN_PATH', os.pathsep.join([
        str(Path(get_package_prefix('usv_vessel_devices')) / 'lib'),
        os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''),
    ]))
    root = ET.parse(PACKAGE / 'worlds/devices_demo.sdf').getroot()
    for i, ns in enumerate(['usv_1', 'usv_2']):
        urdf = xacro.process_file(str(PACKAGE / 'description/demo.urdf.xacro'),
                                  mappings={'namespace': ns}).toxml()
        path = tmp_path / f'{ns}.urdf'
        path.write_text(urdf)
        converted = subprocess.run(['gz', 'sdf', '-p', str(path)],
                                   capture_output=True, text=True, check=True)
        model = ET.fromstring(converted.stdout).find('model')
        model.set('name', ns)
        # The second deck is outside the observer view; neither boat can collide.
        ET.SubElement(model, 'pose').text = f'{i * 10} 0 0 0 0 0'
        root.find('world').append(model)
    world = tmp_path / 'integration.sdf'
    ET.ElementTree(root).write(world)
    log_path = tmp_path / 'gazebo.log'
    log = log_path.open('w')
    process = subprocess.Popen(['gz', 'sim', '-r', '-s', '--headless-rendering', str(world)],
                               stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    context = rclpy.context.Context()
    rclpy.init(context=context)
    node = rclpy.create_node('vessel_devices_integration', context=context)
    executor = rclpy.executors.SingleThreadedExecutor(context=context)
    executor.add_node(node)
    states, images = {}, deque(maxlen=2)
    maximum_mast_speed = [0.0]

    def receive_ros_state(message, name):
        previous = states.get(name)
        if name == 'usv_1' and previous and previous.ready and message.ready:
            dt = (message.header.stamp.sec - previous.header.stamp.sec
                  + (message.header.stamp.nanosec - previous.header.stamp.nanosec) * 1e-9)
            if dt > 0.02:
                maximum_mast_speed[0] = max(maximum_mast_speed[0],
                    abs(message.mast_position - previous.mast_position) / dt)
        states[name] = message

    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions = [node.create_subscription(
        DeviceState, f'/{ns}/devices/state', lambda m, key=ns: receive_ros_state(m, key), qos,
    ) for ns in ['usv_1', 'usv_2']]
    gz = GzNode()
    gz.subscribe(Image, '/vessel_devices/observer/image', lambda image: images.append(image))

    def component_type(name):
        # gz::common::hash64 uses FNV-1a for serialized component type identifiers.
        value = 0xcbf29ce484222325
        for byte in name.encode():
            value = ((value ^ byte) * 0x100000001b3) & ((1 << 64) - 1)
        return value

    visual_cmd_type = component_type('gz_sim_components.VisualCmd')
    name_type = component_type('gz_sim_components.Name')
    visual_commands = {}
    entity_names = {}
    gui_received = [False]

    def receive_gui_state(message):
        gui_received[0] = True
        for entity_id, entity in message.state.entities.items():
            for component in entity.components.values():
                # Map keys are signed int64; component.type contains the uint64 hash.
                if component.type == name_type and not component.remove:
                    entity_names[entity_id] = component.component.decode()
                if component.type == visual_cmd_type and not component.remove:
                    visual_commands[entity_id] = Visual.FromString(component.component)

    gz.subscribe(SerializedStepMap, '/world/devices_demo/state', receive_gui_state)

    def wait(predicate, timeout=20):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            assert process.poll() is None, log_path.read_text()[-6000:]
            executor.spin_once(timeout_sec=0.05)
            if predicate():
                return
        raise AssertionError(f'Timed out; states={states}\n{log_path.read_text()[-4000:]}')

    def call(kind, name, request):
        client = node.create_client(kind, name)
        assert client.wait_for_service(timeout_sec=10), name
        future = client.call_async(request)
        wait(future.done, 10)
        result = future.result()
        node.destroy_client(client)
        return result

    def gz_request(service, request, response_class):
        # Synchronous transport bindings can hold the GIL while Python subscriptions
        # are dispatched. Use a child CLI so subscription callbacks keep progressing.
        result = subprocess.run([
            'gz', 'service', '-s', service,
            '--reqtype', request.DESCRIPTOR.full_name,
            '--reptype', response_class.DESCRIPTOR.full_name,
            '--timeout', '5000', '--req', str(request),
        ], capture_output=True, text=True, timeout=10, check=True)
        response = response_class()
        text_format.Parse(result.stdout, response)
        return response

    def world_control(**kwargs):
        assert gz_request('/world/devices_demo/control', WorldControl(**kwargs), Boolean).data

    def red_pixels(image):
        pixels = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.width, 3)
        return int(np.count_nonzero((pixels[:, :, 0] > 180) & (pixels[:, :, 1] < 80)))

    def component_data(snapshot, entity_id, name):
        return next(component.component for component in
                    snapshot.state.entities[entity_id].components.values()
                    if component.type == component_type('gz_sim_components.' + name))

    def link_pose(snapshot, entity_id):
        # components::Pose serializes gz::math::Pose3d as six ASCII numbers,
        # unlike Material and VisualCmd, which use binary protobuf serialization.
        values = np.fromstring(component_data(snapshot, entity_id, 'Pose').decode(), sep=' ')
        assert len(values) == 6
        return values

    def save_frame(name):
        image = images[-1]
        header = f'P6\n{image.width} {image.height}\n255\n'.encode()
        (tmp_path / name).write_bytes(header + image.data)

    try:
        wait(lambda: len(states) == 2 and all(s.ready for s in states.values())
             and images and gui_received[0], 40)
        # Discovery may happen after entity creation; seed names from a full snapshot.
        receive_gui_state(gz_request('/world/devices_demo/state', Empty(), SerializedStepMap))
        initial_snapshot = gz_request('/world/devices_demo/state', Empty(), SerializedStepMap)
        base_id = next(key for key, name in entity_names.items() if name == 'usv_1/mast_base_link')
        tip_id = next(key for key, name in entity_names.items() if name == 'usv_1/light_mast_link')
        initial_base = link_pose(initial_snapshot, base_id)
        initial_tip = link_pose(initial_snapshot, tip_id)
        mast_id = next(key for key, name in entity_names.items() if name == 'usv_1/mast_link')
        initial_mast = link_pose(initial_snapshot, mast_id)
        lamp_names = list(states['usv_1'].light_names)
        assert lamp_names[:5] == ['left_front', 'left_rear', 'right_front', 'right_rear', 'mast']
        assert len(lamp_names) == 16
        roof_names = ['roof_left_front', 'roof_left_rear', 'roof_right_front', 'roof_right_rear']
        assert lamp_names[-4:] == roof_names
        navigation_ids = {
            lamp: next(key for key, name in entity_names.items() if name == f'usv_1/light_{lamp}_link')
            for lamp in lamp_names if lamp not in roof_names
        }
        roof_ids = {lamp: next(key for key, name in entity_names.items()
                               if name == f'usv_1/light_{lamp}_link') for lamp in roof_names}
        initial_roof = {name: link_pose(initial_snapshot, entity) for name, entity in roof_ids.items()}
        initial_navigation = {name: link_pose(initial_snapshot, entity)
                              for name, entity in navigation_ids.items()}
        assert not any(states['usv_1'].lights_on)
        off_pixels = red_pixels(images[-1])
        save_frame('lights_off.ppm')
        # Every lamp switches independently, without affecting its neighbours/boat 2.
        for name in lamp_names:
            result = call(SetBool, f'/usv_1/devices/lights/{name}/set', SetBool.Request(data=True))
            assert result.success
            wait(lambda: states['usv_1'].lights_on[lamp_names.index(name)])
            assert sum(states['usv_1'].lights_on) == 1
            assert not any(states['usv_2'].lights_on)
            assert call(SetBool, f'/usv_1/devices/lights/{name}/set', SetBool.Request(data=False)).success
            wait(lambda: not any(states['usv_1'].lights_on))
        for name in lamp_names:
            assert call(SetBool, f'/usv_1/devices/lights/{name}/set', SetBool.Request(data=True)).success
        wait(lambda: all(states['usv_1'].lights_on))
        assert not any(states['usv_2'].lights_on)
        # Must affect actual server-side camera rendering, not just state/GUI.
        wait(lambda: red_pixels(images[-1]) > off_pixels + 3)
        visual_id = next(entity_id for entity_id, name in entity_names.items()
                         if name.startswith('usv_1/light_left_front_link') and 'lamp_visual' in name)
        # GUI uses the ECM state stream; scene/info only caches creation-time data.
        state = gz_request('/world/devices_demo/state', Empty(), SerializedStepMap)
        material_type = component_type('gz_sim_components.Material')
        material_data = next(component.component for component in
                             state.state.entities[visual_id].components.values()
                             if component.type == material_type)
        # Late GUI clients create their visual using this persisted current material.
        assert Material.FromString(material_data).emissive.r == pytest.approx(1)
        plugin = ET.fromstring(urdf).find('gazebo/plugin')
        for lamp in plugin.findall('lamp'):
            name = lamp.findtext('name')
            entity_id = next(key for key, visual_name in entity_names.items()
                             if visual_name.startswith(f'usv_1/light_{name}_link')
                             and re.search(r'__lamp_visual(?:_\d+)?$', visual_name))
            material = Material.FromString(component_data(state, entity_id, 'Material'))
            expected = list(map(float, lamp.findtext('color').split()))[:3]
            assert [material.emissive.r, material.emissive.g, material.emissive.b] == pytest.approx(expected)
        housing_ids = [key for key, name in entity_names.items()
                       if name.startswith('usv_1/light_')
                       and re.search(r'__housing_visual(?:_\d+)?$', name)]
        assert len(housing_ids) == len(lamp_names) - 1
        for housing_id in housing_ids:
            housing = Material.FromString(component_data(state, housing_id, 'Material'))
            original_housing = Material.FromString(component_data(initial_snapshot, housing_id, 'Material'))
            assert (housing.emissive.r, housing.emissive.g, housing.emissive.b) == (0, 0, 0)
            assert housing.diffuse == original_housing.diffuse
        save_frame('lights_on.ppm')
        wait(lambda: visual_id in visual_commands
             and visual_commands[visual_id].material.emissive.r > 0.99)

        assert call(SetMastPosition, '/usv_1/devices/mast/set_position',
                    SetMastPosition.Request(position=math.pi / 2)).success
        wait(lambda: states['usv_1'].mast_position > 0.1)
        wait(lambda: states['usv_1'].mast_at_target, 20)
        assert states['usv_1'].mast_position == pytest.approx(math.pi / 2, abs=0.006)
        assert states['usv_2'].mast_position == pytest.approx(0, abs=0.006)
        folded_snapshot = gz_request('/world/devices_demo/state', Empty(), SerializedStepMap)
        folded_base = link_pose(folded_snapshot, base_id)
        folded_tip = link_pose(folded_snapshot, tip_id)
        assert folded_base == pytest.approx(initial_base, abs=1e-8)
        for name, entity_id in roof_ids.items():
            assert link_pose(folded_snapshot, entity_id) == pytest.approx(initial_roof[name], abs=1e-8)
        tip_displacement = float(np.sum((folded_tip[:3] - initial_tip[:3]) ** 2))
        assert tip_displacement > 0.1
        folded_mast = link_pose(folded_snapshot, mast_id)
        pitch = folded_mast[4]
        rotation = np.array([[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0],
                             [-math.sin(pitch), 0, math.cos(pitch)]])
        for name, entity_id in navigation_ids.items():
            folded_navigation = link_pose(folded_snapshot, entity_id)
            local = initial_navigation[name][:3] - initial_mast[:3]
            expected_position = folded_mast[:3] + rotation @ local
            assert folded_navigation[:3] == pytest.approx(expected_position, abs=2e-5)
            assert np.linalg.norm(folded_navigation[:3] - initial_navigation[name][:3]) > 0.1
        save_frame('mast_folded.ppm')
        for bad in [-0.1, math.pi, float('nan'), float('inf')]:
            assert not call(SetMastPosition, '/usv_1/devices/mast/set_position',
                            SetMastPosition.Request(position=bad)).success

        # Check topic path and invalid topic rejection too.
        pub = node.create_publisher(Float64, '/usv_1/devices/mast/cmd', 1)
        light_pub = node.create_publisher(Bool, '/usv_1/devices/lights/mast/cmd', 1)
        wait(lambda: pub.get_subscription_count() > 0 and light_pub.get_subscription_count() > 0)
        pub.publish(Float64(data=0.0))
        light_pub.publish(Bool(data=False))
        wait(lambda: states['usv_1'].mast_target == 0
             and not states['usv_1'].lights_on[lamp_names.index('mast')])
        pub.publish(Float64(data=float('nan')))

        world_control(pause=True)
        # Drain pre-pause DDS snapshots before checking a stable paused sample.
        sample = [states['usv_1']]

        def stable_pause():
            current = states['usv_1']
            if current is sample[0]:
                return False
            previous, sample[0] = sample[0], current
            return (current.header.stamp == previous.header.stamp
                    and current.mast_position == previous.mast_position)

        wait(stable_pause, 5)
        assert call(SetMastPosition, '/usv_1/devices/mast/set_position',
                    SetMastPosition.Request(position=0.3)).success
        wait(lambda: states['usv_1'].mast_target == 0.3)
        paused_position = states['usv_1'].mast_position
        world_control(pause=False)
        wait(lambda: states['usv_1'].mast_at_target and not states['usv_1'].mast_moving, 20)
        assert states['usv_1'].mast_position == pytest.approx(0.3, abs=0.006)
        assert paused_position > 0.3
        assert 0.1 < maximum_mast_speed[0] <= 0.306
        # Holding at an intermediate angle must not let gravity fold it further.
        for _ in range(4):
            previous = states['usv_1']
            wait(lambda: states['usv_1'] is not previous)
            assert states['usv_1'].mast_position == pytest.approx(0.3, abs=0.006)

        assert call(SetMastPosition, '/usv_1/devices/mast/set_position',
                    SetMastPosition.Request(position=0.0)).success
        wait(lambda: states['usv_1'].mast_target == 0 and states['usv_1'].mast_at_target, 20)
        assert states['usv_1'].mast_position == pytest.approx(0, abs=0.006)
        world_control(reset={'all': True}, pause=False)
        wait(lambda: states['usv_1'].mast_target == 0 and not any(states['usv_1'].lights_on))
        wait(lambda: states['usv_1'].mast_at_target)
        wait(lambda: red_pixels(images[-1]) <= off_pixels + 3)
        # Unloading one model shuts down only its own ROS context/executor.
        retired_client = node.create_client(SetBool, '/usv_1/devices/lights/mast/set')
        assert retired_client.wait_for_service(timeout_sec=5)
        assert gz_request('/world/devices_demo/remove',
                          Entity(name='usv_1', type=Entity.MODEL), Boolean).data
        wait(lambda: not retired_client.service_is_ready())
        node.destroy_client(retired_client)
        assert call(SetBool, '/usv_2/devices/lights/right_front/set',
                    SetBool.Request(data=True)).success
        wait(lambda: states['usv_2'].lights_on[2])
        print('PASS: sixteen independent lights, actual red/green/white materials, rendered camera change, '
              'twelve lamps follow doubled mast, four roof lamps stay fixed, full folding/return, fixed base, limits, '
              'topic/service, pause, reset, speed/holding and two-boat isolation/removal')
    finally:
        gz.unsubscribe('/vessel_devices/observer/image')
        gz.unsubscribe('/world/devices_demo/state')
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        executor.shutdown()
        node.destroy_node()
        context.shutdown()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
        log.close()
