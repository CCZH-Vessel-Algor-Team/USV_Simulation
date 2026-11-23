#!/usr/bin/env python3
"""Sequential isolated before/after harbor render benchmark, never the main launch.

Source ROS and the built workspace first. --before-chart is a read-only copy of
the preceding chart variant, including meshes and textures. CPU 100% = one core.
"""
import argparse
from collections import defaultdict
import hashlib
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

import psutil
from PIL import Image


CAMERAS = {'oblique': (-440, 100, 280, -110, 400, 2),
           'topdown': (-90, 420, 650, -90, 420, 0),
           'landing': (-105, 510, 35, -43, 584, 3.2)}


def snapshot(pid):
    process = psutil.Process(pid)
    times, rss = {}, 0
    for item in [process]+process.children(recursive=True):
        try:
            value = item.cpu_times()
            times[item.pid] = value.user+value.system
            rss += item.memory_info().rss
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            pass
    return times, rss/1048576


def run(package, chart, output, seconds):
    from gz.transport13 import Node, NodeOptions
    from gz.msgs10.image_pb2 import Image as ImageMessage
    from gz.msgs10.world_stats_pb2 import WorldStatistics

    output.mkdir(parents=True, exist_ok=False)
    env = os.environ.copy()
    env['GZ_PARTITION'] = 'harbor_benchmark_'+uuid.uuid4().hex
    env['GZ_SIM_RESOURCE_PATH'] = os.pathsep.join([str(package/'worlds/models'),
                                                 str(package/'description/models'),
                                                 env.get('GZ_SIM_RESOURCE_PATH', '')])
    root = ET.parse(package/'worlds/ccs_open_water_crop.sdf').getroot()
    world = root.find('world')
    world.set('name', 'harbor_benchmark')
    world.remove(world.find('gui'))
    if chart:
        world.find("include[name='CCS ENC Chart Obstacles']/uri").text = (chart/'model_gaogongdao.sdf').as_uri()
        world.find("include[name='gaogongdao_local_harbor_v1']/uri").text = (chart/'gaogongdao_harbor.sdf').as_uri()
    for name, (x, y, z, tx, ty, tz) in CAMERAS.items():
        yaw, pitch = math.atan2(ty-y, tx-x), math.atan2(z-tz, math.hypot(tx-x, ty-y))
        world.append(ET.fromstring(f'''<model name="camera_{name}"><static>true</static>
          <pose>{x} {y} {z} 0 {pitch} {yaw}</pose><link name="body">
            <sensor name="camera" type="camera"><always_on>true</always_on><update_rate>2</update_rate>
              <topic>/harbor_benchmark/{name}</topic><camera><horizontal_fov>0.95</horizontal_fov>
                <visibility_mask>268435455</visibility_mask>
                <image><width>1280</width><height>960</height><format>R8G8B8</format></image>
                <clip><near>0.5</near><far>10000</far></clip>
              </camera></sensor></link></model>'''))
    path = output/'world.sdf'
    ET.ElementTree(root).write(path)
    log = (output/'gazebo.log').open('w')
    server = None
    options = NodeOptions()
    options.partition = env['GZ_PARTITION']
    gz = Node(options)
    images, stamps, factors = {}, defaultdict(list), []
    lock = threading.Lock()

    def camera(message, name):
        with lock:
            images[name] = message
            stamps[name].append(time.monotonic())

    def stats(message):
        with lock:
            factors.append((time.monotonic(), message.real_time_factor))

    topics = []
    try:
        for name in CAMERAS:
            topic = '/harbor_benchmark/'+name
            topics.append(topic)
            assert gz.subscribe(ImageMessage, topic, lambda message, key=name: camera(message, key))
        topics.append('/world/harbor_benchmark/stats')
        assert gz.subscribe(WorldStatistics, topics[-1], stats)
        server = subprocess.Popen(
            ['gz', 'sim', '-r', '-s', '--headless-rendering', str(path)], env=env,
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        deadline = time.monotonic()+60
        while time.monotonic() < deadline:
            assert server.poll() is None, (output/'gazebo.log').read_text()[-6000:]
            with lock:
                ready = all(len(stamps[name]) >= 12 for name in CAMERAS) and len(factors) > 5
            if ready:
                break
            time.sleep(.1)
        else:
            raise RuntimeError('Benchmark cameras/stats did not become ready')
        start = last = time.monotonic()
        previous, _ = snapshot(server.pid)
        samples = []
        for _ in range(seconds):
            time.sleep(1)
            assert server.poll() is None, (output/'gazebo.log').read_text()[-6000:]
            now = time.monotonic()
            current, rss = snapshot(server.pid)
            row = {'cpu_percent': 100*sum(value-previous[pid] for pid, value in current.items()
                                          if pid in previous)/(now-last), 'rss_mib': rss}
            try:
                gpu = subprocess.check_output(['nvidia-smi', '--query-gpu=utilization.gpu,memory.used',
                                                '--format=csv,noheader,nounits'], text=True, timeout=3)
                util, memory = map(float, gpu.splitlines()[0].split(','))
                row.update(gpu_global_percent=util, gpu_global_memory_mib=memory)
            except (FileNotFoundError, subprocess.SubprocessError, ValueError):
                pass
            samples.append(row)
            previous, last = current, now
        with lock:
            for name, msg in images.items():
                Image.frombytes('RGB', (msg.width, msg.height), msg.data, 'raw', 'RGB', msg.step).save(output/f'{name}.png')
            rtf = [value for stamp, value in factors if stamp >= start]
            hz = {}
            for name, values in stamps.items():
                values = [stamp for stamp in values if stamp >= start]
                assert len(values) >= 2, f'Camera {name} stopped publishing during sampling'
                hz[name] = (len(values)-1)/(values[-1]-values[0])
        assert rtf, 'World statistics stopped publishing during sampling'
        text = (output/'gazebo.log').read_text()
        for error in ('Unable to find uri', 'could not be resolved', 'Failed to load a world',
                      'CustomMeshShape', 'This submesh will be'):
            assert error not in text, text[-6000:]
        result = {key: round(statistics.mean(row[key] for row in samples), 2) for key in samples[0]}
        geometry = (chart or package/'worlds/models/ccs_chart_obstacles_enc_5km')/'gaogongdao_variant.json'
        result.update(rtf_mean=round(statistics.mean(rtf), 4),
                      image_hz={key: round(value, 3) for key, value in hz.items()},
                      sample_seconds=round(last-start, 2), samples=samples,
                      max_step_size_s=float(world.findtext('physics/max_step_size')),
                      camera_size_px=[1280, 960], camera_target_hz=2,
                      warmup_frames_per_camera=12,
                      scene_geometry_sha256=hashlib.sha256(geometry.read_bytes()).hexdigest())
        (output/'metrics.json').write_text(json.dumps(result, indent=2)+'\n')
        return result
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


if __name__ == '__main__':
    from ament_index_python.packages import get_package_share_directory
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--before-chart', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--seconds', type=int, default=15)
    parser.add_argument('--repeats', type=int, default=2)
    args = parser.parse_args()
    if args.seconds < 5 or args.repeats < 1:
        parser.error('--seconds must be at least 5 and --repeats must be positive')
    for filename in ('model_gaogongdao.sdf', 'gaogongdao_harbor.sdf', 'gaogongdao_variant.json'):
        if not (args.before_chart/filename).is_file():
            parser.error(f'--before-chart is missing {filename}')
    package = Path(get_package_share_directory('usv_sim_full'))
    results = []
    for repeat in range(args.repeats):
        # Counterbalance order; still sequential, never concurrent worlds.
        order = ('before', 'after') if repeat % 2 == 0 else ('after', 'before')
        for mode in order:
            value = run(package, args.before_chart.resolve() if mode == 'before' else None,
                        args.output/f'{mode}_{repeat+1}', args.seconds)
            results.append({'mode': mode, 'repeat': repeat+1, **value})
            print(mode, repeat+1, {k: v for k, v in value.items() if k != 'samples'}, flush=True)
    args.output.joinpath('comparison.json').write_text(json.dumps(results, indent=2)+'\n')
