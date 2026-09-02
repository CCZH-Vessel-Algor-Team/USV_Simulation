#!/usr/bin/env python3

import json
import os
import re
import subprocess
import tempfile
import uuid

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped, Pose
from nav2_colregs_msgs.msg import TrackedShip, TrackedShipList
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from usv_interfaces.msg import Buoy, BuoyArray
from usv_interfaces.srv import (
    ClearDynamicBuoys,
    DeleteDynamicBuoy,
    SetDynamicBuoyConfig,
    SpawnDynamicBuoy,
)
from visualization_msgs.msg import Marker, MarkerArray

VALID_BUOY_TYPES = (
    'rgb', 'rgy', 'yrg', 'bgr', 'bgy', 'gbr', 'rbg', 'ybr', 'ygb',
)


class DynamicBuoy:
    def __init__(self, model_name, buoy_id, x, y, buoy_type, radius_m):
        self.model_name = model_name
        self.buoy_id = buoy_id
        self.x = float(x)
        self.y = float(y)
        self.buoy_type = buoy_type
        self.radius_m = float(radius_m)

    def pose(self):
        pose = Pose()
        pose.position.x = self.x
        pose.position.y = self.y
        pose.position.z = 0.0
        pose.orientation.w = 1.0
        return pose

    def buoy_msg(self):
        msg = Buoy()
        msg.name = self.model_name
        msg.buoy_id.uuid = list(bytes.fromhex(self.buoy_id.replace('-', '')))
        msg.pose = self.pose()
        msg.buoy_type = self.buoy_type
        msg.radius_m = self.radius_m
        msg.color_sequence = self.buoy_type
        return msg

    def tracked_ship(self):
        ts = TrackedShip()
        ts.target_id.uuid = list(bytes.fromhex(self.buoy_id.replace('-', '')))
        ts.pose = self.pose()
        ts.radius = self.radius_m
        return ts


class DynamicBuoyManager(Node):
    def __init__(self):
        super().__init__('dynamic_buoy_manager')
        self._retained_spawn_sdf_paths = []

        self.declare_parameter('world_name', 'sydney_regatta')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('buoy_type', 'rgb')
        self.declare_parameter('radius_m', 0.8)
        self.declare_parameter('spawn_z', 0.0)
        self.declare_parameter('dt', 0.1)

        self.world_name = self.get_parameter('world_name').value
        self.frame_id = self.get_parameter('frame_id').value
        self.dt = float(self.get_parameter('dt').value)

        self.buoys = {}
        self._buoy_counter = 0
        self._last_names_json = '[]'

        self.buoy_pub = self.create_publisher(BuoyArray, '/dynamic_buoy/buoys', 10)
        self.tracked_pub = self.create_publisher(
            TrackedShipList, '/dynamic_buoy/tracked_ships', 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/dynamic_buoy/markers', 10)
        self.names_pub = self.create_publisher(String, '/dynamic_buoy/names', 10)

        self.spawn_srv = self.create_service(
            SpawnDynamicBuoy, '/dynamic_buoy/spawn', self.on_spawn)
        self.delete_srv = self.create_service(
            DeleteDynamicBuoy, '/dynamic_buoy/delete', self.on_delete)
        self.clear_srv = self.create_service(
            ClearDynamicBuoys, '/dynamic_buoy/clear', self.on_clear)
        self.config_srv = self.create_service(
            SetDynamicBuoyConfig, '/dynamic_buoy/set_config', self.on_set_config)

        self.click_sub = self.create_subscription(
            PointStamped, '/dynamic_buoy/clicked_point', self.on_clicked_point, 10)

        self.timer = self.create_timer(self.dt, self.publish_state)

        self.get_logger().info(
            'DynamicBuoyManager ready (VRX light buoys, static spawn)')

    def _read_config(self):
        buoy_type = str(self.get_parameter('buoy_type').value).strip().lower()
        radius_m = float(self.get_parameter('radius_m').value)
        if buoy_type not in VALID_BUOY_TYPES:
            buoy_type = 'rgb'
        return buoy_type, max(0.1, radius_m)

    def on_set_config(self, request, response):
        try:
            buoy_type = request.buoy_type.strip().lower() or 'rgb'
            if buoy_type not in VALID_BUOY_TYPES:
                buoy_type = 'rgb'
            radius_m = max(0.1, float(request.radius_m))
            self.set_parameters([
                Parameter('buoy_type', value=buoy_type),
                Parameter('radius_m', value=radius_m),
            ])
            response.success = True
            response.message = 'buoy config updated'
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        return response

    def on_clicked_point(self, msg):
        buoy_type, radius_m = self._read_config()
        self._buoy_counter += 1
        name = f'dyn_buoy_{self._buoy_counter}'
        self._spawn_buoy_at(name, msg.point.x, msg.point.y, buoy_type, radius_m)

    def on_spawn(self, request, response):
        try:
            if not request.name.strip():
                self._buoy_counter += 1
                name = f'dyn_buoy_{self._buoy_counter}'
            else:
                name = request.name.strip()

            if name in self.buoys:
                response.success = False
                response.message = f"Buoy '{name}' already exists"
                return response

            buoy_type = request.buoy_type.strip().lower() or self._read_config()[0]
            if buoy_type not in VALID_BUOY_TYPES:
                buoy_type = 'rgb'
            radius_m = request.radius_m if request.radius_m > 0.0 else self._read_config()[1]

            self._spawn_buoy_at(
                name,
                request.pose.position.x,
                request.pose.position.y,
                buoy_type,
                radius_m,
            )
            response.success = True
            response.model_name = name
            response.message = f"Buoy '{name}' spawned successfully"
        except Exception as exc:
            response.success = False
            response.message = f'Spawn failed: {exc}'
            self.get_logger().error(f'Spawn failed: {exc}')
        return response

    def on_delete(self, request, response):
        name = request.model_name.strip()
        if name not in self.buoys:
            response.success = False
            response.message = f"Buoy '{name}' not found"
            return response
        try:
            self._remove_gazebo(name)
            del self.buoys[name]
            response.success = True
            response.message = f"Buoy '{name}' deleted"
            self.get_logger().info(f"Deleted dynamic buoy '{name}'")
        except Exception as exc:
            response.success = False
            response.message = f'Delete failed: {exc}'
        return response

    def on_clear(self, request, response):
        names = list(self.buoys.keys())
        for name in names:
            try:
                self._remove_gazebo(name)
                del self.buoys[name]
            except Exception as exc:
                self.get_logger().warn(f"Failed to remove '{name}' during clear: {exc}")
        response.success = True
        response.message = f'Cleared {len(names)} buoys'
        self.get_logger().info(f'Cleared all {len(names)} dynamic buoys')
        return response

    def _spawn_buoy_at(self, name, x, y, buoy_type, radius_m):
        buoy_id = str(uuid.uuid5(uuid.NAMESPACE_DNS, f'buoy:{name}'))
        buoy = DynamicBuoy(name, buoy_id, x, y, buoy_type, radius_m)
        sdf_str = self._generate_buoy_sdf(buoy)
        self._spawn_gazebo(buoy, sdf_str)
        self.buoys[name] = buoy
        self.get_logger().info(
            f'spawned {name} at ({x:.2f},{y:.2f}) type={buoy_type} radius={radius_m:.1f}m')

    def _generate_buoy_sdf(self, buoy):
        template_name = f'robotx_light_buoy_{buoy.buoy_type}'
        try:
            vrx_share = get_package_share_directory('vrx_gz')
        except Exception as exc:
            raise FileNotFoundError(f'vrx_gz package not found: {exc}') from exc

        template_path = os.path.join(vrx_share, 'models', template_name, 'model.sdf')
        if not os.path.isfile(template_path):
            raise FileNotFoundError(f'VRX buoy model not found: {template_path}')

        with open(template_path, 'r', encoding='utf-8') as f:
            sdf = f.read()

        sdf = re.sub(
            rf'<model name="{re.escape(template_name)}">',
            f'<model name="{buoy.model_name}">',
            sdf,
            count=1,
        )
        sdf = sdf.replace(f'{template_name}::', f'{buoy.model_name}::')
        if '<static>' not in sdf:
            sdf = sdf.replace(
                f'<model name="{buoy.model_name}">',
                f'<model name="{buoy.model_name}">\n    <static>true</static>',
                1,
            )
        return sdf

    def _spawn_gazebo(self, buoy, sdf_str):
        spawn_z = float(self.get_parameter('spawn_z').value)
        tmp_sdf_path = None
        try:
            with tempfile.NamedTemporaryFile(mode='w', suffix='.sdf', delete=False) as tmpf:
                tmpf.write(sdf_str)
                tmp_sdf_path = tmpf.name

            cmd = [
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-world', self.world_name,
                '-file', tmp_sdf_path,
                '-name', buoy.model_name,
                '-x', str(buoy.x),
                '-y', str(buoy.y),
                '-z', str(spawn_z),
            ]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
            if result.returncode != 0:
                detail = (result.stderr or result.stdout or '').strip()
                raise RuntimeError(f'Gazebo spawn failed: {detail}')
            self.get_logger().info(
                f"Gazebo entity '{buoy.model_name}' spawned via ros_gz_sim create")
        finally:
            if tmp_sdf_path and os.path.exists(tmp_sdf_path):
                self._retained_spawn_sdf_paths.append(tmp_sdf_path)

    def _remove_gazebo(self, model_name):
        cmd = [
            'gz', 'service', '-s', f'/world/{self.world_name}/remove',
            '--reqtype', 'gz.msgs.Entity',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '5000',
            '--req', f'name: "{model_name}" type: MODEL',
        ]
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
        if result.returncode != 0:
            self.get_logger().warn(f"Failed to remove Gazebo entity '{model_name}'")

    def build_markers(self, stamp):
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        marker_id = 1
        for buoy in self.buoys.values():
            cyl = Marker()
            cyl.header.stamp = stamp
            cyl.header.frame_id = self.frame_id
            cyl.ns = 'dynamic_buoy'
            cyl.id = marker_id
            marker_id += 1
            cyl.type = Marker.CYLINDER
            cyl.action = Marker.ADD
            cyl.pose = buoy.pose()
            cyl.pose.position.z = 0.5
            cyl.scale.x = buoy.radius_m * 2.0
            cyl.scale.y = buoy.radius_m * 2.0
            cyl.scale.z = 1.0
            cyl.color.a = 0.35
            cyl.color.r = 0.1
            cyl.color.g = 0.65
            cyl.color.b = 0.95
            markers.markers.append(cyl)

            label = Marker()
            label.header.stamp = stamp
            label.header.frame_id = self.frame_id
            label.ns = 'dynamic_buoy_label'
            label.id = marker_id
            marker_id += 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = buoy.x
            label.pose.position.y = buoy.y
            label.pose.position.z = 2.5
            label.pose.orientation.w = 1.0
            label.scale.z = 0.8
            label.color.a = 0.95
            label.color.r = 0.9
            label.color.g = 0.95
            label.color.b = 1.0
            label.text = f'{buoy.model_name}\n{buoy.buoy_type.upper()} r={buoy.radius_m:.1f}m'
            markers.markers.append(label)

        return markers

    def publish_state(self):
        stamp = self.get_clock().now().to_msg()

        buoy_msg = BuoyArray()
        buoy_msg.header.stamp = stamp
        buoy_msg.header.frame_id = self.frame_id

        tracked_msg = TrackedShipList()
        tracked_msg.header = buoy_msg.header

        for buoy in sorted(self.buoys.values(), key=lambda b: b.model_name):
            buoy_msg.buoys.append(buoy.buoy_msg())
            tracked_msg.ships.append(buoy.tracked_ship())

        self.buoy_pub.publish(buoy_msg)
        self.tracked_pub.publish(tracked_msg)

        names_json = json.dumps([b.name for b in buoy_msg.buoys])
        if names_json != self._last_names_json:
            self._last_names_json = names_json
            self.names_pub.publish(String(data=names_json))

        self.marker_pub.publish(self.build_markers(stamp))


def main(args=None):
    rclpy.init(args=args)
    node = DynamicBuoyManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for p in getattr(node, '_retained_spawn_sdf_paths', []):
            if p and os.path.isfile(p):
                try:
                    os.remove(p)
                except OSError:
                    pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
