#!/usr/bin/env python3

import json
import math
import uuid

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, PointStamped, Pose, Twist
from nav2_colregs_msgs.msg import TrackedShip, TrackedShipList
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from usv_interfaces.srv import (
    ClearStormFields,
    DeleteStormField,
    SetStormFieldConfig,
)
from usv_interfaces.msg import StormField as StormFieldMsg
from usv_interfaces.msg import StormFieldArray
from visualization_msgs.msg import Marker, MarkerArray


class StormField:
    def __init__(self, name, x, y, radius, drift_heading_deg, drift_speed,
                 weather_validity_duration_s, weather_grid_resolution_m,
                 created_time_sec):
        self.name = name
        self.target_id = str(uuid.uuid5(uuid.NAMESPACE_DNS, f'storm:{name}'))
        self.x = float(x)
        self.y = float(y)
        self.radius = float(radius)
        self.drift_heading_deg = float(drift_heading_deg)
        self.drift_speed = float(drift_speed)
        self.weather_validity_duration_s = float(weather_validity_duration_s)
        self.weather_grid_resolution_m = float(weather_grid_resolution_m)
        self.created_time_sec = float(created_time_sec)

    def update_config(self, radius, drift_heading_deg, drift_speed,
                      weather_validity_duration_s, weather_grid_resolution_m):
        self.radius = float(radius)
        self.drift_heading_deg = float(drift_heading_deg)
        self.drift_speed = float(drift_speed)
        self.weather_validity_duration_s = float(weather_validity_duration_s)
        self.weather_grid_resolution_m = float(weather_grid_resolution_m)

    def _yaw(self):
        return math.pi / 2.0 - math.radians(self.drift_heading_deg)

    def step(self, dt):
        yaw = self._yaw()
        self.x += self.drift_speed * math.cos(yaw) * dt
        self.y += self.drift_speed * math.sin(yaw) * dt

    def pose(self):
        yaw = self._yaw()
        pose = Pose()
        pose.position.x = self.x
        pose.position.y = self.y
        pose.position.z = 0.0
        pose.orientation.w = math.cos(yaw / 2.0)
        pose.orientation.z = math.sin(yaw / 2.0)
        return pose

    def twist(self):
        yaw = self._yaw()
        twist = Twist()
        twist.linear.x = self.drift_speed * math.cos(yaw)
        twist.linear.y = self.drift_speed * math.sin(yaw)
        return twist

    def tracked_ship(self):
        ts = TrackedShip()
        ts.target_id.uuid = list(bytes.fromhex(self.target_id.replace('-', '')))
        ts.pose = self.pose()
        ts.twist = self.twist()
        ts.radius = self.radius
        return ts

    @staticmethod
    def _time_from_sec(sec):
        msg = Time()
        msg.sec = int(sec)
        msg.nanosec = int((sec - msg.sec) * 1e9)
        return msg

    def storm_field_msg(self, now_sec):
        msg = StormFieldMsg()
        msg.name = self.name
        msg.storm_id.uuid = list(bytes.fromhex(self.target_id.replace('-', '')))
        msg.center = Point(x=self.x, y=self.y, z=0.0)
        msg.radius_m = float(self.radius)
        msg.drift_heading_deg = float(self.drift_heading_deg)
        msg.drift_speed_mps = float(self.drift_speed)
        msg.weather_validity_duration_s = float(self.weather_validity_duration_s)
        msg.weather_grid_resolution_m = float(self.weather_grid_resolution_m)
        msg.created_time = self._time_from_sec(self.created_time_sec)
        msg.valid_until = self._time_from_sec(
            self.created_time_sec + self.weather_validity_duration_s)
        msg.remaining_validity_s = max(
            0.0,
            self.created_time_sec + self.weather_validity_duration_s - now_sec)
        return msg

    def is_expired(self, now_sec):
        return (
            self.weather_validity_duration_s > 0.0 and
            now_sec - self.created_time_sec >= self.weather_validity_duration_s
        )


class StormFieldManager(Node):
    def __init__(self):
        super().__init__('storm_field_manager')

        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('tracked_ship_topic', '/dynamic_ship/tracked_ships')
        self.declare_parameter('names_topic', '/storm_field/names')
        self.declare_parameter('marker_topic', '/storm_field/markers')
        self.declare_parameter('storm_field_topic', '/storm_field/storms')
        self.declare_parameter('clicked_point_topic', '/storm_field/clicked_point')
        self.declare_parameter('dt', 0.1)
        self.declare_parameter('drift_heading_deg', 0.0)
        self.declare_parameter('drift_speed', 0.0)
        self.declare_parameter('radius', 30.0)
        self.declare_parameter('weather_validity_duration_s', 3600.0)
        self.declare_parameter('weather_grid_resolution_m', 100.0)

        self.frame_id = self.get_parameter('frame_id').value
        self.dt = float(self.get_parameter('dt').value)
        tracked_ship_topic = self.get_parameter('tracked_ship_topic').value
        names_topic = self.get_parameter('names_topic').value
        marker_topic = self.get_parameter('marker_topic').value
        storm_field_topic = self.get_parameter('storm_field_topic').value
        clicked_point_topic = self.get_parameter('clicked_point_topic').value

        self.storms = {}

        self.tracked_pub = self.create_publisher(TrackedShipList, tracked_ship_topic, 10)
        self.names_pub = self.create_publisher(String, names_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, marker_topic, 10)
        self.storm_field_pub = self.create_publisher(StormFieldArray, storm_field_topic, 10)

        self.click_sub = self.create_subscription(
            PointStamped, clicked_point_topic, self.on_clicked_point, 10)

        self.config_srv = self.create_service(
            SetStormFieldConfig, '/storm_field/set_config', self.on_set_config)
        self.delete_srv = self.create_service(
            DeleteStormField, '/storm_field/delete', self.on_delete)
        self.clear_srv = self.create_service(
            ClearStormFields, '/storm_field/clear', self.on_clear)

        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info(
            f'StormFieldManager publishing TrackedShipList on {tracked_ship_topic}')

    def _read_config(self):
        return (
            float(self.get_parameter('drift_heading_deg').value),
            float(self.get_parameter('drift_speed').value),
            float(self.get_parameter('radius').value),
            float(self.get_parameter('weather_validity_duration_s').value),
            float(self.get_parameter('weather_grid_resolution_m').value),
        )

    def _next_storm_name(self):
        index = 1
        while f'storm_{index}' in self.storms:
            index += 1
        return f'storm_{index}'

    def on_set_config(self, request, response):
        try:
            radius = max(0.1, float(request.radius))
            drift_speed = max(0.0, float(request.drift_speed))
            drift_heading_deg = float(request.drift_heading_deg) % 360.0
            weather_validity_duration_s = max(
                0.0, float(request.weather_validity_duration_s))
            weather_grid_resolution_m = max(
                0.1, float(request.weather_grid_resolution_m))

            self.set_parameters([
                Parameter('drift_heading_deg', value=drift_heading_deg),
                Parameter('drift_speed', value=drift_speed),
                Parameter('radius', value=radius),
                Parameter('weather_validity_duration_s', value=weather_validity_duration_s),
                Parameter('weather_grid_resolution_m', value=weather_grid_resolution_m),
            ])
            for storm in self.storms.values():
                storm.update_config(
                    radius, drift_heading_deg, drift_speed,
                    weather_validity_duration_s, weather_grid_resolution_m)
            response.success = True
            response.message = 'storm config updated'
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        return response

    def on_clicked_point(self, msg):
        (drift_heading_deg, drift_speed, radius,
         weather_validity_duration_s, weather_grid_resolution_m) = self._read_config()
        name = self._next_storm_name()
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        self.storms[name] = StormField(
            name, msg.point.x, msg.point.y, radius,
            drift_heading_deg, drift_speed,
            weather_validity_duration_s, weather_grid_resolution_m,
            now_sec)
        self.get_logger().info(
            f'{name} set at ({msg.point.x:.2f}, {msg.point.y:.2f}) '
            f'radius={radius:.1f}m grid={weather_grid_resolution_m:.1f}m '
            f'validity={weather_validity_duration_s:.0f}s')

    def remove_expired_storms(self, now_sec):
        expired_names = [
            name for name, storm in self.storms.items()
            if storm.is_expired(now_sec)
        ]
        for name in expired_names:
            del self.storms[name]
            self.get_logger().info(f'{name} expired and removed')

    def on_delete(self, request, response):
        name = request.storm_name.strip()
        if not name:
            response.success = False
            response.message = 'storm_name is empty'
            return response
        if name not in self.storms:
            response.success = False
            response.message = f"Storm '{name}' not found"
            return response
        del self.storms[name]
        response.success = True
        response.message = 'storm deleted'
        return response

    def on_clear(self, request, response):
        self.storms.clear()
        response.success = True
        response.message = 'storm cleared'
        return response

    def build_markers(self, stamp):
        markers = MarkerArray()

        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        marker_id = 1
        for storm in self.storms.values():
            disk = Marker()
            disk.header.stamp = stamp
            disk.header.frame_id = self.frame_id
            disk.ns = 'storm_field'
            disk.id = marker_id
            marker_id += 1
            disk.type = Marker.CYLINDER
            disk.action = Marker.ADD
            disk.pose = storm.pose()
            disk.pose.position.z = 0.05
            disk.scale.x = storm.radius * 2.0
            disk.scale.y = storm.radius * 2.0
            disk.scale.z = 0.1
            disk.color.a = 0.32
            disk.color.r = 1.0
            disk.color.g = 0.35
            disk.color.b = 0.08
            markers.markers.append(disk)

            label = Marker()
            label.header.stamp = stamp
            label.header.frame_id = self.frame_id
            label.ns = 'storm_field_label'
            label.id = marker_id
            marker_id += 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = storm.x
            label.pose.position.y = storm.y
            label.pose.position.z = 2.0
            label.pose.orientation.w = 1.0
            label.scale.z = max(1.0, min(4.0, storm.radius * 0.12))
            label.color.a = 0.95
            label.color.r = 1.0
            label.color.g = 0.95
            label.color.b = 0.75
            label.text = (
                f'{storm.name}\n'
                f'grid {storm.weather_grid_resolution_m:.0f}m, '
                f'{storm.weather_validity_duration_s / 60.0:.0f}min')
            markers.markers.append(label)

        return markers

    def control_loop(self):
        msg = TrackedShipList()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        self.remove_expired_storms(now_sec)

        storm_msg = StormFieldArray()
        storm_msg.header = msg.header

        names = []
        for storm in self.storms.values():
            storm.step(self.dt)
            msg.ships.append(storm.tracked_ship())
            storm_msg.storms.append(storm.storm_field_msg(now_sec))
            names.append(storm.name)

        self.tracked_pub.publish(msg)
        self.storm_field_pub.publish(storm_msg)
        self.names_pub.publish(String(data=json.dumps(names)))
        self.marker_pub.publish(self.build_markers(msg.header.stamp))


def main(args=None):
    rclpy.init(args=args)
    node = StormFieldManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
