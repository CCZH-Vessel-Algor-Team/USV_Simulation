#!/usr/bin/env python3

import json
import math
import os
import subprocess
import tempfile
import uuid
import yaml

import rclpy
from geometry_msgs.msg import PointStamped, Pose, Twist
from rclpy.node import Node
from std_msgs.msg import String
from nav2_colregs_msgs.msg import TrackedShip, TrackedShipList
from usv_interfaces.srv import (
    SpawnDynamicShip, DeleteDynamicShip, ClearDynamicShips,
    SetDynamicShipConfig,
)


def _fmt_pose_xyzrpy(xyz, rpy):
    return (
        f"{float(xyz[0]):.6f} {float(xyz[1]):.6f} {float(xyz[2]):.6f} "
        f"{float(rpy[0]):.6f} {float(rpy[1]):.6f} {float(rpy[2]):.6f}"
    )


def _as_vec(raw, length, default=0.0):
    if isinstance(raw, (list, tuple)):
        vals = [float(v) for v in raw[:length]]
    else:
        vals = []
    if len(vals) < length:
        vals.extend([float(default)] * (length - len(vals)))
    return vals


def _resolve_profile_path(path_text, config_base_dir):
    p = str(path_text or '').strip()
    if not p:
        return ''
    if os.path.isabs(p):
        return p
    return os.path.normpath(os.path.join(config_base_dir, p))


class DynamicShip:
    def __init__(self, model_name, target_id, pose, half_distance, shape, speed, color,
                 mesh_profile, node, config_base_dir, world_name):
        self.model_name = model_name
        self.target_id = target_id
        self.shape = shape
        self.speed = speed
        self.half_distance = half_distance
        self.color = color
        self.mesh_profile = mesh_profile
        self.node = node
        self.config_base_dir = config_base_dir
        self.world_name = world_name

        x = pose.position.x
        y = pose.position.y

        def quat_to_yaw(q):
            return math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )

        spawn_yaw = quat_to_yaw(pose.orientation)
        self.spawn_yaw = spawn_yaw

        total_dist = 2.0 * half_distance
        self.waypoint_a = (x, y)
        self.waypoint_b = (
            x + total_dist * math.cos(spawn_yaw),
            y + total_dist * math.sin(spawn_yaw)
        )

        self.current_x = x
        self.current_y = y
        self.direction = 1

        self.cmd_vel_pub = node.create_publisher(
            Twist, f'/model/{model_name}/cmd_vel', 10)

    def start_bridge(self):
        cmd = [
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            f'/model/{self.model_name}/cmd_vel'
            f'@geometry_msgs/msg/Twist]gz.msgs.Twist',
        ]
        self.bridge_process = subprocess.Popen(
            cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def cleanup(self):
        if hasattr(self, 'bridge_process') and self.bridge_process:
            self.bridge_process.terminate()
        if hasattr(self, 'bridge_process'):
            try:
                self.bridge_process.wait(timeout=2)
            except Exception:
                pass

    def world_twist_to_body(self, vx_world, vy_world):
        c = math.cos(self.spawn_yaw)
        s = math.sin(self.spawn_yaw)
        twist = Twist()
        twist.linear.x = c * vx_world + s * vy_world
        twist.linear.y = -s * vx_world + c * vy_world
        twist.angular.z = 0.0
        return twist

    def compute_cmd_vel(self, dt):
        target = self.waypoint_b if self.direction > 0 else self.waypoint_a
        dx = target[0] - self.current_x
        dy = target[1] - self.current_y
        dist = math.hypot(dx, dy)

        if dist < 0.5:
            self.direction *= -1
            target = self.waypoint_b if self.direction > 0 else self.waypoint_a
            dx = target[0] - self.current_x
            dy = target[1] - self.current_y
            dist = math.hypot(dx, dy)

        if dist > 0:
            vx = (dx / dist) * self.speed
            vy = (dy / dist) * self.speed
        else:
            vx = 0.0
            vy = 0.0

        self.current_x += vx * dt
        self.current_y += vy * dt
        return self.world_twist_to_body(vx, vy)

    def get_current_pose(self):
        pose = Pose()
        pose.position.x = self.current_x
        pose.position.y = self.current_y
        pose.position.z = 0.0
        pose.orientation.w = math.cos(self.spawn_yaw / 2.0)
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = math.sin(self.spawn_yaw / 2.0)
        return pose

    def get_current_twist(self):
        target = self.waypoint_b if self.direction > 0 else self.waypoint_a
        dx = target[0] - self.current_x
        dy = target[1] - self.current_y
        dist = math.hypot(dx, dy)
        twist = Twist()
        if dist > 0:
            twist.linear.x = (dx / dist) * self.speed
            twist.linear.y = (dy / dist) * self.speed
        return twist


class DynamicShipManager(Node):
    def __init__(self):
        super().__init__('dynamic_ship_manager')
        self._retained_spawn_sdf_paths = []

        self.declare_parameter('world_name', 'sydney_regatta')
        self.declare_parameter('default_mesh_profile', '')
        self.declare_parameter('config_base_dir', '')

        self.world_name = self.get_parameter('world_name').get_parameter_value().string_value
        self.default_mesh_profile = self.get_parameter('default_mesh_profile').get_parameter_value().string_value
        self.config_base_dir = self.get_parameter('config_base_dir').get_parameter_value().string_value

        if not self.config_base_dir:
            self.config_base_dir = os.path.dirname(os.path.abspath(__file__))

        self.declare_parameter('heading_deg', 0.0)
        self.declare_parameter('speed', 3.0)
        self.declare_parameter('shape', 'mesh_profile')
        self.declare_parameter('half_distance', 50.0)

        self.ships = {}
        self.dt = 0.1

        self.spawn_srv = self.create_service(
            SpawnDynamicShip, '/dynamic_ship/spawn', self.on_spawn)
        self.delete_srv = self.create_service(
            DeleteDynamicShip, '/dynamic_ship/delete', self.on_delete)
        self.clear_srv = self.create_service(
            ClearDynamicShips, '/dynamic_ship/clear', self.on_clear)

        self.tracked_pub = self.create_publisher(
            TrackedShipList, '/dynamic_ship/tracked_ships', 10)

        self.timer = self.create_timer(self.dt, self.control_loop)

        self._ship_counter = 0

        self.click_sub = self.create_subscription(
            PointStamped, '/clicked_point', self.on_clicked_point, 10)

        self.config_srv = self.create_service(
            SetDynamicShipConfig, '/dynamic_ship/set_config',
            self.on_set_config)

        self.names_pub = self.create_publisher(
            String, '/dynamic_ship/names', 10)

    def _read_config(self):
        return (
            self.get_parameter('heading_deg').get_parameter_value().double_value,
            self.get_parameter('speed').get_parameter_value().double_value,
            self.get_parameter('shape').get_parameter_value().string_value,
            self.get_parameter('half_distance').get_parameter_value().double_value,
        )

    def on_set_config(self, request, response):
        try:
            self.set_parameters([
                rclpy.parameter.Parameter(
                    'heading_deg', value=request.heading_deg),
                rclpy.parameter.Parameter(
                    'speed', value=request.speed),
                rclpy.parameter.Parameter(
                    'shape', value=request.shape),
                rclpy.parameter.Parameter(
                    'half_distance', value=request.half_distance),
            ])
            response.success = True
            response.message = 'config updated'
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def on_clicked_point(self, msg):
        heading_deg, speed, shape, half_dist = self._read_config()
        heading = math.radians(heading_deg)

        self._ship_counter += 1
        name = f'dyn_target_{self._ship_counter}'

        self._spawn_ship_at(name, msg.point.x, msg.point.y, heading,
                            half_dist, shape, speed)

    def _spawn_ship_at(self, name, x, y, yaw, half_dist, shape, speed):
        target_id = str(uuid.uuid5(uuid.NAMESPACE_DNS, name))
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = 0.0
        pose.orientation.w = math.cos(yaw / 2.0)
        pose.orientation.z = math.sin(yaw / 2.0)

        color = (1.0, 0.2, 0.2)

        ship = DynamicShip(
            model_name=name, target_id=target_id, pose=pose,
            half_distance=half_dist, shape=shape, speed=speed,
            color=color,
            mesh_profile=self.default_mesh_profile,
            node=self, config_base_dir=self.config_base_dir,
            world_name=self.world_name,
        )

        sdf_str = self._generate_sdf(ship)
        try:
            self._spawn_gazebo(ship, sdf_str)
            ship.start_bridge()
        except Exception as e:
            ship.cleanup()
            self.get_logger().error(f'spawn failed: {e}')
            return

        self.ships[name] = ship
        self.get_logger().info(
            f'clicked at ({x:.2f},{y:.2f}) heading={math.degrees(yaw):.0f}deg '
            f'-> spawned {name} speed={speed}m/s')

    def on_spawn(self, request, response):
        try:
            if not request.name.strip():
                self._ship_counter += 1
                name = f'dyn_target_{self._ship_counter}'
            else:
                name = request.name.strip()

            if name in self.ships:
                response.success = False
                response.message = f"Ship '{name}' already exists"
                return response

            yaw = math.atan2(
                2.0 * (request.pose.orientation.w * request.pose.orientation.z
                       + request.pose.orientation.x * request.pose.orientation.y),
                1.0 - 2.0 * (request.pose.orientation.y * request.pose.orientation.y
                             + request.pose.orientation.z * request.pose.orientation.z))

            half_dist = request.half_distance if request.half_distance > 0 else 50.0
            speed = request.speed if request.speed > 0.0 else 3.0
            shape = request.shape.strip() or 'mesh_profile'

            self._spawn_ship_at(name, request.pose.position.x,
                               request.pose.position.y, yaw,
                               half_dist, shape, speed)
            response.success = True
            response.model_name = name
            response.message = f"Ship '{name}' spawned successfully"
        except Exception as e:
            response.success = False
            response.message = f"Spawn failed: {e}"
            self.get_logger().error(f"Spawn failed: {e}")
        return response

    def on_delete(self, request, response):
        name = request.model_name.strip()
        if name not in self.ships:
            response.success = False
            response.message = f"Ship '{name}' not found"
            return response
        try:
            self._remove_gazebo(name)
            self.ships[name].cleanup()
            del self.ships[name]
            response.success = True
            response.message = f"Ship '{name}' deleted"
            self.get_logger().info(f"Deleted dynamic ship '{name}'")
        except Exception as e:
            response.success = False
            response.message = f"Delete failed: {e}"
        return response

    def on_clear(self, request, response):
        names = list(self.ships.keys())
        for name in names:
            try:
                self._remove_gazebo(name)
                self.ships[name].cleanup()
                del self.ships[name]
            except Exception as e:
                self.get_logger().warn(f"Failed to remove '{name}' during clear: {e}")
        self.get_logger().info(f"Cleared all {len(names)} dynamic ships")
        response.success = True
        response.message = f"Cleared {len(names)} ships"
        return response

    def _generate_sdf(self, ship):
        if ship.shape == 'mesh_profile':
            return self._generate_mesh_profile_sdf(ship)
        elif ship.shape == 'box':
            return self._generate_box_sdf(ship)
        else:
            return self._generate_cylinder_sdf(ship)

    def _generate_mesh_profile_sdf(self, ship):
        profile_path = _resolve_profile_path(ship.mesh_profile, self.config_base_dir)
        if not profile_path or not os.path.isfile(profile_path):
            raise FileNotFoundError(f"mesh_profile not found: {ship.mesh_profile}")

        with open(profile_path, 'r') as f:
            data = yaml.safe_load(f) or {}

        mesh = data.get('mesh') or {}
        mesh_uri = str(mesh.get('uri', '')).strip()
        mesh_scale = _as_vec(mesh.get('scale'), 3, 1.0)
        mesh_rgba = _as_vec(mesh.get('material_rgba'), 4, 1.0)
        mesh_pose = mesh.get('pose_offset') or {}
        mesh_xyz = _as_vec(mesh_pose.get('xyz'), 3, 0.0)
        mesh_rpy = _as_vec(mesh_pose.get('rpy'), 3, 0.0)

        collision_group = data.get('collision_group') or {}
        group_pose = collision_group.get('pose_offset') or {}
        group_xyz = _as_vec(group_pose.get('xyz'), 3, 0.0)
        group_rpy = _as_vec(group_pose.get('rpy'), 3, 0.0)

        collision_xml = []
        for idx, box in enumerate(data.get('boxes') or []):
            box_name = str(box.get('name') or f'collision_{idx}').strip() or f'collision_{idx}'
            size = _as_vec(box.get('size_lwh_m'), 3, 0.0)
            pose_xyz = _as_vec(box.get('pose_xyz_m'), 3, 0.0)
            pose_rpy = _as_vec(box.get('pose_rpy'), 3, 0.0)
            world_xyz = [
                group_xyz[0] + pose_xyz[0],
                group_xyz[1] + pose_xyz[1],
                group_xyz[2] + pose_xyz[2],
            ]
            world_rpy = [
                group_rpy[0] + pose_rpy[0],
                group_rpy[1] + pose_rpy[1],
                group_rpy[2] + pose_rpy[2],
            ]
            collision_xml.append(
                f"""
                    <collision name="{box_name}">
                        <pose>{_fmt_pose_xyzrpy(world_xyz, world_rpy)}</pose>
                        <geometry>
                            <box><size>{size[0]:.6f} {size[1]:.6f} {size[2]:.6f}</size></box>
                        </geometry>
                    </collision>"""
            )

        spawn_z = float(data.get('spawn_z', 0.0))

        r, g, b = ship.color
        a = 1.0

        sdf = f"""<?xml version="1.0" ?>
        <sdf version="1.6">
            <model name="{ship.model_name}">
                <static>false</static>
                <link name="base_link">
                    <gravity>false</gravity>
                    <visual name="visual">
                        <pose>{_fmt_pose_xyzrpy(mesh_xyz, mesh_rpy)}</pose>
                        <geometry>
                            <mesh>
                                <uri>{mesh_uri}</uri>
                                <scale>{mesh_scale[0]:.6f} {mesh_scale[1]:.6f} {mesh_scale[2]:.6f}</scale>
                            </mesh>
                        </geometry>
                        <material>
                            <ambient>{r:.6f} {g:.6f} {b:.6f} {a:.6f}</ambient>
                            <diffuse>{r:.6f} {g:.6f} {b:.6f} {a:.6f}</diffuse>
                            <specular>0.15 0.15 0.10 1.0</specular>
                        </material>
                    </visual>
                    {''.join(collision_xml)}
                    <inertial>
                        <mass>50.0</mass>
                        <inertia>
                            <ixx>4.0</ixx>
                            <iyy>4.0</iyy>
                            <izz>6.0</izz>
                        </inertia>
                    </inertial>
                </link>
                <plugin filename="gz-sim-velocity-control-system" name="gz::sim::systems::VelocityControl">
                    <topic>/model/{ship.model_name}/cmd_vel</topic>
                </plugin>
            </model>
        </sdf>
        """
        return sdf

    def _generate_box_sdf(self, ship):
        name = ship.model_name
        r, g, b = ship.color
        geom = "<box><size>3.6 10.0 2.0</size></box>"
        sdf = f"""<?xml version="1.0" ?>
        <sdf version="1.6">
            <model name="{name}">
                <static>false</static>
                <link name="base_link">
                    <gravity>false</gravity>
                    <visual name="visual">
                        <geometry>{geom}</geometry>
                        <material>
                            <ambient>{r:.6f} {g:.6f} {b:.6f} 1.0</ambient>
                            <diffuse>{r:.6f} {g:.6f} {b:.6f} 1.0</diffuse>
                            <emissive>{r:.6f} {g:.6f} {b:.6f} 1.0</emissive>
                        </material>
                    </visual>
                    <collision name="collision">
                        <geometry>{geom}</geometry>
                    </collision>
                    <inertial>
                        <mass>50.0</mass>
                        <inertia>
                            <ixx>4.0</ixx>
                            <iyy>4.0</iyy>
                            <izz>6.0</izz>
                        </inertia>
                    </inertial>
                </link>
                <plugin filename="gz-sim-velocity-control-system" name="gz::sim::systems::VelocityControl">
                    <topic>/model/{name}/cmd_vel</topic>
                </plugin>
            </model>
        </sdf>
        """
        return sdf

    def _generate_cylinder_sdf(self, ship):
        name = ship.model_name
        r, g, b = ship.color
        geom = "<cylinder><radius>4.0</radius><length>5.0</length></cylinder>"
        sdf = f"""<?xml version="1.0" ?>
        <sdf version="1.6">
            <model name="{name}">
                <static>false</static>
                <link name="base_link">
                    <gravity>false</gravity>
                    <visual name="visual">
                        <geometry>{geom}</geometry>
                        <material>
                            <ambient>{r:.6f} {g:.6f} {b:.6f} 1.0</ambient>
                            <diffuse>{r:.6f} {g:.6f} {b:.6f} 1.0</diffuse>
                            <emissive>{r:.6f} {g:.6f} {b:.6f} 1.0</emissive>
                        </material>
                    </visual>
                    <collision name="collision">
                        <geometry>{geom}</geometry>
                    </collision>
                    <inertial>
                        <mass>50.0</mass>
                        <inertia>
                            <ixx>4.0</ixx>
                            <iyy>4.0</iyy>
                            <izz>6.0</izz>
                        </inertia>
                    </inertial>
                </link>
                <plugin filename="gz-sim-velocity-control-system" name="gz::sim::systems::VelocityControl">
                    <topic>/model/{name}/cmd_vel</topic>
                </plugin>
            </model>
        </sdf>
        """
        return sdf

    def _spawn_gazebo(self, ship, sdf_str):
        z = 0.5
        if ship.shape == 'mesh_profile':
            profile_path = _resolve_profile_path(ship.mesh_profile, self.config_base_dir)
            if profile_path and os.path.isfile(profile_path):
                with open(profile_path, 'r') as f:
                    data = yaml.safe_load(f) or {}
                profile_spawn_z = float(data.get('spawn_z', 0.0))
                if profile_spawn_z != 0.0:
                    z = profile_spawn_z

        tmp_sdf_path = None
        try:
            with tempfile.NamedTemporaryFile(mode='w', suffix='.sdf', delete=False) as tmpf:
                tmpf.write(sdf_str)
                tmp_sdf_path = tmpf.name

            cmd = [
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-world', self.world_name,
                '-file', tmp_sdf_path,
                '-name', ship.model_name,
                '-x', str(ship.current_x),
                '-y', str(ship.current_y),
                '-z', str(z),
                '-Y', str(ship.spawn_yaw),
            ]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
            if result.returncode == 0:
                self.get_logger().info(
                    f"Gazebo entity '{ship.model_name}' spawned via ros_gz_sim create")
            else:
                detail = (result.stderr or result.stdout or '').strip()
                self.get_logger().error(
                    f"Gazebo spawn failed for '{ship.model_name}' (code={result.returncode}): {detail}")
                raise RuntimeError(f"Gazebo spawn failed: {detail}")
        except Exception:
            raise
        finally:
            if tmp_sdf_path and os.path.exists(tmp_sdf_path):
                self._retained_spawn_sdf_paths.append(tmp_sdf_path)

    def _remove_gazebo(self, model_name):
        try:
            cmd = [
                'gz', 'service', '-s', f'/world/{self.world_name}/remove',
                '--reqtype', 'gz.msgs.Entity',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '5000',
                '--req', f'name: "{model_name}" type: MODEL'
            ]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
            if result.returncode != 0:
                self.get_logger().warn(f"Failed to remove Gazebo entity '{model_name}'")
        except Exception as e:
            self.get_logger().warn(f"Error removing '{model_name}': {e}")

    def control_loop(self):
        msg = TrackedShipList()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        names = []
        for ship in self.ships.values():
            twist = ship.compute_cmd_vel(self.dt)
            ship.cmd_vel_pub.publish(twist)

            ts = TrackedShip()
            ts.target_id.uuid = list(bytes.fromhex(ship.target_id.replace('-', '')))
            ts.pose = ship.get_current_pose()
            ts.twist = ship.get_current_twist()
            ts.radius = 5.0
            msg.ships.append(ts)
            names.append(ship.model_name)

        self.tracked_pub.publish(msg)
        if names:
            self.names_pub.publish(String(data=json.dumps(names)))


def main(args=None):
    rclpy.init(args=args)
    node = DynamicShipManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for ship in node.ships.values():
            ship.cleanup()
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
