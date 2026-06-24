#!/usr/bin/env python3

import math
import os
import subprocess
import tempfile
import time
import yaml

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


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


class DynamicObstacle:
    def __init__(self, config, node):
        self.requested_name = str(config.get('name', 'obstacle'))
        # Use a stable, searchable runtime name in Gazebo Entity Tree.
        self.name = self.requested_name if self.requested_name.startswith('dyn_') else f'dyn_{self.requested_name}'
        self.shape = config.get('shape', 'cylinder')
        self.color = config.get('color', 'Red')
        self.speed = config.get('speed', 1.0)
        self.loop = config.get('loop', True)
        self.waypoints = config.get('waypoints', [])
        # 可选尺寸：圆柱 [radius, length]；长方体 [sx, sy, sz]（与 obstacles.fixed_list.size 用法一致）
        self.size = config.get('size')
        self.mesh_profile = config.get('mesh_profile', config.get('profile_path', ''))
        self.spawn_z = config.get('spawn_z')

        self.current_wp_idx = 1 if len(self.waypoints) > 1 else 0
        self.direction = 1 # 1 for forward, -1 for backward
        if len(self.waypoints) > 0:
            self.x = float(self.waypoints[0][0])
            self.y = float(self.waypoints[0][1])
        else:
            self.x = 0.0
            self.y = 0.0
        if config.get('spawn_heading_deg') is not None:
            self.spawn_yaw = math.radians(float(config['spawn_heading_deg']))
        elif len(self.waypoints) > 1:
            dx = float(self.waypoints[1][0]) - float(self.waypoints[0][0])
            dy = float(self.waypoints[1][1]) - float(self.waypoints[0][1])
            self.spawn_yaw = math.atan2(dy, dx) if abs(dx) + abs(dy) > 1e-9 else 0.0
        else:
            self.spawn_yaw = 0.0
            
        self.active = len(self.waypoints) > 1
        
        # ROS 2 Publisher
        self.cmd_vel_pub = node.create_publisher(Twist, f'/model/{self.name}/cmd_vel', 10)
        
        # Start bridge for this obstacle's cmd_vel
        # ros_gz_bridge parameter_bridge /model/{name}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist
        cmd = [
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            f'/model/{self.name}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'
        ]
        self.bridge_process = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def cleanup(self):
        if hasattr(self, 'bridge_process') and self.bridge_process:
            self.bridge_process.terminate()

    def world_twist_to_body(self, vx_world: float, vy_world: float) -> Twist:
        """Gazebo VelocityControl 使用船体系 cmd_vel；航迹速度在 map 系给定。"""
        c = math.cos(self.spawn_yaw)
        s = math.sin(self.spawn_yaw)
        twist = Twist()
        twist.linear.x = c * vx_world + s * vy_world
        twist.linear.y = -s * vx_world + c * vy_world
        twist.angular.z = 0.0
        return twist

class ScenarioManager(Node):
    def __init__(self):
        super().__init__('scenario_manager')
        # ros_gz_sim create 往往先返回、Gazebo 再异步读 -file；立即删临时 SDF 会导致两艘船都插不进。
        self._retained_spawn_sdf_paths: list[str] = []

        self.declare_parameter('config_path', '')
        config_path = self.get_parameter('config_path').get_parameter_value().string_value
        
        if not config_path or not os.path.exists(config_path):
            self.get_logger().error(f"Config file not found: {config_path}")
            return
            
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
            
        self.world_name = self.config.get('environment', {}).get('world_name', 'sydney_regatta')
        scenario_cfg = self.config.get('scenario', {})
        self.obs_configs = scenario_cfg.get('dynamic_obstacles', [])
        
        self.obstacles = []
        self._spawn_delay_timers: list = []

        # Humble 下 rclpy 对 /world/.../SpawnEntity 常无法与 gz 侧服务端匹配；与 ground_truth_gazebo_models
        # 一致，直接使用 ros_gz_sim create（gz-transport），避免 10s 空等与误导性 WARN。
        self.get_logger().info(
            f"动态障碍：使用 ros_gz_sim create 插入世界 '{self.world_name}'。"
        )

        self.spawn_obstacles()
        
        # Timer for kinematic control (10 Hz)
        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.control_loop)

    def get_color_rgba(self, color_name):
        colors = {
            'Red': '1 0 0 1',
            'Green': '0 1 0 1',
            'Blue': '0 0 1 1',
            'Yellow': '1 1 0 1',
            'Black': '0 0 0 1',
            'White': '1 1 1 1'
        }
        return colors.get(color_name, '1 1 1 1')

    def _resolve_profile_path(self, path_text):
        p = str(path_text or '').strip()
        if not p:
            return ''
        if os.path.isabs(p):
            return p
        base = os.path.dirname(os.path.abspath(
            self.get_parameter('config_path').get_parameter_value().string_value
        ))
        return os.path.normpath(os.path.join(base, p))

    def _load_mesh_profile(self, obs_cfg):
        profile_path = self._resolve_profile_path(obs_cfg.mesh_profile)
        if not profile_path or not os.path.isfile(profile_path):
            raise FileNotFoundError(
                f"mesh_profile not found for {obs_cfg.name}: {obs_cfg.mesh_profile!r}"
            )
        with open(profile_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
        if not isinstance(data, dict):
            raise RuntimeError(f"Invalid mesh_profile YAML: {profile_path}")
        mesh = data.get('mesh') or {}
        if not str(mesh.get('uri', '')).strip():
            raise RuntimeError(f"mesh_profile missing mesh.uri: {profile_path}")
        boxes = data.get('boxes') or []
        if not isinstance(boxes, list) or not boxes:
            raise RuntimeError(f"mesh_profile missing boxes[]: {profile_path}")
        return profile_path, data

    def _mesh_profile_min_z(self, profile_data):
        group = profile_data.get('collision_group') or {}
        group_xyz = _as_vec((group.get('pose_offset') or {}).get('xyz'), 3, 0.0)
        min_z = None
        for box in profile_data.get('boxes') or []:
            size = _as_vec(box.get('size_lwh_m'), 3, 0.0)
            pose = _as_vec(box.get('pose_xyz_m'), 3, 0.0)
            bottom = group_xyz[2] + pose[2] - 0.5 * size[2]
            min_z = bottom if min_z is None else min(min_z, bottom)
        return 0.0 if min_z is None else float(min_z)

    def _mesh_profile_spawn_z(self, obs_cfg, profile_data):
        if obs_cfg.spawn_z is not None:
            return float(obs_cfg.spawn_z)
        prof_spawn_z = profile_data.get('spawn_z')
        if prof_spawn_z is not None:
            return float(prof_spawn_z)
        min_z = self._mesh_profile_min_z(profile_data)
        return max(0.0, -min_z)

    def _generate_mesh_profile_sdf(self, obs_cfg):
        profile_path, profile_data = self._load_mesh_profile(obs_cfg)
        mesh = profile_data.get('mesh') or {}
        mesh_uri = str(mesh.get('uri', '')).strip()
        mesh_scale = _as_vec(mesh.get('scale'), 3, 1.0)
        mesh_rgba = _as_vec(mesh.get('material_rgba'), 4, 1.0)
        mesh_pose = mesh.get('pose_offset') or {}
        mesh_xyz = _as_vec(mesh_pose.get('xyz'), 3, 0.0)
        mesh_rpy = _as_vec(mesh_pose.get('rpy'), 3, 0.0)

        collision_group = profile_data.get('collision_group') or {}
        group_pose = collision_group.get('pose_offset') or {}
        group_xyz = _as_vec(group_pose.get('xyz'), 3, 0.0)
        group_rpy = _as_vec(group_pose.get('rpy'), 3, 0.0)

        collision_xml = []
        for idx, box in enumerate(profile_data.get('boxes') or []):
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

        sdf = f"""<?xml version="1.0" ?>
        <sdf version="1.6">
            <model name="{obs_cfg.name}">
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
                            <ambient>{mesh_rgba[0]:.6f} {mesh_rgba[1]:.6f} {mesh_rgba[2]:.6f} {mesh_rgba[3]:.6f}</ambient>
                            <diffuse>{mesh_rgba[0]:.6f} {mesh_rgba[1]:.6f} {mesh_rgba[2]:.6f} {mesh_rgba[3]:.6f}</diffuse>
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
                    <topic>/model/{obs_cfg.name}/cmd_vel</topic>
                </plugin>
            </model>
        </sdf>
        """
        return sdf, profile_path, self._mesh_profile_spawn_z(obs_cfg, profile_data)

    def generate_sdf(self, obs_cfg):
        if obs_cfg.shape == 'mesh_profile':
            sdf, _, _ = self._generate_mesh_profile_sdf(obs_cfg)
            return sdf

        name = obs_cfg.name
        shape = obs_cfg.shape
        color = self.get_color_rgba(obs_cfg.color)
        sz = getattr(obs_cfg, "size", None)

        if shape == 'box':
            if sz is not None and len(sz) >= 3:
                sx, sy, szf = float(sz[0]), float(sz[1]), float(sz[2])
                geom = "<box><size>%.6f %.6f %.6f</size></box>" % (sx, sy, szf)
            else:
                geom = "<box><size>1.2 1.2 1.2</size></box>"
        else:
            if sz is not None and len(sz) >= 2:
                r, h = float(sz[0]), float(sz[1])
                geom = "<cylinder><radius>%.6f</radius><length>%.6f</length></cylinder>" % (r, h)
            else:
                geom = "<cylinder><radius>0.8</radius><length>1.2</length></cylinder>"
            
        sdf = f"""<?xml version="1.0" ?>
        <sdf version="1.6">
            <model name="{name}">
                <static>false</static>
                <link name="base_link">
                    <gravity>false</gravity>
                    <visual name="visual">
                        <geometry>{geom}</geometry>
                        <material>
                            <ambient>{color}</ambient>
                            <diffuse>{color}</diffuse>
                            <emissive>{color}</emissive>
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

    def spawn_obstacles(self):
        for i, cfg in enumerate(self.obs_configs):
            delay = float(cfg.get('spawn_delay_sec', 0.0))
            if delay > 0.0:
                self.get_logger().info(
                    f"动态障碍 '{cfg.get('name', 'obstacle')}' 将在 {delay:.1f}s 后生成。"
                )

                spawn_state = {'done': False}
                timer_holder: list = []

                def _delayed_spawn(ob_cfg=cfg, obs_index=i):
                    if spawn_state['done']:
                        return
                    spawn_state['done'] = True
                    if timer_holder:
                        timer_holder[0].cancel()
                    self._spawn_one_obstacle(ob_cfg, obs_index)

                timer_holder.append(self.create_timer(delay, _delayed_spawn))
                self._spawn_delay_timers.append(timer_holder[0])
            else:
                self._spawn_one_obstacle(cfg, i)

    def _spawn_one_obstacle(self, cfg, index: int):
        obs = DynamicObstacle(cfg, self)
        self.obstacles.append(obs)

        obstacle_sdf = self.generate_sdf(obs)
        self.get_logger().info(
            f"Preparing dynamic obstacle: requested='{obs.requested_name}', runtime='{obs.name}', "
            f"waypoints={obs.waypoints}, speed={obs.speed}"
        )

        self._spawn_with_create_cli(obs, obstacle_sdf)
        if index + 1 < len(self.obs_configs):
            time.sleep(0.3)

    def _spawn_with_create_cli(self, obs, obstacle_sdf):
        """Spawn an obstacle with ros_gz_sim create as a fallback path."""
        x = float(obs.waypoints[0][0]) if len(obs.waypoints) > 0 else 0.0
        y = float(obs.waypoints[0][1]) if len(obs.waypoints) > 0 else 0.0
        z = 0.5
        profile_path = ''
        if obs.shape == 'mesh_profile':
            obstacle_sdf, profile_path, z = self._generate_mesh_profile_sdf(obs)

        tmp_sdf_path = None
        try:
            with tempfile.NamedTemporaryFile(mode='w', suffix='.sdf', delete=False) as tmpf:
                tmpf.write(obstacle_sdf)
                tmp_sdf_path = tmpf.name

            cmd = [
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-world', self.world_name,
                '-file', tmp_sdf_path,
                '-name', obs.name,
                '-x', str(x),
                '-y', str(y),
                '-z', str(z),
                '-Y', str(obs.spawn_yaw),
            ]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
            if result.returncode == 0:
                self.get_logger().info(
                    f"动态障碍已通过 CLI 插入：'{obs.name}'（ros_gz_sim create）。"
                )
                if profile_path:
                    self.get_logger().info(
                        f"动态障碍 {obs.name} 使用 mesh_profile: {profile_path}"
                    )
            else:
                detail = (result.stderr or result.stdout or '').strip()
                self.get_logger().error(
                    f"CLI 生成 {obs.name} 失败（code={result.returncode}）：{detail}"
                )
        except Exception as exc:
            self.get_logger().error(f"Exception while spawning {obs.name} via CLI: {exc}")
        finally:
            if tmp_sdf_path and os.path.exists(tmp_sdf_path):
                self._retained_spawn_sdf_paths.append(tmp_sdf_path)

    def control_loop(self):
        for obs in self.obstacles:
            if not obs.active:
                continue
                
            target_wp = obs.waypoints[obs.current_wp_idx]
            tx, ty = float(target_wp[0]), float(target_wp[1])
            
            dx = tx - obs.x
            dy = ty - obs.y
            dist = math.hypot(dx, dy)
            
            # Check waypoint arrival
            if dist < 0.5:
                if len(obs.waypoints) > 1:
                    if obs.loop:
                        # 往复巡逻逻辑 (Ping-pong)
                        obs.current_wp_idx += obs.direction
                        if obs.current_wp_idx >= len(obs.waypoints):
                            obs.direction = -1
                            obs.current_wp_idx = len(obs.waypoints) - 2
                            if obs.current_wp_idx < 0:
                                obs.current_wp_idx = 0
                        elif obs.current_wp_idx < 0:
                            obs.direction = 1
                            obs.current_wp_idx = 1
                            if obs.current_wp_idx >= len(obs.waypoints):
                                obs.current_wp_idx = 0
                    else:
                        obs.current_wp_idx += 1
                        if obs.current_wp_idx >= len(obs.waypoints):
                            obs.active = False
                            # Stop the obstacle
                            twist = Twist()
                            obs.cmd_vel_pub.publish(twist)
                            continue
                target_wp = obs.waypoints[obs.current_wp_idx]
                tx, ty = float(target_wp[0]), float(target_wp[1])
                dx = tx - obs.x
                dy = ty - obs.y
                dist = math.hypot(dx, dy)
            
            # Kinematic estimation
            if dist > 0:
                vx = (dx / dist) * obs.speed
                vy = (dy / dist) * obs.speed
            else:
                vx = 0.0
                vy = 0.0
                
            obs.x += vx * self.dt
            obs.y += vy * self.dt
            
            obs.cmd_vel_pub.publish(obs.world_twist_to_body(vx, vy))

def main(args=None):
    rclpy.init(args=args)
    node = ScenarioManager()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for obs in node.obstacles:
            obs.cleanup()
        for p in getattr(node, '_retained_spawn_sdf_paths', []):
            if p and os.path.isfile(p):
                try:
                    os.remove(p)
                except OSError:
                    pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
