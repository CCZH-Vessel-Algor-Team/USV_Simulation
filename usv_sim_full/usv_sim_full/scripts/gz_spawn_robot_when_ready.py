#!/usr/bin/env python3
"""
等待 Gazebo 世界与仿真时钟就绪后，再生成船体。

ros_gz_sim create 对 /world/.../create 的请求默认约 5s 超时；CCS 等重载世界
加载常超过该时间，导致 spawn 失败却被误判为成功。此处改用 blocking create
服务并校验模型是否出现在世界中。
"""

from __future__ import annotations

import subprocess
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock


def _sim_time_sec(msg: Clock) -> float:
    return float(msg.clock.sec) + float(msg.clock.nanosec) * 1e-9


class GzSpawnRobotWhenReady(Node):
    """Poll /clock + gz create service, then spawn one robot URDF/SDF entity."""

    def __init__(self) -> None:
        super().__init__('gz_spawn_robot_when_ready')
        self.declare_parameter('world_name', '')
        self.declare_parameter('entity_name', '')
        self.declare_parameter('sdf_file', '')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.5)
        self.declare_parameter('roll', 0.0)
        self.declare_parameter('pitch', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('min_sim_time_sec', 0.01)
        self.declare_parameter('min_clock_messages', 2)
        self.declare_parameter('spawn_stagger_sec', 0.0)
        self.declare_parameter('wait_timeout_sec', 120.0)
        self.declare_parameter('poll_period_sec', 0.5)
        self.declare_parameter('create_service_timeout_ms', 120000)
        self.declare_parameter('max_spawn_attempts', 8)
        self.declare_parameter('spawn_retry_delay_sec', 3.0)

        self._world = str(self.get_parameter('world_name').value).strip()
        self._entity = str(self.get_parameter('entity_name').value).strip()
        self._sdf_file = str(self.get_parameter('sdf_file').value).strip()
        self._min_sim_t = max(0.0, float(self.get_parameter('min_sim_time_sec').value))
        self._min_clock_msgs = max(1, int(self.get_parameter('min_clock_messages').value))
        self._stagger = max(0.0, float(self.get_parameter('spawn_stagger_sec').value))
        self._timeout = max(5.0, float(self.get_parameter('wait_timeout_sec').value))
        self._poll_period = max(0.1, float(self.get_parameter('poll_period_sec').value))
        self._create_timeout_ms = max(
            5000, int(self.get_parameter('create_service_timeout_ms').value)
        )
        self._max_attempts = max(1, int(self.get_parameter('max_spawn_attempts').value))
        self._retry_delay = max(0.5, float(self.get_parameter('spawn_retry_delay_sec').value))

        if not self._entity or not self._sdf_file:
            raise RuntimeError('entity_name and sdf_file must be non-empty')

        self._clock_count = 0
        self._max_sim_t = 0.0
        self._world_svc_ok: Optional[bool] = None
        self._spawned = False
        self._failed = False
        self._spawn_attempts = 0
        self._t0 = time.monotonic()
        self._ready_at: Optional[float] = None
        self._next_spawn_at: Optional[float] = None

        self.create_subscription(Clock, '/clock', self._on_clock, 10)
        self._timer = self.create_timer(self._poll_period, self._poll)

        self.get_logger().info(
            f"等待仿真就绪后 spawn '{self._entity}' "
            f"(world={self._world or 'default'}, file={self._sdf_file}, "
            f"stagger={self._stagger:.1f}s, timeout={self._timeout:.0f}s, "
            f'create_timeout={self._create_timeout_ms}ms, '
            f'max_attempts={self._max_attempts})'
        )

    def _on_clock(self, msg: Clock) -> None:
        self._clock_count += 1
        self._max_sim_t = max(self._max_sim_t, _sim_time_sec(msg))

    def _world_create_service_ready(self) -> bool:
        if not self._world:
            return True
        if self._world_svc_ok is True:
            return True
        svc = f'/world/{self._world}/create'
        blocking_svc = f'{svc}/blocking'
        try:
            out = subprocess.run(
                ['gz', 'service', '-l'],
                capture_output=True,
                text=True,
                timeout=3.0,
                check=False,
            )
            text = (out.stdout or '') + (out.stderr or '')
            if svc in text or blocking_svc in text:
                self._world_svc_ok = True
                self.get_logger().info(
                    f'Gazebo create 服务已就绪: {blocking_svc if blocking_svc in text else svc}'
                )
                return True
            self._world_svc_ok = False
        except Exception as exc:
            self.get_logger().debug(f'gz service -l 检查失败: {exc}')
            self._world_svc_ok = False
        return False

    def _sim_clock_ready(self) -> bool:
        if self._clock_count < self._min_clock_msgs:
            return False
        return self._max_sim_t >= self._min_sim_t

    def _model_exists_in_world(self) -> bool:
        if not self._world:
            return True
        try:
            out = subprocess.run(
                ['gz', 'model', '--list'],
                capture_output=True,
                text=True,
                timeout=10.0,
                check=False,
            )
            text = (out.stdout or '') + (out.stderr or '')
            for line in text.splitlines():
                stripped = line.strip()
                if stripped.startswith('- '):
                    model_name = stripped[2:].strip()
                    if model_name == self._entity:
                        return True
        except Exception as exc:
            self.get_logger().debug(f'gz model --list 检查失败: {exc}')
        return False

    def _poll(self) -> None:
        if self._spawned or self._failed:
            return

        elapsed = time.monotonic() - self._t0
        if elapsed > self._timeout:
            self._failed = True
            self.get_logger().error(
                f'等待 Gazebo/仿真时钟超时 ({self._timeout:.0f}s): '
                f'clock_msgs={self._clock_count}, max_sim_t={self._max_sim_t:.3f}, '
                f'world_svc={self._world_svc_ok}, spawn_attempts={self._spawn_attempts}'
            )
            rclpy.shutdown()
            return

        if not self._world_create_service_ready():
            return
        if not self._sim_clock_ready():
            return

        if self._ready_at is None:
            self._ready_at = time.monotonic()
            if self._stagger > 0:
                self.get_logger().info(
                    f'仿真已运行 (sim_t>={self._min_sim_t:.3f}, '
                    f'clock_msgs={self._clock_count})，'
                    f'等待错开 {self._stagger:.1f}s 后 spawn'
                )
            else:
                self.get_logger().info(
                    f'仿真已运行 (sim_t>={self._min_sim_t:.3f}, '
                    f'clock_msgs={self._clock_count})，开始 spawn'
                )

        if self._stagger > 0 and (time.monotonic() - self._ready_at) < self._stagger:
            return

        if self._next_spawn_at is not None and time.monotonic() < self._next_spawn_at:
            return

        if self._model_exists_in_world():
            self._spawned = True
            self.get_logger().info(
                f"船体 '{self._entity}' 已存在于 Gazebo 世界 '{self._world}'"
            )
            self._timer.cancel()
            rclpy.shutdown()
            return

        if self._spawn_attempts >= self._max_attempts:
            self._failed = True
            self.get_logger().error(
                f"spawn 失败：已尝试 {self._spawn_attempts} 次，"
                f"模型 '{self._entity}' 仍未出现在世界中"
            )
            rclpy.shutdown()
            return

        self._spawn_entity()

    def _spawn_entity(self) -> None:
        self._spawn_attempts += 1
        x = float(self.get_parameter('x').value)
        y = float(self.get_parameter('y').value)
        z = float(self.get_parameter('z').value)
        roll = float(self.get_parameter('roll').value)
        pitch = float(self.get_parameter('pitch').value)
        yaw = float(self.get_parameter('yaw').value)

        if not self._world:
            self._failed = True
            self.get_logger().error('world_name 为空，无法调用 Gazebo create 服务')
            rclpy.shutdown()
            return

        service = f'/world/{self._world}/create/blocking'
        req = (
            f'sdf_filename: "{self._sdf_file}"\n'
            f'name: "{self._entity}"\n'
            'pose {\n'
            f'  position {{ x: {x} y: {y} z: {z} }}\n'
            f'  orientation {{ x: 0.0 y: 0.0 z: 0.0 w: 1.0 }}\n'
            '}\n'
            'allow_renaming: false\n'
        )
        if abs(roll) > 1e-9 or abs(pitch) > 1e-9 or abs(yaw) > 1e-9:
            import math

            cr = math.cos(roll * 0.5)
            sr = math.sin(roll * 0.5)
            cp = math.cos(pitch * 0.5)
            sp = math.sin(pitch * 0.5)
            cy = math.cos(yaw * 0.5)
            sy = math.sin(yaw * 0.5)
            qw = cr * cp * cy + sr * sp * sy
            qx = sr * cp * cy - cr * sp * sy
            qy = cr * sp * cy + sr * cp * sy
            qz = cr * cp * sy - sr * sp * cy
            req = (
                f'sdf_filename: "{self._sdf_file}"\n'
                f'name: "{self._entity}"\n'
                'pose {\n'
                f'  position {{ x: {x} y: {y} z: {z} }}\n'
                f'  orientation {{ x: {qx} y: {qy} z: {qz} w: {qw} }}\n'
                '}\n'
                'allow_renaming: false\n'
            )

        cmd = [
            'gz', 'service',
            '-s', service,
            '--reqtype', 'gz.msgs.EntityFactory',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', str(self._create_timeout_ms),
            '--req', req,
        ]

        self.get_logger().info(
            f"第 {self._spawn_attempts}/{self._max_attempts} 次 spawn "
            f"'{self._entity}' via {service} (timeout={self._create_timeout_ms}ms)"
        )
        try:
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=(self._create_timeout_ms / 1000.0) + 10.0,
                check=False,
            )
        except subprocess.TimeoutExpired:
            detail = f'Gazebo create 服务调用超时 ({self._create_timeout_ms}ms)'
            self.get_logger().warn(detail)
            self._schedule_retry(detail)
            return

        detail = (result.stderr or result.stdout or '').strip()
        if result.returncode != 0 or 'data: true' not in detail.lower():
            self.get_logger().warn(
                f'spawn 尝试失败 (code={result.returncode}): {detail or "(无输出)"}'
            )
            self._schedule_retry(detail)
            return

        if not self._model_exists_in_world():
            self.get_logger().warn(
                f"create 服务返回成功，但模型 '{self._entity}' 尚未出现在世界中"
            )
            self._schedule_retry(detail)
            return

        self._spawned = True
        self.get_logger().info(
            f"船体 '{self._entity}' 已成功插入 Gazebo 世界 '{self._world}'"
        )
        if detail:
            self.get_logger().info(detail)
        self._timer.cancel()
        rclpy.shutdown()

    def _schedule_retry(self, detail: str) -> None:
        if self._spawn_attempts >= self._max_attempts:
            self._failed = True
            self.get_logger().error(
                f"spawn 最终失败 ({self._spawn_attempts} 次): {detail}"
            )
            rclpy.shutdown()
            return
        self._next_spawn_at = time.monotonic() + self._retry_delay
        self.get_logger().info(
            f'{self._retry_delay:.1f}s 后重试 spawn ({self._spawn_attempts}/'
            f'{self._max_attempts})'
        )


def main(args=None):
    rclpy.init(args=args)
    node = GzSpawnRobotWhenReady()
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
