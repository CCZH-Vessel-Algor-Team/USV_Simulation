#!/usr/bin/env python3
"""
等待 Gazebo 世界与仿真时钟就绪后，再调用 ros_gz_sim create 生成船体。
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
        self.declare_parameter('create_cli_timeout_sec', 60.0)

        self._world = str(self.get_parameter('world_name').value).strip()
        self._entity = str(self.get_parameter('entity_name').value).strip()
        self._sdf_file = str(self.get_parameter('sdf_file').value).strip()
        self._min_sim_t = max(0.0, float(self.get_parameter('min_sim_time_sec').value))
        self._min_clock_msgs = max(1, int(self.get_parameter('min_clock_messages').value))
        self._stagger = max(0.0, float(self.get_parameter('spawn_stagger_sec').value))
        self._timeout = max(5.0, float(self.get_parameter('wait_timeout_sec').value))
        self._poll_period = max(0.1, float(self.get_parameter('poll_period_sec').value))
        self._create_timeout = max(
            10.0, float(self.get_parameter('create_cli_timeout_sec').value)
        )

        if not self._entity or not self._sdf_file:
            raise RuntimeError('entity_name and sdf_file must be non-empty')

        self._clock_count = 0
        self._max_sim_t = 0.0
        self._world_svc_ok: Optional[bool] = None
        self._spawned = False
        self._failed = False
        self._t0 = time.monotonic()
        self._ready_at: Optional[float] = None

        self.create_subscription(Clock, '/clock', self._on_clock, 10)
        self._timer = self.create_timer(self._poll_period, self._poll)

        self.get_logger().info(
            f"等待仿真就绪后 spawn '{self._entity}' "
            f"(world={self._world or 'default'}, file={self._sdf_file}, "
            f"stagger={self._stagger:.1f}s, timeout={self._timeout:.0f}s)"
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
        try:
            out = subprocess.run(
                ['gz', 'service', '-l'],
                capture_output=True,
                text=True,
                timeout=3.0,
                check=False,
            )
            text = (out.stdout or '') + (out.stderr or '')
            if svc in text:
                self._world_svc_ok = True
                self.get_logger().info(f'Gazebo create 服务已就绪: {svc}')
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

    def _poll(self) -> None:
        if self._spawned or self._failed:
            return

        elapsed = time.monotonic() - self._t0
        if elapsed > self._timeout:
            self._failed = True
            self.get_logger().error(
                f'等待 Gazebo/仿真时钟超时 ({self._timeout:.0f}s): '
                f'clock_msgs={self._clock_count}, max_sim_t={self._max_sim_t:.3f}, '
                f'world_svc={self._world_svc_ok}'
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

        self._spawn_entity()

    def _spawn_entity(self) -> None:
        x = float(self.get_parameter('x').value)
        y = float(self.get_parameter('y').value)
        z = float(self.get_parameter('z').value)
        roll = float(self.get_parameter('roll').value)
        pitch = float(self.get_parameter('pitch').value)
        yaw = float(self.get_parameter('yaw').value)

        cmd = ['ros2', 'run', 'ros_gz_sim', 'create']
        if self._world:
            cmd.extend(['-world', self._world])
        cmd.extend([
            '-name', self._entity,
            '-file', self._sdf_file,
            '-x', str(x),
            '-y', str(y),
            '-z', str(z),
            '-R', str(roll),
            '-P', str(pitch),
            '-Y', str(yaw),
        ])

        self.get_logger().info(f"调用 ros_gz_sim create: {' '.join(cmd)}")
        try:
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=self._create_timeout,
                check=False,
            )
        except subprocess.TimeoutExpired:
            self._failed = True
            self.get_logger().error(
                f'ros_gz_sim create 超时 ({self._create_timeout:.0f}s): {self._entity}'
            )
            rclpy.shutdown()
            return

        detail = (result.stderr or result.stdout or '').strip()
        if result.returncode != 0:
            self._failed = True
            self.get_logger().error(
                f'spawn 失败 (code={result.returncode}): {detail}'
            )
            rclpy.shutdown()
            return

        self._spawned = True
        self.get_logger().info(f"船体 '{self._entity}' 已成功插入 Gazebo")
        if detail:
            self.get_logger().info(detail)
        self._timer.cancel()
        rclpy.shutdown()


def main(args=None) -> None:
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
