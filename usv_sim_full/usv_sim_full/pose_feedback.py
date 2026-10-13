"""Gazebo 位姿反馈的纯函数与轨迹滤波工具。

供 ``dynamic_ship_manager_node`` 将开环积分真值替换为 Gazebo 物理
实体位姿，避免 TS 层 / 融合链位置与激光雷达扫描位置偏移。
"""

from __future__ import annotations

import math
from typing import Optional, Tuple

PoseSample = Tuple[float, float, float, float]  # x, y, yaw, stamp_sec


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """从四元数提取平面偏航角。

    :param x: 四元数 x 分量
    :param y: 四元数 y 分量
    :param z: 四元数 z 分量
    :param w: 四元数 w 分量
    :return: 平面偏航角，单位弧度
    """
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def is_feedback_fresh(
    feedback_stamp_sec: Optional[float],
    now_sec: float,
    timeout_sec: float = 1.0,
) -> bool:
    """判断位姿反馈是否在超时窗口内。

    :param feedback_stamp_sec: 反馈时间戳（秒），None 表示无反馈
    :param now_sec: 当前时间（秒）
    :param timeout_sec: 允许的最大延时（秒）
    :return: 反馈存在且未超时返回 True
    """
    if feedback_stamp_sec is None:
        return False
    return 0.0 <= (now_sec - feedback_stamp_sec) <= timeout_sec


def is_feedback_fresh_ns(stamp_ns: Optional[int], now_ns: int, timeout_sec: float) -> bool:
    """Check observation age without rounding an absolute timestamp to float.

    :param stamp_ns: Observation time in nanoseconds, or None.
    :param now_ns: Current simulation time in nanoseconds.
    :param timeout_sec: Maximum observation age in seconds.
    :return: Whether the observation is neither future-dated nor expired.
    """
    return stamp_ns is not None and 0 <= now_ns - stamp_ns <= int(timeout_sec * 1e9)


def fresh_sample(
    sample: Optional[PoseSample],
    now_sec: float,
    timeout_sec: float = 1.0,
) -> Optional[PoseSample]:
    """返回未超时的位姿样本，超时或缺失时返回 None。

    :param sample: (x, y, yaw, stamp_sec) 位姿样本，None 表示无反馈
    :param now_sec: 当前时间（秒）
    :param timeout_sec: 允许的最大延时（秒）
    :return: 新鲜样本本身，或 None
    """
    if sample is None:
        return None
    if not is_feedback_fresh(sample[3], now_sec, timeout_sec):
        return None
    return sample


class PoseTracker:
    """用连续位姿样本差分并一阶低通估计平面速度。"""

    def __init__(self, tau_sec: float = 0.3) -> None:
        self._tau_sec = max(0.0, float(tau_sec))
        self._prev: Optional[Tuple[float, float, int]] = None
        self._velocity: Tuple[float, float] = (0.0, 0.0)
        self._ready = False

    @property
    def velocity(self) -> Tuple[float, float]:
        """当前滤波后的平面速度 (vx, vy)，单位 m/s。"""
        return self._velocity

    @property
    def ready(self) -> bool:
        """Whether two advancing observations have established a velocity."""
        return self._ready

    @property
    def stamp_ns(self) -> Optional[int]:
        """Timestamp of the last accepted observation, in nanoseconds."""
        return None if self._prev is None else self._prev[2]

    def reset(self) -> None:
        """清空历史样本与速度。"""
        self._prev = None
        self._velocity = (0.0, 0.0)
        self._ready = False

    def update(self, x: float, y: float, stamp_sec: float) -> Tuple[float, float]:
        """输入新位姿样本，返回滤波后的平面速度。

        :param x: 实体 x 坐标（米）
        :param y: 实体 y 坐标（米）
        :param stamp_sec: 样本时间戳（秒）
        :return: 滤波后的速度 (vx, vy)，单位 m/s
        """
        if not math.isfinite(stamp_sec):
            return self._velocity
        return self.update_ns(x, y, round(stamp_sec * 1e9))

    def update_ns(self, x: float, y: float, stamp_ns: int,
                  timeout_sec: Optional[float] = None) -> Tuple[float, float]:
        """Update from exact observation time; ignore duplicates and reordering.

        :param x: World-frame X position in metres.
        :param y: World-frame Y position in metres.
        :param stamp_ns: Integer observation timestamp in nanoseconds.
        :param timeout_sec: Reset history across a longer gap, if supplied.
        :return: Filtered world-frame velocity in metres per second.
        """
        if not isinstance(stamp_ns, int) or not all(map(math.isfinite, (x, y))):
            return self._velocity
        if self._prev is not None:
            dt_ns = stamp_ns - self._prev[2]
            if dt_ns <= 0:
                return self._velocity
            if timeout_sec is not None and dt_ns > int(timeout_sec * 1e9):
                self.reset()
        if self._prev is not None:
            dt = (stamp_ns - self._prev[2]) * 1e-9
            raw_vx = (x - self._prev[0]) / dt
            raw_vy = (y - self._prev[1]) / dt
            if not all(map(math.isfinite, (raw_vx, raw_vy))):
                self.reset()
            else:
                alpha = dt / (self._tau_sec + dt)
                self._velocity = (
                    (1.0 - alpha) * self._velocity[0] + alpha * raw_vx,
                    (1.0 - alpha) * self._velocity[1] + alpha * raw_vy,
                )
                self._ready = True
        self._prev = (x, y, stamp_ns)
        return self._velocity
