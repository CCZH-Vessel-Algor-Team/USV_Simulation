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
    return (now_sec - feedback_stamp_sec) <= timeout_sec


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
        self._prev: Optional[Tuple[float, float, float]] = None
        self._velocity: Tuple[float, float] = (0.0, 0.0)

    @property
    def velocity(self) -> Tuple[float, float]:
        """当前滤波后的平面速度 (vx, vy)，单位 m/s。"""
        return self._velocity

    def reset(self) -> None:
        """清空历史样本与速度。"""
        self._prev = None
        self._velocity = (0.0, 0.0)

    def update(self, x: float, y: float, stamp_sec: float) -> Tuple[float, float]:
        """输入新位姿样本，返回滤波后的平面速度。

        :param x: 实体 x 坐标（米）
        :param y: 实体 y 坐标（米）
        :param stamp_sec: 样本时间戳（秒）
        :return: 滤波后的速度 (vx, vy)，单位 m/s
        """
        if self._prev is not None:
            dt = stamp_sec - self._prev[2]
            if dt > 1e-6:
                raw_vx = (x - self._prev[0]) / dt
                raw_vy = (y - self._prev[1]) / dt
                if self._tau_sec <= 0.0:
                    self._velocity = (raw_vx, raw_vy)
                else:
                    alpha = dt / (self._tau_sec + dt)
                    self._velocity = (
                        self._velocity[0] + alpha * (raw_vx - self._velocity[0]),
                        self._velocity[1] + alpha * (raw_vy - self._velocity[1]),
                    )
        self._prev = (x, y, stamp_sec)
        return self._velocity
