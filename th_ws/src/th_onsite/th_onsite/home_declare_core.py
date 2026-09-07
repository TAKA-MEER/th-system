"""
home_declare_core.py
====================
待機場所宣言の純ロジック（ROS2 非依存）

当日、機体を待機場所に置いて宣言 → SLAM 自己位置と待機場所ピン(kind=HOME)の
ずれを照合し、大きければ警告する。

ROS2 ノード (home_declarer.py) はこのモジュールを import して使用する。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple


@dataclass
class HomeDeclareParams:
    tolerance_m: float = 0.30
    tolerance_deg: float = 15.0


def pose_offset(robot_xy_yaw: Tuple[float, float, float],
                home_xy_yaw: Tuple[float, float, float]) -> Tuple[float, float]:
    """(offset_m, offset_deg)。offset_deg は (-180, 180] へ正規化した絶対値。

    robot_xy_yaw / home_xy_yaw はそれぞれ (x, y, yaw) [m, m, rad]。
    """
    rx, ry, ryaw = robot_xy_yaw
    hx, hy, hyaw = home_xy_yaw

    offset_m = math.hypot(rx - hx, ry - hy)

    d_deg = math.degrees(hyaw - ryaw)
    d_deg = (d_deg + 180.0) % 360.0 - 180.0  # (-180, 180] へ正規化
    offset_deg = abs(d_deg)

    return offset_m, offset_deg


def within_tolerance(offset_m: float, offset_deg: float,
                     params: HomeDeclareParams) -> bool:
    return offset_m <= params.tolerance_m and offset_deg <= params.tolerance_deg
