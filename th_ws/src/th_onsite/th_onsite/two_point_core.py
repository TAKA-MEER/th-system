"""
two_point_core.py
==================
2 点指示の純ロジック（ROS2 非依存）

-方位計算・点間距離検証・対象妥当性判定・base_link→map 座標合成。
ROS2 ノード (pin_registrar.py) はこのモジュールを import して使用する。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple


@dataclass
class TwoPointParams:
    min_spacing_m: float = 0.30
    min_confidence: float = 0.50


def two_point_yaw(x1: float, y1: float, x2: float, y2: float) -> float:
    """P1→P2 方向の方位角 [rad] を返す（東=+x=0, 北=+y=π/2）。"""
    return math.atan2(y2 - y1, x2 - x1)


def spacing_ok(x1: float, y1: float, x2: float, y2: float,
               params: TwoPointParams) -> Tuple[bool, float]:
    """点間距離が min_spacing_m 以上なら (True, 距離)。"""
    d = math.hypot(x2 - x1, y2 - y1)
    return d >= params.min_spacing_m, d


def is_target_valid(is_lost: bool, confidence: float,
                    params: TwoPointParams) -> Tuple[bool, str]:
    """登録対象の妥当性判定（lost / 信頼度不足で拒否）。"""
    if is_lost:
        return False, "対象を見失っています"
    if confidence < params.min_confidence:
        return False, f"追跡信頼度が不足しています (confidence={confidence:.2f})"
    return True, "OK"


def compose_map_pose(robot_map_x: float, robot_map_y: float,
                     robot_map_yaw: float, person_bl_x: float,
                     person_bl_y: float) -> Tuple[float, float]:
    """base_link 相対の対象位置を map フレーム座標に変換する。

    並進 = robot_map に回転行列を適用した person_bl を加算。
    """
    cos_yaw = math.cos(robot_map_yaw)
    sin_yaw = math.sin(robot_map_yaw)
    map_x = robot_map_x + cos_yaw * person_bl_x - sin_yaw * person_bl_y
    map_y = robot_map_y + sin_yaw * person_bl_x + cos_yaw * person_bl_y
    return map_x, map_y
