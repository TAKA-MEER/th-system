"""
summon_navigator_core.py
=========================
summon_navigator のコアロジック（ROS2 非依存）

テスト可能性のため、ROS2 インポートを一切含まない純粋な Python モジュール。
ROS2 ノード側 (summon_navigator.py) はこのモジュールを import して使用する。

呼び寄せ: base_link 相対の試験員位置 (/person/status) から、手前
stop_distance_m の位置で停止する base_link 相対ゴール (x, y, yaw) を
一度だけ計算する単発方式。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class SummonParams:
    stop_distance_m: float = 1.0   # m 試験員手前でこの距離を残して停止
    min_confidence:  float = 0.5   # 0.0-1.0 これ未満の confidence では起動を拒否


def is_target_valid(is_lost: bool, confidence: float,
                     params: SummonParams) -> Tuple[bool, str]:
    """呼び寄せの起動可否を判定する（誤切替防止）。"""
    if is_lost:
        return False, "対象を見失っています"
    if confidence < params.min_confidence:
        return False, f"追跡信頼度が不足しています (confidence={confidence:.2f})"
    return True, "OK"


def compute_stop_goal(person_x: float, person_y: float,
                       params: SummonParams) -> Optional[Tuple[float, float, float]]:
    """base_link 相対の試験員位置から base_link 相対ゴール (x, y, yaw) を計算する。

    ロボット→試験員の直線上、試験員の stop_distance_m 手前で停止する。
    既に stop_distance_m 以内にいる場合はゴール不要として None を返す。
    """
    d = math.hypot(person_x, person_y)
    if d <= params.stop_distance_m:
        return None
    scale = (d - params.stop_distance_m) / d
    goal_x = person_x * scale
    goal_y = person_y * scale
    yaw = math.atan2(person_y, person_x)
    return goal_x, goal_y, yaw
