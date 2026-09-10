"""
venue_nav_core.py
==================
venue_navigator の純ロジック（ROS2 非依存）

-旋回誤差の正規化・旋回指令のクランプ・到着判定。
ROS2 ノード (venue_navigator.py) はこのモジュールを import して使用する。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple


@dataclass
class VenueNavParams:
    align_tolerance_rad: float = 0.09     # ~5deg。evt.align_done を出す閾値
    align_kp: float = 1.2                 # 旋回 P ゲイン
    align_w_max_rps: float = 0.6          # 旋回上限（指令生成側の上限）
    blocked_recheck_period_s: float = 2.0
    arrival_xy_tol_m: float = 0.30


def yaw_error(current_yaw: float, target_yaw: float) -> float:
    """current→target のヨー差 [rad]。(-pi, pi] へ正規化する。"""
    err = target_yaw - current_yaw
    err = (err + math.pi) % (2.0 * math.pi) - math.pi
    return err


def align_cmd_wz(current_yaw: float, target_yaw: float,
                 params: VenueNavParams) -> Tuple[float, bool]:
    """旋回指令 (angular_z, done)。|err| < tol なら (0.0, True)。"""
    err = yaw_error(current_yaw, target_yaw)
    if abs(err) < params.align_tolerance_rad:
        return 0.0, True
    wz = params.align_kp * err
    wz = max(-params.align_w_max_rps, min(params.align_w_max_rps, wz))
    return wz, False


def arrived(robot_x: float, robot_y: float, goal_x: float, goal_y: float,
            params: VenueNavParams) -> bool:
    """ロボット xy がゴール xy から arrival_xy_tol_m 以内か（euclid）。"""
    return math.hypot(robot_x - goal_x, robot_y - goal_y) < params.arrival_xy_tol_m


def should_unblock_for_arrival(robot_xy, goal, params: VenueNavParams) -> bool:
    """BLOCKED 中に到着圏内なら evt.unblocked で NAV に戻すべきか（2026-09-10）。

    盤前で膠着（footprint が膨張域に食い込んで RPP が毎回 ABORT）しても、
    xy が arrival_xy_tol_m 以内なら再探索を繰り返さず NAV に戻して
    _align_timer の到着フォールバックに完了させる。

    robot_xy: (x, y) or None。goal: {'x','y',...} or None。どちらか無ければ False。
    """
    if robot_xy is None or goal is None:
        return False
    return arrived(robot_xy[0], robot_xy[1],
                   float(goal['x']), float(goal['y']), params)


def find_home_goal(pins) -> dict | None:
    """kind == 'HOME' のピンからゴール dict {x, y, yaw} を探す。

    pins: 各要素が `kind` / `x` / `y` / `yaw` を持つ dict の列。
    HOME ピンが無ければ None（ノードは evt.blocked を出して待つ）。
    """
    if pins is None:
        return None
    for pin in pins:
        if getattr(pin, 'get', None) is not None and pin.get('kind') == 'HOME':
            return {'x': float(pin['x']), 'y': float(pin['y']),
                    'yaw': float(pin['yaw'])}
    return None
