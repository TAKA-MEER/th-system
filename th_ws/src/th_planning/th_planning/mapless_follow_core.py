"""
mapless_follow_core.py
=======================
MAP(SLAM占有格子・Nav2)を必要としない、純粋な軌跡追従モードのコアロジック
（ROS2 非依存）。

- 試験員の軌跡(trail)を遡った点へ Pure Pursuit で直接 (v, ω) を計算する
  （Nav2 は使わない。座標変換・軌跡ロジックは follow_planner_core.py の
  既存関数をそのまま再利用する）。
- 進路上の障害物判定は costmap ではなく生の LiDAR スキャン(/scan_filtered
  相当の ranges 配列)を直接走査して行う。
- 距離帯は TRACKING(追従中) / STOPPED(停止中) の2状態のみ。試験員が
  近づいても退避せずその場停止し、離れたら追従を再開する。
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from enum import Enum, auto
from typing import Deque, List, Optional, Tuple

from .follow_planner_core import (
    Point2D, Pose2D,
    to_absolute, to_relative,
    update_trail, get_trail_goal,
    pure_pursuit_control,
)


class MaplessState(Enum):
    TRACKING = auto()   # 追従中
    STOPPED  = auto()   # 停止中（試験員接近 / 障害物 / 姿勢・スキャン未確立）


@dataclass
class MaplessFollowParams:
    # 軌跡追従
    lookback_distance:       float = 1.0    # m 軌跡追従時に遡る距離
    trail_sample_interval_m: float = 0.05   # m 軌跡点を追加する最小移動距離
    trail_max_points:        int   = 2000   # 軌跡の最大保持点数

    # 試験員接近時の停止/再開（ヒステリシス）
    stop_distance:           float = 1.0    # m これ未満で停止
    resume_distance:         float = 1.3    # m これ以上で追従再開

    # 進路上障害物チェック（生 LiDAR スキャン）
    obstacle_check_distance_m:     float = 1.0   # m この距離未満に障害物があれば停止
    obstacle_check_half_width_deg: float = 20.0  # deg 進行方位を中心とした走査半幅

    # Pure Pursuit 駆動
    v_max:        float = 0.3   # m/s
    k_ang:        float = 2.0   # 角度ゲイン
    stop_radius_m: float = 0.2  # m ゴール到達とみなす半径

    update_rate_hz: float = 10.0


@dataclass
class MaplessOutput:
    linear_x:  float = 0.0
    angular_z: float = 0.0
    state:     MaplessState = MaplessState.TRACKING
    # "tracking" / "person_close" / "obstacle_ahead" / "no_pose" / "no_scan"
    reason:    str = "tracking"


def next_mapless_state(
    current_state: MaplessState,
    distance: float,
    stop_distance: float,
    resume_distance: float,
) -> MaplessState:
    """試験員との距離に基づき TRACKING/STOPPED をヒステリシス付きで切り替える"""
    if current_state == MaplessState.TRACKING:
        if distance < stop_distance:
            return MaplessState.STOPPED
        return MaplessState.TRACKING

    # STOPPED
    if distance >= resume_distance:
        return MaplessState.TRACKING
    return MaplessState.STOPPED


def is_path_blocked(
    ranges: List[float],
    angle_min: float,
    angle_increment: float,
    direction_rad: float,
    half_width_rad: float,
    max_check_dist: float,
) -> bool:
    """
    direction_rad ± half_width_rad の範囲にある LiDAR レンジ値のうち、
    max_check_dist 未満のものが1つでもあれば True(障害物あり)を返す。
    inf/nan は障害物なしとして扱う。
    """
    for i, r in enumerate(ranges):
        if not math.isfinite(r):
            continue
        angle = angle_min + i * angle_increment
        diff = math.atan2(math.sin(angle - direction_rad), math.cos(angle - direction_rad))
        if abs(diff) <= half_width_rad and r < max_check_dist:
            return True
    return False


class MaplessFollowCore:
    """
    follow_planner_mapless の中核ロジック（ROS2 非依存）。
    ROS2 ノードから update_rate_hz で update() を呼び出す。
    """

    def __init__(self, params: MaplessFollowParams = None):
        self.params = params or MaplessFollowParams()
        self.state  = MaplessState.TRACKING
        self.trail: Deque[Point2D] = deque(maxlen=self.params.trail_max_points)

    def update(
        self,
        person_x: float,
        person_y: float,
        robot_pose_odom: Optional[Pose2D],
        scan_ranges: Optional[List[float]],
        scan_angle_min: float = 0.0,
        scan_angle_increment: float = 0.0,
    ) -> MaplessOutput:
        distance = math.hypot(person_x, person_y)
        self.state = next_mapless_state(
            self.state, distance,
            self.params.stop_distance, self.params.resume_distance)

        if robot_pose_odom is None:
            return MaplessOutput(state=self.state, reason="no_pose")

        person_abs = to_absolute(robot_pose_odom, (person_x, person_y))
        update_trail(self.trail, person_abs, self.params.trail_sample_interval_m)

        if self.state == MaplessState.STOPPED:
            return MaplessOutput(state=self.state, reason="person_close")

        if scan_ranges is None:
            return MaplessOutput(state=self.state, reason="no_scan")

        goal_abs = get_trail_goal(self.trail, self.params.lookback_distance,
                                   robot_pos=(robot_pose_odom[0], robot_pose_odom[1]))
        rel_goal = to_relative(robot_pose_odom, goal_abs)
        bearing = math.atan2(rel_goal[1], rel_goal[0])

        if is_path_blocked(
                scan_ranges, scan_angle_min, scan_angle_increment,
                bearing, math.radians(self.params.obstacle_check_half_width_deg),
                self.params.obstacle_check_distance_m):
            return MaplessOutput(state=self.state, reason="obstacle_ahead")

        v, w = pure_pursuit_control(
            (0.0, 0.0, 0.0), rel_goal,
            v_max=self.params.v_max, k_ang=self.params.k_ang,
            stop_radius=self.params.stop_radius_m)
        return MaplessOutput(linear_x=v, angular_z=w, state=self.state, reason="tracking")

    def clear_trail(self) -> None:
        """試験員ロスト→再検出時に呼ぶ。軌跡の不連続を防ぐ。"""
        self.trail.clear()

    def reset(self) -> None:
        """モード切替時などにリセットする"""
        self.state = MaplessState.TRACKING
        self.trail.clear()
