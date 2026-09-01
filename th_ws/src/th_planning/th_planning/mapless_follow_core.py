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
    v_max:         float = 0.3   # m/s
    k_ang:         float = 2.0   # 角度ゲイン
    stop_radius_m: float = 0.2   # m ゴール到達とみなす半径
    w_max_rad_s:   float = 1.0   # rad/s 旋回速度の上限
                                  # (実機検証: 無制限だと角度誤差π×k_angのような
                                  #  物理的に不可能な指令が出る。follow_planner_core 参照)

    # 速度プロファイル（試験員との距離ベースの追従速度・加減速レート）
    # lookback_distance は目標点の"位置"を決めるだけで速度には使わない
    # (軌跡追従目標は常に近距離にあるため、目標点までの距離で速度を決めると
    #  常に低速に頭打ちになってしまう)。
    max_linear_accel_mps2:  float = 1.0   # m/s^2 加速側のレート制限 (急発進防止)
    max_linear_decel_mps2:  float = 2.0   # m/s^2 減速側のレート制限 兼 制動距離計算の減速度
                                            # (停止応答は加速より優先されるべきなので別枠。
                                            #  実機の実制動性能を超えない値にすること)
    max_angular_accel_rad_s2: float = 3.0  # rad/s^2 旋回速度のレート制限
                                            # (実機検証: これが無いと軌跡のノイズによる
                                            #  bearing の揺れが angular_z に瞬時に反映され、
                                            #  1周期で ±w_max まで飛んで機体が激しく揺れる)

    update_rate_hz: float = 10.0


@dataclass
class MaplessOutput:
    linear_x:  float = 0.0
    angular_z: float = 0.0
    state:     MaplessState = MaplessState.TRACKING
    # "tracking" / "person_close" / "obstacle_ahead" / "no_pose" / "no_scan"
    reason:    str = "tracking"


def should_stop_for_lost(
    is_lost: bool,
    lost_elapsed_sec: float,
    debounce_sec: float,
) -> bool:
    """
    is_lost によるロスト停止をデバウンスする。DR-SPAAM は約2Hzでしか推論
    されないため、1フレームの検知ミスだけで is_lost が一瞬 true になり得る。
    debounce_sec 未満の継続では「まだ見失っていない」とみなし、停止させない
    (実機検証: これが無いと十分離れていても頻繁に一瞬停止する)。
    障害物検知・接近時の停止はこの対象外(follow_planner_mapless.py 側で
    別途、即時停止のまま扱う)。
    """
    return is_lost and lost_elapsed_sec >= debounce_sec


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
    person_angle_rad: Optional[float] = None,
    person_range_m: Optional[float] = None,
    person_exclude_tolerance_m: float = 0.3,
) -> bool:
    """
    direction_rad ± half_width_rad の範囲にある LiDAR レンジ値のうち、
    max_check_dist 未満のものが1つでもあれば True(障害物あり)を返す。
    inf/nan は障害物なしとして扱う。

    person_angle_rad / person_range_m を指定すると、その方向・距離
    (± person_exclude_tolerance_m) にある反射は追従対象自身の脚とみなして
    無視する（生スキャンは追従対象を障害物と区別できないため、指定が無いと
    lookback_distance と obstacle_check_distance_m が近い設定の場合に
    追従対象自身で誤検知し停止を繰り返す）。追従対象より手前にある実際の
    障害物は距離が異なるため通常どおり検知される。
    """
    for i, r in enumerate(ranges):
        if not math.isfinite(r):
            continue
        angle = angle_min + i * angle_increment
        diff = math.atan2(math.sin(angle - direction_rad), math.cos(angle - direction_rad))
        if abs(diff) > half_width_rad or r >= max_check_dist:
            continue

        if person_angle_rad is not None and person_range_m is not None:
            person_diff = math.atan2(
                math.sin(angle - person_angle_rad), math.cos(angle - person_angle_rad))
            if abs(person_diff) <= half_width_rad and abs(r - person_range_m) <= person_exclude_tolerance_m:
                continue

        return True
    return False


def observe_cone(
    ranges: List[float],
    angle_min: float,
    angle_increment: float,
    direction_rad: float,
    half_width_rad: float,
) -> Optional[float]:
    """
    direction_rad ± half_width_rad の扇内にある有限レンジの最小値を返す。

    is_path_blocked() の bool 判定を最近傍距離に拡張したもの
    (DetailedDesign-safety.md §3.4.3。C++ 移植版は
    th_safety::observe_cone()、`th_safety/src/obstacle_limiter_core.cpp`。
    LimiterStatus.nearest_obstacle_m のような診断値として使う想定で、
    is_path_blocked() と違い max_check_dist によるフィルタは行わない
    (「ブロックされているか」ではなく「最近傍がどこか」を返すのが目的
    なので、距離の閾値判定は呼び出し側の責務にする)。

    is_path_blocked() と異なり person_* による追従対象除外は行わない。
    これはこの関数の抜け漏れではなく意図的な違い —— C++ 側
    obstacle_limiter_core は「どのレンジが追従対象(人物)か」を知らない
    設計にしてある(DetailedDesign-safety.md WP-SAFE-03 §1「作らない」:
    追従対象の除外は後段の責務ではない)。この Python 版もそれに揃える。
    除外が要る場面(mapless_follow の障害物判定)は今後も is_path_blocked()
    を使うこと。

    見つからなければ None(コーン内に条件を満たす有限レンジが1つも無い
    = 「空き」。C++版 ConeObservation{covered=true, nearest_m=+inf} に相当)。

    アルゴリズムは is_path_blocked() と同じ「全ビームの角度を都度計算して
    走査する」方式(角度ベース)。C++ 版は scan_geometry.sector_indices()
    でコーンの両端を先に添字化し、その添字区間だけを走査する(添字ベース)。
    全周スキャン(360°を1周する angle_min/angle_increment の構成)なら
    両者は一致するはずだが、境界のビーム1本で食い違う可能性がある
    (両者の等価性は test_obstacle_cone_equivalence.cpp で検証する。
    scan がコーンの一部しかカバーしていない場合の「未観測」の扱いは
    この関数には無い概念で、C++ 版の covered=false と単純には対応しない
    ―― この関数はどの入力に対しても常に「見つかった/見つからなかった」
    の2値でしか答えない。詳細は DetailedDesign-open.md N-12 を参照)。
    """
    nearest: Optional[float] = None
    for i, r in enumerate(ranges):
        if not math.isfinite(r) or r < 0.0:
            continue
        angle = angle_min + i * angle_increment
        diff = math.atan2(math.sin(angle - direction_rad), math.cos(angle - direction_rad))
        if abs(diff) > half_width_rad:
            continue
        if nearest is None or r < nearest:
            nearest = r
    return nearest


def mapless_target_speed(
    distance_to_person: float,
    stop_distance: float,
    max_decel_mps2: float,
    v_max: float,
) -> float:
    """
    試験員との実距離を元に目標速度を決める（制動距離ベース）。
    stop_distance に達するまでに max_decel_mps2 の減速度で確実に速度0まで
    落とせる速度を上限とする: v = sqrt(2 * max_decel_mps2 * margin)。
    線形ランプと違い v_max をいくつに設定しても「その場で急停止しても
    追従対象に届く前に止まれる」速度を上回らないため、v_max を上げても
    追従対象への追突を防げる。
    """
    margin = distance_to_person - stop_distance
    if margin <= 0.0:
        return 0.0
    return min(v_max, math.sqrt(2.0 * max_decel_mps2 * margin))


def rate_limit(target: float, prev: float, max_delta: float,
               max_delta_down: float = None) -> float:
    """
    1制御周期での変化量を制限する（急加減速防止）。
    max_delta_down を指定すると減少側だけ別の上限を使える
    (停止応答を加速立ち上がりより速くするため)。省略時は増減とも max_delta。
    """
    if max_delta_down is None:
        max_delta_down = max_delta
    if target > prev + max_delta:
        return prev + max_delta
    if target < prev - max_delta_down:
        return prev - max_delta_down
    return target


class MaplessFollowCore:
    """
    follow_planner_mapless の中核ロジック（ROS2 非依存）。
    ROS2 ノードから update_rate_hz で update() を呼び出す。
    """

    def __init__(self, params: MaplessFollowParams = None):
        self.params = params or MaplessFollowParams()
        self.state  = MaplessState.TRACKING
        self.trail: Deque[Point2D] = deque(maxlen=self.params.trail_max_points)
        self._prev_linear_x: float = 0.0
        self._prev_angular_z: float = 0.0

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
            self._prev_linear_x = 0.0
            self._prev_angular_z = 0.0
            return MaplessOutput(state=self.state, reason="no_pose")

        person_abs = to_absolute(robot_pose_odom, (person_x, person_y))
        update_trail(self.trail, person_abs, self.params.trail_sample_interval_m)

        if self.state == MaplessState.STOPPED:
            self._prev_linear_x = 0.0
            self._prev_angular_z = 0.0
            return MaplessOutput(state=self.state, reason="person_close")

        if scan_ranges is None:
            self._prev_linear_x = 0.0
            self._prev_angular_z = 0.0
            return MaplessOutput(state=self.state, reason="no_scan")

        goal_abs = get_trail_goal(self.trail, self.params.lookback_distance,
                                   robot_pos=(robot_pose_odom[0], robot_pose_odom[1]))
        rel_goal = to_relative(robot_pose_odom, goal_abs)
        bearing = math.atan2(rel_goal[1], rel_goal[0])

        if is_path_blocked(
                scan_ranges, scan_angle_min, scan_angle_increment,
                bearing, math.radians(self.params.obstacle_check_half_width_deg),
                self.params.obstacle_check_distance_m,
                person_angle_rad=math.atan2(person_y, person_x),
                person_range_m=distance):
            self._prev_linear_x = 0.0
            self._prev_angular_z = 0.0
            return MaplessOutput(state=self.state, reason="obstacle_ahead")

        _, target_w = pure_pursuit_control(
            (0.0, 0.0, 0.0), rel_goal,
            v_max=self.params.v_max, k_ang=self.params.k_ang,
            stop_radius=self.params.stop_radius_m,
            w_max=self.params.w_max_rad_s)

        target_v = mapless_target_speed(
            distance, self.params.stop_distance,
            self.params.max_linear_decel_mps2, self.params.v_max)
        # 大きく曲がる必要がある時は先に向き直れるよう並進を絞る
        # (曲がりきれず壁に衝突するのを防ぐ。follow_planner_core の
        #  pure_pursuit_control と同じ考え方)
        target_v *= max(0.0, math.cos(bearing))
        max_up   = self.params.max_linear_accel_mps2 / self.params.update_rate_hz
        max_down = self.params.max_linear_decel_mps2 / self.params.update_rate_hz
        v = rate_limit(target_v, self._prev_linear_x, max_up, max_down)
        self._prev_linear_x = v

        # 旋回速度も同様にレート制限する。軌跡(trail)の点は試験員位置検知の
        # ノイズをそのまま含むため、制限が無いと bearing のわずかな揺れが
        # 1周期で ±w_max までの旋回指令に直結し機体が激しく揺れる。
        max_ang_delta = self.params.max_angular_accel_rad_s2 / self.params.update_rate_hz
        w = rate_limit(target_w, self._prev_angular_z, max_ang_delta)
        self._prev_angular_z = w

        return MaplessOutput(linear_x=v, angular_z=w, state=self.state, reason="tracking")

    def clear_trail(self) -> None:
        """試験員ロスト→再検出時に呼ぶ。軌跡の不連続を防ぐ。"""
        self.trail.clear()

    def notify_lost(self) -> None:
        """
        試験員ロスト中に ROS2 ノード側から毎周期呼ぶ。前回速度を 0 にしておくことで、
        再検出時に rate_limit がロスト直前の速度から再開する（＝停止せず進み続ける）
        のを防ぎ、0 から滑らかに加速し直す。
        """
        self._prev_linear_x = 0.0
        self._prev_angular_z = 0.0

    def reset(self) -> None:
        """モード切替時などにリセットする"""
        self.state = MaplessState.TRACKING
        self.trail.clear()
        self._prev_linear_x = 0.0
        self._prev_angular_z = 0.0
