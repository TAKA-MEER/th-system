"""
follow_planner_core.py
======================
follow_planner のコアロジック（ROS2 非依存）

テスト可能性のため、ROS2 インポートを一切含まない純粋な Python モジュール。
ROS2 ノード側 (follow_planner.py) はこのモジュールを import して使用する。

距離帯ベース方式（followLogic.md v2）:
  TRACKING (d >= d_prepare)           : 試験員の軌跡を遡った点を Nav2 ゴールとして送る
  PREPARE  (d_evade <= d < d_prepare) : 前進せず、地図上の最空きスペース方向へ旋回して待機
  EVADING  (d < d_evade)              : 退避方向へ Pure Pursuit で前進する

軌跡 (trail) と退避方向/退避ゴールは絶対座標系（costmap の frame_id、通常 odom）で
保持する。base_link 相対座標系だとロボット自身の移動で過去の点の意味が変わって
しまうため、ROS2 ノード側から渡される robot_pose_odom (base_link の odom 系での
姿勢) を使って都度変換する。
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from enum import Enum, auto
from typing import Deque, Optional, Tuple


Point2D = Tuple[float, float]
Pose2D  = Tuple[float, float, float]   # (x, y, theta)


# ──────────────────────────────────────────────────────────────────
# 内部状態
# ──────────────────────────────────────────────────────────────────
class FollowState(Enum):
    TRACKING = auto()   # 状態A: 軌跡追従
    PREPARE  = auto()   # 状態B: 退避準備
    EVADING  = auto()   # 状態C: 退避


# ──────────────────────────────────────────────────────────────────
# パラメータ群（外部から差し替え可能）
# ──────────────────────────────────────────────────────────────────
@dataclass
class FollowParams:
    # 軌跡追従 (TRACKING)
    lookback_distance:          float = 1.0    # m 軌跡追従時に遡る距離
    trail_sample_interval_m:    float = 0.05   # m 軌跡点を追加する最小移動距離
    trail_max_points:           int   = 2000   # 軌跡の最大保持点数

    # 状態遷移（距離帯）
    d_prepare:                  float = 3.0    # m TRACKING/PREPARE の切替距離
    d_evade:                    float = 2.0    # m PREPARE/EVADING の切替距離
    distance_hysteresis_m:      float = 0.2    # m 復帰判定に加える余裕距離（チャタリング防止）

    # 退避方向探索 (PREPARE)
    evade_scan_directions:      int   = 16     # 空きスペース探索の走査方向数
    evade_scan_max_dist:        float = 3.0    # m 空きスペース探索の最大走査距離
    evade_route_length_m:       float = 2.0    # m 退避ルートの長さ
    retreat_check_clearance:    float = 0.5    # m 退避方向に必要な最小自由空間

    # 退避走行 (EVADING)
    retreat_speed:               float = 0.15  # m/s 退避速度（= v_evade）
    evade_k_ang:                  float = 2.0  # 旋回 / Pure Pursuit の角度ゲイン
    evade_stop_radius_m:          float = 0.2  # m Pure Pursuit の停止半径
    evade_orient_tolerance_rad:   float = 0.05 # rad 向き合わせ完了とみなす角度誤差

    # 共通
    goal_deadzone_m:             float = 0.30  # m ゴール更新のデッドゾーン
    update_rate_hz:               float = 10.0


# ──────────────────────────────────────────────────────────────────
# 座標変換（base_link 相対 ⇔ 絶対座標系）
# ──────────────────────────────────────────────────────────────────
def to_absolute(robot_pose: Pose2D, rel_point: Point2D) -> Point2D:
    """base_link 相対座標を絶対座標系（odom 等）に変換する"""
    x_r, y_r, theta_r = robot_pose
    c, s = math.cos(theta_r), math.sin(theta_r)
    return (
        x_r + rel_point[0] * c - rel_point[1] * s,
        y_r + rel_point[0] * s + rel_point[1] * c,
    )


def to_relative(robot_pose: Pose2D, abs_point: Point2D) -> Point2D:
    """絶対座標系（odom 等）の点を base_link 相対座標に変換する"""
    x_r, y_r, theta_r = robot_pose
    dx, dy = abs_point[0] - x_r, abs_point[1] - y_r
    c, s = math.cos(theta_r), math.sin(theta_r)
    return (dx * c + dy * s, -dx * s + dy * c)


# ──────────────────────────────────────────────────────────────────
# 軌跡追従（状態A: TRACKING）
# ──────────────────────────────────────────────────────────────────
def update_trail(trail: Deque[Point2D], person_pos: Point2D, min_delta: float = 0.05) -> None:
    """試験員の軌跡バッファへ点を追加する（前回点から一定距離以上移動した場合のみ）"""
    if not trail or math.hypot(person_pos[0] - trail[-1][0], person_pos[1] - trail[-1][1]) > min_delta:
        trail.append(person_pos)


def get_trail_goal(
    trail: Deque[Point2D],
    lookback_distance: float,
    robot_pos: Point2D = (0.0, 0.0),
) -> Point2D:
    """
    軌跡上を現在点から lookback_distance だけ遡った点を目標地点として返す。
    trail が空の場合は robot_pos、1点しかない場合はその点を返す。
    """
    if len(trail) == 0:
        return robot_pos
    if len(trail) < 2:
        return trail[-1]

    cum_dist = 0.0
    pts = list(trail)
    for i in range(len(pts) - 1, 0, -1):
        seg = math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
        cum_dist += seg
        if cum_dist >= lookback_distance:
            return pts[i - 1]
    return pts[0]


# ──────────────────────────────────────────────────────────────────
# 距離帯による状態遷移（ヒステリシス付き）
# ──────────────────────────────────────────────────────────────────
def next_follow_state(
    current_state: FollowState,
    distance: float,
    d_prepare: float,
    d_evade: float,
    hysteresis: float,
) -> FollowState:
    """距離に基づき次の状態を返す。復帰判定には hysteresis 分の余裕を要求する。"""
    if current_state == FollowState.TRACKING:
        if distance < d_prepare:
            return FollowState.PREPARE
        return FollowState.TRACKING

    if current_state == FollowState.PREPARE:
        if distance < d_evade:
            return FollowState.EVADING
        if distance >= d_prepare + hysteresis:
            return FollowState.TRACKING
        return FollowState.PREPARE

    # EVADING
    if distance >= d_evade + hysteresis:
        return FollowState.PREPARE
    return FollowState.EVADING


# ──────────────────────────────────────────────────────────────────
# 退避方向探索（状態B: PREPARE）
# ──────────────────────────────────────────────────────────────────
def find_nearest_open_direction(
    clearance_fn,                   # callable(dx, dy) -> float: 単位ベクトル方向への自由距離 [m]
    num_directions: int = 16,
    max_check_dist: float = 3.0,
    min_clearance: float = 0.5,
) -> Tuple[float, float]:
    """
    ロボット位置を中心に放射状に num_directions 方向を走査し、
    min_clearance 以上の自由空間を確保できる中で最も自由距離の大きい方向を返す。

    戻り値: (best_angle_rad, best_clear_dist)
    採用可能な方向が無い場合は best_clear_dist = -1.0 を返す（呼び出し側で判定する）。
    """
    best_angle = 0.0
    best_clear_dist = -1.0
    step = 2.0 * math.pi / num_directions

    for k in range(num_directions):
        angle = k * step
        dx, dy = math.cos(angle), math.sin(angle)
        clear_dist = min(clearance_fn(dx, dy), max_check_dist)

        if clear_dist >= min_clearance and clear_dist > best_clear_dist:
            best_clear_dist = clear_dist
            best_angle = angle

    return best_angle, best_clear_dist


def compute_evade_goal(
    robot_pos_abs: Point2D,
    escape_angle_abs: float,
    route_length: float,
) -> Point2D:
    """絶対座標系でのロボット位置と退避角度から、退避ルート終端点を返す"""
    return (
        robot_pos_abs[0] + route_length * math.cos(escape_angle_abs),
        robot_pos_abs[1] + route_length * math.sin(escape_angle_abs),
    )


def orient_to_evade_route(
    robot_pose: Pose2D,
    escape_angle: float,
    k_ang: float = 2.0,
    angle_tolerance: float = 0.05,
) -> Tuple[float, float]:
    """退避方向(絶対角)へ機体を向ける。前進速度は常に 0。"""
    _, _, theta_r = robot_pose
    angle_err = math.atan2(math.sin(escape_angle - theta_r), math.cos(escape_angle - theta_r))

    if abs(angle_err) < angle_tolerance:
        return (0.0, 0.0)

    return (0.0, k_ang * angle_err)


# ──────────────────────────────────────────────────────────────────
# Pure Pursuit（状態C: EVADING）
# ──────────────────────────────────────────────────────────────────
def pure_pursuit_control(
    robot_pose: Pose2D,
    goal: Point2D,
    v_max: float = 0.5,
    k_ang: float = 2.0,
    stop_radius: float = 0.3,
) -> Tuple[float, float]:
    x_r, y_r, theta_r = robot_pose
    dx, dy = goal[0] - x_r, goal[1] - y_r
    dist = math.hypot(dx, dy)

    if dist < stop_radius:
        return (0.0, 0.0)

    target_angle = math.atan2(dy, dx)
    angle_err = math.atan2(math.sin(target_angle - theta_r), math.cos(target_angle - theta_r))

    v = v_max * min(max(dist / 1.5, 0.0), 1.0)
    omega = k_ang * angle_err
    return (v, omega)


# ──────────────────────────────────────────────────────────────────
# メインステートマシン
# ──────────────────────────────────────────────────────────────────
class FollowPlannerCore:
    """
    follow_planner の中核ロジック（ROS2 非依存）。
    ROS2 ノードから 10Hz で update() を呼び出す。
    """

    def __init__(self, params: FollowParams = None):
        self.params = params or FollowParams()
        self.state  = FollowState.TRACKING

        self.trail: Deque[Point2D] = deque(maxlen=self.params.trail_max_points)

        self._prev_goal:        Optional[Point2D] = None
        self._escape_angle_abs: Optional[float]    = None
        self._evade_goal_abs:   Optional[Point2D]  = None

    def update(
        self,
        person_x: float,
        person_y: float,
        t: float,
        clearance_fn,                                  # callable(dx,dy)->float
        robot_pose_odom: Optional[Pose2D],              # (x,y,theta) odom(costmap)系
    ) -> "PlannerOutput":
        """
        1制御周期分の計算を行い PlannerOutput を返す。

        戻り値の kind:
          "nav_goal"   → (x,y,yaw) を Nav2 へ送る（base_link 座標）
          "retreat"    → (linear_x, angular_z) を /cmd_vel_retreat へ直接送る
          "stop"       → 退避不可 / 絶対姿勢未確立で停止
          "no_update"  → デッドゾーン内・絶対姿勢未確立などで変化なし
        """
        distance = math.hypot(person_x, person_y)

        next_state = next_follow_state(
            self.state, distance,
            self.params.d_prepare, self.params.d_evade,
            self.params.distance_hysteresis_m)

        if next_state != self.state and next_state in (FollowState.TRACKING, FollowState.PREPARE):
            # TRACKING へ復帰、または PREPARE へ（再)突入 → 退避方向を再計算させる
            self._escape_angle_abs = None
            self._evade_goal_abs   = None
        self.state = next_state

        if self.state == FollowState.TRACKING:
            return self._handle_tracking(person_x, person_y, robot_pose_odom)

        if self.state == FollowState.PREPARE:
            return self._handle_prepare(clearance_fn, robot_pose_odom)

        return self._handle_evading(clearance_fn, robot_pose_odom)

    def _handle_tracking(
        self, person_x: float, person_y: float, robot_pose_odom: Optional[Pose2D]
    ) -> "PlannerOutput":
        if robot_pose_odom is None:
            # 絶対姿勢が無いと軌跡を正しく蓄積できないため、この周期はスキップする
            return PlannerOutput(kind="no_update")

        person_abs = to_absolute(robot_pose_odom, (person_x, person_y))
        update_trail(self.trail, person_abs, self.params.trail_sample_interval_m)

        goal_abs = get_trail_goal(self.trail, self.params.lookback_distance,
                                   robot_pos=(robot_pose_odom[0], robot_pose_odom[1]))
        gx, gy = to_relative(robot_pose_odom, goal_abs)
        goal_yaw = math.atan2(person_y - gy, person_x - gx)
        return self._nav_goal_output(gx, gy, goal_yaw)

    def _handle_prepare(self, clearance_fn, robot_pose_odom: Optional[Pose2D]) -> "PlannerOutput":
        if robot_pose_odom is None:
            return PlannerOutput(kind="stop", retreat_blocked=True)

        if self._escape_angle_abs is None:
            if not self._scan_escape_direction(clearance_fn, robot_pose_odom):
                return PlannerOutput(kind="stop", retreat_blocked=True)

        lx, az = orient_to_evade_route(
            robot_pose_odom, self._escape_angle_abs,
            self.params.evade_k_ang, self.params.evade_orient_tolerance_rad)
        return PlannerOutput(kind="retreat", linear_x=lx, angular_z=az)

    def _handle_evading(self, clearance_fn, robot_pose_odom: Optional[Pose2D]) -> "PlannerOutput":
        if robot_pose_odom is None:
            return PlannerOutput(kind="stop", retreat_blocked=True)

        if self._escape_angle_abs is None:
            # PREPARE を経ずに来ることは想定していないが、念のためこの周期で走査する
            if not self._scan_escape_direction(clearance_fn, robot_pose_odom):
                return PlannerOutput(kind="stop", retreat_blocked=True)

        if self._evade_goal_abs is None:
            self._evade_goal_abs = compute_evade_goal(
                (robot_pose_odom[0], robot_pose_odom[1]),
                self._escape_angle_abs,
                self.params.evade_route_length_m)

        lx, az = pure_pursuit_control(
            robot_pose_odom, self._evade_goal_abs,
            v_max=self.params.retreat_speed,
            k_ang=self.params.evade_k_ang,
            stop_radius=self.params.evade_stop_radius_m)
        return PlannerOutput(kind="retreat", linear_x=lx, angular_z=az)

    def _scan_escape_direction(self, clearance_fn, robot_pose_odom: Pose2D) -> bool:
        angle_rel, clear_dist = find_nearest_open_direction(
            clearance_fn,
            num_directions=self.params.evade_scan_directions,
            max_check_dist=self.params.evade_scan_max_dist,
            min_clearance=self.params.retreat_check_clearance)
        if clear_dist < 0.0:
            return False
        self._escape_angle_abs = robot_pose_odom[2] + angle_rel
        return True

    def _nav_goal_output(self, gx: float, gy: float, gyaw: float) -> "PlannerOutput":
        """デッドゾーン判定付きで nav_goal を返す"""
        if self._prev_goal is not None:
            dx = gx - self._prev_goal[0]
            dy = gy - self._prev_goal[1]
            if math.hypot(dx, dy) < self.params.goal_deadzone_m:
                return PlannerOutput(kind="no_update")
        self._prev_goal = (gx, gy)
        return PlannerOutput(kind="nav_goal", goal_x=gx, goal_y=gy, goal_yaw=gyaw)

    def clear_prev_goal(self) -> None:
        """デッドゾーンをリセットして次回必ずゴールを送出する"""
        self._prev_goal = None

    def clear_trail(self) -> None:
        """試験員ロスト→再検出時に呼ぶ。軌跡の不連続（過去点の再利用）を防ぐ。"""
        self.trail.clear()

    def reset(self) -> None:
        """モード切替時などにリセットする"""
        self.state = FollowState.TRACKING
        self.trail.clear()
        self._prev_goal        = None
        self._escape_angle_abs = None
        self._evade_goal_abs   = None


@dataclass
class PlannerOutput:
    kind:            str   = "no_update"   # "nav_goal"/"retreat"/"stop"/"no_update"
    # nav_goal 時
    goal_x:          float = 0.0
    goal_y:          float = 0.0
    goal_yaw:        float = 0.0
    # retreat 時
    linear_x:        float = 0.0
    angular_z:       float = 0.0
    # 退避不可フラグ
    retreat_blocked: bool  = False
