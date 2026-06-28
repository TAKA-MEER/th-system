"""
follow_planner_core.py
======================
follow_planner のコアロジック（ROS2 非依存）

テスト可能性のため、ROS2 インポートを一切含まない純粋な Python モジュール。
ROS2 ノード側 (follow_planner.py) はこのモジュールを import して使用する。
"""

from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List, Optional, Tuple


# ── 型エイリアス ──────────────────────────────────────────────────
Point2D   = Tuple[float, float]   # (x, y) メートル
Vector2D  = Tuple[float, float]   # (vx, vy) m/s


# ──────────────────────────────────────────────────────────────────
# 内部状態
# ──────────────────────────────────────────────────────────────────
class FollowState(Enum):
    NORMAL_FOLLOW      = auto()   # 4.3.3 通常追従
    STATIC_REPOSITION  = auto()   # 4.3.4 静止時再配置
    RETREAT            = auto()   # 4.3.7 近接退避


# ──────────────────────────────────────────────────────────────────
# パラメータ群（外部から差し替え可能）
# ──────────────────────────────────────────────────────────────────
@dataclass
class FollowParams:
    # 4.3.3 通常追従
    follow_distance_target:    float = 1.5    # m
    follow_distance_min:       float = 0.8    # m
    follow_distance_max:       float = 3.0    # m
    follow_angle_offset_max:   float = 30.0   # deg（斜め後方の最大オフセット）
    corridor_width_threshold:  float = 1.5    # m（真後ろ追従に切替える通路幅）
    lidar_fov_deg:             float = 360.0  # deg（有効視野角）
    lidar_blind_angle_ranges:  List[Tuple[float, float]] = field(
        default_factory=list)                 # [(start_deg, end_deg), ...]
    vel_history_sec:           float = 1.0    # 速度推定に使う位置履歴長

    # 4.3.4 静止時再配置
    static_speed_threshold:    float = 0.05   # m/s（静止判定速度）
    static_time_threshold:     float = 2.0    # s（静止継続時間）
    candidate_count:           int   = 16     # 候補点数（放射状）
    candidate_radius:          float = 1.5    # m（候補点の半径）
    work_front_exclusion_deg:  float = 60.0   # deg（作業前方除外角度の半角）

    # 4.3.7 近接退避
    retreat_trigger_distance:  float = 0.8    # m
    retreat_release_distance:  float = 1.2    # m
    closing_speed_threshold:   float = 0.3    # m/s
    retreat_check_clearance:   float = 0.5    # m（退避方向の必要自由空間）
    retreat_speed:             float = 0.15   # m/s

    # 進行路上退避
    approach_perpendicular_deg: float = 45.0  # deg 垂直判定の半角（この範囲で前後逃げを優先）

    # 共通
    path_approach_angle_deg:   float = 60.0   # deg（試験員がロボット方向へ向かっているとみなす角度差閾値）
    goal_deadzone_m:           float = 0.30   # m
    update_rate_hz:            float = 10.0


# ──────────────────────────────────────────────────────────────────
# 位置履歴バッファ
# ──────────────────────────────────────────────────────────────────
class PositionHistory:
    """試験員の位置履歴を保持し、速度ベクトルを推定する"""

    def __init__(self, max_sec: float = 2.0, rate_hz: float = 10.0):
        maxlen = int(max_sec * rate_hz) + 1
        self._buf: deque[Tuple[float, float, float]] = deque(maxlen=maxlen)  # (t, x, y)
        self._last_vel: Vector2D = (0.0, 0.0)

    def update(self, x: float, y: float, t: float) -> None:
        self._buf.append((t, x, y))
        if len(self._buf) >= 2:
            self._last_vel = self._estimate_vel()

    def _estimate_vel(self) -> Vector2D:
        """最小二乗法で速度ベクトルを推定（2点以上必要）"""
        if len(self._buf) < 2:
            return (0.0, 0.0)
        oldest = self._buf[0]
        newest = self._buf[-1]
        dt = newest[0] - oldest[0]
        if dt < 1e-3:
            return (0.0, 0.0)
        vx = (newest[1] - oldest[1]) / dt
        vy = (newest[2] - oldest[2]) / dt
        return (vx, vy)

    @property
    def velocity(self) -> Vector2D:
        return self._last_vel

    @property
    def speed(self) -> float:
        vx, vy = self._last_vel
        return math.hypot(vx, vy)

    @property
    def direction_rad(self) -> float:
        """進行方向 [rad]（静止中は直前値を保持）"""
        vx, vy = self._last_vel
        if math.hypot(vx, vy) < 0.02:
            return math.atan2(vy, vx) if (vx != 0 or vy != 0) else 0.0
        return math.atan2(vy, vx)

    def closing_speed_to_robot(self) -> float:
        """ロボット（原点）への接近速度 [m/s]（正=接近）"""
        if len(self._buf) < 2:
            return 0.0
        p0 = self._buf[-2]
        p1 = self._buf[-1]
        dt = p1[0] - p0[0]
        if dt < 1e-3:
            return 0.0
        d0 = math.hypot(p0[1], p0[2])
        d1 = math.hypot(p1[1], p1[2])
        return (d0 - d1) / dt

    def clear(self) -> None:
        self._buf.clear()
        self._last_vel = (0.0, 0.0)


# ──────────────────────────────────────────────────────────────────
# 静止検知
# ──────────────────────────────────────────────────────────────────
class StaticDetector:
    """試験員が静止（作業中）かどうかを判定する"""

    def __init__(self, speed_threshold: float = 0.05, time_threshold: float = 2.0):
        self._speed_th    = speed_threshold
        self._time_th     = time_threshold
        self._static_since: Optional[float] = None

    def update(self, speed: float, t: float) -> bool:
        """is_static を返す"""
        if speed > self._speed_th:
            self._static_since = None
            return False
        if self._static_since is None:
            self._static_since = t
        return (t - self._static_since) >= self._time_th

    def reset(self) -> None:
        self._static_since = None


# ──────────────────────────────────────────────────────────────────
# 退避ヒステリシス
# ──────────────────────────────────────────────────────────────────
class RetreatHysteresis:
    """前進・後退のハンチングを防ぐヒステリシス機構（4.3.7 §6）"""

    def __init__(self, trigger: float, release: float, closing_th: float):
        assert release > trigger, "release > trigger でなければヒステリシスにならない"
        self._trigger    = trigger
        self._release    = release
        self._closing_th = closing_th
        self._active     = False

    def update(self, dist: float, closing_speed: float) -> bool:
        if not self._active:
            # 非退避中 → トリガー条件で退避開始
            if dist < self._trigger or closing_speed > self._closing_th:
                self._active = True
        else:
            # 退避中 → 両方の解除条件を満たしたら終了
            if dist > self._release and closing_speed <= self._closing_th:
                self._active = False
        return self._active

    @property
    def is_active(self) -> bool:
        return self._active

    def reset(self) -> None:
        self._active = False


# ──────────────────────────────────────────────────────────────────
# 候補点スコアリング（4.3.4）
# ──────────────────────────────────────────────────────────────────
@dataclass
class CandidatePoint:
    x: float
    y: float
    angle_from_person_rad: float   # 試験員から見た方向（ロボット基準）
    score: float = 0.0
    feasible: bool = True


def generate_candidate_points(
    person_x: float,
    person_y: float,
    person_facing_rad: float,       # 試験員の正面方向（ロボット基準）
    radius: float,
    count: int,
    work_exclusion_half_deg: float  # 正面方向からの除外角度（半角）
) -> List[CandidatePoint]:
    """
    試験員を中心に放射状に候補点を生成する。
    試験員の正面方向（配電盤がある方向）付近は除外する。
    """
    candidates: List[CandidatePoint] = []
    excl_half = math.radians(work_exclusion_half_deg)
    step = 2 * math.pi / count

    for i in range(count):
        angle = i * step          # ロボット基準の絶対角度
        # 試験員から見た相対角度（正面を 0 として）
        rel = angle - person_facing_rad
        # 正規化 -π〜π
        while rel >  math.pi: rel -= 2 * math.pi
        while rel < -math.pi: rel += 2 * math.pi

        # 正面付近を除外（配電盤作業スペース）
        if abs(rel) < excl_half:
            continue

        cx = person_x + radius * math.cos(angle)
        cy = person_y + radius * math.sin(angle)
        candidates.append(CandidatePoint(
            x=cx, y=cy,
            angle_from_person_rad=angle
        ))
    return candidates


def score_candidate(
    candidate: CandidatePoint,
    robot_x: float,
    robot_y: float,
    person_x: float,
    person_y: float,
    person_facing_rad: float,
    is_in_fov_fn,           # callable(x, y) → bool
    is_clear_fn,            # callable(x, y) → bool （costmap）
    move_cost_weight: float = 1.0,
    fov_violation_penalty: float = 1000.0,
    blocked_penalty: float  = 500.0,
) -> float:
    """
    候補点のスコアを計算する（低いほど良い）。
    スコア = 移動コスト + ペナルティ
    """
    score = 0.0

    # costmap チェック
    if not is_clear_fn(candidate.x, candidate.y):
        score += blocked_penalty
        candidate.feasible = False

    # 視野角チェック
    if not is_in_fov_fn(candidate.x - person_x, candidate.y - person_y):
        score += fov_violation_penalty

    # 移動距離コスト（現在位置からの距離）
    move_dist = math.hypot(candidate.x - robot_x, candidate.y - robot_y)
    score += move_cost_weight * move_dist

    candidate.score = score
    return score


# ──────────────────────────────────────────────────────────────────
# LiDAR 視野角判定
# ──────────────────────────────────────────────────────────────────
class FovChecker:
    """
    ロボットから見た相対ベクトルが LiDAR 有効視野角内かどうかを判定する。
    死角範囲 (blind_angle_ranges) は除外する。
    """

    def __init__(self, fov_deg: float = 360.0,
                 blind_ranges: List[Tuple[float, float]] = None):
        self._fov_half = math.radians(fov_deg / 2.0)
        self._blind    = [(math.radians(s), math.radians(e))
                          for s, e in (blind_ranges or [])]

    def is_in_fov(self, dx: float, dy: float) -> bool:
        """(dx, dy) がロボット基準の相対ベクトル"""
        angle = math.atan2(dy, dx)
        # 360° FOV の場合は死角チェックのみ
        if self._fov_half >= math.pi:
            return not self._in_blind(angle)
        return abs(angle) <= self._fov_half and not self._in_blind(angle)

    def _in_blind(self, angle_rad: float) -> bool:
        a = angle_rad % (2 * math.pi)
        for (s, e) in self._blind:
            s = s % (2 * math.pi)
            e = e % (2 * math.pi)
            if s <= e:
                if s <= a <= e:
                    return True
            else:
                if a >= s or a <= e:
                    return True
        return False

    def clamp_offset(self, desired_offset_deg: float,
                     person_angle_rad: float) -> float:
        """
        角度オフセットを適用した結果が死角に入る場合、
        視野角内に収まるよう縮小する。
        返り値: 実際に使用するオフセット [deg]
        """
        for offset in range(int(desired_offset_deg), -1, -1):
            test_angle = person_angle_rad + math.radians(offset)
            if self.is_in_fov(math.cos(test_angle), math.sin(test_angle)):
                return float(offset)
        return 0.0


# ──────────────────────────────────────────────────────────────────
# 通路幅評価
# ──────────────────────────────────────────────────────────────────
def estimate_corridor_width(
    query_x: float,
    query_y: float,
    query_yaw: float,
    costmap_query_fn,      # callable(x, y) → bool (True = free)
    probe_dist: float = 3.0,
    probe_step: float = 0.05,
) -> float:
    """
    ロボットの横方向（左右）に障害物をプローブし、通路幅を推定する。
    costmap_query_fn: (x,y) が空きならば True を返す関数

    返り値: 推定通路幅 [m]
    """
    perp = query_yaw + math.pi / 2.0   # 横方向角度

    def probe_side(sign: float) -> float:
        for d in [probe_step * i for i in range(1, int(probe_dist / probe_step) + 1)]:
            px = query_x + sign * d * math.cos(perp)
            py = query_y + sign * d * math.sin(perp)
            if not costmap_query_fn(px, py):
                return d
        return probe_dist

    left  = probe_side( 1.0)
    right = probe_side(-1.0)
    return left + right


# ──────────────────────────────────────────────────────────────────
# 追従ゴール計算（4.3.3）
# ──────────────────────────────────────────────────────────────────
def compute_follow_goal(
    person_x:   float,
    person_y:   float,
    person_vel: Vector2D,
    params:     FollowParams,
    corridor_width: float,
    fov_checker: FovChecker,
) -> Tuple[float, float, float]:
    """
    通常追従ゴール (goal_x, goal_y, goal_yaw) を base_link 座標系で計算する。

    アルゴリズム (4.3.3):
      1. 試験員の進行方向を速度ベクトルから推定
      2. 通路幅に応じて追従角度オフセットを決定
      3. 視野角制約を適用しオフセットを縮小
      4. 距離制約に基づいてゴール位置を算出
    """
    dist = math.hypot(person_x, person_y)

    # ── 1. 試験員の進行方向 ──────────────────────────────────────
    vx, vy = person_vel
    person_dir = math.atan2(vy, vx) if math.hypot(vx, vy) > 0.02 else math.atan2(person_y, person_x)

    # ── 2. 通路幅による角度オフセット決定 ──────────────────────────
    if corridor_width < params.corridor_width_threshold:
        desired_offset_deg = 0.0           # 狭い通路 → 真後ろ
    else:
        # 開けた空間 → 斜め後方（最大 follow_angle_offset_max）
        desired_offset_deg = params.follow_angle_offset_max

    # ── 3. 視野角制約で縮小 ─────────────────────────────────────
    person_angle = math.atan2(person_y, person_x)
    actual_offset_deg = fov_checker.clamp_offset(desired_offset_deg, person_angle)

    # ── 4. ゴール位置計算 ────────────────────────────────────────
    # 試験員の後方 follow_distance_target m の点
    # オフセット: 試験員進行方向の逆方向から actual_offset_deg 回転
    behind_dir = person_dir + math.pi + math.radians(actual_offset_deg)

    goal_x = person_x + params.follow_distance_target * math.cos(behind_dir)
    goal_y = person_y + params.follow_distance_target * math.sin(behind_dir)

    # ゴール姿勢: 試験員を向く
    goal_yaw = math.atan2(person_y - goal_y, person_x - goal_x)

    return (goal_x, goal_y, goal_yaw)


# ──────────────────────────────────────────────────────────────────
# 退避速度指令計算（4.3.7）
# ──────────────────────────────────────────────────────────────────
def compute_retreat_cmd(
    person_x:   float,
    person_y:   float,
    retreat_speed: float,
    clearance_fn,   # callable(dx, dy) → float: 退避方向の自由空間距離
    min_clearance:  float,
) -> Optional[Tuple[float, float]]:
    """
    退避方向速度指令 (linear_x, angular_z) を計算する。
    自由空間が不十分な場合は None を返す（その場停止）。

    クローラーは非ホロノミックなため:
      - 試験員が前方にいる → 後退（linear_x < 0）
      - 試験員が横・後方にいる → 旋回してから後退の 2 フェーズ
    ただし緊急退避ではまず距離を開けることを優先し、
    試験員から遠ざかる方向の linear_x を直接出す簡略版を採用する。
    """
    dist = math.hypot(person_x, person_y)
    if dist < 1e-3:
        return (0.0, 0.0)

    # 試験員から遠ざかる方向（逆ベクトル）
    away_x = -person_x / dist
    away_y = -person_y / dist

    # 自由空間チェック
    clearance = clearance_fn(away_x, away_y)
    if clearance < min_clearance:
        return None   # 退避不可 → 呼び出し側で停止

    # クローラーは y 方向に直接移動できないため、
    # x 成分（前後）のみを使い、y 偏差は angular_z で補正
    linear_x  = away_x * retreat_speed
    angular_z = away_y * retreat_speed * 2.0   # 横方向誤差を旋回で補正（ゲイン暫定）

    return (linear_x, angular_z)


# ──────────────────────────────────────────────────────────────────
# 進行路上退避速度指令計算
# ──────────────────────────────────────────────────────────────────
def compute_approach_escape_cmd(
    person_dir:         float,   # 試験員の進行方向 (rad, base_link +x 基準)
    escape_speed:       float,   # m/s（= retreat_speed を流用）
    perp_threshold_deg: float,   # 垂直判定の半角 [deg]
    clearance_fn,                # callable(dx, dy) → float: その方向の自由空間距離
    min_clearance:      float,   # m 最低限必要な自由空間
) -> Optional[Tuple[float, float]]:
    """
    試験員の進行方向上にロボットがいる場合の退避速度指令 (linear_x, angular_z) を計算する。

    優先順位:
      1. クローラ前後軸（±x）が試験員進行方向と垂直かつスペースあり
           → ±x 方向（前進 or 後退）でスペースの広い方向へ逃げる
      2. 平行 or 垂直でもスペースなし
           → 試験員の進行方向そのままに速度指令を発行（後退含む）
      スペースが全方向不足 → None（停止）

    base_link 座標系。ロボット前進方向 = +x 軸。
    """
    # alpha: ロボット前後軸(x軸)と試験員進行方向の角度差 [0, π]
    alpha = abs(person_dir)
    if alpha > math.pi:
        alpha = 2.0 * math.pi - alpha

    perp_rad = math.radians(perp_threshold_deg)
    is_perp  = abs(alpha - math.pi / 2.0) <= perp_rad

    if is_perp:
        fwd_clear = clearance_fn(1.0, 0.0)
        bwd_clear = clearance_fn(-1.0, 0.0)
        if max(fwd_clear, bwd_clear) >= min_clearance:
            if fwd_clear >= bwd_clear:
                return (escape_speed, 0.0)
            else:
                return (-escape_speed, 0.0)
        # スペース不足 → 試験員進行方向へフォールスルー

    # 試験員の進行方向に沿って退避（前進 or 後退）
    esc_x = math.cos(person_dir)
    esc_y = math.sin(person_dir)
    if clearance_fn(esc_x, esc_y) < min_clearance:
        return None   # 退避不可

    # 非ホロノミック補正: x 成分で前後移動、y 成分は angular_z で補正
    linear_x  = esc_x * escape_speed
    angular_z = esc_y * escape_speed * 2.0
    return (linear_x, angular_z)


# ──────────────────────────────────────────────────────────────────
# メインステートマシン
# ──────────────────────────────────────────────────────────────────
class FollowPlannerCore:
    """
    follow_planner の中核ロジック（ROS2 非依存）。
    ROS2 ノードから 10Hz で update() を呼び出す。
    """

    def __init__(self, params: FollowParams = None):
        self.params         = params or FollowParams()
        self.state          = FollowState.NORMAL_FOLLOW
        self.pos_history    = PositionHistory(
            max_sec=self.params.vel_history_sec * 2,
            rate_hz=self.params.update_rate_hz)
        self.static_detector = StaticDetector(
            speed_threshold=self.params.static_speed_threshold,
            time_threshold=self.params.static_time_threshold)
        self.retreat_hyst   = RetreatHysteresis(
            trigger=self.params.retreat_trigger_distance,
            release=self.params.retreat_release_distance,
            closing_th=self.params.closing_speed_threshold)
        self.fov_checker    = FovChecker(
            fov_deg=self.params.lidar_fov_deg,
            blind_ranges=self.params.lidar_blind_angle_ranges)
        self._prev_goal: Optional[Point2D] = None
        self._static_goal:  Optional[Tuple[float, float, float]] = None

    def update(
        self,
        person_x:       float,
        person_y:       float,
        t:              float,
        corridor_width: float,                      # m
        costmap_clear_fn,                           # callable(x,y)→bool
        retreat_clearance_fn,                       # callable(dx,dy)→float
    ) -> "PlannerOutput":
        """
        1 制御周期分の計算を行い PlannerOutput を返す。

        戻り値の kind:
          "nav_goal"   → (x,y,yaw) を Nav2 へ送る（base_link 座標）
          "retreat"    → (linear_x, angular_z) を /cmd_vel_retreat へ直接送る
          "stop"       → 退避不可で停止（/cmd_vel_retreat にゼロを送る）
          "no_update"  → デッドゾーン内・変化なし（何もしない）
        """
        # 位置履歴を更新
        self.pos_history.update(person_x, person_y, t)

        dist          = math.hypot(person_x, person_y)
        closing_speed = self.pos_history.closing_speed_to_robot()
        person_speed  = self.pos_history.speed
        is_static     = self.static_detector.update(person_speed, t)

        # ── 優先1: 近接退避チェック（距離のみ）──────────────────
        # closing_speed によるトリガーは廃止。接近中の退避は優先1.5が担当する。
        retreat_needed = self.retreat_hyst.update(dist, 0.0)

        if retreat_needed:
            self.state = FollowState.RETREAT
            cmd = compute_retreat_cmd(
                person_x, person_y,
                self.params.retreat_speed,
                retreat_clearance_fn,
                self.params.retreat_check_clearance)
            if cmd is None:
                return PlannerOutput(kind="stop", retreat_blocked=True)
            lx, az = cmd
            return PlannerOutput(kind="retreat", linear_x=lx, angular_z=az)

        # ── 優先1.5: 進行路上退避 ────────────────────────────────
        # 試験員が動いており、かつロボットが試験員の進行経路上にいる場合に
        # compute_approach_escape_cmd() で速度指令を生成する。
        if person_speed > self.params.static_speed_threshold:
            vx, vy = self.pos_history.velocity
            if math.hypot(vx, vy) > 0.02:
                person_dir     = math.atan2(vy, vx)
                person_to_robot = math.atan2(-person_y, -person_x)
                diff = math.atan2(
                    math.sin(person_dir - person_to_robot),
                    math.cos(person_dir - person_to_robot))
                if abs(diff) < math.radians(self.params.path_approach_angle_deg):
                    self.state = FollowState.RETREAT
                    cmd = compute_approach_escape_cmd(
                        person_dir,
                        self.params.retreat_speed,
                        self.params.approach_perpendicular_deg,
                        retreat_clearance_fn,
                        self.params.retreat_check_clearance)
                    if cmd is not None:
                        lx, az = cmd
                        return PlannerOutput(kind="retreat", linear_x=lx, angular_z=az)
                    # 逃げ場なし → 試験員が通り過ぎるのを待つ
                    return PlannerOutput(kind="stop", retreat_blocked=True)

        # ── 優先2: 静止時再配置（4.3.4）────────────────────────
        if is_static:
            if self.state != FollowState.STATIC_REPOSITION:
                self.state = FollowState.STATIC_REPOSITION
                self._static_goal = None   # 候補点を再計算

            goal = self._compute_static_goal(
                person_x, person_y, corridor_width,
                costmap_clear_fn)
            if goal is None:
                return PlannerOutput(kind="no_update")
            gx, gy, gyaw = goal
            return self._nav_goal_output(gx, gy, gyaw)

        # ── 通常追従（4.3.3）────────────────────────────────────
        # 注意: static_detector.reset() はここで呼ばない。
        # StaticDetector 内で移動速度が speed_threshold を超えたとき
        # 自動的にタイマーがリセットされる（StaticDetector.update() 参照）。
        # ここで reset() を呼ぶと静止タイマーが毎周期リセットされ
        # 静止検知が永遠に発動しなくなる。
        self.state = FollowState.NORMAL_FOLLOW
        self._static_goal = None

        gx, gy, gyaw = compute_follow_goal(
            person_x, person_y,
            self.pos_history.velocity,
            self.params,
            corridor_width,
            self.fov_checker)
        return self._nav_goal_output(gx, gy, gyaw)

    def _compute_static_goal(
        self,
        person_x:       float,
        person_y:       float,
        corridor_width: float,
        costmap_clear_fn,
    ) -> Optional[Tuple[float, float, float]]:
        """
        静止時の再配置目標を計算する（4.3.4）。
        既に計算済みであれば再利用する（_static_goal キャッシュ）。
        """
        if self._static_goal is not None:
            return self._static_goal

        # 試験員の正面方向を推定（直前の移動方向から）
        facing_rad = self.pos_history.direction_rad

        # 候補点生成
        candidates = generate_candidate_points(
            person_x, person_y,
            facing_rad,
            radius=self.params.candidate_radius,
            count=self.params.candidate_count,
            work_exclusion_half_deg=self.params.work_front_exclusion_deg)

        if not candidates:
            return None

        # スコアリング
        for c in candidates:
            score_candidate(
                c, 0.0, 0.0,    # ロボットは原点（base_link 系）
                person_x, person_y,
                facing_rad,
                is_in_fov_fn=lambda dx, dy: self.fov_checker.is_in_fov(dx, dy),
                is_clear_fn=costmap_clear_fn)

        # 実行可能な候補を最小コスト順にソート
        feasible = [c for c in candidates if c.feasible]
        if not feasible:
            feasible = candidates   # 全候補不可でも最良を選ぶ（停止よりマシ）
        best = min(feasible, key=lambda c: c.score)

        goal_yaw = math.atan2(person_y - best.y, person_x - best.x)
        self._static_goal = (best.x, best.y, goal_yaw)
        return self._static_goal

    def _nav_goal_output(
        self, gx: float, gy: float, gyaw: float
    ) -> "PlannerOutput":
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

    def reset(self) -> None:
        """モード切替時などにリセットする"""
        self.state = FollowState.NORMAL_FOLLOW
        self.retreat_hyst.reset()
        self.static_detector.reset()
        self._prev_goal   = None
        self._static_goal = None


@dataclass
class PlannerOutput:
    kind:           str     = "no_update"   # "nav_goal"/"retreat"/"stop"/"no_update"
    # nav_goal 時
    goal_x:         float   = 0.0
    goal_y:         float   = 0.0
    goal_yaw:       float   = 0.0
    # retreat 時
    linear_x:       float   = 0.0
    angular_z:      float   = 0.0
    # 退避不可フラグ
    retreat_blocked: bool   = False
