"""
route_replay_core.py
====================
教示経路の再生コアロジック（ROS2 非依存）

記録した点列を pure-pursuit で辿り、(v, w) 指令を返す。逆再生は点列を
反転 (各 yaw に pi 加算) することで実現する。ROS2 インポートを一切含まない。

純コアの線速度は一律 cruise_speed_mps とし、旋回が大きいほど落としたい
場合に備え cos(alpha) 係数をかけられる式を採用する:
    v = cruise_speed_mps * max(0.2, cos(alpha))
alpha は 0 に近いほど真直ぐ、±pi ほど大きく旋回する。cos(alpha) が負になる
極端な真後ろ目標でも最小 0.2 倍に絞って停止させないよう下駄を履かせる。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Sequence, Tuple

Pose2D = Tuple[float, float, float]   # (x, y, theta[rad])


# ──────────────────────────────────────────────────────────────────
# パラメータ群（外部から差し替え可能）
# ──────────────────────────────────────────────────────────────────
@dataclass
class ReplayParams:
    lookahead_m:        float = 0.40   # m 目標点選択の前方参照距離
    cruise_speed_mps:   float = 0.25   # m/s 巡航速度
    max_yaw_rate_rps:   float = 0.8    # rad/s 最大旋回速度
    arrive_dist_m:      float = 0.20   # m 最終点にこの距離まで近づいたら到着
    yaw_tol_rad:        float = 0.10   # rad 超信地旋回の完了判定


@dataclass
class ReplayCommand:
    v: float
    w: float
    target_index: int
    arrived: bool


def normalize_angle(a: float) -> float:
    """角度を [-pi, pi] に正規化する"""
    return math.atan2(math.sin(a), math.cos(a))


# ──────────────────────────────────────────────────────────────────
# 逆再生支援
# ──────────────────────────────────────────────────────────────────
def reverse_points(points: Sequence[Pose2D]) -> List[Pose2D]:
    """点列を逆順にし、各 yaw を normalize_angle(yaw + pi) に回す"""
    return [(x, y, normalize_angle(yaw + math.pi)) for (x, y, yaw) in reversed(points)]


# ──────────────────────────────────────────────────────────────────
# 旋回
# ──────────────────────────────────────────────────────────────────
def rotate_toward(
    current_yaw: float, target_yaw: float, params: ReplayParams,
) -> Tuple[float, bool]:
    """超信地旋回 (v=0) で target_yaw へ向ける。

    返り値: (w, done)。done は角度差の絶対値 <= yaw_tol_rad。そのとき w=0.0。
    w は差の符号 × max_yaw_rate_rps。
    """
    diff = normalize_angle(target_yaw - current_yaw)
    if abs(diff) <= params.yaw_tol_rad:
        return (0.0, True)
    return (math.copysign(params.max_yaw_rate_rps, diff), False)


# ──────────────────────────────────────────────────────────────────
# Pure Pursuit
# ──────────────────────────────────────────────────────────────────
def advance_index(
    robot: Pose2D, points: Sequence[Pose2D], current_index: int, params: ReplayParams,
) -> int:
    """ロボットが通過した点まで current_index を前進させる小関数。

    現在の目標点から前へ、ロボットの後方/側方に既にある点はスキップする。
    単純な実装: 現在点より後の点で、ロボット位置を「過ぎた」点を指し進める。
    """
    rx, ry, _ = robot
    new_index = current_index
    for i in range(current_index, len(points)):
        px, py, _ = points[i]
        # ロボット位置から前方にある点 (i > current_index) で、
        # その点を超えて進んだ (ロボットが目標点より先にある / 横を通過) 場合は前進
        dx, dy = px - rx, py - ry
        if i > current_index and dx * dx + dy * dy < params.arrive_dist_m * params.arrive_dist_m:
            new_index = i
        else:
            break
    return new_index


def pure_pursuit(
    robot: Pose2D,
    points: Sequence[Pose2D],
    params: ReplayParams,
    from_index: int = 0,
) -> ReplayCommand:
    """記録点列に沿って (v, w) を返す pure-pursuit。

    1. from_index 以降で、ロボット位置から lookahead_m 以上離れた最初の点を
       目標にし、無ければ最終点を目標にする。target_index はその点の添字。
    2. 最終点までの距離 <= arrive_dist_m なら (0, 0, last, True)。
    3. それ以外:
         alpha = 目標点へのベアリングの base_link 相対角
         w = clamp(2 * cruise * sin(alpha) / lookahead, ±max_yaw_rate_rps)
         v = cruise * max(0.2, cos(alpha))   # 旋回が大きいほど落とす
    """
    if not points:
        return ReplayCommand(0.0, 0.0, 0, True)

    last_index = len(points) - 1
    rx, ry, _ = robot

    # 1. ルックアヘッド目標選択
    target_index = last_index
    for i in range(from_index, len(points)):
        px, py, _ = points[i]
        if math.hypot(px - rx, py - ry) >= params.lookahead_m:
            target_index = i
            break
    tx, ty, _ = points[target_index]

    # 2. 到着判定（最終点までの距離）
    lx, ly, _ = points[last_index]
    if math.hypot(lx - rx, ly - ry) <= params.arrive_dist_m:
        return ReplayCommand(0.0, 0.0, last_index, True)

    # 3. pure-pursuit
    # WAIVER(demo): W-02 走行中の自己位置補正なし（オドメトリ＋ジャイロの /odom だけで辿る）
    bearing = math.atan2(ty - ry, tx - rx)
    alpha = normalize_angle(bearing - robot[2])
    w = 2.0 * params.cruise_speed_mps * math.sin(alpha) / params.lookahead_m
    w = max(-params.max_yaw_rate_rps, min(params.max_yaw_rate_rps, w))
    v = params.cruise_speed_mps * max(0.2, math.cos(alpha))
    return ReplayCommand(v, w, target_index, False)
