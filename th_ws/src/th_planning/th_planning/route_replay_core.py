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
from dataclasses import dataclass, replace
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


# ──────────────────────────────────────────────────────────────────
# 再生速度スケール（WS-9T: WebUI から /replay/speed_scale で可変にする）
# ──────────────────────────────────────────────────────────────────
def scale_replay_params(
    base: ReplayParams, ratio: float, cruise_min: float, yaw_min: float,
    lookahead_min: float,
) -> ReplayParams:
    """再生速度の比率 ratio(0..1) で cruise / yaw / lookahead を最小端〜base の間に
    線形補間する。

    base.cruise_speed_mps / base.max_yaw_rate_rps / base.lookahead_m を「最大端」、
    cruise_min / yaw_min / lookahead_min を「最小端」とし、ratio=0 で最小・
    ratio=1 で最大。範囲外の ratio はクランプする。arrive_dist_m / yaw_tol_rad は
    変えない。

    lookahead_m を cruise と一緒にスケールする理由（2026-09-05、実機で確認した
    ふらつき対策）: pure_pursuit の応答は実質的に「lookahead_m / cruise_speed_mps」
    （目標点までの先読み時間）で damping が決まる。以前は lookahead_m を固定
    (cruise に関係なく一定) にしていたため、cruise が上がるほど先読み時間が
    短くなり（例: 低速 0.40/0.15≈2.7s に対し高速 0.40/0.45≈0.9s）、高速再生時に
    応答が過敏になってふらついていた。lookahead も cruise と一緒にスケールする
    ことで、高速側の先読み時間を底上げする。lookahead_min は現行の低速側の
    挙動（既に問題ないと確認済み）をそのまま保つ値にすること。
    """
    r = 0.0 if ratio < 0.0 else 1.0 if ratio > 1.0 else float(ratio)
    return replace(
        base,
        cruise_speed_mps=cruise_min + (base.cruise_speed_mps - cruise_min) * r,
        max_yaw_rate_rps=yaw_min + (base.max_yaw_rate_rps - yaw_min) * r,
        lookahead_m=lookahead_min + (base.lookahead_m - lookahead_min) * r,
    )


def normalize_angle(a: float) -> float:
    """角度を [-pi, pi] に正規化する"""
    return math.atan2(math.sin(a), math.cos(a))


def ramp_toward(current: float, target: float, max_step: float) -> float:
    """current を target へ 1 周期分（最大 max_step）だけ近づける。

    再生指令を階段状に出すと指令と実ホイール速度が乖離し safety_monitor の
    runaway 判定（比 1.5 / 500ms）に引っかかる（実機 DRIVE_RUNAWAY）。毎ティック
    この関数で滑らかに追従させる（crawler_teleop.py の _ramp と同型）。
    """
    if max_step <= 0.0:
        return target
    if target > current + max_step:
        return current + max_step
    if target < current - max_step:
        return current - max_step
    return target


# ──────────────────────────────────────────────────────────────────
# 逆再生支援
# ──────────────────────────────────────────────────────────────────
def reverse_points(points: Sequence[Pose2D]) -> List[Pose2D]:
    """点列を逆順にし、各 yaw を normalize_angle(yaw + pi) に回す"""
    return [(x, y, normalize_angle(yaw + math.pi)) for (x, y, yaw) in reversed(points)]


# ──────────────────────────────────────────────────────────────────
# 現在地合わせ（WAIVER(demo): W-01 の実害対策）
# ──────────────────────────────────────────────────────────────────
def align_path_to_current(
    points: Sequence[Pose2D],
    recorded_start: Pose2D,
    current: Pose2D,
) -> List[Pose2D]:
    """記録経路を「記録始点 pose → 現在 pose」の 2D 剛体変換で移す。

    LiDAR による初期姿勢推定を省略している（W-01）ため、ロボットが記録始点に
    いない状態で再生を始めると、絶対 odom 座標の経路へ横から寄せる不自然な
    挙動になる。代わりに「今いる場所を記録の始点とみなす」ことで、記録した
    経路の"形"をその場から辿れるようにする。

    変換: 記録始点まわりに dtheta = current_yaw - recorded_start_yaw だけ回し、
    記録始点が現在位置に重なるよう平行移動する。返る点列の先頭は current に一致する。
    """
    sx, sy, syaw = recorded_start
    cx, cy, cyaw = current
    dtheta = normalize_angle(cyaw - syaw)
    cos_d, sin_d = math.cos(dtheta), math.sin(dtheta)
    out: List[Pose2D] = []
    for (x, y, yaw) in points:
        rx, ry = x - sx, y - sy
        nx = cx + rx * cos_d - ry * sin_d
        ny = cy + rx * sin_d + ry * cos_d
        out.append((nx, ny, normalize_angle(yaw + dtheta)))
    return out


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
    """ロボットが通過した点まで current_index を前進させる。

    セグメント idx→idx+1 について、ロボットがその終端の足を越えた
    (セグメントへの射影 t >= 1)、または次の点 idx+1 に arrive_dist_m 以内まで
    近づいたら index を 1 つ進める。進めなくなるまで繰り返す。

    旧実装はループが range(current_index, ...) で始まり、初回反復の
    `i > current_index` が必ず False → `else: break` で**常に current_index を
    返す no-op** だった。そのため replay_runner._from_index が 0 に固着し、
    pure_pursuit の探索窓が動かず経路の 2 点目以降へ進めなかった。
    """
    if not points:
        return 0
    idx = max(0, min(current_index, len(points) - 1))
    rx, ry, _ = robot
    ad_sq = params.arrive_dist_m * params.arrive_dist_m
    while idx + 1 < len(points):
        px, py, _ = points[idx]
        nx, ny, _ = points[idx + 1]
        seg_x, seg_y = nx - px, ny - py
        seg_sq = seg_x * seg_x + seg_y * seg_y
        near_next = (nx - rx) ** 2 + (ny - ry) ** 2 <= ad_sq
        passed = seg_sq > 1e-9 and ((rx - px) * seg_x + (ry - py) * seg_y) >= seg_sq
        if near_next or passed or seg_sq <= 1e-9:
            idx += 1
        else:
            break
    return idx


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
    rx, ry, ryaw = robot
    cos_y, sin_y = math.cos(ryaw), math.sin(ryaw)

    # 1. ルックアヘッド目標選択。ロボット後方の点はスキップする
    #    （後方点を目標にすると alpha≈±π で U ターンし、経路始点との往復に固着する）。
    target_index = last_index
    for i in range(from_index, len(points)):
        px, py, _ = points[i]
        dx, dy = px - rx, py - ry
        if dx * cos_y + dy * sin_y < 0.0:      # ロボット進行方向の背後
            continue
        if math.hypot(dx, dy) >= params.lookahead_m:
            target_index = i
            break
    tx, ty, _ = points[target_index]

    # 2. 到着判定（最終セグメントを過ぎた or 最終点まで arrive_dist_m 以内）
    lx, ly, _ = points[last_index]
    if from_index >= last_index or math.hypot(lx - rx, ly - ry) <= params.arrive_dist_m:
        return ReplayCommand(0.0, 0.0, last_index, True)

    # 3. pure-pursuit
    # WAIVER(demo): W-02（縮小）— この純コアは pose の出どころを問わない。
    # replay_runner が map フレーム経路なら slam_toolbox の map→base_link を、
    # odom フォールバックなら /odom を渡す（前者は走行中も連続補正あり）。
    bearing = math.atan2(ty - ry, tx - rx)
    alpha = normalize_angle(bearing - ryaw)
    w = 2.0 * params.cruise_speed_mps * math.sin(alpha) / params.lookahead_m
    w = max(-params.max_yaw_rate_rps, min(params.max_yaw_rate_rps, w))
    v = params.cruise_speed_mps * max(0.2, math.cos(alpha))
    return ReplayCommand(v, w, target_index, False)
