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
    arrival_xy_tol_m: float = 0.12


# SM-3.1.2-118: 盤前移動・呼び寄せ・待機場所へ戻る途中は、PAUSE を含むどの状態でも
# 行き先を選べない。th_state の transitions.yaml T-NAV-goto-reject と同じ集合。
# 選べる状態（AT_HOME の IDLE_H/PAUSE・AT_PANEL・IDLE など）の列挙ではなく、
# 選べない側を列挙する（/system/state 未受信の空モードで拒否して起動直後の
# 呼び出しを巻き込まないため）。
GOTO_BLOCKED_MODES = frozenset({'PANEL_NAV', 'SUMMON', 'HOME_NAV'})


def select_pin_allowed(mode: str) -> bool:
    """/onsite/select_pin（行き先の保留）を受け付けてよいモードか。"""
    return mode not in GOTO_BLOCKED_MODES


def select_pin_reject_message(mode: str, state: str) -> str:
    """select_pin を拒否するときの利用者向け理由（日本語）。"""
    if state == 'PAUSE':
        return '一時停止中は行き先を選べません。「走行」で続きを走るか、「中断」してから選んでください'
    return '移動中は行き先を選べません。着いてから、または「中断」してから選んでください'


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


# Nav2 が SUCCEEDED を返したとき、機体がゴールからこれ以上離れていたら鵜呑みにしない [m]。
# 通常の許容差(arrival_xy_tol_m・Nav2 の goal checker)よりずっと大きい値にして、
# 正規の到着を弾かない。
SPURIOUS_SUCCESS_DIST_M = 0.5


def success_is_plausible(robot_xy, goal, max_dist: float = SPURIOUS_SUCCESS_DIST_M) -> bool:
    """Nav2 の FollowPath が SUCCEEDED を返したのが本当に到着か（純関数）。

    2026-10-09 実機: Nav2 controller の tf バッファの map→odom が約 57 秒止まり
    (`Transform data too old`)、ゴールを変換できないまま「Reached the goal!」と
    即成功を返した。venue_navigator がそれを信じて、ゴールの 1.7 m 手前で
    「待機場所に着いた」として向き合わせを始め、戻れなかった。
    位置が分からない(None)ときは判定できないので信じる(従来どおり)。
    """
    if robot_xy is None or goal is None:
        return True
    return math.hypot(robot_xy[0] - float(goal['x']),
                      robot_xy[1] - float(goal['y'])) <= max_dist


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


# ── 塞がれた経路の先の確認（Spec-onsite §6.3。2026-10-12）────────────────
# 「通れるようになった」＝残りの経路の先が、いまの障害物マップで空いていること。
# 走り出せたかどうかでは判定しない（実機 2026-10-10 は障害物へ向かって走り直す往復）。

# 経路の先を確認する距離 [m]（機体の現在位置に最も近い経路上の点から経路に沿って）。
PATH_AHEAD_CHECK_M = 1.5

# 通れない値の下限（nav_msgs/OccupancyGrid の値。Nav2 costmap の公開値）。
# 致死(100)だけでなく内接(99)も通れない扱いにする。機体は 0.55 m 角で、
# 経路中心が内接域なら機体は壁に触れるため。内接より低い値(膨張域 1〜98)は通れる。
# 未知(-1)は障害物ではないので通れる扱い（経路計画が未知を許す設定と揃える）。
COSTMAP_BLOCKED_MIN = 99

# 障害物マップがこれ以上更新されていなければ「確認できない＝空いていない」[s]。
COSTMAP_STALE_S = 5.0


def _cell_blocked(grid, x: float, y: float) -> bool:
    """map 座標 (x, y) のセルが通れないか。範囲外は確認できないので True。

    grid: width/height/resolution/origin_x/origin_y/data を持つもの。
    原点の向きは無回転（Nav2 の costmap は常に無回転）として扱う。
    """
    res = float(grid.resolution)
    ix = int(math.floor((x - float(grid.origin_x)) / res))
    iy = int(math.floor((y - float(grid.origin_y)) / res))
    if ix < 0 or iy < 0 or ix >= grid.width or iy >= grid.height:
        return True
    return int(grid.data[iy * grid.width + ix]) >= COSTMAP_BLOCKED_MIN


def path_ahead_clear(path_xy, robot_xy, grid,
                     ahead_m: float = PATH_AHEAD_CHECK_M) -> bool:
    """経路の先 ahead_m [m] が空いているか（純関数）。

    path_xy: 経路の (x, y) の列（map 系）。robot_xy: 機体位置 (x, y)。
    機体に最も近い経路上の点から経路に沿って ahead_m まで、点の間も
    解像度の半分刻みで補間して、通れないセルが 1 つでもあれば False。
    経路が空・機体位置不明・マップ無しは確認できないので False。
    """
    if grid is None or robot_xy is None or not path_xy:
        return False
    pts = [(float(p[0]), float(p[1])) for p in path_xy]
    rx, ry = float(robot_xy[0]), float(robot_xy[1])
    start = min(range(len(pts)),
                key=lambda i: (pts[i][0] - rx) ** 2 + (pts[i][1] - ry) ** 2)
    step = max(float(grid.resolution) * 0.5, 1e-3)
    if _cell_blocked(grid, *pts[start]):
        return False
    travelled = 0.0
    for i in range(start, len(pts) - 1):
        ax, ay = pts[i]
        bx, by = pts[i + 1]
        seg = math.hypot(bx - ax, by - ay)
        if seg <= 0.0:
            continue
        remain = ahead_m - travelled
        end = min(seg, remain)
        n = max(1, int(math.ceil(end / step)))
        for k in range(1, n + 1):
            d = end * k / n
            t = d / seg
            if _cell_blocked(grid, ax + (bx - ax) * t, ay + (by - ay) * t):
                return False
        travelled += end
        if travelled >= ahead_m - 1e-9:
            break
    return True


# ── 引いた経路を機体の形で検査する（Spec-onsite §6.1.1。2026-10-22）─────────
# Nav2 の経路計画は機体を点（内接円 0.275 m 相当）として扱うので、0.55 m 角の機体が
# 通れない狭い隙間にも経路を引く。走り出す前に、経路の各点に機体の四角形をその点の
# 進行方向に置いて、経路計画の地図（/global_costmap/costmap）の通れない区画に当たらないか
# 確かめる。詳細は DetailedDesign-onsite.md「経路の機体形状検査」。

# 機体の半幅 [m]。nav2_params.yaml の global_costmap の footprint（±0.275）と同じ値。
FOOTPRINT_HALF_M = 0.275

# 当たりとみなす値の下限（OccupancyGrid の値）。致死(100)だけを当たりにする。
# 内接(99)は「中心が壁から内接円（0.275 m）以内」の印で、四角形そのものを置いて見る
# この検査で 99 まで含めると内接円ぶん二重に太る。未知(-1)は当たりにしない
# （経路計画が未知を許す設定と揃える。会場地図の外周の未知で通れる通路を拒否しない）。
FOOTPRINT_BLOCKED_MIN = 100

# 経路に沿って機体を置く間隔 [m]。機体の幅(0.55)より十分小さく、置いた四角形どうしが
# 大きく重なるので、細い障害物のすり抜けは無い。0.05 刻みの全点より約 3 倍速い。
FOOTPRINT_STRIDE_M = 0.15

# 進行方向を決めるとき、前後それぞれこの距離の点を結ぶ [m]（経路の 8 近傍のギザギザで
# 向きが揺れて、斜めの通路を誤って拒否しないように均す）。
FOOTPRINT_HEADING_WINDOW_M = 0.10

# 出発点と行き先は機体がその場で回るので、向きを変えて当たらない向きが 1 つでもあれば
# 通す。その場回転で試す向きの数（360 度を等分）。
FOOTPRINT_SPIN_HEADINGS = 16


def footprint_collision_at(grid, x: float, y: float, yaw: float,
                           half: float = FOOTPRINT_HALF_M):
    """map 座標 (x, y)・向き yaw に一辺 2*half の正方形を置いたときの当たり（純関数）。

    戻り値: 当たらなければ None、当たれば最初に見つけた通れないセルの
    (セル中心 x, セル中心 y, 値)。セルは解像度ぶんの正方形として扱い、機体の正方形と
    重なれば（辺が触れるだけは除く）当たり。セル中心だけで見ると壁の厚みの半セルぶん
    （0.025 m）甘くなり、壁面の間隔が機体幅 0.55 m に足りない隙間を通してしまう。
    中心 (x, y) が地図の外なら確認できないので当たり（値 -1）。地図の外にはみ出す
    部分は見ない。grid: width/height/resolution/origin_x/origin_y/data を持つもの。
    """
    res = float(grid.resolution)
    ox, oy = float(grid.origin_x), float(grid.origin_y)
    w, h = int(grid.width), int(grid.height)
    cx = int(math.floor((x - ox) / res))
    cy = int(math.floor((y - oy) / res))
    if cx < 0 or cy < 0 or cx >= w or cy >= h:
        return (x, y, -1)
    c, s = math.cos(yaw), math.sin(yaw)
    ext = half * (abs(c) + abs(s))
    ix0 = max(0, int(math.floor((x - ext - ox) / res)))
    ix1 = min(w - 1, int(math.floor((x + ext - ox) / res)))
    iy0 = max(0, int(math.floor((y - ext - oy) / res)))
    iy1 = min(h - 1, int(math.floor((y + ext - oy) / res)))
    data = grid.data
    lim = half + 0.5 * res * (abs(c) + abs(s)) - 1e-9   # 機体の辺 ± セルの射影の半分
    for iy in range(iy0, iy1 + 1):
        row = iy * w
        dy = oy + (iy + 0.5) * res - y
        for ix in range(ix0, ix1 + 1):
            v = int(data[row + ix])
            if v < FOOTPRINT_BLOCKED_MIN:
                continue
            dx = ox + (ix + 0.5) * res - x
            if abs(dx * c + dy * s) <= lim and abs(-dx * s + dy * c) <= lim:
                return (ox + (ix + 0.5) * res, oy + (iy + 0.5) * res, v)
    return None


def _spin_clear(grid, x: float, y: float, half: float) -> bool:
    """その場で向きを変えて、当たらない向きが 1 つでもあるか。"""
    for k in range(FOOTPRINT_SPIN_HEADINGS):
        yaw = -math.pi + 2.0 * math.pi * k / FOOTPRINT_SPIN_HEADINGS
        if footprint_collision_at(grid, x, y, yaw, half) is None:
            return True
    return False


def _arclength_sampler(pts, stride: float):
    """経路の弧長 s から位置を返す関数と、stride 刻みの弧長列（終点を含む）、総長。"""
    cum = [0.0]
    for i in range(1, len(pts)):
        cum.append(cum[-1] + math.hypot(pts[i][0] - pts[i - 1][0],
                                         pts[i][1] - pts[i - 1][1]))
    total = cum[-1]

    def at(s: float):
        s = max(0.0, min(total, s))
        lo, hi = 0, len(pts) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if cum[mid] <= s:
                lo = mid
            else:
                hi = mid
        seg = cum[hi] - cum[lo]
        t = 0.0 if seg <= 0.0 else (s - cum[lo]) / seg
        return (pts[lo][0] + (pts[hi][0] - pts[lo][0]) * t,
                pts[lo][1] + (pts[hi][1] - pts[lo][1]) * t)

    s_list = [k * stride for k in range(int(total // stride) + 1)]
    if total - s_list[-1] > 1e-6:
        s_list.append(total)
    return s_list, total, at


def path_footprint_collision(path_xy, grid, goal_yaw=None,
                             half: float = FOOTPRINT_HALF_M,
                             stride: float = FOOTPRINT_STRIDE_M):
    """計画した経路を機体の形で検査する（純関数）。

    path_xy: 経路の (x, y) の列（map 系）。goal_yaw: 行き先の最終向き [rad]（無ければ見ない）。
    戻り値: 全点で当たらなければ None。当たれば最初の当たりの dict
      {'index': 間引いた点の番号, 'x', 'y', 'yaw': 置いた向き, 'cell': (x, y, 値)}。
    ・各点の向きは前後 FOOTPRINT_HEADING_WINDOW_M の点を結ぶ向き（進行方向）。
    ・最初の点（出発点）と最後の点（行き先）は機体がその場で回るので、進行方向で
      当たっても、向きを変えて当たらない向きが 1 つでもあれば通す。
    ・最後の点は、行き先の最終向き goal_yaw でも当たらないこと（これは必ず見る）。
    ・grid が None／経路が空なら確認できないので None（呼び側が扱いを決める）。
    """
    if grid is None or not path_xy:
        return None
    pts = [(float(p[0]), float(p[1])) for p in path_xy]
    s_list, total, at = _arclength_sampler(pts, max(stride, 1e-3))
    last = len(s_list) - 1
    win = FOOTPRINT_HEADING_WINDOW_M
    for i, s in enumerate(s_list):
        x, y = at(s)
        if total > 1e-6:
            ax, ay = at(s - win)
            bx, by = at(s + win)
            yaw = math.atan2(by - ay, bx - ax)
        else:
            yaw = 0.0 if goal_yaw is None else float(goal_yaw)
        hit = footprint_collision_at(grid, x, y, yaw, half)
        if hit is not None and i in (0, last) and _spin_clear(grid, x, y, half):
            hit = None
        if hit is not None:
            return {'index': i, 'x': x, 'y': y, 'yaw': yaw, 'cell': hit}
        if i == last and goal_yaw is not None:
            hit = footprint_collision_at(grid, x, y, float(goal_yaw), half)
            if hit is not None:
                return {'index': i, 'x': x, 'y': y, 'yaw': float(goal_yaw),
                        'cell': hit}
    return None
