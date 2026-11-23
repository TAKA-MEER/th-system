"""clearance_core.py — 角が描く円の中の障害物の確認の純粋コア（8-g P5b）。

正本は docs/plan/detailed/DetailedDesign-measure.md §6.2 P5b と
docs/plan/spec/Spec-measure.md §4.4。

- `measure_clearance` ノードが `/scan_filtered` を `base_link` へ変換した点と、
  局所コストマップ（`/local_costmap/costmap`）の占有セルから、
  `base_link` 中心・半径 0.389 m（機体の外接。0.55 m 角の半対角）の円の中に
  障害物があるかを判定する。
- スキャンが `arm_clearance_scan_max_age_ms` より古い・TF が取れないときは
  「確認できない」＝ clear=false（理由つき）。

ROS2 を import しない（`th_state/state_core.py` と同じ制約）。
"""
from __future__ import annotations

import math

# 機体の角が描く円の半径 [m]。0.55 m 角の半対角（外接）。Spec-measure.md §4.4。
CORNER_RADIUS_M = 0.389

# 通れないとみなす OccupancyGrid の値の下限（venue_nav_core の
# COSTMAP_BLOCKED_MIN と同じ値。致死(100)だけでなく内接(99)も通れない）。
COSTMAP_BLOCKED_MIN = 99

REASON_CLEAR = "clear"
REASON_TF_UNAVAILABLE = "tf_unavailable"
REASON_SCAN_STALE = "scan_stale"
REASON_OBSTACLE_SCAN = "obstacle_scan"
REASON_OBSTACLE_COSTMAP = "obstacle_costmap"


def points_in_circle(points_xy, radius: float = CORNER_RADIUS_M) -> bool:
    """点の列のうち 1 つでも半径 radius の円の中にあるか。"""
    for x, y in points_xy:
        if math.hypot(float(x), float(y)) <= radius:
            return True
    return False


def circle_blocked_by_costmap(grid, center_x: float, center_y: float,
                              radius: float = CORNER_RADIUS_M) -> bool:
    """占有セル（値 >= COSTMAP_BLOCKED_MIN）が円の中にあるか。

    grid: width/height/resolution/origin_x/origin_y/data を持つもの
    （OccupancyGrid の map 系）。center: 円の中心（base_link 中心を grid の
    座標系へ変換したもの）。範囲外のセルは見ない（未知は障害物ではない）。
    """
    res = float(grid.resolution)
    if res <= 0:
        return True
    ox, oy = float(grid.origin_x), float(grid.origin_y)
    w, h = int(grid.width), int(grid.height)
    ix0 = max(0, int(math.floor((center_x - radius - ox) / res)))
    ix1 = min(w - 1, int(math.floor((center_x + radius - ox) / res)))
    iy0 = max(0, int(math.floor((center_y - radius - oy) / res)))
    iy1 = min(h - 1, int(math.floor((center_y + radius - oy) / res)))
    data = grid.data
    r2 = radius * radius
    for iy in range(iy0, iy1 + 1):
        cy = oy + (iy + 0.5) * res
        for ix in range(ix0, ix1 + 1):
            if int(data[iy * w + ix]) < COSTMAP_BLOCKED_MIN:
                continue
            cx = ox + (ix + 0.5) * res
            if (cx - center_x) ** 2 + (cy - center_y) ** 2 <= r2:
                return True
    return False


def decide_clear(tf_ok: bool, scan_age_ms: float | None,
                 max_age_ms: float, scan_blocked: bool,
                 costmap_blocked: bool) -> tuple[bool, str]:
    """(clear, reason) を決める。判定順: TF → スキャンの鮮度 → スキャン → コストマップ。

    scan_age_ms が None（スキャン未受信）なら古い扱い（scan_stale）。
    """
    if not tf_ok:
        return False, REASON_TF_UNAVAILABLE
    if scan_age_ms is None or scan_age_ms > max_age_ms:
        return False, REASON_SCAN_STALE
    if scan_blocked:
        return False, REASON_OBSTACLE_SCAN
    if costmap_blocked:
        return False, REASON_OBSTACLE_COSTMAP
    return True, REASON_CLEAR

# 8-g P3（DetailedDesign-measure.md §6.4）: 並進の「進む先」の空き距離。
# 機体の形 0.55 m 角（半幅 BODY_HALF_M）が障害物に当たるまでの距離から
# リミッタの止まった位置の余裕 floor_margin_m を引いたもの。
BODY_HALF_M = 0.275


def _in_blind(angle_rad: float, blind_ranges_rad) -> bool:
    for a0, a1 in blind_ranges_rad or []:
        if a0 <= angle_rad <= a1:
            return True
    return False


def free_distance(points_xy, forward: bool, blind_ranges_rad=(),
                  floor_margin_m: float = 0.0,
                  max_range_m: float | None = None) -> float:
    """まっすぐ進む（forward=True）／退くときの free 距離 [m]。

    掃いた範囲 = 幅 [-0.275, 0.275] の corridor。障害物点が corridor 内で
    一番近いものまでの「形が当たるまでの距離」- floor_margin。見える範囲の外は
    空きとする（max_range_m で上限。返りの無い方向は端までを空き）。
    死角にかかる向きは 0（確かめられないものを空きと扱わない）。
    """
    direction = 1.0 if forward else -1.0
    # 死角判定: 進む向きの角が死角にかかるとき 0。
    ang = 0.0 if forward else math.pi
    if _in_blind(ang, blind_ranges_rad):
        return 0.0
    best = None
    for x, y in points_xy or []:
        x = float(x); y = float(y)
        if abs(y) > BODY_HALF_M:
            continue
        t = x * direction          # 進む向きの座標（>0 が先）
        if t <= 0:
            continue
        if best is None or t < best:
            best = t
    if best is None:
        free = float(max_range_m) if max_range_m else 10.0
    else:
        free = (best - BODY_HALF_M) - float(floor_margin_m)
    return max(0.0, free)


def free_distance_from_grid(grid, center_x: float, center_y: float,
                            forward: bool, dir_x: float, dir_y: float,
                            blind_ranges_rad=(), floor_margin_m: float = 0.0,
                            max_range_m: float | None = None) -> float:
    """コストマップ版。grid は占有セルを持つもの、中心 (center_x, center_y) は
    base_link 中心を grid 座標系へ変換したもの、(dir_x, dir_y) は
    base_link の +x を grid 座標系へ変換した単位ベクトル。"""
    ang = 0.0 if forward else math.pi
    if _in_blind(ang, blind_ranges_rad):
        return 0.0
    res = float(grid.resolution)
    if res <= 0:
        return 0.0
    direction = 1.0 if forward else -1.0
    best = None
    w, h = int(grid.width), int(grid.height)
    data = grid.data
    for iy in range(h):
        cy = float(grid.origin_y) + (iy + 0.5) * res
        for ix in range(w):
            if int(data[iy * w + ix]) < COSTMAP_BLOCKED_MIN:
                continue
            cx = float(grid.origin_x) + (ix + 0.5) * res
            dx, dy = cx - center_x, cy - center_y
            t = dx * dir_x + dy * dir_y
            lateral = -dx * dir_y + dy * dir_x
            scaled = t * direction
            if scaled <= 0 or abs(lateral) > BODY_HALF_M:
                continue
            if best is None or scaled < best:
                best = scaled
    if best is None:
        free = float(max_range_m) if max_range_m else 10.0
    else:
        free = (best - BODY_HALF_M) - float(floor_margin_m)
    return max(0.0, free)


def combine_free(scan_free: float, map_free: float,
                 scan_ok: bool, map_ok: bool) -> float:
    """スキャンとコストマップの free の小さい方。届いていない側は見ない
    （空き扱いではなく無視する。両方届かなければ 0 にはならないが、
    state_manager 側の鮮度で途絶は 0 に倒される）。"""
    vals = []
    if scan_ok:
        vals.append(scan_free)
    if map_ok:
        vals.append(map_free)
    if not vals:
        return 0.0
    return max(0.0, min(vals))
