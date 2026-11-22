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
