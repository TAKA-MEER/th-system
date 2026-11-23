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


# 8-g P4（DetailedDesign-measure.md §3.5・§6.5）: 腕が振れる範囲の確認。
# `region` は回ったあとの base_link 基準（x 前・y 左 [m]）。LEFT は機体の
# 中心まわりに +90°、RIGHT は −90° 回した位置を、回る前の今の位置から
# 確かめる。機体の形 0.55 m 角の内側は見ない（除かないと腕の取り付け部
# のそばの死角マスクのかかったパイプで毎回 UNKNOWN になる）。
RESULT_CLEAR = "CLEAR"
RESULT_BLOCKED = "BLOCKED"
RESULT_UNKNOWN = "UNKNOWN"
REASON_BLIND_SECTOR = "blind_sector"

SIDE_ROTATION_RAD = {
    "LEFT": math.pi / 2.0,
    "RIGHT": -math.pi / 2.0,
    "CURRENT": 0.0,
}
_ARM_SIDES = frozenset(SIDE_ROTATION_RAD)

# 死角判定の標本化の刻み [m]。範囲を細かく標本化し、laser_link から見た
# 向き（laser と base の向きは同じ前提。P3 の free_*_m の死角判定と同じ）
# が blind_angle_ranges に入る標本があるかを確かめる。
_BLIND_SAMPLE_STEP_M = 0.05
_BLIND_EDGE_STEP_M = 0.025


def rotate_point(x: float, y: float, angle_rad: float) -> tuple[float, float]:
    """点 (x, y) を原点まわりに angle_rad 回した点。"""
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    return (x * c - y * s, x * s + y * c)


def rotate_polygon(points, angle_rad: float) -> list[tuple[float, float]]:
    """多角形の頂点の列を原点まわりに回したもの。"""
    return [rotate_point(float(x), float(y), angle_rad) for x, y in points]


def inside_body_square(x: float, y: float) -> bool:
    """機体の形 0.55 m 角（|x|・|y| ≤ 0.275 m）の内側か。"""
    return abs(float(x)) <= BODY_HALF_M and abs(float(y)) <= BODY_HALF_M


def point_in_polygon(x: float, y: float, polygon) -> bool:
    """多角形の中か（境界ちょうどを含む）。ray casting。"""
    poly = [(float(px), float(py)) for px, py in polygon]
    n = len(poly)
    if n < 3:
        return False
    x, y = float(x), float(y)
    # 境界（辺上）も中として数える。
    for i in range(n):
        x0, y0 = poly[i]
        x1, y1 = poly[(i + 1) % n]
        dx, dy = x1 - x0, y1 - y0
        cross = (x - x0) * dy - (y - y0) * dx
        if abs(cross) <= 1e-9:
            dot = (x - x0) * dx + (y - y0) * dy
            if 0.0 <= dot <= dx * dx + dy * dy:
                return True
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if (yi > y) != (yj > y):
            x_int = xi + (y - yi) * (xj - xi) / (yj - yi)
            if x < x_int:
                inside = not inside
        j = i
    return inside


def point_in_region(x: float, y: float, polygon) -> bool:
    """範囲（多角形）の内側で、機体の内側を除いた所か。"""
    if inside_body_square(x, y):
        return False
    return point_in_polygon(x, y, polygon)


def any_point_in_region(points_xy, polygon) -> bool:
    """障害物点のうち 1 つでも範囲（機体の内側を除く）の中にあるか。"""
    for x, y in points_xy or []:
        if point_in_region(float(x), float(y), polygon):
            return True
    return False


def _sample_region(polygon):
    """範囲を細かく標本化した点の列（機体の内側の点は含めない）。

    bounding box の格子（_BLIND_SAMPLE_STEP_M）と、辺に沿った標本
    （_BLIND_EDGE_STEP_M。細い帯の目安）・頂点を含む。
    """
    poly = [(float(px), float(py)) for px, py in polygon]
    if len(poly) < 3:
        return []
    samples = []
    xs = [p[0] for p in poly]
    ys = [p[1] for p in poly]
    x, y = min(xs), min(ys)
    while y <= max(ys) + 1e-9:
        xx = x
        while xx <= max(xs) + 1e-9:
            if point_in_region(xx, y, poly):
                samples.append((xx, y))
            xx += _BLIND_SAMPLE_STEP_M
        y += _BLIND_SAMPLE_STEP_M
    n = len(poly)
    for i in range(n):
        x0, y0 = poly[i]
        x1, y1 = poly[(i + 1) % n]
        length = math.hypot(x1 - x0, y1 - y0)
        steps = max(1, int(math.ceil(length / _BLIND_EDGE_STEP_M)))
        for k in range(steps + 1):
            t = k / steps
            ex, ey = x0 + (x1 - x0) * t, y0 + (y1 - y0) * t
            if point_in_region(ex, ey, poly):
                samples.append((ex, ey))
    return samples


def region_overlaps_blind(polygon, blind_ranges_rad) -> bool:
    """範囲が LiDAR の死角にかかるか。

    範囲を細かく標本化し（機体の内側を除く）、標本の平均位置の向きを
    代表向きとして、laser_link から見た向き（laser と base の向きは同じ
    前提。P3 の free_*_m と同じ）が blind_angle_ranges に入るかで決める。

    標本のどれか 1 つでも死角に入るかで決めると、矩形の隅（機体の斜め前）が
    支柱の死角セクタ（registry の 4 セクタ）に当たって、どの向きも
    `blind_sector` になり、腕の確認が常に `UNKNOWN` になる（§3.5 が
    機体の内側を除いて「毎回 UNKNOWN」を避けた意図に反する）。
    範囲全体が死角の中にあるときは代表向きも死角に入るため、この
    決め方でも拾える。
    """
    if not blind_ranges_rad:
        return False
    samples = _sample_region(polygon)
    if not samples:
        return False
    mx = sum(x for x, _ in samples) / len(samples)
    my = sum(y for _, y in samples) / len(samples)
    if math.hypot(mx, my) < 1e-9:
        # 代表向きが取れない形（原点のまわりを囲む範囲など）は、標本の
        # どれかが死角に入るかで決める。
        return any(_in_blind(math.atan2(y, x), blind_ranges_rad)
                   for x, y in samples)
    return _in_blind(math.atan2(my, mx), blind_ranges_rad)


def polygon_blocked_by_costmap(grid, origin_x: float, origin_y: float,
                               basis_x, basis_y, polygon) -> bool:
    """占有セル（値 >= COSTMAP_BLOCKED_MIN）が範囲の中にあるか。

    grid: width/height/resolution/origin_x/origin_y/data を持つもの。
    (origin_x, origin_y) は base_link 原点を grid の座標系へ変換したもの、
    basis_x・basis_y は base_link の +x・+y を grid の座標系へ変換した
    単位ベクトル。範囲外のセルは見ない（未知は障害物ではない）。
    セルの中心を base_link 座標へ戻して判定する（境界の言い換えで
    ずれないため）。
    """
    res = float(grid.resolution)
    if res <= 0:
        return True
    poly = [(float(px), float(py)) for px, py in polygon]
    if len(poly) < 3:
        return False
    # 範囲の corners を grid 座標へ変換して bounding box を出す。
    gx = [origin_x + basis_x[0] * px + basis_y[0] * py for px, py in poly]
    gy = [origin_y + basis_x[1] * px + basis_y[1] * py for px, py in poly]
    ox, oy = float(grid.origin_x), float(grid.origin_y)
    w, h = int(grid.width), int(grid.height)
    ix0 = max(0, int(math.floor((min(gx) - ox) / res)))
    ix1 = min(w - 1, int(math.floor((max(gx) - ox) / res)))
    iy0 = max(0, int(math.floor((min(gy) - oy) / res)))
    iy1 = min(h - 1, int(math.floor((max(gy) - oy) / res)))
    data = grid.data
    for iy in range(iy0, iy1 + 1):
        cy = oy + (iy + 0.5) * res
        for ix in range(ix0, ix1 + 1):
            if int(data[iy * w + ix]) < COSTMAP_BLOCKED_MIN:
                continue
            cx = ox + (ix + 0.5) * res
            dx, dy = cx - origin_x, cy - origin_y
            bx = dx * basis_x[0] + dy * basis_x[1]
            by = dx * basis_y[0] + dy * basis_y[1]
            if point_in_region(bx, by, poly):
                return True
    return False


def decide_side(tf_ok: bool, scan_age_ms: float | None, max_age_ms: float,
                blocked: bool, blind_hit: bool) -> tuple[str, str]:
    """1 つの向きの (result, unknown_reason) を決める。判定順: スキャンの
    鮮度（古い・届いていない・TF が取れない）→ 障害物 → 死角。
    見える物があれば死角があっても塞がり。UNKNOWN 以外の理由は空文字。
    """
    if not tf_ok or scan_age_ms is None or scan_age_ms > max_age_ms:
        return RESULT_UNKNOWN, REASON_SCAN_STALE
    if blocked:
        return RESULT_BLOCKED, ""
    if blind_hit:
        return RESULT_UNKNOWN, REASON_BLIND_SECTOR
    return RESULT_CLEAR, ""


def validate_arm_request(points, sides) -> bool:
    """引数が bad_argument か。

    `region` の頂点が 3 未満・座標が有限でない／`sides` が空・
    LEFT／RIGHT／CURRENT 以外・重複 → True。
    """
    pts = list(points) if points is not None else []
    if len(pts) < 3:
        return True
    for px, py in pts:
        if not (math.isfinite(float(px)) and math.isfinite(float(py))):
            return True
    sds = list(sides) if sides is not None else []
    if not sds:
        return True
    seen = set()
    for s in sds:
        if s not in _ARM_SIDES or s in seen:
            return True
        seen.add(s)
    return False
