"""test_measure_clearance_core.py — th_measure/clearance_core の純コア試験。ROS 不要。

角が描く円（半径 0.389 m）の中の障害物判定・古いスキャン・TF 無しを縛る。
正本は docs/plan/detailed/DetailedDesign-measure.md §6.2 P5b と
docs/plan/spec/Spec-measure.md §4.4。
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'th_measure'))

from th_measure.clearance_core import (CORNER_RADIUS_M, circle_blocked_by_costmap,  # noqa: E402
                                       decide_clear, points_in_circle)


class _Grid:
    def __init__(self, w=200, h=200, res=0.05, ox=-5.0, oy=-5.0):
        self.width, self.height, self.resolution = w, h, res
        self.origin_x, self.origin_y = ox, oy
        self.data = [0] * (w * h)

    def paint(self, x0, x1, val, y0=-0.5, y1=0.5):
        for iy in range(self.height):
            y = self.origin_y + (iy + 0.5) * self.resolution
            for ix in range(self.width):
                x = self.origin_x + (ix + 0.5) * self.resolution
                if x0 <= x < x1 and y0 <= y <= y1:
                    self.data[iy * self.width + ix] = val


def test_circle_radius_is_outer_radius_of_half_meter_square():
    assert abs(CORNER_RADIUS_M - 0.389) < 1e-9


def test_points_in_circle_boundary():
    # 円の中（境界ちょうどを含む）と外。
    assert points_in_circle([(0.1, 0.0)])
    assert points_in_circle([(-CORNER_RADIUS_M, 0.0)])
    assert not points_in_circle([(CORNER_RADIUS_M + 0.01, 0.0)])
    assert not points_in_circle([])
    assert not points_in_circle([(2.0, 2.0)])


def test_costmap_circle_blocked():
    g = _Grid()
    g.paint(0.3, 0.35, 100)
    assert circle_blocked_by_costmap(g, 0.0, 0.0)
    g2 = _Grid()
    g2.paint(1.0, 1.1, 100)
    assert not circle_blocked_by_costmap(g2, 0.0, 0.0)
    g3 = _Grid()
    g3.paint(0.3, 0.35, 99)
    assert circle_blocked_by_costmap(g3, 0.0, 0.0)
    # 膨張域（99 未満）は通れる扱い（経路の先の確認と同じ値）。
    g4 = _Grid()
    g4.paint(0.3, 0.35, 98)
    assert not circle_blocked_by_costmap(g4, 0.0, 0.0)
    # 未知（-1）は障害物ではない。
    g5 = _Grid()
    g5.paint(0.3, 0.35, -1)
    assert not circle_blocked_by_costmap(g5, 0.0, 0.0)


def test_decide_clear_order():
    # TF 無し → tf_unavailable（スキャンの鮮度より先）。
    assert decide_clear(False, 10_000.0, 500.0, True, True) == (
        False, "tf_unavailable")
    # 古いスキャン（未受信 None を含む） → scan_stale。
    assert decide_clear(True, 501.0, 500.0, True, True) == (
        False, "scan_stale")
    assert decide_clear(True, None, 500.0, False, False) == (
        False, "scan_stale")
    # 境界ちょうどは古くない（超えたら）。
    assert decide_clear(True, 500.0, 500.0, False, False) == (True, "clear")
    # スキャンの障害物 → obstacle_scan（コストマップより先）。
    assert decide_clear(True, 10.0, 500.0, True, True) == (
        False, "obstacle_scan")
    # コストマップの障害物 → obstacle_costmap。
    assert decide_clear(True, 10.0, 500.0, False, True) == (
        False, "obstacle_costmap")
    # 空いていれば clear。
    assert decide_clear(True, 10.0, 500.0, False, False) == (True, "clear")


# ---------- 8-g P3: free_forward_m / free_backward_m ----------
from th_measure.clearance_core import (BODY_HALF_M, combine_free,  # noqa: E402
                                       free_distance)


def _blind_rad(deg_pairs):
    import math
    return tuple((math.radians(a), math.radians(b))
                 for a, b in zip(deg_pairs[::2], deg_pairs[1::2]))


def test_body_half_is_quarter_meter():
    assert abs(BODY_HALF_M - 0.275) < 1e-9


def test_free_forward_subtracts_margin():
    # 壁 0.9 m: 形が当たるまで 0.625 から余裕 0.3 を引く。
    free = free_distance([(0.9, 0.0)], True, floor_margin_m=0.3)
    assert abs(free - 0.325) < 1e-6
    # 余裕を引かないと 0.5 も通る（変異チェック d の裏付け）。
    assert free_distance([(0.9, 0.0)], True, floor_margin_m=0.0) > 0.5


def test_free_backward_uses_rear_points():
    assert abs(free_distance([(-0.9, 0.0)], False, floor_margin_m=0.3) - 0.325) < 1e-6
    # 前の点は後ろの free に数えない。
    assert abs(free_distance([(0.9, 0.0)], False, floor_margin_m=0.3,
                             max_range_m=10.0) - 10.0) < 1e-6


def test_free_zero_on_blind_stale_and_tf():
    # 死角にかかる向き（後ろ 180° が死角）→ 0。
    assert free_distance([(-0.9, 0.0)], False,
                         blind_ranges_rad=_blind_rad([170, 190]),
                         floor_margin_m=0.3) == 0.0
    assert combine_free(0.0, 0.0, False, False) == 0.0


def test_free_ignores_off_corridor_and_uses_min_of_sources():
    # corridor の外の点（y=0.5）は進む先に数えない。
    assert free_distance([(0.9, 0.5)], True, floor_margin_m=0.3,
                         max_range_m=10.0) == 10.0
    assert combine_free(10.0, 0.3, True, True) == 0.3
    assert combine_free(10.0, 0.0, True, False) == 10.0


# ---------- 8-g P4: 腕が振れる範囲の確認 ----------
from th_measure.clearance_core import (decide_side, point_in_region,  # noqa: E402
                                       region_overlaps_blind, rotate_polygon,
                                       validate_arm_request)

_RECT = [(0.0, -0.275), (1.0, -0.275), (1.0, 0.275), (0.0, 0.275)]


def test_arm_left_rotation_extends_to_plus_y():
    # LEFT（+90°）の範囲は左（+y）に出る。RECT は x 0〜1.0・y ±0.275。
    left = rotate_polygon(_RECT, 1.5707963267948966)
    assert point_in_region(0.0, 0.6, left)
    assert not point_in_region(0.6, 0.0, left)
    # RIGHT（−90°）は右（−y）に出る。
    right = rotate_polygon(_RECT, -1.5707963267948966)
    assert point_in_region(0.0, -0.6, right)
    assert not point_in_region(0.0, 0.6, right)
    # CURRENT は回さない。
    assert point_in_region(0.6, 0.0, _RECT)
    assert not point_in_region(0.0, 0.6, _RECT)


def test_arm_body_interior_points_not_counted():
    # 機体の内側（x 0.1・y 0）の点はどの向きでも数えない。
    for poly in (_RECT, rotate_polygon(_RECT, 1.5707963267948966),
                 rotate_polygon(_RECT, -1.5707963267948966)):
        assert not point_in_region(0.1, 0.0, poly)


def test_arm_outside_points_not_counted():
    # 範囲の外の点は数えない（x 1.1・y ±0.276 の外、背面）。
    assert not point_in_region(1.1, 0.0, _RECT)
    assert not point_in_region(0.6, 0.276, _RECT)
    assert not point_in_region(-0.6, 0.0, _RECT)


def test_arm_blind_only_is_unknown_and_visible_beats_blind():
    import math
    left = rotate_polygon(_RECT, math.pi / 2.0)
    blind_90 = ((math.radians(80.0), math.radians(100.0)),)
    # 死角だけにかかる → blind_sector。
    assert region_overlaps_blind(left, blind_90) is True
    assert decide_side(True, 10.0, 500.0, False, True) == (
        "UNKNOWN", "blind_sector")
    # 見える障害物と死角の両方 → BLOCKED（見える物があれば塞がり）。
    assert decide_side(True, 10.0, 500.0, True, True) == ("BLOCKED", "")
    # 右の範囲は 90° 帯に触れない。
    right = rotate_polygon(_RECT, -math.pi / 2.0)
    assert region_overlaps_blind(right, blind_90) is False


def test_arm_registry_blind_does_not_cover_rect_sides():
    # registry の blind_angle_ranges（4 セクタ）では、§3.5 の長方形のどの
    # 向きも死角にならない。近い部分（0.65 m 以内）の楔は判定から除く
    # （2026-11-23 ユーザー決定。Spec-measure.md §5.2 の既知のリスク）。
    import math
    blind = _blind_rad([-132.8, -117.5, -59.5, -38.9,
                        45.9, 61.5, 130.3, 143.5])
    for ang in (0.0, math.pi / 2.0, -math.pi / 2.0):
        assert region_overlaps_blind(
            rotate_polygon(_RECT, ang), blind, 0.65) is False
    # 右だけを覆う値では右だけが死角になる（試験 d と同じ値）。
    right_blind = _blind_rad([-100.0, -80.0])
    assert region_overlaps_blind(
        rotate_polygon(_RECT, -math.pi / 2.0), right_blind, 0.65) is True
    assert region_overlaps_blind(
        rotate_polygon(_RECT, math.pi / 2.0), right_blind, 0.65) is False


def test_arm_near_zero_makes_rect_blind():
    # arm_clearance_blind_near_m を 0 にすると、近い楔も数えて同じ範囲が
    # blind_sector になる（近い部分を除く判定を外す変異を掴む）。
    import math
    blind = _blind_rad([-132.8, -117.5, -59.5, -38.9,
                        45.9, 61.5, 130.3, 143.5])
    for ang in (0.0, math.pi / 2.0, -math.pi / 2.0):
        assert region_overlaps_blind(
            rotate_polygon(_RECT, ang), blind, 0.0) is True


def test_arm_wide_region_blind_far_from_center():
    # 範囲の幅を広げて 0.65 m より遠くで支柱の死角にかかる形（平均の向きは
    # 32.5° で死角に入らない）。代表の向きの案なら見逃す。
    import math
    blind = _blind_rad([-132.8, -117.5, -59.5, -38.9,
                        45.9, 61.5, 130.3, 143.5])
    wide = [(0.0, -0.275), (1.0, -0.275), (1.0, 0.9), (0.0, 0.9)]
    assert region_overlaps_blind(wide, blind, 0.65) is True


def test_arm_stale_is_all_unknown():
    # 古いスキャン（未受信・超え）→ 全部 UNKNOWN scan_stale。
    for age in (None, 500.1, 10000.0):
        assert decide_side(True, age, 500.0, False, False) == (
            "UNKNOWN", "scan_stale")
    # TF 無しも scan_stale（古い扱い）。
    assert decide_side(False, 10.0, 500.0, False, False) == (
        "UNKNOWN", "scan_stale")
    # 境界ちょうど（500）は古くない。
    assert decide_side(True, 500.0, 500.0, False, False) == ("CLEAR", "")


def test_arm_bad_argument_cases():
    # 頂点 2 つ → bad_argument。
    assert validate_arm_request([(0.0, 0.0), (1.0, 0.0)], ['LEFT']) is True
    # sides に UP → bad_argument。
    assert validate_arm_request(_RECT, ['UP']) is True
    # sides 空・重複・座標が有限でないも bad_argument。
    assert validate_arm_request(_RECT, []) is True
    assert validate_arm_request(_RECT, ['LEFT', 'LEFT']) is True
    assert validate_arm_request([(0.0, 0.0), (1.0, 0.0), (float('nan'), 0.0)],
                                ['LEFT']) is True
    assert validate_arm_request([(0.0, 0.0), (1.0, 0.0), (float('inf'), 0.0)],
                                ['LEFT']) is True
    # 正常な要求は bad_argument でない。
    assert validate_arm_request(_RECT, ['LEFT', 'RIGHT', 'CURRENT']) is False
