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
