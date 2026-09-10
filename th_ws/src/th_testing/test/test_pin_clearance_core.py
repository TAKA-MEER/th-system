"""pin_clearance_core の純ロジックテスト（ROS2 不要）"""
import math
import os
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), '..', '..', 'th_onsite'))

from th_onsite.pin_clearance_core import (  # noqa: E402
    clearance_verdict, nearest_lethal_distance, retreat_pose)


def _grid(w, h, occupied_cells):
    """occupied_cells: (i, j) の集合を 100 に、それ以外 0。"""
    g = [0] * (w * h)
    for (i, j) in occupied_cells:
        g[j * w + i] = 100
    return g


class TestNearestLethalDistance:
    # 10x10 セル・resolution 0.1m・origin (0,0) → world 0..1m
    W = H = 10
    RES = 0.1

    def test_wall_adjacent(self):
        # (5,5) が占有。点 (0.55, 0.5)=セル中心 → 距離 0
        g = _grid(self.W, self.H, {(5, 5)})
        d = nearest_lethal_distance(g, self.W, self.H, self.RES, 0, 0,
                                    0.55, 0.55, 0.5)
        assert d < 0.05

    def test_wall_0p3_away(self):
        # (8,5) 占有。点 (0.55, 0.55) から x 方向に 3 セル ≈ 0.3m
        g = _grid(self.W, self.H, {(8, 5)})
        d = nearest_lethal_distance(g, self.W, self.H, self.RES, 0, 0,
                                    0.55, 0.55, 0.6)
        assert 0.25 < d < 0.35

    def test_no_wall_in_range(self):
        g = _grid(self.W, self.H, {(9, 9)})
        d = nearest_lethal_distance(g, self.W, self.H, self.RES, 0, 0,
                                    0.15, 0.15, 0.3)
        assert d == float('inf')

    def test_empty_grid(self):
        assert nearest_lethal_distance([], 0, 0, 0.1, 0, 0, 0, 0, 1.0) == float('inf')

    def test_unknown_cells_ignored(self):
        g = [-1] * (self.W * self.H)
        assert nearest_lethal_distance(g, self.W, self.H, self.RES, 0, 0,
                                       0.5, 0.5, 1.0) == float('inf')


class TestClearanceVerdict:
    def test_ok(self):
        assert clearance_verdict(0.5, 0.45) == 'ok'
        assert clearance_verdict(0.45, 0.45) == 'ok'

    def test_warn(self):
        assert clearance_verdict(0.26, 0.45) == 'warn'
        assert clearance_verdict(0.0, 0.45) == 'warn'


class TestRetreatPose:
    def test_retreat_along_neg_yaw(self):
        # yaw=0（+x を向いている）→ -x 方向へ退避
        x2, y2 = retreat_pose(3.0, -3.0, 0.0, 0.25)
        assert math.isclose(x2, 2.75)
        assert math.isclose(y2, -3.0)

    def test_retreat_yaw_90(self):
        # yaw=pi/2（+y を向いている）→ -y 方向へ退避
        x2, y2 = retreat_pose(3.0, -3.0, math.pi / 2, 0.25)
        assert math.isclose(x2, 3.0, abs_tol=1e-9)
        assert math.isclose(y2, -3.25)
