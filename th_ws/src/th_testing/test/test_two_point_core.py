"""two_point_core の純ロジックテスト (WP-ONSITE-01。ROS2 不要)"""
import math
import os
import sys

# ── パスを通す（colcon build 前にも直接 pytest できるように）──
sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), '..', '..', 'th_onsite'))

from th_onsite.two_point_core import (TwoPointParams, compose_map_pose,  # noqa: E402
                                      is_target_valid, spacing_ok, two_point_yaw)


def approx(a, b, eps=1e-6):
    assert abs(a - b) < eps, f'{a} != {b}'


class TestTwoPointYaw:
    def test_east(self):
        # 東 (+x) = 0
        approx(two_point_yaw(0, 0, 1, 0), 0.0)

    def test_north(self):
        # 北 (+y) = π/2
        approx(two_point_yaw(0, 0, 0, 1), math.pi / 2)

    def test_west(self):
        # 西 = ±π
        w = two_point_yaw(0, 0, -1, 0)
        approx(abs(w), math.pi)

    def test_south(self):
        # 南 = -π/2
        approx(two_point_yaw(0, 0, 0, -1), -math.pi / 2)


class TestSpacingOk:
    def test_spacing_ok(self):
        ok, dist = spacing_ok(0, 0, 0.5, 0, TwoPointParams(min_spacing_m=0.3))
        assert ok is True
        approx(dist, 0.5)

    def test_spacing_ng(self):
        ok, dist = spacing_ok(0, 0, 0.2, 0, TwoPointParams(min_spacing_m=0.3))
        assert ok is False
        approx(dist, 0.2)

    def test_hypot_consistent(self):
        _, d1 = spacing_ok(1, 2, 4, 6, TwoPointParams())
        assert d1 == math.hypot(4 - 1, 6 - 2)


class TestIsTargetValid:
    def test_lost(self):
        ok, _ = is_target_valid(True, 0.9, TwoPointParams())
        assert ok is False

    def test_low_confidence(self):
        ok, _ = is_target_valid(False, 0.4, TwoPointParams(min_confidence=0.5))
        assert ok is False

    def test_valid(self):
        ok, _ = is_target_valid(False, 0.6, TwoPointParams(min_confidence=0.5))
        assert ok is True

    def test_boundary(self):
        # しきい値ちょうどでも合格
        ok, _ = is_target_valid(False, 0.5, TwoPointParams(min_confidence=0.5))
        assert ok is True


class TestComposeMapPose:
    def test_origin_yaw0(self):
        # ロボットが map 原点・yaw=0 のとき person(1,0)→(1,0)
        x, y = compose_map_pose(0, 0, 0, 1, 0)
        approx(x, 1)
        approx(y, 0)

    def test_yaw_pi_over2(self):
        # ロボット yaw=π/2 のとき person(1,0)→(0,1)
        x, y = compose_map_pose(0, 0, math.pi / 2, 1, 0)
        approx(x, 0)
        approx(y, 1)

    def test_translation(self):
        # ロボットが (10,5)・yaw=0 のとき person(1,0)→(11,5)
        x, y = compose_map_pose(10, 5, 0, 1, 0)
        approx(x, 11)
        approx(y, 5)
