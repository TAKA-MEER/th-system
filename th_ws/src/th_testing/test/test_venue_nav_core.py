"""venue_nav_core の純ロジックテスト (WP-ONSITE-02。ROS2 不要)"""
import math
import os
import sys

# ── パスを通す（colcon build 前にも直接 pytest できるように）──
sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), '..', '..', 'th_onsite'))

from th_onsite.venue_nav_core import (  # noqa: E402
    VenueNavParams, align_cmd_wz, arrived, find_home_goal,
    should_unblock_for_arrival, yaw_error)


def approx(a, b, eps=1e-6):
    assert abs(a - b) < eps, f'{a} != {b}'


class TestYawError:
    def test_basic_positive(self):
        approx(yaw_error(0, math.pi / 2), math.pi / 2)

    def test_basic_negative(self):
        approx(yaw_error(math.pi / 2, 0), -math.pi / 2)

    def test_wrap_plus_pi(self):
        # (3, -3): 差 = -6。(-6+π) % 2π - π ≈ +0.283
        approx(yaw_error(3, -3), 0.283185307179586, 1e-3)

    def test_wrap_minus_pi(self):
        # (-3, 3): 差 = 6。(6+π) % 2π - π ≈ -0.283
        approx(yaw_error(-3, 3), -0.283185307179586, 1e-3)


class TestAlignCmdWz:
    def test_small_error_done(self):
        # 誤差 0.05 < tol 0.09 → (0.0, True)
        wz, done = align_cmd_wz(0, 0.05, VenueNavParams(align_tolerance_rad=0.09))
        assert wz == 0.0
        assert done is True

    def test_large_error_clamp(self):
        # 誤差 1.0 → clamp(1.2*1.0, ±0.6)=0.6, False
        wz, done = align_cmd_wz(0, 1.0, VenueNavParams())
        approx(wz, 0.6)
        assert done is False

    def test_negative_error_clamp(self):
        # 誤差 -1.0 → -0.6, False
        wz, done = align_cmd_wz(0, -1.0, VenueNavParams())
        approx(wz, -0.6)
        assert done is False


class TestArrived:
    def test_arrived_true(self):
        # (0,0)-(0.2,0) tol 0.3 → True
        assert arrived(0, 0, 0.2, 0, VenueNavParams(arrival_xy_tol_m=0.30)) is True

    def test_arrived_false(self):
        # (0,0)-(0.5,0) → False
        assert arrived(0, 0, 0.5, 0, VenueNavParams(arrival_xy_tol_m=0.30)) is False


class TestShouldUnblockForArrival:
    P = VenueNavParams(arrival_xy_tol_m=0.30)

    def test_within_tol_true(self):
        # 実機で膠着した 0.259m は tol 0.30 内 → True（BLOCKED から NAV に戻す）
        assert should_unblock_for_arrival(
            (3.656, -3.684), {'x': 3.509, 'y': -3.897}, self.P) is True

    def test_outside_tol_false(self):
        assert should_unblock_for_arrival(
            (0.0, 0.0), {'x': 1.0, 'y': 0.0}, self.P) is False

    def test_none_robot_false(self):
        assert should_unblock_for_arrival(None, {'x': 0.0, 'y': 0.0}, self.P) is False

    def test_none_goal_false(self):
        assert should_unblock_for_arrival((0.0, 0.0), None, self.P) is False


class TestFindHomeGoal:
    def test_home_found(self):
        # HOME ピン (1,2,yaw=0) を返す（PANEL があっても HOME を優先）
        pins = [
            {'kind': 'PANEL', 'x': 9, 'y': 9, 'yaw': 0.5},
            {'kind': 'HOME', 'x': 1, 'y': 2, 'yaw': 0},
        ]
        assert find_home_goal(pins) == {'x': 1, 'y': 2, 'yaw': 0}

    def test_no_home_returns_none(self):
        # PANEL のみ → None
        pins = [{'kind': 'PANEL', 'x': 9, 'y': 9, 'yaw': 0.5}]
        assert find_home_goal(pins) is None

    def test_empty_returns_none(self):
        assert find_home_goal([]) is None

    def test_none_returns_none(self):
        assert find_home_goal(None) is None
