"""venue_nav_core の純ロジックテスト (WP-ONSITE-02。ROS2 不要)"""
import math
import os
import sys

# ── パスを通す（colcon build 前にも直接 pytest できるように）──
sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), '..', '..', 'th_onsite'))

from th_onsite.venue_nav_core import (  # noqa: E402
    VenueNavParams, align_cmd_wz, arrived, yaw_error)


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
