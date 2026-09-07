"""test_home_declare_core.py — 待機場所宣言の純ロジックテスト"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'th_onsite'))

import math

from th_onsite.home_declare_core import (
    HomeDeclareParams, pose_offset, within_tolerance,
)


def _near(a, b, tol=1e-6):
    assert abs(a - b) < tol, f'{a} != {b} (tol={tol})'


class TestPoseOffset:
    def test_same_pose(self):
        """robot (0,0,0), home (0,0,0) → (0.0, 0.0)"""
        m, d = pose_offset((0, 0, 0), (0, 0, 0))
        _near(m, 0.0)
        _near(d, 0.0)

    def test_position_only(self):
        """robot (0.2,0,0), home (0,0,0) → (0.2, 0.0)"""
        m, d = pose_offset((0.2, 0, 0), (0, 0, 0))
        _near(m, 0.2)
        _near(d, 0.0)

    def test_yaw_only_90(self):
        """robot (0,0, π/2), home (0,0,0) → (0.0, 90.0)"""
        m, d = pose_offset((0, 0, math.pi / 2), (0, 0, 0))
        _near(m, 0.0)
        _near(d, 90.0)

    def test_yaw_wrap_180(self):
        """robot (0,0,-3.0), home (0,0,3.0) → offset_deg ≈ 16.2（±180ラップ・絶対値）"""
        m, d = pose_offset((0, 0, -3.0), (0, 0, 3.0))
        _near(m, 0.0)
        _near(d, 16.225323, tol=1e-3)

    def test_diagonal_position(self):
        """robot (3,4,0), home (0,0,0) → (5.0, 0.0)"""
        m, d = pose_offset((3, 4, 0), (0, 0, 0))
        _near(m, 5.0)
        _near(d, 0.0)


class TestWithinTolerance:
    def test_within(self):
        p = HomeDeclareParams(tolerance_m=0.3, tolerance_deg=15.0)
        assert within_tolerance(0.2, 10, p) is True

    def test_position_over(self):
        p = HomeDeclareParams(tolerance_m=0.3, tolerance_deg=15.0)
        assert within_tolerance(0.4, 10, p) is False

    def test_angle_over(self):
        p = HomeDeclareParams(tolerance_m=0.3, tolerance_deg=15.0)
        assert within_tolerance(0.2, 20, p) is False

    def test_boundary(self):
        p = HomeDeclareParams(tolerance_m=0.3, tolerance_deg=15.0)
        assert within_tolerance(0.30, 15.0, p) is True
