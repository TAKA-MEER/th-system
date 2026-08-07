"""
test_face_planner_logic.py
=============================
FACING(超信地旋回のみで正対・展示用)モードのコアロジック単体テスト

ROS2 なし・純粋 Python で実行可能。
pytest で実行: pytest test/test_face_planner_logic.py -v
"""

import math
import sys
import os
import pytest

# ── パスを通す(colcon build 前にも直接 pytest できるように)
sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), '..', '..', 'th_planning'))

from th_planning.face_planner_core import (
    FaceParams, FaceCore, bearing_to, face_control, rate_limit_angular,
)


DEFAULT_PARAMS = FaceParams(
    k_ang                     = 2.0,
    w_max_rad_s                = 0.5,
    max_angular_accel_rad_s2   = 3.0,
    max_angular_decel_rad_s2   = 6.0,
    angle_tolerance_rad        = 0.05,
    min_confidence              = 0.5,
    lost_debounce_sec           = 0.3,
    update_rate_hz               = 10.0,
)


def _params(**overrides):
    return FaceParams(**{**DEFAULT_PARAMS.__dict__, **overrides})


class TestBearingTo:
    def test_directly_ahead_is_zero(self):
        assert bearing_to(2.0, 0.0) == pytest.approx(0.0)

    def test_to_the_left(self):
        assert bearing_to(0.0, 2.0) == pytest.approx(math.pi / 2)

    def test_to_the_right(self):
        assert bearing_to(0.0, -2.0) == pytest.approx(-math.pi / 2)

    def test_behind(self):
        assert abs(bearing_to(-2.0, 0.0)) == pytest.approx(math.pi)


class TestFaceControl:
    def test_within_tolerance_is_zero(self):
        assert face_control(0.01, k_ang=2.0, angle_tolerance=0.05) == 0.0

    def test_proportional_outside_tolerance(self):
        assert face_control(0.5, k_ang=2.0, angle_tolerance=0.05) == pytest.approx(1.0)

    def test_sign_follows_error(self):
        assert face_control(-0.5, k_ang=2.0, angle_tolerance=0.05) == pytest.approx(-1.0)


class TestFaceCore:
    def test_ramps_toward_target_not_instant(self):
        core = FaceCore(_params(w_max_rad_s=2.0, max_angular_accel_rad_s2=3.0, update_rate_hz=10.0))
        out1 = core.update(0.0, 2.0, confidence=1.0, is_lost=False, lost_elapsed_sec=0.0)
        out2 = core.update(0.0, 2.0, confidence=1.0, is_lost=False, lost_elapsed_sec=0.0)
        assert out1.angular_z == pytest.approx(0.3)
        assert out2.angular_z == pytest.approx(0.6)

    def test_w_max_clamps(self):
        core = FaceCore(_params(w_max_rad_s=0.5, max_angular_accel_rad_s2=100.0))
        out = None
        for _ in range(20):
            out = core.update(0.0, 2.0, confidence=1.0, is_lost=False, lost_elapsed_sec=0.0)
        assert out.angular_z == pytest.approx(0.5)

    def test_aligned_within_tolerance_gives_zero_and_reason_aligned(self):
        core = FaceCore(_params())
        out = core.update(2.0, 0.01, confidence=1.0, is_lost=False, lost_elapsed_sec=0.0)
        assert out.angular_z == pytest.approx(0.0, abs=1e-9)
        assert out.reason == "aligned"

    def test_facing_away_gives_tracking_reason(self):
        core = FaceCore(_params())
        out = core.update(0.0, 2.0, confidence=1.0, is_lost=False, lost_elapsed_sec=0.0)
        assert out.reason == "tracking"
        assert out.angular_z > 0.0

    def test_brief_lost_flicker_does_not_stop(self):
        core = FaceCore(_params())
        core.update(0.0, 2.0, confidence=1.0, is_lost=False, lost_elapsed_sec=0.0)
        out = core.update(0.0, 2.0, confidence=1.0, is_lost=True, lost_elapsed_sec=0.05)
        assert out.reason != "person_lost"

    def test_sustained_lost_stops(self):
        core = FaceCore(_params())
        out = core.update(0.0, 2.0, confidence=1.0, is_lost=True, lost_elapsed_sec=0.5)
        assert out.angular_z == 0.0
        assert out.reason == "person_lost"

    def test_low_confidence_stops(self):
        core = FaceCore(_params())
        out = core.update(0.0, 2.0, confidence=0.2, is_lost=False, lost_elapsed_sec=0.0)
        assert out.angular_z == 0.0
        assert out.reason == "low_confidence"

    def test_reset_clears_rate_limit_state(self):
        core = FaceCore(_params(w_max_rad_s=2.0, max_angular_accel_rad_s2=3.0, update_rate_hz=10.0))
        for _ in range(10):
            core.update(0.0, 2.0, confidence=1.0, is_lost=False, lost_elapsed_sec=0.0)
        core.reset()
        out = core.update(0.0, 2.0, confidence=1.0, is_lost=False, lost_elapsed_sec=0.0)
        assert 0.0 < out.angular_z <= 0.3 + 1e-9   # ゼロから再加速(急に w_max へ飛ばない)

    def test_decelerates_faster_than_accelerates(self):
        # w_max=2.0 まで加速側レート(0.3/tick)で積み上げてから、目標が急に
        # 正面(bearing=0)になったと想定して減速側レート(0.6/tick)で落ちることを確認
        core = FaceCore(_params(w_max_rad_s=2.0, k_ang=100.0,
                                 max_angular_accel_rad_s2=3.0, max_angular_decel_rad_s2=6.0,
                                 update_rate_hz=10.0))
        out = None
        for _ in range(10):
            out = core.update(0.0, 2.0, confidence=1.0, is_lost=False, lost_elapsed_sec=0.0)
        assert out.angular_z == pytest.approx(2.0)   # w_max に張り付いている

        out = core.update(2.0, 0.0, confidence=1.0, is_lost=False, lost_elapsed_sec=0.0)
        # 減速レート 6.0/10Hz = 0.6/tick 分だけ落ちる(加速レート 0.3/tick より速い)
        assert out.angular_z == pytest.approx(2.0 - 0.6)

    def test_lost_stop_is_rate_limited_not_instant(self):
        core = FaceCore(_params(w_max_rad_s=2.0, k_ang=100.0,
                                 max_angular_accel_rad_s2=3.0, max_angular_decel_rad_s2=6.0,
                                 update_rate_hz=10.0))
        for _ in range(10):
            core.update(0.0, 2.0, confidence=1.0, is_lost=False, lost_elapsed_sec=0.0)
        out = core.update(0.0, 2.0, confidence=1.0, is_lost=True, lost_elapsed_sec=0.5)
        assert out.reason == "person_lost"
        assert out.angular_z == pytest.approx(2.0 - 0.6)   # 瞬時ゼロではなく減速レートで落ちる


class TestRateLimitAngular:
    def test_decelerating_uses_decel_delta(self):
        # |target| < |prev| は減速: 減速側の上限を使う
        assert rate_limit_angular(0.0, 1.0, max_accel_delta=0.3, max_decel_delta=0.6) == pytest.approx(0.4)

    def test_accelerating_uses_accel_delta(self):
        # |target| > |prev| は加速: 加速側の上限を使う
        assert rate_limit_angular(1.0, 0.0, max_accel_delta=0.3, max_decel_delta=0.6) == pytest.approx(0.3)

    def test_reaches_target_without_overshoot_when_within_delta(self):
        assert rate_limit_angular(0.05, 0.1, max_accel_delta=0.3, max_decel_delta=0.6) == pytest.approx(0.05)
