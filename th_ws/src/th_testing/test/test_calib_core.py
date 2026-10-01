"""test_calib_core.py — 校正の計算・合否判定（純粋関数。ホストの素の pytest で走る）。

DetailedDesign-maintenance.md §7 の #4（sanity を通らない値で Step 3 へ進めない）・
#5（検証で許容範囲を超えたら適用前に戻る。判定部）・#8（A10）に対応する。
"""
import math
import os
import sys

import pytest

_MAINT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "th_maintenance"))
sys.path.insert(0, _MAINT)

from th_maintenance import calib_core as cc  # noqa: E402


def test_sanity_rejects_nan_inf_and_out_of_range():
    assert cc.sanity(1.0, 0.5, 2.0)
    assert not cc.sanity(math.nan, 0.5, 2.0)
    assert not cc.sanity(math.inf, 0.5, 2.0)
    assert not cc.sanity(0.0, 0.5, 2.0)
    assert not cc.sanity(10.0, 0.5, 2.0)


def test_corrected_wheel_radius_scale_short_run_shrinks_scale():
    # 1.0 m 走ったつもりが 0.98 m しか進んでいない → スケールを 0.98 倍
    assert cc.corrected_wheel_radius_scale(1.0, 1.0, 0.98) == pytest.approx(0.98)
    # 現在のスケールが 0.99 なら掛け合わせる
    assert cc.corrected_wheel_radius_scale(0.99, 2.0, 1.96) == pytest.approx(0.99 * 0.98)


def test_corrected_values_nan_on_invalid_input():
    assert math.isnan(cc.corrected_wheel_radius_scale(1.0, 0.0, 1.0))
    assert math.isnan(cc.corrected_wheel_radius_scale(1.0, 1.0, 0.0))
    assert math.isnan(cc.corrected_wheel_radius_scale(1.0, 1.0, -1.0))
    assert math.isnan(cc.corrected_wheel_base(0.39, 0.0, 180.0))
    assert math.isnan(cc.corrected_wheel_base(0.39, 180.0, math.nan))


def test_corrected_wheel_base_overrotation_enlarges_base():
    # 回りすぎた（実測 190 > 指令 180）→ 車輪間距離を大きく見積もり直す
    assert cc.corrected_wheel_base(0.39, 180.0, 190.0) == pytest.approx(0.39 * 190.0 / 180.0)


def test_corrected_wheel_radius_matches_design_formula():
    assert cc.corrected_wheel_radius(0.05, 1.0, 0.9) == pytest.approx(0.045)


@pytest.mark.parametrize("scale,ok", [
    (1.0, True), (1.10, True), (0.90, True),           # 境界ちょうどは合格（丸め許容）
    (1.1001, False), (0.8999, False), (1.5, False),
    (0.0, False), (-1.0, False), (math.nan, False),
])
def test_a10_boundary(scale, ok):
    assert cc.a10_ok(scale, 0.10) is ok


def test_linear_preview_not_sane_when_a10_exceeded():
    # 比は sanity の範囲（0.85）だが A10 を超える → 適用しない
    new = cc.corrected_wheel_radius_scale(1.0, 1.0, 0.85)
    assert cc.ratio_sane(0.85, 1.0)
    assert not cc.linear_preview_sane(1.0, 0.85, new, 0.10)
    new_ok = cc.corrected_wheel_radius_scale(1.0, 1.0, 0.95)
    assert cc.linear_preview_sane(1.0, 0.95, new_ok, 0.10)


def test_linear_preview_not_sane_for_orders_of_magnitude():
    # cm と m の取り違え（100 と入れた）
    assert not cc.linear_preview_sane(1.0, 100.0, 100.0, 0.10)
    assert not cc.linear_preview_sane(1.0, 0.0, math.nan, 0.10)


def test_rotation_preview_sane():
    new = cc.corrected_wheel_base(0.39, 180.0, 190.0)
    assert cc.rotation_preview_sane(180.0, 190.0, new, 0.39)
    assert not cc.rotation_preview_sane(180.0, 18.0, cc.corrected_wheel_base(0.39, 180.0, 18.0), 0.39)
    assert not cc.rotation_preview_sane(180.0, math.nan, math.nan, 0.39)


def test_verify_linear_within_and_exceeding_tolerance():
    assert cc.verify_linear(1.0, 1.01, 0.02).ok
    v = cc.verify_linear(1.0, 1.05, 0.02)
    assert not v.ok and v.reason.startswith("error_exceeds_tolerance")


def test_verify_rotation_within_and_exceeding_tolerance():
    assert cc.verify_rotation(180.0, 181.5, 3.0).ok
    assert not cc.verify_rotation(180.0, 185.0, 3.0).ok


@pytest.mark.parametrize("tol", [None, math.nan, 0.0, -1.0])
def test_verify_fails_safe_when_tolerance_undefined(tol):
    """許容範囲が未確定（placeholder）なら合格にしない。"""
    assert cc.verify_linear(1.0, 1.0, tol) == cc.Verdict(False, "tolerance_undefined")
    assert cc.verify_rotation(180.0, 180.0, tol) == cc.Verdict(False, "tolerance_undefined")


def test_verify_rejects_invalid_measurement():
    assert not cc.verify_linear(1.0, 0.0, 0.02).ok
    assert not cc.verify_rotation(180.0, math.nan, 3.0).ok


def test_imu_all_calibrated_requires_all_four_at_3():
    assert cc.imu_all_calibrated(0xFF)
    assert not cc.imu_all_calibrated(0xFE)          # mag=2
    assert not cc.imu_all_calibrated(0xBF)          # sys=2
    assert not cc.imu_all_calibrated(0x00)
