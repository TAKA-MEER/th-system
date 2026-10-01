"""test_calib_blind_core.py — 校正 BLIND の純粋関数（ホストの素の pytest で走る）。

死角マスクは「その角度の点を障害物として見ない」設定。広げすぎを弾く上限と、
検証（選んだ範囲の外に残る写り込みのずれ）を縛る。
"""
import math
import os
import sys

import pytest

_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(_SRC, "th_maintenance"))
sys.path.insert(0, os.path.join(_SRC, "th_perception"))

from th_maintenance import blind_core as bc  # noqa: E402
from th_maintenance import check_core  # noqa: E402


# ── 正規化・結合 ─────────────────────────────────────
def test_normalize_deg_range():
    assert bc.normalize_deg(190.0) == pytest.approx(-170.0)
    assert bc.normalize_deg(-180.0) == pytest.approx(180.0)
    assert bc.normalize_deg(540.0) == pytest.approx(180.0)


def test_sector_width_wraps():
    assert bc.sector_width_deg(10.0, 40.0) == pytest.approx(30.0)
    assert bc.sector_width_deg(170.0, -170.0) == pytest.approx(20.0)
    assert bc.sector_width_deg(5.0, 5.0) == 0.0


def test_empty_selection_is_valid_deletion():
    sel = bc.validate_selection([])
    assert sel.ok and sel.ranges == ()


@pytest.mark.parametrize("bad", [[[1.0]], [[1, 2, 3]], "x", [[math.nan, 1.0]], [[0.0, math.inf]],
                                 [["a", "b"]]])
def test_invalid_format_rejected(bad):
    assert bc.validate_selection(bad).reason == "invalid_format"


def test_zero_width_rejected():
    assert bc.validate_selection([[40.0, 40.0]]).reason == "zero_width"


def test_overlapping_sectors_are_merged_before_limit():
    sel = bc.validate_selection([[0.0, 20.0], [10.0, 28.0]])
    assert sel.ok
    assert sel.ranges == ((0.0, 28.0),)


def test_merge_across_plus_minus_180():
    sel = bc.validate_selection([[170.0, 180.0], [-180.0, -165.0]])
    assert sel.ok
    assert len(sel.ranges) == 1
    assert bc.sector_width_deg(*sel.ranges[0]) == pytest.approx(25.0)


def test_splitting_cannot_dodge_sector_limit():
    """12° ずつ 3 つ（互いに接する）= 結合後 36° → 1 区間 30° の上限で弾く。"""
    sel = bc.validate_selection([[0.0, 12.0], [12.0, 24.0], [24.0, 36.0]])
    assert not sel.ok and sel.reason.startswith("sector_too_wide")


# ── 上限（A: 1 区間 30°・総幅 90°・8 区間）────────────
def test_sector_limit_boundary():
    assert bc.validate_selection([[0.0, 30.0]]).ok
    sel = bc.validate_selection([[0.0, 30.5]])
    assert not sel.ok and sel.reason.startswith("sector_too_wide")


def test_total_limit_boundary():
    four = [[i * 60.0, i * 60.0 + 22.5] for i in range(4)]       # 90° ちょうど
    assert bc.validate_selection(four).ok
    over = [[i * 60.0, i * 60.0 + 23.0] for i in range(4)]       # 92°
    sel = bc.validate_selection(over)
    assert not sel.ok and sel.reason.startswith("total_too_wide")


def test_sector_count_limit():
    nine = [[i * 40.0 - 180.0 + 1, i * 40.0 - 180.0 + 6] for i in range(9)]   # 5° × 9 = 45°
    sel = bc.validate_selection(nine)
    assert not sel.ok and sel.reason.startswith("too_many_sectors")
    assert bc.validate_selection(nine[:8]).ok


def test_current_registry_mask_is_within_limits():
    """出荷時の実機のマスク（4 区間・合計約 68°）は上限内でなければならない。"""
    flat = [-132.8, -117.5, -59.5, -38.9, 45.9, 61.5, 130.3, 143.5]
    sel = bc.validate_selection([list(p) for p in bc.pairs_from_flat(flat)])
    assert sel.ok, sel.reason
    assert sel.total_deg == pytest.approx(15.3 + 20.6 + 15.6 + 13.2)


def test_flat_roundtrip():
    pairs = [(1.0, 2.0), (3.0, 4.0)]
    assert bc.pairs_from_flat(bc.flat_from_pairs(pairs)) == pairs


# ── 検証（選んだ範囲の外に残るずれ）──────────────────
def _scan_with_blocks(blocks, n=360, near=0.3, far=3.0):
    """1° 刻み・angle_min=-180°。blocks=[(a0,a1)] の度の範囲は近距離、他は遠距離。"""
    out = []
    for i in range(n):
        a = -180.0 + i
        out.append(near if bc.in_ranges(a, blocks) else far)
    return out


def test_residual_zero_when_selection_covers_band():
    est = [(0.0, 20.0)]
    assert bc.residual_offset_deg(est, [(-5.0, 25.0)]) == pytest.approx(0.0)


def test_residual_measures_protrusion():
    est = [(0.0, 20.0)]
    off = bc.residual_offset_deg(est, [(0.0, 15.0)])
    assert off == pytest.approx(5.0, abs=0.3)


def test_residual_unselected_band_is_its_width():
    est = [(100.0, 120.0)]
    off = bc.residual_offset_deg(est, [(0.0, 20.0)])
    assert off == pytest.approx(20.0, abs=0.3)


def test_verify_tolerance_boundary_and_undefined():
    est = [(0.0, 20.0)]
    assert bc.verify_blind(est, [(0.0, 15.0)], 5.0).ok            # 5.0°（許容ちょうど）
    assert not bc.verify_blind(est, [(0.0, 14.0)], 5.0).ok        # 6.0°
    v = bc.verify_blind(est, [(0.0, 20.0)], None)
    assert not v.ok and v.reason == "tolerance_undefined"


def test_verify_from_scan_end_to_end_with_estimator():
    """推定器（check_core）→ 残りのずれ → 判定。選択が写り込みを覆っていれば合格。"""
    block = [(40.0, 60.0)]
    frames = [_scan_with_blocks(block) for _ in range(5)]
    persistent = bc.persistent_near_ranges(frames, near_m=0.6)
    est = bc.shift_bands(check_core.estimate_blind_sectors(persistent, 1.0), -180.0)
    assert est, "写り込み帯が推定できていない"
    assert bc.verify_blind(est, [(38.0, 62.0)], 5.0).ok
    assert not bc.verify_blind(est, [(100.0, 120.0)], 5.0).ok


def test_persistent_ignores_transient_obstacle():
    block = [(40.0, 60.0)]
    steady = _scan_with_blocks(block)
    passer = _scan_with_blocks(block + [(-100.0, -60.0)])    # 1 フレームだけ人が通った
    p = bc.persistent_near_ranges([steady, steady, passer, steady], near_m=0.6)
    est = check_core.estimate_blind_sectors(p, 1.0)
    assert all(not (s < 280 and e > 250) for s, e in est)      # -100..-60 は 260..300


# ── プレビュー（lidar_filter と同じ添字）─────────────
def test_count_masked_matches_lidar_filter_indexing():
    ranges = [2.0] * 360
    ranges[10] = math.inf
    n = bc.count_masked([(0.0, 20.0)], math.radians(-180.0), math.radians(1.0), ranges)
    assert n in (20, 21)
    assert bc.count_masked([], math.radians(-180.0), math.radians(1.0), ranges) == 0


def test_unmasked_inside_zero_for_filtered_scan():
    ranges = [2.0] * 360
    inf_in = list(ranges)
    for i in range(180, 201):
        inf_in[i] = math.inf
    assert bc.unmasked_inside([(0.0, 20.0)], math.radians(-180.0), math.radians(1.0), inf_in) == 0
    assert bc.unmasked_inside([(0.0, 20.0)], math.radians(-180.0), math.radians(1.0), ranges) > 0
