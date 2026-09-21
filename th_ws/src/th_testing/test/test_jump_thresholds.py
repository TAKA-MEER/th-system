"""test_jump_thresholds.py — B′（jump）閾値の実走回帰試験。

Spec-safety.md §3.5.0「実走で分かったこと」（2026-09-21）。
registry.yaml の値から Params を組む（試験の中で閾値を直書きしない）。
旧値（並進 0.11 m／回転 0.0175 rad）に戻すと発火しない側が赤くなる。

- 発火しない: 実走で観測した正常な補正の最悪値
  （並進 0.37 m／回転 10.5°＝0.1833 rad。その場旋回中）
- 発火する: 桁の違う飛び（過去の地図積み上がり不具合の規模。
  数 m・100〜200°）
"""
from __future__ import annotations

import os

import pytest
import yaml

from th_state.localization_health_core import (
    Params,
    TransformSample,
    detect_jump,
)

REGISTRY_YAML = os.path.join(
    os.path.dirname(__file__), "..", "..", "th_params", "config", "registry.yaml")


def _registry_params() -> Params:
    """registry.yaml の実値から Params を組む。閾値は直書きしない。"""
    with open(REGISTRY_YAML, encoding="utf-8") as f:
        rows = {row["name"]: row for row in yaml.safe_load(f)}
    return Params(
        stale_ms=rows["localization_stale_ms"]["value"],
        warmup_ms=rows["localization_warmup_ms"]["value"],
        expected_nodes=tuple(rows["localization_expected_nodes"]["value"]),
        jump_window_ms=rows["jump_window_ms"]["value"],
        jump_translation_m=rows["jump_translation_m"]["value"],
        jump_rotation_rad=rows["jump_rotation_rad"]["value"],
    )


def _jump(trans_m: float, rot_rad: float):
    """0.5 秒 window の map→odom の動き (trans_m, rot_rad) に対する判定。
    時刻差 500 ms は stale（2000 ms）以内なので不連続ガードに掛からない。"""
    p = _registry_params()
    prev = TransformSample(t_ms=100_000, x=0.0, y=0.0, yaw=0.0)
    curr = TransformSample(t_ms=100_500, x=trans_m, y=0.0, yaw=rot_rad)
    return detect_jump(prev, curr, p)


# ============================================================================
# 発火しない: 実走の正常な補正の最悪値（いずれもその場旋回中）
# ============================================================================

@pytest.mark.parametrize("trans_m,rot_rad", [
    (0.37, 0.0),        # 並進の最大
    (0.0, 0.1833),      # 回転の最大（10.5°）
    (0.254, 0.117),     # p99 級の同時寄り
    (0.37, 0.1833),     # 両方の最大が同時
])
def test_observed_worst_correction_does_not_fire(trans_m, rot_rad):
    """実走の最悪値では jump を出さない。旧閾値（0.11／0.0175）に戻すと
    いずれも超えるため赤くなる（変異チェック済み）。"""
    r = _jump(trans_m, rot_rad)
    assert r.trans_m == pytest.approx(trans_m)
    assert r.rot_rad == pytest.approx(rot_rad)
    assert r.is_jump is False, (
        f"正常な補正 ({trans_m} m, {rot_rad} rad) で誤発火: {r}")


# ============================================================================
# 発火する: 桁の違う飛び（地図の読み直し積み上がり不具合の規模）
# ============================================================================

@pytest.mark.parametrize("trans_m,rot_rad", [
    (2.0, 0.0),     # 数 m の飛び（並進）
    (0.0, 2.6),     # 100〜200° の飛び（回転。π 未満なので wrap しない）
    (3.0, 3.0),     # 両方とも桁違い
])
def test_gross_misalignment_fires(trans_m, rot_rad):
    """桁の違う飛びは jump を出す。閾値を上げすぎる方向への退行を防ぐ。"""
    r = _jump(trans_m, rot_rad)
    assert r.is_jump is True, (
        f"桁違いの飛び ({trans_m} m, {rot_rad} rad) で発火しない: {r}")
