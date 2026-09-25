"""test_maintenance_registry_driven.py — WP-MAINT-01（始業点検 OPCHECK）の registry 駆動の試験。

W-03 / W-13 と同じ流儀。registry.yaml（正本）と check_core.py（合否判定コア）と
生成物（opcheck_runner.yaml）の 3 点が一致していることを機械で固定する。

- registry.yaml の WP-MAINT-01 ブロック 9 行は status: given（方針値）。
  consumers に opcheck_runner と params_audit を持つ。
- 共有行: v_check / scan_stale_ms（既存）。OPCHECK は scan_stale_ms を
  LiDAR 死活・周期の判定に「借用」し、consumers に opcheck_runner を追加する。
- 値の出どころは registry の note ・CheckParams の docstring を参照。
ROS2 不要（素の pytest で走る）。
"""
from __future__ import annotations

import dataclasses
import os
import sys
import tempfile

import pytest
import yaml

_REPO_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
_LAUNCH_DIR = os.path.join(_REPO_SRC, "th_bringup", "launch")
_PARAMS_SRC = os.path.join(_REPO_SRC, "th_params")
_MAINTENANCE_PKG = os.path.join(_REPO_SRC, "th_maintenance")

for _p in (_LAUNCH_DIR, _PARAMS_SRC, _MAINTENANCE_PKG):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import params_generation as pg  # noqa: E402
from th_maintenance import check_core  # noqa: E402

REGISTRY_YAML = os.path.join(_PARAMS_SRC, "config", "registry.yaml")

# WP-MAINT-01 で新設した行。
_OPCHECK_NAMES = [
    "motor_deadband_mps",
    "motor_follow_min_ratio",
    "imu_bias_max_rad_s",
    "imu_wz_implausible_rad_s",
    "opcheck_imu_window_s",
    "opcheck_deadman_timeout_s",
    "opcheck_spin_w_rad_s",
    "opcheck_blind_tolerance_deg",
    "opcheck_scan_coverage_gap_deg",
]

# OPCHECK が借用する既存行（新規作成しない）。
_SHARED_NAMES = ["v_check", "scan_stale_ms"]


def _read(path: str) -> str:
    with open(path, encoding="utf-8") as f:
        return f.read()


def _subprocess_env() -> dict:
    env = dict(os.environ)
    env["PYTHONPATH"] = _PARAMS_SRC + os.pathsep + env.get("PYTHONPATH", "")
    return env


def _registry_rows() -> dict:
    with open(REGISTRY_YAML, encoding="utf-8") as f:
        return {row["name"]: row for row in yaml.safe_load(f)}


# ============================================================================
# 1. registry の行 ↔ CheckParams の対応
# ============================================================================

def test_check_core_fields_match_registry():
    """CheckParams のフィールド集合が registry の対応行と 1 対 1 で一致する。"""
    fields = set(check_core.CheckParams.__dataclass_fields__)
    assert fields == set(_OPCHECK_NAMES) | set(_SHARED_NAMES), (
        "check_core の CheckParams と registry の行がずれている（どちらか一方だけ足した）")


def test_opcheck_rows_are_given_class_b():
    reg = _registry_rows()
    for name in _OPCHECK_NAMES:
        row = reg[name]
        assert row["status"] == "given", f"{name}: 方針値なのに given でない"
        assert row["class"] == "b", f"{name}: class が b でない"
        assert "opcheck_runner" in (row.get("consumers") or []), (
            f"{name}: consumers に opcheck_runner が無い")
        assert "params_audit" in (row.get("consumers") or []), (
            f"{name}: consumers に params_audit が無い（監査対象から漏れる）")


def test_scan_stale_ms_consumers_extended():
    row = _registry_rows()["scan_stale_ms"]
    assert "opcheck_runner" in row["consumers"], (
        "scan_stale_ms を OPCHECK が借用していない（LiDAR 死活・周期の判定）")
    assert "obstacle_limiter" in row["consumers"], (
        "scan_stale_ms の元の消費者を外している")


def test_opcheck_rows_are_float():
    reg = _registry_rows()
    for name in _OPCHECK_NAMES:
        assert type(reg[name]["value"]) is float, (
            f"{name}: registry の値が float でない（CheckParams は float 宣言）")


def test_check_core_params_constructible_from_registry():
    """registry の値で CheckParams を構築できる（値・型が破綻しない）。"""
    reg = _registry_rows()
    kwargs = {name: reg[name]["value"] for name in _SHARED_NAMES + _OPCHECK_NAMES}
    p = check_core.CheckParams(**kwargs)
    assert p.v_check == pytest.approx(0.05)
    assert p.scan_stale_ms == pytest.approx(300)
    assert p.motor_deadband_mps == pytest.approx(0.03)


def test_no_placeholder_row_blocks_opcheck_runner():
    """opcheck_runner を consumers に持つ行は placeholder ではない（A8 が armed にならない）。"""
    reg = _registry_rows()
    blockers = [r["name"] for r in reg.values()
                if "opcheck_runner" in (r.get("consumers") or [])
                and r["status"] == "placeholder"]
    assert blockers == [], (
        f"opcheck_runner を consumers に持つ placeholder 行がある（launch が止まる）: {blockers}")


def test_opcheck_runner_in_registry_nodes():
    assert "opcheck_runner" in pg.REGISTRY_NODES, (
        "REGISTRY_NODES に opcheck_runner が無い（A8 の対象に含まれない）")


# ============================================================================
# 2. 生成物 opcheck_runner.yaml に値が載る
# ============================================================================

@pytest.mark.parametrize("stage", [1, 4])
def test_generated_opcheck_runner_yaml_carries_values(stage):
    """stage 1/4 とも生成が通り（A8）、opcheck_runner.yaml に値が載る。"""
    reg = _registry_rows()
    with tempfile.TemporaryDirectory() as tmp:
        out_dir = os.path.join(tmp, "generated")
        pg.run_generation(stage=stage, sim=False, nodes=list(pg.REGISTRY_NODES),
                          out_dir=out_dir, registry_path=REGISTRY_YAML,
                          env=_subprocess_env())
        with open(os.path.join(out_dir, "opcheck_runner.yaml"), encoding="utf-8") as f:
            doc = yaml.safe_load(f)
        params = doc["opcheck_runner"]["ros__parameters"]
        for name in _OPCHECK_NAMES + _SHARED_NAMES:
            assert name in params, f"stage={stage}: opcheck_runner.yaml に {name} が無い"
            assert params[name] == reg[name]["value"], (
                f"stage={stage}: {name}: 生成値 {params[name]!r} != registry {reg[name]['value']!r}")


def test_generated_does_not_carry_unrelated_placeholder():
    """生成物は共有行のみ。OPCHECK が使わない他ノードの行（例: lidar_timeout_ms）は載らない。"""
    with tempfile.TemporaryDirectory() as tmp:
        out_dir = os.path.join(tmp, "generated")
        pg.run_generation(stage=1, sim=False, nodes=["opcheck_runner"],
                          out_dir=out_dir, registry_path=REGISTRY_YAML,
                          env=_subprocess_env())
        with open(os.path.join(out_dir, "opcheck_runner.yaml"), encoding="utf-8") as f:
            doc = yaml.safe_load(f)
        params = doc["opcheck_runner"]["ros__parameters"]
        assert "lidar_timeout_ms" not in params, (
            "lidar_timeout_ms が生成物に載っている（placeholder 鎖・LD を巻き込む）")


# ============================================================================
# 3. check_core の判定にレジストリ値がそのまま使われる（ast 静的検査）
# ============================================================================

def test_check_core_reads_params_only_via_dataclass():
    """check_core.py が registry.yaml をファイル読取していない（値の二重管理が無い）。"""
    src = _read(os.path.join(_MAINTENANCE_PKG, "th_maintenance", "check_core.py"))
    assert "import yaml" not in src, \
        "check_core.py が yaml を import している（registry を直接読む二重管理）"
    assert "open(" not in src, \
        "check_core.py がファイルを開いている（判定ロジックが pure でない）"
    assert "def judge_" in src


def test_opcheck_values_not_hardcoded_in_judges():
    """判定のしきい値は CheckParams 経由（ハードコード禁止）。"""
    src = _read(os.path.join(_MAINTENANCE_PKG, "th_maintenance", "check_core.py"))
    for name in _OPCHECK_NAMES:
        assert name in src, (
            f"check_core.py に {name} への参照が無い（判定に使っていない）")
    assert set(check_core.CheckParams.__dataclass_fields__) == set(_OPCHECK_NAMES) | set(_SHARED_NAMES)