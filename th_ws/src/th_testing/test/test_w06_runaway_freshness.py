"""test_w06_runaway_freshness.py — W-06 の②（DRIVE_RUNAWAY の鮮度ゲート）の試験。

Spec-safety.md §3.5.3。実測が新鮮なときだけ判定し、古いあいだは凍結する
（保持時間を進めも戻しもせず、フォルト状態も変えない）。

- registry の runaway_feedback_stale_ms が 250・class b・status given・
  consumers safety_monitor・spec_ref Spec-safety.md §3.5.3
- safety_monitor.cpp の既定値が registry と一致（値・型）
- 生成 yaml に載る（stage 1/4）
- 整合: stale < esp32_timeout_ms かつ < runaway_hold_ms
  （registry と生成 yaml から読む。将来どちらかを動かして不整合にしたら赤）
- 配線: safety_monitor.cpp が純関数を実際に呼んでいる
  （純関数だけ試験して本体が別のロジックのままだと保護にならない）
- 決定値の一致: runaway_hold_ms（1000）/ runaway_ratio（1.5）/
  runaway_zero_threshold（0.08）が registry・C++ 既定値・生成 yaml の
  三者で一致する（Spec-safety.md §3.5.4。W-06 ③⑤）
"""
from __future__ import annotations

import os
import re
import sys
import tempfile

import pytest
import yaml

_REPO_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
_LAUNCH_DIR = os.path.join(_REPO_SRC, "th_bringup", "launch")
_PARAMS_SRC = os.path.join(_REPO_SRC, "th_params")

for _p in (_LAUNCH_DIR, _PARAMS_SRC):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import params_generation as pg  # noqa: E402

REGISTRY_YAML = os.path.join(_PARAMS_SRC, "config", "registry.yaml")
SAFETY_CPP = os.path.join(_REPO_SRC, "th_safety", "src", "safety_monitor.cpp")

_EXPECTED_STALE_MS = 250


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


def _cpp_default(name: str) -> int | float:
    """safety_monitor.cpp の declare_parameter("<name>", <リテラル>) を抜く。"""
    src = _read(SAFETY_CPP)
    m = re.search(rf'declare_parameter\("{re.escape(name)}",\s*([^)]+)\)', src)
    assert m, f"safety_monitor.cpp が {name} を宣言していない"
    lit = m.group(1).strip()
    return eval(lit, {"__builtins__": {}})  # noqa: S307 — リテラルのみ


def _generate(stage: int) -> dict:
    with tempfile.TemporaryDirectory() as tmp:
        out_dir = os.path.join(tmp, "generated")
        pg.run_generation(stage=stage, sim=False, nodes=list(pg.REGISTRY_NODES),
                           out_dir=out_dir, registry_path=REGISTRY_YAML,
                           env=_subprocess_env())
        with open(os.path.join(out_dir, "safety_monitor.yaml"), encoding="utf-8") as f:
            return yaml.safe_load(f)["safety_monitor"]["ros__parameters"]


# ============================================================================
# 1. registry の行が正しい（値・分類・消費者・spec 参照）
# ============================================================================

def test_registry_row_is_given_250_for_safety_monitor():
    reg = _registry_rows()
    assert "runaway_feedback_stale_ms" in reg, (
        "registry.yaml に runaway_feedback_stale_ms が無い")
    row = reg["runaway_feedback_stale_ms"]
    assert row["unit"] == "ms"
    assert row["class"] == "b"
    assert row["status"] == "given", (
        "given でない（measured と偽らない）。出発点 250ms は実測で詰める前の仮値")
    assert row["value"] == _EXPECTED_STALE_MS, (
        f"registry={row['value']!r} != 期待 {_EXPECTED_STALE_MS!r}")
    assert row["consumers"] == ["safety_monitor"], (
        f"consumers が {row['consumers']!r}。safety_monitor のみ")
    assert "Spec-safety.md §3.5.3" in (row.get("spec_ref") or ""), (
        "spec_ref に Spec-safety.md §3.5.3 が無い")


def test_node_default_matches_registry_with_type():
    """ノード既定値が registry と値・型ともに一致する（rclpy が起動失敗しない）。"""
    reg = _registry_rows()
    default = _cpp_default("runaway_feedback_stale_ms")
    assert default == reg["runaway_feedback_stale_ms"]["value"], (
        f"safety_monitor.cpp 既定={default!r} != registry={reg['runaway_feedback_stale_ms']['value']!r}")
    assert type(default) is type(reg["runaway_feedback_stale_ms"]["value"]), (
        "型不一致（rclpy が起動失敗する）")
    assert 'get_parameter("runaway_feedback_stale_ms")' in _read(SAFETY_CPP), (
        "宣言しているだけで読んでいない")


# ============================================================================
# 2. 生成 yaml に載る
# ============================================================================

@pytest.mark.parametrize("stage", [1, 4])
def test_generated_yaml_carries_stale_value(stage):
    """stage 1/4 とも生成が通り（A8）、safety_monitor.yaml に 250 が載る。"""
    params = _generate(stage)
    assert params.get("runaway_feedback_stale_ms") == _EXPECTED_STALE_MS, (
        f"stage={stage}: safety_monitor.yaml に runaway_feedback_stale_ms=250 が無い"
        f"（あるのは {params.get('runaway_feedback_stale_ms')!r}）")


# ============================================================================
# 3. 整合: stale < esp32_timeout_ms かつ < runaway_hold_ms
# ============================================================================

@pytest.mark.parametrize("stage", [1, 4])
def test_stale_is_shorter_than_timeouts_in_generated(stage):
    """生成 yaml から読む。将来どちらかを動かして不整合にしたら赤くなる。"""
    params = _generate(stage)
    stale = params["runaway_feedback_stale_ms"]
    esp32_timeout = params["esp32_timeout_ms"]
    hold = params["runaway_hold_ms"]
    assert stale < esp32_timeout, (
        f"runaway_feedback_stale_ms({stale}) が esp32_timeout_ms({esp32_timeout}) 以上。"
        "凍結の窓が ESP32_DISCONNECTED より長くなっている")
    assert stale < hold, (
        f"runaway_feedback_stale_ms({stale}) が runaway_hold_ms({hold}) 以上。"
        "鮮度窓が保持時間より長くなっている")


def test_stale_is_shorter_than_hold_in_registry():
    """registry 側（esp32_timeout_ms は derived のため hold のみ）。"""
    reg = _registry_rows()
    assert reg["runaway_feedback_stale_ms"]["value"] < reg["runaway_hold_ms"]["value"]


# ============================================================================
# 4. 配線: safety_monitor.cpp が純関数を実際に使っている
# ============================================================================

def test_wiring_calls_pure_functions():
    """「配線したつもり」が本番の経路を通っていない事故の再発防止。

    純関数だけ試験して本体が別のロジックのままだと保護にならない。
    特に凍結中の解除（else 節での updateFaultState(..., false)）は
    「発火した DRIVE_RUNAWAY を古い実測で解除しない」という中核を壊すため、
    呼び出し箇所の形まで縛る（差し戻し指摘）。
    """
    src = _read(SAFETY_CPP)
    # コメントを除いた実コードで見る（コメント中の言及で誤検知しないため）。
    code = re.sub(r"//.*", "", src)
    assert "is_runaway_feedback_fresh(" in code, (
        "safety_monitor.cpp が is_runaway_feedback_fresh を呼んでいない")
    assert "update_runaway_with_freshness(" in code, (
        "safety_monitor.cpp が update_runaway_with_freshness を呼んでいない")
    # 鮮度は t - last_esp32_time_ と esp32_alive_ から出す。
    assert "last_esp32_time_" in code and "esp32_alive_" in code
    # runaway の HoldTimer 直叩きは残っていない（純関数経由のみ）。
    assert "runaway_hold_.update(" not in code, (
        "runaway_hold_.update を直接呼んでいる。update_runaway_with_freshness 経由にすること"
        "（凍結が素通りになる）")
    # DRIVE_RUNAWAY の報告は凍結判定（has_value）の内側の 1 箇所だけ。
    # 凍結中（nullopt）の else 節で updateFaultState(..., false) による解除を
    # 足されると has_value という文字列は残るため、箇所数と引数まで見る。
    assert 'updateFaultState("DRIVE_RUNAWAY"' in code
    assert "has_value()" in code, (
        "凍結（nullopt）のときに updateFaultState を呼ばないガードが無い")
    assert code.index("has_value()") < code.index('updateFaultState("DRIVE_RUNAWAY"'), (
        "updateFaultState(DRIVE_RUNAWAY) が has_value ガードの前に置かれている")
    calls = re.findall(
        r'updateFaultState\(\s*"DRIVE_RUNAWAY"\s*,([^)]*)\)', code)
    assert len(calls) == 1, (
        f'updateFaultState("DRIVE_RUNAWAY", ...) が {len(calls)} 箇所ある。'
        "凍結中の解除パスが足されていないか確認すること")
    assert calls[0].strip() == "*runaway", (
        f'引数が *runaway でない: {calls[0].strip()!r}')
    assert 'updateFaultState("DRIVE_RUNAWAY", false)' not in code, (
        "凍結中にフォルトを解除している（has_value の else 節で false 報告）")
    assert re.search(
        r'if\s*\(\s*runaway\.has_value\(\)\s*\)\s*\{[^}]*\}\s*else', code) is None, (
        "has_value() ブロックに else 節がある。凍結中は何もしないこと")


# ============================================================================
# 5. 決定値の一致（Spec-safety.md §3.5.4。W-06 ③⑤）
# ============================================================================

def test_decided_values_match_registry_and_cpp():
    """runaway_hold_ms（1000）/ runaway_ratio（1.5）/
    runaway_zero_threshold（0.08）が registry と C++ 既定値で一致する。
    本体が別の値のままだと保護にならない（変異で赤くなることを確認済み）。"""
    reg = _registry_rows()
    assert reg["runaway_hold_ms"]["value"] == 1000
    assert reg["runaway_ratio"]["value"] == 1.5
    assert reg["runaway_zero_threshold"]["value"] == 0.08
    # esp32_timeout_ms は derived のまま（値を持たない）。
    assert reg["esp32_timeout_ms"]["status"] == "derived"
    src = _read(SAFETY_CPP)
    assert _cpp_default("runaway_hold_ms") == 1000
    assert _cpp_default("runaway_ratio") == 1.5
    assert _cpp_default("runaway_zero_threshold") == 0.08


@pytest.mark.parametrize("stage", [1, 4])
def test_generated_yaml_carries_tuned_values(stage):
    """生成 yaml に決定値が載る（本番の経路。既定値のフォールバック頼みにしない）。"""
    params = _generate(stage)
    assert params.get("runaway_hold_ms") == 1000, (
        f"stage={stage}: runaway_hold_ms が 1000 でない "
        f"（あるのは {params.get('runaway_hold_ms')!r}）")
    assert params.get("runaway_zero_threshold") == 0.08, (
        f"stage={stage}: runaway_zero_threshold が 0.08 でない "
        f"（あるのは {params.get('runaway_zero_threshold')!r}）")
    assert params.get("runaway_ratio") == 1.5
