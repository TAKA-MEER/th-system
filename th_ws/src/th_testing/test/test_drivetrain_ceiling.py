"""
test_drivetrain_ceiling.py
==========================
SG-A14（1b-10）の再現・回帰試験。ROS2 なし・純粋 Python（ホストで走る）。

生产経路: `th_params.export.resolve_registry()` は `export.main()`（CLI 本体）と
launch の `params_generation.run_generation()` が呼ぶのと同じ解決関数であり、
`th_ws/data/generated/*.yaml` の値の出どころである。ここでは実物の
`registry.yaml` を読んで解決し、以下を縛る:

1. `drivetrain_ceiling_mps` がファームの出力上限 (PID_OUT_MAX=200) ÷
   フィードフォワード係数 (PID_KFF=280) = 約 0.71 m/s であること
   （`th_ws/esp32/src/config.h`、`Spec-params.md` §1）。
2. そこから `v_max` が 0.71 × 0.8 ≈ 0.568 m/s に導出されること
   （`derive.v_max_from_ceiling`。`Spec-params.md` §2）。
3. 生成物（tracked の `th_ws/data/generated/`）の `v_max` が解決値と一致すること
   （生成スクリプトでの作り直し忘れを検出する）。
"""
import math
import os

import pytest
import yaml

from th_params import derive, export

_SRC_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
_GENERATED_DIR = os.path.abspath(os.path.join(
    os.path.dirname(__file__), '..', '..', '..', 'data', 'generated'))


def test_ceiling_matches_firmware_limits(registry_rows):
    """registry の天井がファームの 200 ÷ 280 ≈ 0.71 と一致すること。"""
    row = next(r for r in registry_rows if r['name'] == 'drivetrain_ceiling_mps')
    assert row['value'] == pytest.approx(200.0 / 280.0, abs=0.005), (
        f"drivetrain_ceiling_mps={row['value']} がファームの出力上限 "
        f"200 ÷ フィードフォワード係数 280 ≈ 0.714 と食い違う "
        f"(th_ws/esp32/src/config.h の PID_OUT_MAX / PID_KFF)")
    assert row['value'] == pytest.approx(0.71, abs=0.005)


def test_v_max_derives_to_0568_from_ceiling(registry_rows):
    """本番の解決経路で v_max ≈ 0.568 になること（0.71 × 0.8）。"""
    resolved = export.resolve_registry(registry_rows)
    status, value = resolved['v_max']
    assert status == 'derived'
    assert value == pytest.approx(0.71 * 0.8, rel=1e-9)
    assert value == pytest.approx(0.568, abs=0.005)
    # 導出式そのものも直接縛る（式が変わったらここが赤になる）。
    assert derive.v_max_from_ceiling(0.71, 0.8) == pytest.approx(value, rel=1e-9)


def test_generated_v_max_matches_resolved(registry_rows):
    """生成物の v_max が registry の解決値と一致すること（作り直し忘れの検出）。"""
    resolved = export.resolve_registry(registry_rows)
    _, expected = resolved['v_max']
    errors, _warnings = export.run_assertions(
        registry_rows, resolved, stage=0, nodes=None)
    assert errors == [], f"設定値の検査 (A1 など) が通らない: {errors}"
    checked = 0
    for fname in sorted(os.listdir(_GENERATED_DIR)):
        if not fname.endswith('.yaml'):
            continue
        with open(os.path.join(_GENERATED_DIR, fname), encoding='utf-8') as f:
            doc = yaml.safe_load(f) or {}
        for node_body in doc.values():
            params = (node_body or {}).get('ros__parameters') or {}
            if 'v_max' in params:
                checked += 1
                assert params['v_max'] == pytest.approx(expected, rel=1e-9), (
                    f"{fname} の v_max={params['v_max']} が解決値 {expected} と食い違う。"
                    f"生成スクリプトで作り直すこと（手で書き換えない）")
    assert checked > 0, "生成物に v_max が 1 件も無い（検査自体が空振りしている）"


def test_firmware_constants_unchanged():
    """根拠側（ファームの 200 / 280）が変わっていないこと。変わったら天井の
    再計算が要るのでここが赤になって知らせる。"""
    path = os.path.abspath(os.path.join(
        _SRC_ROOT, '..', 'esp32', 'src', 'config.h'))
    src = open(path, encoding='utf-8').read()
    assert '#define PID_KFF           280.0f' in src
    assert '#define PID_OUT_MAX       200.0f' in src
    assert math.isclose(200.0 / 280.0, 0.7142857, rel_tol=1e-6)
