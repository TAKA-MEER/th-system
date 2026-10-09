"""test_w01p5_localize_registry.py — W-01 P5 の registry 配線の試験（ホスト・ROS2 不要）。

P1〜P4 で `LOCALIZE_DEFAULTS` に仮置きしていた 6 値を registry.yaml へ移した
ことの固定。W-03 / SG-B21 と同じ流儀（registry ↔ ノード既定値 ↔ 生成 yaml の
3 点一致）だが、確度 3 件と resume は実スキャン待ちの placeholder のため切り分ける:

- given（探索窓 2 件）: registry ↔ ノード既定値 ↔ 生成 yaml が同値・同型で載る
- placeholder（確度 3 件・resume）: 生成物に載らない・ノードの既定値が
  `LOCALIZE_DEFAULTS` と同値（値は変えない）・stage 5 以降は A8 が armed

rclpy の型の罠（`test_maintenance_param_types.py` 相当）もここで縛る:
replay_runner は 6 件すべて `float(...)` で宣言し、registry の given 値も
float であること（INTEGER↔DOUBLE を変換せず起動時に落ちる）。
"""
from __future__ import annotations

import ast
import os
import sys
import tempfile

import pytest
import yaml

_REPO_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
_LAUNCH_DIR = os.path.join(_REPO_SRC, "th_bringup", "launch")
_PARAMS_SRC = os.path.join(_REPO_SRC, "th_params")
_PLANNING_PKG = os.path.join(_REPO_SRC, "th_planning")

for _p in (_LAUNCH_DIR, _PARAMS_SRC):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import params_generation as pg  # noqa: E402

REGISTRY_YAML = os.path.join(_PARAMS_SRC, "config", "registry.yaml")
REPLAY_RUNNER_PY = os.path.join(_REPO_SRC, "th_planning", "scripts", "replay_runner.py")
LOCALIZE_CORE_PY = os.path.join(_PLANNING_PKG, "th_planning", "localize_core.py")

# (名前, 分類, 状態)。確度 3 件と resume は実スキャン（P6）待ちの placeholder、
# 探索窓 2 件は m_sep_m からの逆算で出発点に置ける given。
_EXPECTED = {
    "localize_match_low": ("c", "placeholder"),
    "localize_margin_low": ("c", "placeholder"),
    "localize_margin_min": ("c", "placeholder"),
    "search_radius_m": ("b", "given"),
    "widen_radius_m": ("b", "given"),
    "resume_max_dist_m": ("c", "placeholder"),
}

# P1〜P4 の仮置き値。変えたら P6 の実測として別作業にする。
# localize_match_low だけは 2026-10-09 の実機実測（localize_core.py の注記）で 0.8→0.70。
_FALLBACK_VALUES = {
    "localize_match_low": 0.70,
    "localize_margin_low": 0.2,
    "localize_margin_min": 0.05,
    "search_radius_m": 5.0,
    "widen_radius_m": 10.0,
    "resume_max_dist_m": 2.0,
}


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


def _localize_defaults() -> dict:
    """localize_core.py の LOCALIZE_DEFAULTS を AST から読む。

    モジュール自体は numpy が要るためホストでは import できない。値は全て
    リテラルなので AST を eval する（自リポジトリのリテラル）。
    """
    tree = ast.parse(_read(LOCALIZE_CORE_PY), filename=LOCALIZE_CORE_PY)
    for node in tree.body:
        targets = (node.targets if isinstance(node, ast.Assign)
                   else [node.target] if isinstance(node, ast.AnnAssign) else [])
        if any(isinstance(t, ast.Name) and t.id == "LOCALIZE_DEFAULTS" for t in targets):
            expr = ast.Expression(node.value)
            return dict(eval(compile(expr, LOCALIZE_CORE_PY, "eval"), {}))  # noqa: S307
    raise AssertionError("localize_core.py に LOCALIZE_DEFAULTS が無い")


def _node_declared_floats() -> dict:
    """replay_runner.py の declare_parameter('name', float(...)) を抜く。

    戻り値は {name: True/False}（True＝float() で宣言＝DOUBLE）。
    """
    tree = ast.parse(_read(REPLAY_RUNNER_PY), filename=REPLAY_RUNNER_PY)
    out: dict[str, bool] = {}
    for node in ast.walk(tree):
        if not (isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == "declare_parameter"
                and len(node.args) >= 2):
            continue
        name, value = node.args[0], node.args[1]
        if not (isinstance(name, ast.Constant) and isinstance(name.value, str)):
            continue
        out[name.value] = (
            isinstance(value, ast.Call)
            and isinstance(value.func, ast.Name)
            and value.func.id == "float")
    return out


@pytest.fixture(scope="module")
def generated_stage4():
    with tempfile.TemporaryDirectory() as tmp:
        out_dir = os.path.join(tmp, "generated")
        pg.run_generation(stage=4, sim=False, nodes=list(pg.REGISTRY_NODES),
                          out_dir=out_dir, registry_path=REGISTRY_YAML,
                          env=_subprocess_env(),
                          calib_dir=os.path.join(tmp, "no_calib"))
        with open(os.path.join(out_dir, "replay_runner.yaml"), encoding="utf-8") as f:
            yield yaml.safe_load(f)["replay_runner"]["ros__parameters"]


# ============================================================================
# 1. registry.yaml の 6 行（分類・状態・値・consumers）
# ============================================================================

def test_registry_has_six_w01p5_rows():
    reg = _registry_rows()
    for name, (cls, status) in _EXPECTED.items():
        assert name in reg, f"registry.yaml に {name} が無い（W-01 P5）"
        assert reg[name]["class"] == cls, f"{name}: class が {cls} でない"
        assert reg[name]["status"] == status, f"{name}: status が {status} でない"
        assert reg[name]["consumers"] == ["replay_runner"], (
            f"{name}: consumers が [replay_runner] でない")
        assert not reg[name].get("derived_from"), (
            f"{name}: 導出式を持たない（given／placeholder のみ）")


def test_placeholders_are_tbd_and_stage5():
    """確度 3 件・resume は TBD＋blocking＋stage 5（W-15 の教訓。起動は止めない）。"""
    from th_params import schema
    reg = _registry_rows()
    for name in ("localize_match_low", "localize_margin_low",
                 "localize_margin_min", "resume_max_dist_m"):
        assert reg[name]["value"] == schema.TBD, f"{name}: placeholder は TBD のみ"
        assert reg[name]["blocking"] is True, f"{name}: blocking:true が無い"
        assert reg[name]["blocking_from_stage"] == 5, (
            f"{name}: blocking_from_stage が 5 でない")


def test_given_radii_carry_values():
    """探索窓 2 件は given の値を持つ（値は変えない）。"""
    reg = _registry_rows()
    assert reg["search_radius_m"]["value"] == 5.0
    assert reg["widen_radius_m"]["value"] == 10.0
    assert isinstance(reg["search_radius_m"]["value"], float)
    assert isinstance(reg["widen_radius_m"]["value"], float)


# ============================================================================
# 2. 生成（A8 が armed にならない stage では通り、stage 5 では止まる）
# ============================================================================

@pytest.mark.parametrize("stage", [1, 2, 3, 4])
def test_generation_passes_before_stage5(stage):
    with tempfile.TemporaryDirectory() as tmp:
        out_dir = os.path.join(tmp, "generated")
        pg.run_generation(stage=stage, sim=False, nodes=list(pg.REGISTRY_NODES),
                          out_dir=out_dir, registry_path=REGISTRY_YAML,
                          env=_subprocess_env(),
                          calib_dir=os.path.join(tmp, "no_calib"))
        assert os.path.exists(os.path.join(out_dir, "replay_runner.yaml"))


def test_generation_armed_at_stage5():
    """stage 5 では placeholder のままの行で起動を拒否する（W-15 の設計どおり。
    P6 の実測で埋めるまで stage 5 の運用には出さない）。"""
    with tempfile.TemporaryDirectory() as tmp:
        out_dir = os.path.join(tmp, "generated")
        with pytest.raises(pg.GenerationError) as exc:
            pg.run_generation(stage=5, sim=False, nodes=list(pg.REGISTRY_NODES),
                              out_dir=out_dir, registry_path=REGISTRY_YAML,
                              env=_subprocess_env(),
                              calib_dir=os.path.join(tmp, "no_calib"))
    assert "localize_match_low" in str(exc.value), (
        "stage 5 の拒否理由に確度の行が無い")


# ============================================================================
# 3. 生成 yaml への載り／載らなさ（値・型）
# ============================================================================

def test_generated_yaml_carries_given_radii(generated_stage4):
    assert generated_stage4["search_radius_m"] == 5.0
    assert generated_stage4["widen_radius_m"] == 10.0
    assert isinstance(generated_stage4["search_radius_m"], float)
    assert isinstance(generated_stage4["widen_radius_m"], float)


def test_generated_yaml_omits_placeholders(generated_stage4):
    """placeholder の 4 件は生成物に載らない（sanitize が落とす）。
    載っていたら起動時に TBD 文字列で落ちるか、誤った値で動く。"""
    for name in ("localize_match_low", "localize_margin_low",
                 "localize_margin_min", "resume_max_dist_m"):
        assert name not in generated_stage4, (
            f"{name}: placeholder が生成 {name}.yaml に載っている")


# ============================================================================
# 4. ノードの既定値＝LOCALIZE_DEFAULTS（値は変えない）・宣言型
# ============================================================================

def test_localize_defaults_unchanged():
    """P1〜P4 の仮置き値が変わっていないこと（値の変更は P6 の実測の作業）。"""
    defaults = _localize_defaults()
    for name, value in _FALLBACK_VALUES.items():
        assert defaults[name] == value, (
            f"LOCALIZE_DEFAULTS[{name!r}] が {value!r} でない（{defaults[name]!r}）")


def test_node_declares_all_six_as_double():
    """replay_runner が 6 件すべて float()（＝DOUBLE）で宣言していること。
    宣言を忘れる・int で宣言する変異ではここが赤くなる（rclpy は
    INTEGER↔DOUBLE を変換せず、生成 yaml と型が違うと起動時に落ちる）。"""
    declared = _node_declared_floats()
    for name in _EXPECTED:
        assert name in declared, f"{name}: replay_runner が宣言していない"
        assert declared[name], (
            f"{name}: float() で宣言していない（INTEGER↔DOUBLE の罠）")
