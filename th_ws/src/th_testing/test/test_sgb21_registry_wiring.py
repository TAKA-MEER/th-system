"""test_sgb21_registry_wiring.py — SG-B21（1b-10）の registry 配線の試験。

W-03 / W-13 と同じ流儀。registry.yaml（正本）とノードの `declare_parameter`
既定値と生成物（`<node>.yaml`）の 3 点が一致していることを機械で固定する。
SG-B21 は「consumers に挙がっているのに読み手がいない行」の除去が本体なので、
**載っていることの照合だけでなく、載っていないことの照合（否定試験）**も持つ。
外した consumers が戻っても赤くなる。

- 肯定: 行が consumers にあり、ノードが同じ名を宣言し、生成 yaml のノード節に
  同値・同型で載る（値の変更なし・INTEGER↔DOUBLE の事故防止）。
- 否定: 外した consumers のノード節にその行が載らない。ノード側も宣言しない。
- `link_gap_p99_ms` は safety_monitor に残すが直接読まない（間接消費）。
  生成物に dict で載ることと、C++ 側がこの名を宣言していないことを縛る。

ROS2 不要（素の pytest で走る）。
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
_MAINTENANCE_PKG = os.path.join(_REPO_SRC, "th_maintenance")

for _p in (_LAUNCH_DIR, _PARAMS_SRC, _MAINTENANCE_PKG):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import params_generation as pg  # noqa: E402
from th_maintenance import blind_core  # noqa: E402

REGISTRY_YAML = os.path.join(_PARAMS_SRC, "config", "registry.yaml")

# (registry 行, ノード, ノードのソース。既定値の取り出し方は _node_defaults が吸収)
_POSITIVE = [
    ("auto_select_hold_s", "person_tracker_bridge",
     os.path.join(_REPO_SRC, "th_perception", "scripts", "person_tracker_bridge.py")),
    ("esp32_ws_port", "esp32_bridge",
     os.path.join(_REPO_SRC, "th_esp32_bridge", "scripts", "esp32_bridge.py")),
    ("opcheck_estop_stale_ms", "opcheck_runner",
     os.path.join(_REPO_SRC, "th_maintenance", "scripts", "opcheck_runner.py")),
    ("opcheck_estop_stale_ms", "opcheck_auto",
     os.path.join(_REPO_SRC, "th_maintenance", "scripts", "opcheck_auto.py")),
    ("opcheck_estop_release_timeout_ms", "opcheck_runner",
     os.path.join(_REPO_SRC, "th_maintenance", "scripts", "opcheck_runner.py")),
    ("opcheck_motor_too_brief_ms", "opcheck_runner",
     os.path.join(_REPO_SRC, "th_maintenance", "scripts", "opcheck_runner.py")),
    ("opcheck_lidar_no_data_ms", "opcheck_runner",
     os.path.join(_REPO_SRC, "th_maintenance", "scripts", "opcheck_runner.py")),
    ("calib_linear_distance_m", "calib_runner",
     os.path.join(_REPO_SRC, "th_maintenance", "scripts", "calib_runner.py")),
    ("calib_rotation_deg", "calib_runner",
     os.path.join(_REPO_SRC, "th_maintenance", "scripts", "calib_runner.py")),
    ("calib_run_timeout_ms", "calib_runner",
     os.path.join(_REPO_SRC, "th_maintenance", "scripts", "calib_runner.py")),
    ("calib_blind_verify_frames", "calib_runner",
     os.path.join(_REPO_SRC, "th_maintenance", "scripts", "calib_runner.py")),
    ("calib_blind_verify_timeout_ms", "calib_runner",
     os.path.join(_REPO_SRC, "th_maintenance", "scripts", "calib_runner.py")),
    ("calib_state_stale_ms", "calib_runner",
     os.path.join(_REPO_SRC, "th_maintenance", "scripts", "calib_runner.py")),
    ("calib_odom_stale_ms", "calib_runner",
     os.path.join(_REPO_SRC, "th_maintenance", "scripts", "calib_runner.py")),
]

# (registry 行, 外したノード, 外したノードの生成 yaml に載ってはならない)
_NEGATIVE = [
    ("two_point_spacing_m", "pin_registrar"),
    ("auto_select_hold_s", "pin_registrar"),
    ("link_quality_regression_ratio", "safety_monitor"),
    ("nav_tolerance_m", "venue_navigator"),
    ("nav_tolerance_deg", "venue_navigator"),
]


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


def _node_defaults(path: str) -> dict:
    """`declare_parameter('name', <リテラル or モジュール定数 or PARS 展開>)` を抜く。

    - リテラル宣言（esp32_bridge 等）は直接読む。
    - `DEFAULT_*` モジュール定数（person_tracker_bridge）は W-13 と同じく解決する。
    - `for name, value in PARS.items(): declare_parameter(name, value)`
     （opcheck_runner / opcheck_auto / calib_runner）は PARS の AST を eval する
      （calib の `blind_core.MAX_*` は名前空間で渡す）。
    """
    src = _read(path)
    tree = ast.parse(src, filename=path)
    consts = {}
    for node in ast.walk(tree):
        if (isinstance(node, ast.Assign) and len(node.targets) == 1
                and isinstance(node.targets[0], ast.Name)
                and isinstance(node.value, ast.Constant)):
            consts[node.targets[0].id] = node.value.value
    pars = None
    for node in tree.body:
        targets = (node.targets if isinstance(node, ast.Assign)
                   else [node.target] if isinstance(node, ast.AnnAssign) else [])
        if any(isinstance(t, ast.Name) and t.id == "PARS" for t in targets):
            expr = ast.Expression(node.value)
            pars = dict(eval(compile(expr, path, "eval"),  # noqa: S307（自リポジトリのリテラル）
                             {"blind_core": blind_core}))
    out = dict(pars) if pars is not None else {}
    for node in ast.walk(tree):
        if not (isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == "declare_parameter"
                and len(node.args) >= 2):
            continue
        name, value = node.args[0], node.args[1]
        if not (isinstance(name, ast.Constant) and isinstance(name.value, str)):
            continue
        if isinstance(value, ast.Constant):
            out[name.value] = value.value
        elif isinstance(value, ast.Name) and value.id in consts:
            out[name.value] = consts[value.id]
    return out


def _ros_type(value) -> str:
    if isinstance(value, bool):
        return "bool"
    if isinstance(value, int):
        return "int"
    if isinstance(value, float):
        return "float"
    if isinstance(value, str):
        return "str"
    if isinstance(value, list):
        return "list"
    if isinstance(value, dict):
        return "dict"
    return type(value).__name__


@pytest.fixture(scope="module")
def generated_stage4():
    with tempfile.TemporaryDirectory() as tmp:
        out_dir = os.path.join(tmp, "generated")
        pg.run_generation(stage=4, sim=False, nodes=list(pg.REGISTRY_NODES),
                          out_dir=out_dir, registry_path=REGISTRY_YAML,
                          env=_subprocess_env(),
                          calib_dir=os.path.join(tmp, "no_calib"))
        docs = {}
        for name in os.listdir(out_dir):
            if not name.endswith(".yaml"):
                continue
            with open(os.path.join(out_dir, name), encoding="utf-8") as f:
                docs[name[:-5]] = yaml.safe_load(f)[name[:-5]]["ros__parameters"]
        yield docs


# ============================================================================
# 1. consumers の状態（decision-1 の固定）
# ============================================================================

def test_consumers_match_decision():
    reg = _registry_rows()
    assert reg["two_point_spacing_m"]["consumers"] == ["params_audit"]
    assert reg["auto_select_hold_s"]["consumers"] == ["person_tracker_bridge"]
    assert reg["link_quality_regression_ratio"]["consumers"] == ["params_audit"]
    assert reg["nav_tolerance_m"]["consumers"] == ["params_audit"]
    assert reg["nav_tolerance_deg"]["consumers"] == ["params_audit"]
    assert "safety_monitor" in reg["link_gap_p99_ms"]["consumers"]
    assert "間接消費" in reg["link_gap_p99_ms"]["note"]
    assert reg["esp32_ws_port"]["consumers"] == ["esp32_bridge"]


# ============================================================================
# 2. 肯定: registry ↔ ノード既定値 ↔ 生成 yaml（値・型）
# ============================================================================

@pytest.mark.parametrize("name,node,path", _POSITIVE)
def test_positive_row_is_wired(name, node, path, generated_stage4):
    reg = _registry_rows()
    assert node in (reg[name].get("consumers") or []), (
        f"{name}: consumers に {node} が無い")
    defaults = _node_defaults(path)
    assert name in defaults, f"{name}: {os.path.basename(path)} が宣言していない"
    assert defaults[name] == reg[name]["value"], (
        f"{name}: ノード既定 {defaults[name]!r} != registry {reg[name]['value']!r}")
    assert _ros_type(defaults[name]) == _ros_type(reg[name]["value"]), (
        f"{name}: ノード既定と registry の型が違う（rclpy は INTEGER↔DOUBLE を変換しない）")
    params = generated_stage4[node]
    assert name in params, f"{name}: 生成 {node}.yaml に載っていない"
    assert params[name] == reg[name]["value"], (
        f"{name}: 生成値 {params[name]!r} != registry {reg[name]['value']!r}")
    assert _ros_type(params[name]) == _ros_type(reg[name]["value"]), (
        f"{name}: 生成値と registry の型が違う")


# ============================================================================
# 3. 否定: 外した consumers の生成 yaml に載らない・ノードも宣言しない
# ============================================================================

@pytest.mark.parametrize("name,node", _NEGATIVE)
def test_removed_consumer_carries_nothing(name, node, generated_stage4):
    reg = _registry_rows()
    assert node not in (reg[name].get("consumers") or []), (
        f"{name}: consumers に {node} が戻っている")
    assert name not in generated_stage4[node], (
        f"{name}: 生成 {node}.yaml に載っている（consumers を外したのに漏れている）")


def test_pin_registrar_does_not_declare_removed_names():
    defaults = _node_defaults(os.path.join(
        _REPO_SRC, "th_onsite", "scripts", "pin_registrar.py"))
    assert "two_point_spacing_m" not in defaults
    assert "auto_select_hold_s" not in defaults


def test_safety_monitor_does_not_declare_regression_ratio():
    src = _read(os.path.join(_REPO_SRC, "th_safety", "src", "safety_monitor.cpp"))
    assert "link_quality_regression_ratio" not in src, (
        "safety_monitor.cpp が link_quality_regression_ratio を宣言している"
        "（使う設計が無いのにダミー宣言している）")


def test_link_gap_p99_is_indirect_only(generated_stage4):
    """link_gap_p99_ms は生成物に載るが、ノードは直接読まない（導出 timeout 経由）。"""
    params = generated_stage4["safety_monitor"]
    assert params["link_gap_p99_ms"] == {"esp32": 161, "lidar": 154}
    src = _read(os.path.join(_REPO_SRC, "th_safety", "src", "safety_monitor.cpp"))
    assert "link_gap_p99_ms" not in src


# ============================================================================
# 4. stage 1/4 とも生成が通る（A8 が armed にならない）
# ============================================================================

@pytest.mark.parametrize("stage", [1, 4])
def test_generation_passes_stages(stage):
    with tempfile.TemporaryDirectory() as tmp:
        out_dir = os.path.join(tmp, "generated")
        pg.run_generation(stage=stage, sim=False, nodes=list(pg.REGISTRY_NODES),
                          out_dir=out_dir, registry_path=REGISTRY_YAML,
                          env=_subprocess_env(),
                          calib_dir=os.path.join(tmp, "no_calib"))
        for name in ("opcheck_runner", "opcheck_auto", "calib_runner",
                     "esp32_bridge", "person_tracker_bridge"):
            assert os.path.exists(os.path.join(out_dir, f"{name}.yaml")), (
                f"stage={stage}: {name}.yaml が無い")
