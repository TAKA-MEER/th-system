"""test_w13_registry_driven.py — W-13（試験場内・人物追跡の registry 駆動）の試験。

test_w03_registry_driven.py と同じ形。違い:
- person_tracker_bridge は `DEFAULT_*` モジュール定数で宣言するため、
  AST 抽出で定数を解決する
- `tracker_lost_grace_ms` は W-15 の placeholder 行のため対象外
  （触らないことを別試験で縛る）
- `clear_distance_m` は導出値と live 値が食い違っているため launch で
  live 値をピン留めする（順序試験で縛る）
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

for _p in (_LAUNCH_DIR, _PARAMS_SRC):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import params_generation as pg  # noqa: E402

REGISTRY_YAML = os.path.join(_PARAMS_SRC, "config", "registry.yaml")
BRINGUP_PY = os.path.join(_LAUNCH_DIR, "bringup.launch.py")

_W13_NODES = {
    "pin_registrar": os.path.join(_REPO_SRC, "th_onsite", "scripts",
                                  "pin_registrar.py"),
    "venue_navigator": os.path.join(_REPO_SRC, "th_onsite", "scripts",
                                    "venue_navigator.py"),
    "wait_clear_gate": os.path.join(_REPO_SRC, "th_onsite", "scripts",
                                    "wait_clear_gate.py"),
    "home_declarer": os.path.join(_REPO_SRC, "th_onsite", "scripts",
                                  "home_declarer.py"),
    "person_tracker_bridge": os.path.join(_REPO_SRC, "th_perception", "scripts",
                                          "person_tracker_bridge.py"),
}

# 移さないもの（パス・トピック名・サービス名・フレーム名・配線フラグ・ID）。
_W13_NOT_MOVED = {
    "venue_dir", "map_frame", "base_frame",
    "candidates_topic", "select_service", "reset_service",
}

# W-15 のため対象外（placeholder のまま。触らない）。
_W15_REMAINDER = {"tracker_lost_grace_ms"}

# 導出値のため値突き合わせの対象外（別試験で縛る）。旧ピン留めは外した。
_DERIVED = {"clear_distance_m"}


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
    """`declare_parameter('name', <リテラル or モジュール定数>)` を AST で抜く。
    person_tracker_bridge の DEFAULT_* はモジュール直下の代入から解決する。"""
    src = _read(path)
    tree = ast.parse(src, filename=path)
    consts = {}
    for node in ast.walk(tree):
        if (isinstance(node, ast.Assign) and len(node.targets) == 1
                and isinstance(node.targets[0], ast.Name)
                and isinstance(node.value, ast.Constant)):
            consts[node.targets[0].id] = node.value.value
    out = {}
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


def _moved_names() -> dict:
    """ノードごとに移行対象の {name: default}。配線・W-15・導出値を除く。"""
    out = {}
    for node, path in _W13_NODES.items():
        for name, default in _node_defaults(path).items():
            if name in _W13_NOT_MOVED or name in _W15_REMAINDER or name in _DERIVED:
                continue
            out.setdefault(node, {})[name] = default
    return out


# ============================================================================
# 1. registry の行がノード既定値と一致する（値の変更なし・型一致）
# ============================================================================

def test_registry_matches_node_defaults_with_types():
    reg = _registry_rows()
    for node, names in _moved_names().items():
        for name, default in names.items():
            assert name in reg, f"registry.yaml に {name} が無い"
            row = reg[name]
            assert row["status"] == "given", (
                f"{name} が given でない（measured と偽らない）")
            assert node in row["consumers"], (
                f"{name} の consumers に {node} が無い")
            assert row["value"] == default, (
                f"{name}: registry={row['value']!r} != ノード既定={default!r}"
                "（値を変えてはいけない）")
            assert type(row["value"]) is type(default), (
                f"{name}: 型不一致 registry={type(row['value']).__name__} "
                f"!= ノード={type(default).__name__}（rclpy が起動失敗する）")


def test_not_moved_names_stay_out_of_registry():
    reg = _registry_rows()
    for name in _W13_NOT_MOVED:
        assert name not in reg, (
            f"{name} が registry にある（配線設定は対象外）")


def test_w15_rows_untouched():
    """W-15 の 2 行は placeholder・blocking_from_stage 5 のまま。触らない。"""
    reg = _registry_rows()
    for name in ("tracker_lost_grace_ms", "person_position_sigma_m"):
        row = reg[name]
        assert row["status"] == "placeholder", f"{name} の status を変えている"
        assert row["blocking_from_stage"] == 5, (
            f"{name} の blocking_from_stage を変えている（W-15）")


def test_no_waiver_w13_in_src():
    """完了条件2: WAIVER(demo): W-13 が th_ws/src に残っていない
    （W-15 のタグは残す）。"""
    bad_w13 = []
    for root, _dirs, files in os.walk(_REPO_SRC):
        if ".pytest_cache" in root or "__pycache__" in root:
            continue
        for fn in files:
            if not fn.endswith((".py", ".cpp", ".hpp", ".yaml")):
                continue
            if fn in ("test_w13_registry_driven.py", "test_w03_registry_driven.py"):
                continue  # この試験自身が文字列に言及するため除外
            p = os.path.join(root, fn)
            with open(p, encoding="utf-8", errors="replace") as f:
                content = f.read()
            if "WAIVER(demo): W-13" in content:
                bad_w13.append(os.path.relpath(p, _REPO_SRC))
    assert not bad_w13, f"WAIVER(demo): W-13 が残っている: {bad_w13}"


def test_clear_distance_derived_and_unpinned():
    """clear_distance_m は導出値（0.975）が効き、ピン留めが無いこと。
    ノード既定 1.0 との差（2.5 cm）は意図した変更（spec 差分）。"""
    reg = _registry_rows()
    row = reg["clear_distance_m"]
    assert row["status"] == "derived", "clear_distance_m が derived でない"
    assert "person_margin_m" in (row.get("derived_from") or []), (
        "derived_from に person_margin_m が無い")
    nodes = _bringup_nodes_by_name("wait_clear_gate")
    assert len(nodes) == 1
    seg = ast.get_source_segment(_read(BRINGUP_PY), nodes[0]) or ""
    assert "clear_distance_m" not in seg, (
        "ピン留めが残っている（生成 yaml の 0.975 が効かない）")


# ============================================================================
# 2. 生成 yaml に載る（完了条件1の静的側・完了条件3）
# ============================================================================

def _generate(stage: int) -> dict:
    with tempfile.TemporaryDirectory() as tmp:
        out_dir = os.path.join(tmp, "generated")
        pg.run_generation(stage=stage, sim=False, nodes=list(pg.REGISTRY_NODES),
                           out_dir=out_dir, registry_path=REGISTRY_YAML,
                           env=_subprocess_env())
        docs = {}
        for node in list(_W13_NODES) + ["state_manager"]:
            with open(os.path.join(out_dir, f"{node}.yaml"), encoding="utf-8") as f:
                docs[node] = yaml.safe_load(f)[node]["ros__parameters"]
        return docs


@pytest.mark.parametrize("stage", [1, 4])
def test_generated_yaml_carries_w13_values(stage):
    """stage 1/4 とも生成が通り（A8）、各ファイルに W-13 の値が載る。
    tracker_lost_grace_ms は null→除去される（W-15 のまま起動する根拠）。"""
    docs = _generate(stage)
    reg = _registry_rows()
    for node, names in _moved_names().items():
        for name, default in names.items():
            assert name in docs[node], (
                f"stage={stage}: {node}.yaml に {name} が無い")
            assert docs[node][name] == default
            assert reg[name]["status"] == "given"
    # W-15 行は生成物から落ち、ノード既定（1500）で動く。
    assert "tracker_lost_grace_ms" not in docs["person_tracker_bridge"], (
        "placeholder が生成物に残っている（D3 違反・ノードが起動失敗する）")
    # 共有行が両方に載る。
    assert docs["state_manager"]["target_confidence_min"] == 0.5
    assert docs["person_tracker_bridge"]["target_confidence_min"] == 0.5


@pytest.mark.parametrize("stage", [1, 4])
def test_generated_clear_distance_is_formula_value(stage):
    """完了条件1: 生成 yaml の clear_distance_m が式どおり 0.975 になる。
    ノード既定 1.0 との差は意図した変更（float 誤差は approx で吸収）。"""
    docs = _generate(stage)
    assert docs["wait_clear_gate"]["clear_distance_m"] == pytest.approx(0.975)


# ============================================================================
# 3. launch が生成 yaml を渡している
# ============================================================================

def _bringup_nodes_by_name(name: str) -> list:
    """bringup の Node 定義を探す。person_tracker_bridge は TimerAction の
    actions 内に居るため、ネストも辿る。"""
    tree = ast.parse(_read(BRINGUP_PY), filename=BRINGUP_PY)
    rows = []

    def _node_named(arg):
        if not (isinstance(arg, ast.Call)
                and isinstance(arg.func, ast.Name) and arg.func.id == "Node"):
            return False
        return any(kw.arg == "name" and isinstance(kw.value, ast.Constant)
                   and kw.value.value == name for kw in arg.keywords)

    for node in ast.walk(tree):
        if (isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == "append"
                and isinstance(node.func.value, ast.Name)
                and node.func.value.id == "nodes"
                and node.args):
            arg = node.args[0]
            if _node_named(arg):
                rows.append(arg)
                continue
            # TimerAction(period=..., actions=[...]) のネストを辿る。
            if (isinstance(arg, ast.Call) and isinstance(arg.func, ast.Name)):
                for kw in arg.keywords:
                    if kw.arg != "actions" or not isinstance(kw.value, ast.List):
                        continue
                    for elt in kw.value.elts:
                        if _node_named(elt):
                            rows.append(elt)
    return rows


def _generated_yaml_refs(node_call: ast.Call) -> list:
    refs = []
    for kw in node_call.keywords:
        if kw.arg != "parameters" or not isinstance(kw.value, ast.List):
            continue
        for elt in kw.value.elts:
            if not (isinstance(elt, ast.Call)
                    and isinstance(elt.func, ast.Attribute)
                    and elt.func.attr == "join"):
                continue
            args = elt.args
            if (len(args) >= 2 and isinstance(args[0], ast.Name)
                    and args[0].id == "GENERATED_DIR"
                    and isinstance(args[1], ast.Constant)):
                refs.append(args[1].value)
    return refs


def test_launch_passes_generated_yaml():
    for name in ("pin_registrar", "venue_navigator", "wait_clear_gate",
                 "home_declarer", "person_tracker_bridge"):
        nodes = _bringup_nodes_by_name(name)
        assert nodes, f"bringup に name={name!r} が無い"
        for node in nodes:
            assert f"{name}.yaml" in _generated_yaml_refs(node), (
                f"name={name!r} が生成 yaml を渡していない")
