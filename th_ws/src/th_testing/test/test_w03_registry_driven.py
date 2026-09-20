"""test_w03_registry_driven.py — W-03（教示・再生の registry 駆動）の試験。

- registry の 25 行が現行ノードの既定値と一致すること（値の変更なし）
- 生成 yaml に載ること（完了条件1の静的側）
- A8 が armed にならないこと（stage 1〜7 で run_generation が通る）
- launch が生成 yaml を渡していること（onsite の factor=1 上書きの順序を含む）
- WAIVER(demo): W-03 が th_ws/src に残っていないこと（完了条件2）
- int/float の型一致（rclpy の未宣言型推論と生成 yaml の型がずれると
  ノードが起動失敗する。CLAUDE.md「環境の癖」）
"""
from __future__ import annotations

import ast
import os
import sys
import tempfile

import pytest
import yaml

_REPO_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
_SRC_ROOT = os.path.join(_REPO_SRC)
_LAUNCH_DIR = os.path.join(_REPO_SRC, "th_bringup", "launch")
_PARAMS_SRC = os.path.join(_REPO_SRC, "th_params")

for _p in (_LAUNCH_DIR, _PARAMS_SRC):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import params_generation as pg  # noqa: E402

REGISTRY_YAML = os.path.join(_PARAMS_SRC, "config", "registry.yaml")
BRINGUP_PY = os.path.join(_LAUNCH_DIR, "bringup.launch.py")

# ノードの declare 既定値 → (registry 名, ファイル)。registry 名とノードの
# パラメータ名は一致させる（export が consumers で振り分ける仕組みのため。
# 改名は別作業）。
_W03_NODES = {
    "route_recorder": os.path.join(_REPO_SRC, "th_planning", "scripts",
                                   "route_recorder.py"),
    "replay_runner": os.path.join(_REPO_SRC, "th_planning", "scripts",
                                   "replay_runner.py"),
    "map_downsampler": os.path.join(_REPO_SRC, "th_planning", "scripts",
                                     "map_downsampler.py"),
}

# 移さないもの（パス・トピック名・フレーム名・配線フラグ・ID）。
# ここにある名前が registry に現れたら分類ミス。
_NOT_MOVED = {
    "routes_dir", "use_map_frame", "map_frame", "base_frame",
    "map_session_id", "odom_topic", "odom_filtered_topic",
    "map_topic", "output_topic",
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


def _node_defaults(path: str) -> dict:
    """`declare_parameter('name', <リテラル>)` を AST で抜く。"""
    tree = ast.parse(_read(path), filename=path)
    out = {}
    for node in ast.walk(tree):
        if not (isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == "declare_parameter"
                and len(node.args) >= 2):
            continue
        name, value = node.args[0], node.args[1]
        if isinstance(name, ast.Constant) and isinstance(name.value, str):
            if isinstance(value, ast.Constant):
                out[name.value] = value.value
    return out


def _w03_names() -> set:
    names = set()
    for node, path in _W03_NODES.items():
        for name in _node_defaults(path):
            if name not in _NOT_MOVED:
                names.add(name)
    return names


# ============================================================================
# 1. registry の行がノード既定値と一致する（値の変更なし・型一致）
# ============================================================================

def test_registry_matches_node_defaults_with_types():
    """25 行すべてが、対応ノードの既定値と値・型ともに一致する。
    値が違えば「変えた」、型が違えば rclpy が起動失敗する。"""
    reg = _registry_rows()
    for node, path in _W03_NODES.items():
        for name, default in _node_defaults(path).items():
            if name in _NOT_MOVED:
                continue
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
    """配線系（パス・トピック・フレーム・フラグ・ID）は registry に入れない。"""
    reg = _registry_rows()
    for name in _NOT_MOVED:
        assert name not in reg, (
            f"{name} が registry にある（配線設定は対象外）")


def test_no_waiver_w03_in_src():
    """完了条件2: WAIVER(demo): W-03 が th_ws/src に残っていない。"""
    bad = []
    for root, _dirs, files in os.walk(_REPO_SRC):
        if ".pytest_cache" in root or "__pycache__" in root:
            continue
        for fn in files:
            if not fn.endswith((".py", ".cpp", ".hpp", ".yaml")):
                continue
            if fn == "test_w03_registry_driven.py":
                continue  # この試験自身が文字列に言及するため除外
            p = os.path.join(root, fn)
            with open(p, encoding="utf-8", errors="replace") as f:
                if "WAIVER(demo): W-03" in f.read():
                    bad.append(os.path.relpath(p, _REPO_SRC))
    assert not bad, f"WAIVER(demo): W-03 が残っている: {bad}"


# ============================================================================
# 2. 生成 yaml に載る（完了条件1の静的側・完了条件5）
# ============================================================================

def _generate(stage: int) -> dict:
    with tempfile.TemporaryDirectory() as tmp:
        out_dir = os.path.join(tmp, "generated")
        pg.run_generation(stage=stage, sim=False, nodes=list(pg.REGISTRY_NODES),
                           out_dir=out_dir, registry_path=REGISTRY_YAML,
                           env=_subprocess_env())
        docs = {}
        for node in ("route_recorder", "replay_runner", "map_downsampler"):
            with open(os.path.join(out_dir, f"{node}.yaml"), encoding="utf-8") as f:
                docs[node] = yaml.safe_load(f)[node]["ros__parameters"]
        return docs


@pytest.mark.parametrize("stage", [1, 4])
def test_generated_yaml_carries_w03_values(stage):
    """stage 1/4 とも生成が通り（A8）、3 ファイルに W-03 の値が載る。"""
    docs = _generate(stage)
    reg = _registry_rows()
    for node, path in _W03_NODES.items():
        for name, default in _node_defaults(path).items():
            if name in _NOT_MOVED:
                continue
            assert name in docs[node], (
                f"stage={stage}: {node}.yaml に {name} が無い")
            assert docs[node][name] == default, (
                f"stage={stage}: {node}.yaml の {name}={docs[node][name]!r} "
                f"!= ノード既定 {default!r}")
            assert reg[name]["status"] == "given"


# ============================================================================
# 3. launch が生成 yaml を渡している（順序＝後勝ちを含む）
# ============================================================================

def _bringup_nodes_by_name(name: str) -> list:
    tree = ast.parse(_read(BRINGUP_PY), filename=BRINGUP_PY)
    rows = []
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == "append"
                and isinstance(node.func.value, ast.Name)
                and node.func.value.id == "nodes"
                and node.args):
            arg = node.args[0]
            if not (isinstance(arg, ast.Call)
                    and isinstance(arg.func, ast.Name) and arg.func.id == "Node"):
                continue
            for kw in arg.keywords:
                if (kw.arg == "name" and isinstance(kw.value, ast.Constant)
                        and kw.value.value == name):
                    rows.append(arg)
    return rows


def _generated_yaml_refs(node_call: ast.Call) -> list:
    """parameters=[...] の中の os.path.join(GENERATED_DIR, '<f>.yaml') を集める。"""
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
    """3 ノード（＋onsite 2 個目）が生成 yaml を parameters= に持つ。"""
    for name, fname in (("route_recorder", "route_recorder.yaml"),
                        ("replay_runner", "replay_runner.yaml"),
                        ("map_downsampler", "map_downsampler.yaml"),
                        ("onsite_map_downsampler", "map_downsampler.yaml")):
        nodes = _bringup_nodes_by_name(name)
        assert nodes, f"bringup に name={name!r} が無い"
        for node in nodes:
            assert fname in _generated_yaml_refs(node), (
                f"name={name!r} が {fname} を渡していない")


def test_onsite_override_comes_after_generated():
    """onsite インスタンスは生成 yaml の後に factor=1 を置く（後勝ち）。
    順序が逆だと registry の 4 で上書きされ F-2（factor=1 固定）が壊れる。"""
    nodes = _bringup_nodes_by_name("onsite_map_downsampler")
    assert len(nodes) == 1
    params = None
    for kw in nodes[0].keywords:
        if kw.arg == "parameters" and isinstance(kw.value, ast.List):
            params = kw.value.elts
    assert params is not None and len(params) == 2
    first, second = params
    assert isinstance(first, ast.Call), "1 番目が生成 yaml でない"
    assert isinstance(second, ast.Dict), "2 番目が inline dict でない"
    keys = [k.value for k in second.keys
            if isinstance(k, ast.Constant)]
    assert "factor" in keys, "onsite の上書きに factor が無い"
