"""test_localization_health_launch.py — WP-SAFE-05 の配線試験（ホスト・ROS2 不要）。

- registry.yaml の localization_* 4 行の存在と値・consumers
- 生成 yaml（params_generation）への載り（完了条件5）
- bringup.launch.py の条件付き有効化（完了条件4の静的側）
- safety_monitor.cpp の配線（宣言・ゲート・保持・購読）
- LocalizationHealth.msg の reason 定義（B を出さない）

振る舞い（発火・保持・途絶）の試験は test_localization_lost.py（Docker）が持つ。
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

BRINGUP_PY = os.path.join(_LAUNCH_DIR, "bringup.launch.py")
REGISTRY_YAML = os.path.join(_PARAMS_SRC, "config", "registry.yaml")
SAFETY_CPP = os.path.join(_REPO_SRC, "th_safety", "src", "safety_monitor.cpp")
HEALTH_MSG = os.path.join(_REPO_SRC, "th_system_msgs", "msg", "LocalizationHealth.msg")
HEALTH_NODE_PY = os.path.join(_REPO_SRC, "th_state", "scripts", "localization_health.py")


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
# registry.yaml の 4 行（SD-9。ノード内リテラルにしない）
# ============================================================================

def test_registry_has_four_localization_rows():
    reg = _registry_rows()
    for name in ("localization_stale_ms", "localization_topic_timeout_ms",
                 "localization_warmup_ms", "localization_expected_nodes"):
        assert name in reg, f"registry.yaml に {name} が無い（SD-9）"
        assert reg[name]["status"] == "given"
        assert reg[name]["consumers"], f"{name} の consumers が空（A9）"


def test_registry_values_are_sane():
    reg = _registry_rows()
    assert reg["localization_stale_ms"]["value"] == 2000
    assert reg["localization_topic_timeout_ms"]["value"] == 5000
    assert reg["localization_warmup_ms"]["value"] == 30000
    assert set(reg["localization_expected_nodes"]["value"]) == {"slam_toolbox", "amcl"}
    assert "safety_monitor" in reg["localization_topic_timeout_ms"]["consumers"]
    assert "localization_health" in reg["localization_stale_ms"]["consumers"]


def test_registry_jump_rows_are_remeasured_values():
    """B′ の 3 行。2026-09-21 の実走で決め直した値（Spec-safety.md §3.5.0
    「実走で分かったこと」）。「Autoware の既定を出発点にする」は誤りだった
    ため note に残さない。jump_window_ms は変えない。"""
    reg = _registry_rows()
    for name in ("jump_window_ms", "jump_translation_m", "jump_rotation_rad"):
        assert name in reg, f"registry.yaml に {name} が無い（SD-9）"
        assert reg[name]["consumers"] == ["localization_health"]
    assert reg["jump_window_ms"]["value"] == 500
    assert reg["jump_translation_m"]["value"] == 1.0
    assert reg["jump_rotation_rad"]["value"] == 0.5
    for name in ("jump_translation_m", "jump_rotation_rad"):
        note = reg[name].get("note") or ""
        assert "0.37 m" in note or "10.5" in note, (
            f"{name} の note に実走の最大値が無い")
        assert "2.5" in note, (
            f"{name} の note に「正常な補正の最大の約 2.5 倍」が無い")
        assert "Spec-safety.md" in note, (
            f"{name} の note に根拠（Spec-safety.md §3.5.0）が無い")
        assert "Autoware" not in note, (
            f"{name} の note に誤った「Autoware の既定を出発点」が残っている")


# ============================================================================
# 生成 yaml への載り（完了条件5）
# ============================================================================

def test_generated_yaml_carries_localization_params():
    """実物の registry.yaml → export → 生成物の safety_monitor.yaml /
    localization_health.yaml に localization_* が載ること。"""
    with tempfile.TemporaryDirectory() as tmp:
        out_dir = os.path.join(tmp, "generated")
        pg.run_generation(stage=3, sim=False, nodes=list(pg.REGISTRY_NODES),
                           out_dir=out_dir, registry_path=REGISTRY_YAML,
                           env=_subprocess_env())

        health_path = os.path.join(out_dir, "localization_health.yaml")
        assert os.path.isfile(health_path), "localization_health.yaml が生成されていない"
        with open(health_path, encoding="utf-8") as f:
            health = yaml.safe_load(f)["localization_health"]["ros__parameters"]
        assert health["localization_stale_ms"] == 2000
        assert health["localization_warmup_ms"] == 30000
        assert set(health["localization_expected_nodes"]) == {"slam_toolbox", "amcl"}
        # B′ の 3 件も載ること（完了条件5）。
        assert health["jump_window_ms"] == 500
        assert health["jump_translation_m"] == 1.0
        assert health["jump_rotation_rad"] == 0.5

        with open(os.path.join(out_dir, "safety_monitor.yaml"), encoding="utf-8") as f:
            safety = yaml.safe_load(f)["safety_monitor"]["ros__parameters"]
        assert safety["localization_topic_timeout_ms"] == 5000


# ============================================================================
# bringup.launch.py の条件付き有効化（完了条件4の静的側）
# ============================================================================

def _find_function(tree: ast.AST, name: str) -> ast.FunctionDef:
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return node
    raise AssertionError(f"関数 {name} が見つからない")


def _nodes_by_name(tree: ast.AST, name: str) -> list:
    rows = []
    func = _find_function(tree, "generate_launch_description")
    for node in ast.walk(func):
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


def test_safety_monitor_has_two_variants_with_and_without_localization():
    """localization あり／なしの safety_monitor 定義が If/Unless で排他的にある。
    W-06 の runaway 除外は両方とも維持する。"""
    tree = ast.parse(_read(BRINGUP_PY), filename=BRINGUP_PY)
    nodes = _nodes_by_name(tree, "safety_monitor")
    assert len(nodes) == 2, (
        f"safety_monitor の定義は2件のはず（実際 {len(nodes)} 件）")

    def _base_targets() -> list:
        for node in ast.walk(tree):
            if (isinstance(node, ast.Assign)
                    and any(isinstance(t, ast.Name) and t.id == "SAFETY_ENABLED_TARGETS"
                            for t in node.targets)
                    and isinstance(node.value, (ast.List, ast.Tuple))):
                return [e.value for e in node.value.elts if isinstance(e, ast.Constant)]
        raise AssertionError("SAFETY_ENABLED_TARGETS の代入が見つからない")

    def _targets(node) -> list:
        # enabled_targets は変数参照（SAFETY_ENABLED_TARGETS）または
        # 変数＋リストの BinOp（+ ['localization']）で渡す。素の List ではない。
        for kw in node.keywords:
            if kw.arg != "parameters" or not isinstance(kw.value, ast.List):
                continue
            for elt in kw.value.elts:
                if not isinstance(elt, ast.Dict):
                    continue
                for k, v in zip(elt.keys, elt.values):
                    if not (isinstance(k, ast.Constant)
                            and k.value == "enabled_targets"):
                        continue
                    if isinstance(v, ast.Name) and v.id == "SAFETY_ENABLED_TARGETS":
                        return _base_targets()
                    if isinstance(v, ast.BinOp) and isinstance(v.op, ast.Add):
                        extra = []
                        for side in (v.left, v.right):
                            if isinstance(side, ast.List):
                                extra += [e.value for e in side.elts
                                          if isinstance(e, ast.Constant)]
                        return _base_targets() + extra
        return []

    with_loc = [n for n in nodes if "localization" in _targets(n)]
    without_loc = [n for n in nodes if "localization" not in _targets(n)]
    assert len(with_loc) == 1 and len(without_loc) == 1, (
        "片方だけに localization が入っていない（両定義を確認）")
    assert "IfCondition" in ast.dump(with_loc[0]), "あり側が IfCondition でない"
    assert "UnlessCondition" in ast.dump(without_loc[0]), "なし側が UnlessCondition でない"
    for n in nodes:
        assert "runaway" not in _targets(n), (
            "W-06: runaway を戻してはいけない（範囲外）")


def test_localization_gate_matches_slam_and_amcl_conditions():
    """ゲート条件が SLAM 条件 OR AMCL 条件であること。
    brief の「stage>=3 または enable_route_slam」だけでは map_yaml 指定＋
    stage<3＋enable_route_slam の矛盾した呼び出しで誤発火するため、
    map_yaml の両分岐を見る（実装時の是正）。"""
    src = _read(BRINGUP_PY)
    tree = ast.parse(src, filename=BRINGUP_PY)
    func = _find_function(tree, "generate_launch_description")
    found = None
    for node in ast.walk(func):
        if (isinstance(node, ast.Assign)
                and any(isinstance(t, ast.Name) and t.id == "localization_enabled"
                        for t in node.targets)):
            found = node
    assert found is not None, "localization_enabled の定義が無い"
    seg = ast.get_source_segment(src, found) or ""
    assert "map_yaml" in seg, "ゲートが map_yaml を見ていない"
    assert "enable_route_slam" in seg, "ゲートが enable_route_slam を見ていない"
    assert "stage" in seg, "ゲートが stage を見ていない"


def test_publisher_uses_the_same_gate():
    """localization_health ノードが safety と同じ条件式で起動すること。
    （条件がずれると、監視だけ有効／publisher だけ起動の不一致が起きる）"""
    tree = ast.parse(_read(BRINGUP_PY), filename=BRINGUP_PY)
    nodes = _nodes_by_name(tree, "localization_health")
    assert len(nodes) == 1, "localization_health の定義が1件でない"
    dump = ast.dump(nodes[0])
    assert "IfCondition" in dump and "localization_enabled" in dump, (
        "publisher が localization_enabled 条件で起動していない")


# ============================================================================
# safety_monitor.cpp の配線
# ============================================================================

def test_safety_cpp_wiring():
    """宣言・ゲート・保持・購読が揃っていること。振る舞いは Docker で見る。"""
    src = _read(SAFETY_CPP)
    assert 'declare_parameter("localization_topic_timeout_ms"' in src
    assert 'get_parameter("localization_topic_timeout_ms")' in src
    assert 'targetEnabled("localization")' in src
    assert 'updateFaultState("LOCALIZATION_LOST"' in src
    assert "localization_dead_hold_.update(" in src
    assert '"/safety/localization_health"' in src
    assert "localization_health.hpp" in src


# ============================================================================
# LocalizationHealth.msg の reason 定義（B を出さない）
# ============================================================================

def test_health_msg_defines_reasons_and_reserves_low_confidence():
    src = _read(HEALTH_MSG)
    for token in ("bool ok", "string reason", "transform_age_sec", "node_present",
                  "stale", "node_down", "low_confidence", "jump"):
        assert token in src, f"LocalizationHealth.msg に {token!r} が無い"
    assert "出さない" in src, "low_confidence を出さない旨の注記が無い"


# ============================================================================
# ノード配線（localization_health.py。本文書の試験だけでは「通るだけ」に
# なるため、純粋関数と実装の接続点を縛る。DEV-01A の結びつき試験と同型）
# ============================================================================

def test_health_node_wires_detect_jump():
    """ノードが detect_jump を呼び、結果を reason=jump に載せること。
    呼び出し削除・reason 付け替えで赤くなる。"""
    src = _read(HEALTH_NODE_PY)
    # 呼び出し点そのものを縛る（docstring の `detect_jump()` という文字列だけ
    # では、呼び出し削除で赤くならない。DEV-01A 変異C の教訓）。
    assert "jump = detect_jump(self._prev_sample, curr, p)" in src, (
        "jump 判定の呼び出しが無い")
    assert "REASON_JUMP" in src, "reason=jump への載せ替えが無い"
    assert "self._prev_sample = None" in src, (
        "TF 不連続時の履歴捨てが無い（読み直し直後の飛びを拾ってしまう）")
    # タイマ周期は比較周期（jump_window_ms）に連動する。数値リテラル禁止。
    assert "jump_window_ms" in src
    assert "create_timer" in src
    # jump は起動猶予中に出さない（A・C と同じ考え方）。
    assert "in_warmup" in src
    # jump パラメータ 3 件の宣言（既定値なし・外部から必ず渡す。R2）。
    for param in ("jump_window_ms", "jump_translation_m", "jump_rotation_rad"):
        assert f"'{param}'" in src, f"パラメータ '{param}' の宣言が無い"
