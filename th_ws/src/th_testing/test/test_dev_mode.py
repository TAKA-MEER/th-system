"""test_dev_mode.py — WP-DEV-01A（開発モードの土台）の試験。

ROS2 不要・ホストの素の pytest で走る（`connectivity_core` の純粋関数と、
launch / ノードのソーステキストに対する AST・文字列検査だけ）。

対応する完了条件:
  1. `dev_mode:=true` で機器なしでも `INIT/CHECK` を抜けられる
     → dev 分岐（4 項目除外）＋ E-Stop 未受信でも通す gate の試験
  2. `dev_mode:=false`（既定）では今と完全に同じ挙動
     → dev 既定 OFF・sim 分岐不変・既存テスト群がそのまま緑の試験
  3. `dev_mode` パラメータは `safety_monitor` / `obstacle_limiter` に渡さない。
     両ノードは `/system/dev_mode` を購読し effective の項目だけに反応する
     （2026-09-23 改定。Spec-safety.md §10）→ launch の AST 試験＋項目名の一致試験。
     振る舞いは th_safety の gtest（test_dev_mode_core）と
     test_dev_mode_safety_node.py（launch_testing）で縛る。
  5. 開発モードに入っただけでは通常運用と同じ（項目の既定は全部 OFF）
     → connectivity_checker の宣言・launch の dev_ignore 引数の試験
  4. 物理 E-Stop 押下中は `dev_mode:=true` でも `evt.link_ok` が出ない
     → `should_emit_link_ok()` の真理値表試験

変異チェック（実装をわざと壊して赤くなること）の記録はコミットメッセージにある。
"""
from __future__ import annotations

import ast
import os
import re

import pytest

from th_state.connectivity_core import Params, evaluate, should_emit_link_ok


_SRC_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
BRINGUP_PY = os.path.join(_SRC_ROOT, "th_bringup", "launch", "bringup.launch.py")
CHECKER_PY = os.path.join(_SRC_ROOT, "th_state", "scripts", "connectivity_checker.py")
DEV_CORE_HPP = os.path.join(_SRC_ROOT, "th_safety", "include", "th_safety", "dev_mode_core.hpp")
WEB_DEV_STATE_JS = os.path.join(_SRC_ROOT, "..", "web_ui", "src", "ros", "devModeState.js")


def _read(path: str) -> str:
    with open(path, encoding="utf-8") as f:
        return f.read()


def _params(**overrides):
    base = dict(
        esp32_alive_timeout_ms=3000,
        scan_expected_points=360,
        required_nodes=("state_manager", "safety_monitor"),
    )
    base.update(overrides)
    return Params(**base)


# 機器が何も無い状態（受信なし・ノードなし・点数不一致）。
_NOTHING_KWARGS = dict(
    now_ms=10_000,
    last_fb_ms=None,
    last_cmd_ms=None,
    last_scan_ms=None,
    scan_points=0,
    present_nodes=(),
)


# ============================================================================
# 1. dev 分岐（connectivity_core.evaluate）
# ============================================================================

def test_dev_ignore_link_bypasses_all_four_items():
    """完了条件1: dev 実効時は ESP32 の2項目・lidar・nodes の4項目すべてを除外する。

    `sim` 分岐より広い（lidar を含む）。完了条件が「ESP32 もラズパイも
    繋がっていない状態」なので、lidar を残すと IDLE に到達できない。
    """
    p = _params(dev_ignore_link=True)
    r = evaluate(p=p, **_NOTHING_KWARGS)
    assert r.esp32_feedback is True
    assert r.esp32_loopback is True
    assert r.lidar is True
    assert r.nodes is True
    assert r.missing_nodes == ()
    assert r.all_ok()


def test_dev_off_by_default():
    """完了条件2: dev は既定 OFF（安全側）。何も無い状態では通らない。"""
    p = _params()
    assert p.dev_ignore_link is False
    r = evaluate(p=p, **_NOTHING_KWARGS)
    assert not r.all_ok()


def test_sim_branch_unchanged():
    """完了条件2（続き）: 既存の sim 分岐を壊さない。sim は lidar を残す。"""
    p = _params(sim=True)
    r = evaluate(p=p, **_NOTHING_KWARGS)
    assert r.esp32_feedback is True
    assert r.esp32_loopback is True
    assert r.nodes is True
    # Gazebo でも /scan は出る想定なので、lidar の判定は残る。
    assert r.lidar is False
    assert not r.all_ok()


def test_sim_and_dev_are_idempotent():
    """sim と dev の両方が真でも all_ok（除外の除外でおかしくならない）。"""
    p = _params(sim=True, dev_ignore_link=True)
    r = evaluate(p=p, **_NOTHING_KWARGS)
    assert r.all_ok()


# ============================================================================
# 2. gate 真理値表（should_emit_link_ok）
# ============================================================================

def _ok_report():
    return evaluate(
        p=_params(),
        now_ms=10_000,
        last_fb_ms=9_500,
        last_cmd_ms=9_600,
        last_scan_ms=9_800,
        scan_points=360,
        present_nodes=("state_manager", "safety_monitor"),
    )


def _ng_report():
    return evaluate(p=_params(), **_NOTHING_KWARGS)


def test_gate_normal_path_unchanged():
    """完了条件2（続き）: dev 無効時の gate は従来式と同一。"""
    ok, ng = _ok_report(), _ng_report()
    assert ok.all_ok() and not ng.all_ok()
    # 全部揃い＋E-Stop 受信済み＋非押下 → 出す
    assert should_emit_link_ok(ok, True, False, False) is True
    # E-Stop 未受信 → 出さない（L-2 のフェイルセーフ既定・CL-B-6）
    assert should_emit_link_ok(ok, False, False, False) is False
    # 押下中 → 出さない
    assert should_emit_link_ok(ok, True, True, False) is False
    # 項目 NG → 出さない
    assert should_emit_link_ok(ng, True, False, False) is False


def test_gate_pressed_never_passes_even_in_dev():
    """完了条件4: 押下中は dev 実効時でも出さない（無視できないものの境界）。"""
    ok, ng = _ok_report(), _ng_report()
    assert should_emit_link_ok(ok, True, True, True) is False
    assert should_emit_link_ok(ng, True, True, True) is False
    # 未受信＋押下（受信前の初期値 False と区別できない组合せ）も出さない
    assert should_emit_link_ok(ng, False, True, True) is False


def test_gate_dev_passes_without_estop_seen():
    """完了条件1の裏側: ESP32 不在時は /safety/estop_hw 自体が無音なので、
    未受信でも通す（押下判明時を除く）。通した事実はノードがログに残す。"""
    ng = _ng_report()
    assert should_emit_link_ok(ng, False, False, True) is True
    assert should_emit_link_ok(_ok_report(), True, False, True) is True


# ============================================================================
# 3. launch 定義の AST 検査（bringup.launch.py）
# ============================================================================

def _find_declare_arg(tree: ast.AST, name: str):
    """DeclareLaunchArgument('<name>', ...) 呼び出しを探して返す（無ければ None）。"""
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call)
                and isinstance(node.func, ast.Name)
                and node.func.id == "DeclareLaunchArgument"
                and node.args
                and isinstance(node.args[0], ast.Constant)
                and node.args[0].value == name):
            return node
    return None


def _find_nodes_by_name(tree: ast.AST, name: str) -> list:
    """`nodes.append(Node(..., name='<name>', ...))` の Node 呼び出し一覧。"""
    rows = []
    for node in ast.walk(tree):
        if not (isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == "append"
                and isinstance(node.func.value, ast.Name)
                and node.func.value.id == "nodes"
                and node.args):
            continue
        arg = node.args[0]
        if not (isinstance(arg, ast.Call)
                and isinstance(arg.func, ast.Name) and arg.func.id == "Node"):
            continue
        for kw in arg.keywords:
            if (kw.arg == "name" and isinstance(kw.value, ast.Constant)
                    and kw.value.value == name):
                rows.append(arg)
    return rows


def test_bringup_declares_dev_mode_arg_default_false():
    """やること§1: dev_mode 引数の宣言（既定 false）。"""
    src = _read(BRINGUP_PY)
    assert os.path.isfile(BRINGUP_PY), "bringup.launch.py が無い"
    tree = ast.parse(src, filename=BRINGUP_PY)
    call = _find_declare_arg(tree, "dev_mode")
    assert call is not None, "bringup.launch.py に DeclareLaunchArgument('dev_mode') が無い"
    defaults = {kw.arg: kw.value for kw in call.keywords}
    assert "default_value" in defaults, "dev_mode 引数に default_value が無い"
    assert (isinstance(defaults["default_value"], ast.Constant)
            and defaults["default_value"].value == "false"), (
        "dev_mode の既定は 'false' でなければならない（完了条件2）")


def test_bringup_passes_dev_mode_only_to_connectivity_checker():
    """やること§1: dev_mode を受け取るのは connectivity_checker だけ。
    If/Unless で排他的に true/false の両定義があること。

    注意: `ast.dump()` は識別子も single-quote で出すため、条件式の変数参照
    （`IfCondition(dev_mode)`）に `'dev_mode'` の文字列検査がヒットしてしまう。
    parameters 辞書の**キー**として厳密に見る（変異Cの教訓）。
    """
    src = _read(BRINGUP_PY)
    tree = ast.parse(src, filename=BRINGUP_PY)
    nodes = _find_nodes_by_name(tree, "connectivity_checker")
    assert len(nodes) == 2, (
        f"connectivity_checker の Node 定義は If/Unless の2件のはず（実際 {len(nodes)} 件）")
    values = set()
    for node in nodes:
        seg = ast.dump(node)
        assert "IfCondition" in seg or "UnlessCondition" in seg, (
            "connectivity_checker の定義に条件が無い")
        assert "dev_mode" in seg, "条件が dev_mode を見ていない"
        found = _node_param_dict_entries(node)
        assert "dev_mode" in found, (
            "connectivity_checker の parameters 辞書に 'dev_mode' キーが無い"
            "（条件だけ残して受け渡しを削ると起動時既定のままになる）")
        values.add(found["dev_mode"])
    assert values == {True, False}, (
        f"If/Unless の両定義で dev_mode=True/False を渡すこと（実際 {values}）")


def _node_param_dict_entries(node_call: ast.Call) -> dict:
    """Node(...) の parameters=[..., {key: value}, ...] の辞書エントリを集める。"""
    entries = {}
    for kw in node_call.keywords:
        if kw.arg != "parameters" or not isinstance(kw.value, ast.List):
            continue
        for elt in kw.value.elts:
            if not isinstance(elt, ast.Dict):
                continue
            for k, v in zip(elt.keys, elt.values):
                if (isinstance(k, ast.Constant) and isinstance(k.value, str)
                        and isinstance(v, ast.Constant)):
                    entries[k.value] = v.value
    return entries


def test_bringup_does_not_pass_dev_mode_to_safety_or_limiter():
    """完了条件3: dev_mode が safety_monitor / obstacle_limiter に現れない。
    構造的な保証（両ノードが値を知らなければ無効化は起きない）。"""
    src = _read(BRINGUP_PY)
    tree = ast.parse(src, filename=BRINGUP_PY)
    for name in ("safety_monitor", "obstacle_limiter"):
        nodes = _find_nodes_by_name(tree, name)
        assert nodes, f"bringup.launch.py に name={name!r} の Node 定義が無い"
        for node in nodes:
            seg = ast.get_source_segment(src, node) or ""
            assert "dev_mode" not in seg, (
                f"name={name!r} の Node 定義に dev_mode が混入している（完了条件3違反）")
    # 保証の意図を示すコメントが両箇所に残っていること（消したら赤）。
    assert src.count("# dev_mode は渡さない") >= 2, (
        "dev_mode 非伝播のコメントが消えている（safety_monitor 側・obstacle_limiter 側の2箇所）")


def test_bringup_logs_dev_mode_at_startup():
    """やること§1: 起動ログに dev_mode=true/false を出す（stage ログと同じ場所）。"""
    src = _read(BRINGUP_PY)
    assert "dev_mode=" in src, "起動ログに dev_mode= が無い"


# ============================================================================
# 4. ノード実装の結びつき（connectivity_checker.py。本文書の試験だけでは
#    「通るだけ」になるため、純粋関数の試験と実装の接続点を縛る）
# ============================================================================

def test_checker_uses_pure_gate_and_publishes_dev_state():
    """ノードが should_emit_link_ok を使い、/system/dev_mode を出すこと。
    旧インライン式への復帰・トピック削除で赤くなる。"""
    src = _read(CHECKER_PY)
    assert "should_emit_link_ok" in src, "gate に純粋関数を使っていない"
    assert "report.all_ok() and self._estop_seen" not in src, (
        "旧インライン gate 式が残っている（純粋関数への置換が不完全）")
    assert "'/system/dev_mode'" in src, "/system/dev_mode の発行が無い"
    for param in ("dev_mode", "dev_ignore_at_start"):
        assert f"'{param}'" in src, f"パラメータ '{param}' の宣言が無い"


# ============================================================================
# 5. 項目（2026-09-23 改定。Spec-safety.md §10）
# ============================================================================

EXPECTED_DEV_ITEMS = ("link", "lidar_fault", "scan_stop", "battery", "opcheck", "auto_brake")


def _checker_dev_items() -> tuple:
    tree = ast.parse(_read(CHECKER_PY), filename=CHECKER_PY)
    for node in ast.walk(tree):
        if (isinstance(node, ast.Assign) and len(node.targets) == 1
                and isinstance(node.targets[0], ast.Name)
                and node.targets[0].id == "_DEV_ITEMS"):
            return tuple(ast.literal_eval(node.value))
    raise AssertionError("connectivity_checker.py に _DEV_ITEMS が無い")


def test_dev_items_match_across_checker_web_and_cpp():
    """項目名が 3 か所（正本・WebUI・C++）で一致する。ずれると画面で選んでも効かない。"""
    assert _checker_dev_items() == EXPECTED_DEV_ITEMS
    hpp = _read(DEV_CORE_HPP)
    assert 'kDevItemLidarFault = "lidar_fault"' in hpp
    assert 'kDevItemScanStop   = "scan_stop"' in hpp


def test_dev_items_match_web():
    """WebUI の DEV_ITEMS も同じ。Docker（web_ui 未マウント）ではスキップし、ホストで縛る。"""
    if not os.path.isfile(WEB_DEV_STATE_JS):
        pytest.skip("web_ui がこの環境に無い（Docker の th_robot は src だけをマウントする）")
    js = _read(WEB_DEV_STATE_JS)
    m = re.search(r"export const DEV_ITEMS = \[([^\]]*)\]", js)
    assert m, "devModeState.js に DEV_ITEMS が無い"
    assert tuple(re.findall(r"'([a-z_]+)'", m.group(1))) == EXPECTED_DEV_ITEMS


def test_checker_dev_ignore_defaults_are_false():
    """開発モードに入っただけでは何も外さない（項目の既定は偽）。"""
    src = _read(CHECKER_PY)
    assert "self.declare_parameter(f'dev_ignore_{item}', False)" in src, (
        "dev_ignore_* の既定が偽で宣言されていない")
    assert "for item in self._DEV_ITEMS:" in src
    assert "declare_parameter('dev_ignore_link', True)" not in src


def test_bringup_declares_dev_ignore_arg_and_passes_it_as_string():
    """launch 引数 dev_ignore（既定 ''）を connectivity_checker の dev_ignore_at_start に
    文字列として渡す（両定義とも）。"""
    src = _read(BRINGUP_PY)
    tree = ast.parse(src, filename=BRINGUP_PY)
    call = _find_declare_arg(tree, "dev_ignore")
    assert call is not None, "DeclareLaunchArgument('dev_ignore') が無い"
    defaults = {kw.arg: kw.value for kw in call.keywords}
    assert isinstance(defaults.get("default_value"), ast.Constant)
    assert defaults["default_value"].value == "", "dev_ignore の既定は '' のはず"
    nodes = _find_nodes_by_name(tree, "connectivity_checker")
    assert len(nodes) == 2
    for node in nodes:
        seg = ast.get_source_segment(src, node) or ""
        assert "'dev_ignore_at_start': ParameterValue(dev_ignore, value_type=str)" in seg


def test_safety_nodes_follow_dev_mode_topic_only_via_core():
    """safety_monitor / obstacle_limiter は /system/dev_mode を購読し、鮮度込みの
    dev_item_effective() で自分の項目だけを見る（生の JSON を直接読まない）。"""
    for fname, const in (("safety_monitor.cpp", "kDevItemLidarFault"),
                         ("obstacle_limiter.cpp", "kDevItemScanStop")):
        src = _read(os.path.join(_SRC_ROOT, "th_safety", "src", fname))
        assert '"/system/dev_mode"' in src, f"{fname} が /system/dev_mode を購読していない"
        assert "parse_dev_effective(msg->data)" in src, f"{fname} が effective を読んでいない"
        assert "dev_item_effective(" in src
        assert f"th_safety::{const}" in src, f"{fname} が項目 {const} を見ていない"
