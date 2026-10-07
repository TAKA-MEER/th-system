"""
test_teach_record_broken.py
=============================
1b-7 SG-B18（SM-3.1.2-019）: 記録の連続性が切れたら evt.record_broken を出す。

route_recorder が自己位置の姿勢を見て切れ目（途絶・飛び）を検出し、
/system/event に evt.record_broken を出す（T-TEACH-06 → IDLE＋ask_save の入口）。
しきい値は registry.yaml（route_gap_timeout_ms・route_jump_m）で、ノード内
リテラルにしない（W-03 の検査が値・型・生成物を縛る。ここでは宣言の存在と
既定値が registry と一致すること＋配線の AST を縛る）。

ROS2 不要・純粋 Python＋AST。launch 側は test_teach_record_broken_node.py。
"""
import ast
import os
import sys

import pytest
import yaml

_PLAN_SCRIPTS = os.path.abspath(os.path.join(
    os.path.dirname(__file__), '..', '..', 'th_planning', 'scripts'))
ROUTE_RECORDER = os.path.join(_PLAN_SCRIPTS, 'route_recorder.py')

_PARAMS_CONFIG = os.path.abspath(os.path.join(
    os.path.dirname(__file__), '..', '..', 'th_params', 'config'))
REGISTRY_YAML = os.path.join(_PARAMS_CONFIG, 'registry.yaml')


def _read(path: str) -> str:
    with open(path, encoding='utf-8') as f:
        return f.read()


def _tree(path: str) -> ast.AST:
    return ast.parse(_read(path), filename=path)


def _method_def(tree, name):
    return next(
        (n for n in ast.walk(tree)
         if isinstance(n, ast.FunctionDef) and n.name == name), None)


def _method_calls_self(tree, method, callee) -> bool:
    node = _method_def(tree, method)
    if node is None:
        return False
    return any(
        isinstance(n, ast.Call)
        and isinstance(n.func, ast.Attribute)
        and isinstance(n.func.value, ast.Name) and n.func.value.id == 'self'
        and n.func.attr == callee
        for n in ast.walk(ast.Module(body=node.body, type_ignores=[])))


def _method_calls_name(tree, method, name) -> bool:
    node = _method_def(tree, method)
    if node is None:
        return False
    return any(
        isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
        and n.func.id == name
        for n in ast.walk(ast.Module(body=node.body, type_ignores=[])))


def _declare_defaults(path: str) -> dict:
    """`declare_parameter('name', <リテラル>)` を AST で抜く。"""
    out = {}
    for node in ast.walk(_tree(path)):
        if not (isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == 'declare_parameter'
                and len(node.args) >= 2):
            continue
        name, value = node.args[0], node.args[1]
        if isinstance(name, ast.Constant) and isinstance(name.value, str):
            if isinstance(value, ast.Constant):
                out[name.value] = value.value
    return out


def _registry_row(name: str) -> dict:
    with open(REGISTRY_YAML, encoding='utf-8') as f:
        rows = {row['name']: row for row in yaml.safe_load(f)}
    assert name in rows, f'registry.yaml に {name} が無い'
    return rows[name]


# ── しきい値の宣言（registry と同名・同値・同型） ──────────────────────
@pytest.mark.parametrize("name", ['route_gap_timeout_ms', 'route_jump_m'])
def test_continuity_thresholds_declared_matching_registry(name):
    """連続性切れのしきい値を registry と同名で宣言し、既定値が一致すること。
    変異: 宣言を消す・既定値を変えると赤くなる（W-03 の汎用検査も縛るが、
    ここでは SG-B18 の対象として明示する）。"""
    defaults = _declare_defaults(ROUTE_RECORDER)
    assert name in defaults, f'route_recorder.py が {name} を宣言していない'
    row = _registry_row(name)
    assert row['status'] == 'given'
    assert 'route_recorder' in row['consumers']
    assert row['value'] == defaults[name], (
        f"{name}: registry={row['value']!r} != ノード既定={defaults[name]!r}")
    assert type(row['value']) is type(defaults[name])


# ── 配線（_sample_timer → _update_continuity → 純関数 → 発行） ──────────
def test_sample_timer_updates_continuity():
    """_sample_timer が _update_continuity を呼ぶこと（REC 以外の状態でも見る）。
    変異: 呼び出しを消すと赤くなる（ESTOP・手押し中の移動を見逃す）。"""
    tree = _tree(ROUTE_RECORDER)
    assert _method_calls_self(tree, '_sample_timer', '_update_continuity') is True, (
        '_sample_timer が _update_continuity を呼んでいない')


def test_update_continuity_uses_pure_judges_and_latches():
    """_update_continuity が純判定（gap／jump）を使い、切れたらラッチすること。
    変異: 純関数のどちらかを呼ばなくすると赤くなる。"""
    tree = _tree(ROUTE_RECORDER)
    assert _method_calls_name(tree, '_update_continuity', 'pose_gap_broken') is True
    assert _method_calls_name(tree, '_update_continuity', 'pose_jump_broken') is True
    assert _method_calls_self(tree, '_update_continuity', '_on_record_broken') is True


def test_broken_latches_and_freezes_sampling():
    """_on_record_broken が _broken を立て、点の追加が凍結されること。
    変異: ラッチを消すと赤くなる（切れ目以降の点が混ざる）。
    変異: add_pose の凍結条件を消すと赤くなる。"""
    tree = _tree(ROUTE_RECORDER)
    node = _method_def(tree, '_on_record_broken')
    assert node is not None, '_on_record_broken が無い'
    sets_broken = any(
        isinstance(n, ast.Assign)
        and len(n.targets) == 1 and isinstance(n.targets[0], ast.Attribute)
        and isinstance(n.targets[0].value, ast.Name) and n.targets[0].value.id == 'self'
        and n.targets[0].attr == '_broken'
        and isinstance(n.value, ast.Constant) and n.value.value is True
        for n in ast.walk(ast.Module(body=node.body, type_ignores=[])))
    assert sets_broken, '_on_record_broken が self._broken = True にしない'
    sample = _method_def(tree, '_sample_timer')
    assert sample is not None, '_sample_timer が無い'
    src = ast.dump(sample)
    assert '_broken' in src, '_sample_timer が _broken を見ていない（凍結しない）'


def test_broken_event_published_to_system_event():
    """evt.record_broken を /system/event に出すこと（T-TEACH-06 の入口）。
    変異: イベント名を変える・発行先を変えると赤くなる。"""
    src = _read(ROUTE_RECORDER)
    assert "'evt.record_broken'" in src, 'evt.record_broken の発行が無い'
    assert "'/system/event'" in src, '/system/event への発行が無い'
    tree = _tree(ROUTE_RECORDER)
    assert _method_calls_self(
        tree, '_on_record_broken', '_publish_record_broken') is True


def test_broken_reemitted_while_in_teach():
    """ラッチ中は教示系にいる間出し続けること（ESTOP 中に切れた場合、
    戻ってから T-TEACH-06 が拾う）。変異: 再送出を消すと赤くなる。"""
    tree = _tree(ROUTE_RECORDER)
    assert _method_calls_self(
        tree, '_status_timer', '_publish_record_broken') is True, (
        '_status_timer が _publish_record_broken を呼ばない')


def test_broken_latch_cleared_on_leaving_teach_and_on_start():
    """教示系を抜けたらラッチを下ろす（W-4 が引き継ぐ）・新しい教示で
    まっさらに戻すこと。変異: クリアを消すと赤くなる。"""
    tree = _tree(ROUTE_RECORDER)
    for method in ('_on_state',):
        node = _method_def(tree, method)
        assert node is not None
        clears = any(
            isinstance(n, ast.Assign)
            and len(n.targets) == 1 and isinstance(n.targets[0], ast.Attribute)
            and isinstance(n.targets[0].value, ast.Name)
            and n.targets[0].value.id == 'self'
            and n.targets[0].attr == '_broken'
            and isinstance(n.value, ast.Constant) and n.value.value is False
            for n in ast.walk(ast.Module(body=node.body, type_ignores=[])))
        assert clears, f'{method} が self._broken = False にしない'
    # start_record 分岐で _broken = False に戻す。
    found = False
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == '_on_effect':
            for n in ast.walk(node):
                if (isinstance(n, ast.Assign)
                        and len(n.targets) == 1
                        and isinstance(n.targets[0], ast.Attribute)
                        and isinstance(n.targets[0].value, ast.Name)
                        and n.targets[0].value.id == 'self'
                        and n.targets[0].attr == '_broken'
                        and isinstance(n.value, ast.Constant)
                        and n.value.value is False):
                    found = True
    assert found, 'start_record 分岐で self._broken = False にしていない'
