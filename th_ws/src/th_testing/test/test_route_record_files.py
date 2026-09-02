"""
test_route_record_files.py
===========================
経路ファイルの保存系（WS-9H）のテスト。route_record_core の純関数
（save_route_atomic / finalize_route_file / autosave_path / previous_path /
list_finalized_route_files）を、実ファイルで検証する。

ROS2 なし・純粋 Python。pytest の tmp_path フィクスチャを使って実ディレクトリを
作る。

背景: 教示は 10 分前後かかる。いまは finalize（画面「保存」）の 1 回だけしか
ファイルに書かず、その間の点列はメモリにしか無い（落ちると 10 分の成果が消える）。
さらに同名で保存し直すと旧版 <id>.json が丸ごと消える（EXCEPTION-LEDGER W-04）。
ここでは「途中経過の .wip 自動保存」「旧版の .prev 退避」「原子的書き込み」を固定する。
"""

import ast
import json
import os
import sys

import pytest

# route_recorder.py のソースを ast で静的検証するためのパス（WS-9H-2）
_SCRIPTS = os.path.abspath(os.path.join(
    os.path.dirname(__file__), '..', '..', 'th_planning', 'scripts'))
ROUTE_RECORDER = os.path.join(_SCRIPTS, 'route_recorder.py')

# ── パスを通す（colcon build 前にも直接 pytest できるように）
sys.path.insert(0, os.path.join(
    os.path.dirname(__file__),
    '..', '..', 'th_planning', 'th_planning'))

from route_record_core import (
    autosave_path, previous_path, finalized_path,
    save_route_atomic, finalize_route_file, list_finalized_route_files,
)


def _mkroutes(tmp_path):
    d = tmp_path / 'routes'
    d.mkdir()
    return str(d)


def test_save_route_atomic_writes_readable_json(tmp_path):
    routes_dir = _mkroutes(tmp_path)
    dest = finalized_path(routes_dir, 'r')
    data = {'id': 'r', 'points': [[0.0, 0.0, 0.0], [1.0, 1.0, 0.1]]}
    save_route_atomic(dest, data)
    assert json.load(open(dest, encoding='utf-8')) == data


def test_save_route_atomic_uses_temp_then_replace(tmp_path, monkeypatch):
    """一時ファイルに書いてから os.replace で差し替える（半分書きの JSON を読ませない）。

    変更: save_route_atomic を「本体へ直接 open して書く」に変えると、os.replace が
    呼ばれず、このテストが落ちる（変異チェック 2）。
    """
    import route_record_core as m
    dest = finalized_path(str(tmp_path), 'r')
    calls = []
    real_replace = m.os.replace

    def fake_replace(src, dst):
        calls.append((src, dst))
        real_replace(src, dst)

    monkeypatch.setattr(m.os, 'replace', fake_replace)
    save_route_atomic(dest, {'id': 'r'})
    assert len(calls) == 1, 'os.replace が 1 回だけ呼ばれること（原子的差し替え）'
    _src, dst = calls[0]
    assert dst == dest                     # 差し替え先は本体
    assert _src != dest                    # 元は一時ファイル（名前が本体と異なる）
    assert os.path.basename(_src).startswith('.')   # 一時ファイルは隠し名


def test_save_route_atomic_leaves_no_partial_or_temp(tmp_path):
    routes_dir = _mkroutes(tmp_path)
    dest = finalized_path(routes_dir, 'r')
    save_route_atomic(dest, {'id': 'r'})
    # 書き込み直後は完成品の .json だけが残り、一時ファイルが残らない
    assert sorted(os.listdir(routes_dir)) == ['r.json']
    assert json.load(open(dest, encoding='utf-8')) == {'id': 'r'}


def test_finalize_route_file_backs_up_existing_to_prev(tmp_path):
    routes_dir = _mkroutes(tmp_path)
    d1 = {'id': 'r', 'points': [[0.0, 0.0, 0.0]]}
    d2 = {'id': 'r', 'points': [[9.0, 9.0, 0.0]]}
    finalize_route_file(routes_dir, 'r', d1)
    finalize_route_file(routes_dir, 'r', d2)
    assert json.load(open(finalized_path(routes_dir, 'r'), encoding='utf-8')) == d2
    assert json.load(open(previous_path(routes_dir, 'r'), encoding='utf-8')) == d1


def test_finalize_route_file_keeps_only_latest_and_one_previous(tmp_path):
    routes_dir = _mkroutes(tmp_path)
    finalize_route_file(routes_dir, 'r', {'g': 1})
    finalize_route_file(routes_dir, 'r', {'g': 2})
    finalize_route_file(routes_dir, 'r', {'g': 3})
    # 最新と 1 つ前の 2 つだけが残る（世代は積み上がらない）
    assert sorted(os.listdir(routes_dir)) == ['r.json', 'r.prev']
    assert json.load(open(finalized_path(routes_dir, 'r'), encoding='utf-8')) == {'g': 3}
    assert json.load(open(previous_path(routes_dir, 'r'), encoding='utf-8')) == {'g': 2}


def test_finalize_route_file_removes_leftover_autosave(tmp_path):
    routes_dir = _mkroutes(tmp_path)
    wip = routes_dir + '/r.wip'
    with open(wip, 'w', encoding='utf-8') as f:
        json.dump({'id': 'r'}, f)
    finalize_route_file(routes_dir, 'r', {'id': 'r'})
    assert not os.path.exists(wip)                      # 途中経過は消える
    assert os.path.exists(finalized_path(routes_dir, 'r'))


def test_catalog_list_excludes_wip_and_prev(tmp_path):
    routes_dir = _mkroutes(tmp_path)
    # 完成品だけを一覧に載せ、途中経過(.wip)・1 世代前(.prev)は出さない。
    for fname in ('a.json', 'b.wip', 'c.prev', 'd.wip'):
        with open(os.path.join(routes_dir, fname), 'w', encoding='utf-8') as f:
            f.write('{}')
    assert list_finalized_route_files(routes_dir) == ['a.json']


def test_dotdot_and_slash_do_not_escape_routes_dir(tmp_path):
    base = str(tmp_path / 'base')
    os.makedirs(base)
    # `../evil` や `a/b` は routes_dir の外へ書けない
    finalize_route_file(base, '../evil', {'id': 'evil'})
    finalize_route_file(base, 'a/b', {'id': 'b'})
    made = os.listdir(base)
    # 作られたファイル名はすべて平ら（パス区切りを含まない＝サブディレクトリへも
    # 外へも逃げていない）。.. は名前の一部として字面に残ることはあるが、区切りが
    # 無ければ base の外には書けない。
    assert all('/' not in n and '\\' not in n for n in made)
    # tmp_path 直下（base の外）に evil 系のファイルは無い
    assert not os.path.exists(os.path.join(str(tmp_path), 'evil.json'))
    assert not os.path.exists(os.path.join(str(tmp_path), 'evil'))
    assert all(os.path.dirname(os.path.join(base, n)) == base for n in made)
    # 無害化した名前に .json で保存されている
    assert os.path.exists(finalized_path(base, '../evil'))
    assert os.path.exists(finalized_path(base, 'a/b'))
    assert 'a_b.json' in made


def test_autosave_path_sanitizes_dotdot(tmp_path):
    routes_dir = _mkroutes(tmp_path)
    p = autosave_path(routes_dir, '../../evil')
    assert p.startswith(routes_dir)
    assert '/../' not in '/' + p


# ============================================================================
# WS-9H-2: 2 回目の教示で自動保存が効かない欠陥の修正を固定する静的テスト。
# ノードは立てず、route_recorder.py のソースを ast で読む（test_route_preview_bandwidth
# の _read / ast.parse の書き方に倣う）。
#
# 欠陥: 自動保存の間引き用状態 (_last_autosave_s / _last_autosaved_points /
# _autosave_logged) が __init__ で 1 回初期化されるだけで start_record でリセット
# されなかった。start_record は毎回新しい RouteRecorderCore を作り点数が 0 から
# 数え直しになるのに、前の教示の点数 (例 1000) が残っていると _maybe_autosave の
# 「点が増えていないときは書かない」(point_count <= _last_autosaved_points) が
# 常に成立し、2 本目の自動保存が丸ごと抑止される。録り直しこそ本機能の主用途。
# resume_record は同じ recorder を使い続けるためリセットしてはいけない。
# ============================================================================


def _read(path: str) -> str:
    with open(path, encoding='utf-8') as f:
        return f.read()


def _tree(path: str) -> ast.AST:
    return ast.parse(_read(path), filename=path)


def _iter_effect_branches(funcdef):
    """_on_effect の if name == '...' / elif のチェーンを (効果名, body) 列として返す。

    _on_effect は先頭に `name = msg.name` を持つため、関数本体の最初の If 文を探す。
    """
    first_if = next((s for s in funcdef.body if isinstance(s, ast.If)), None)
    node = first_if
    while isinstance(node, ast.If):
        if (isinstance(node.test, ast.Compare)
                and isinstance(node.test.left, ast.Name) and node.test.left.id == 'name'
                and len(node.test.ops) == 1 and isinstance(node.test.ops[0], ast.Eq)
                and isinstance(node.test.comparators[0], ast.Constant)):
            yield node.test.comparators[0].value, node.body
        if node.orelse and isinstance(node.orelse[0], ast.If):
            node = node.orelse[0]
        else:
            break


def _branch_calls_self_method(body_nodes, method: str) -> bool:
    """分岐 body の中で self.<method>() を呼び出しているか。"""
    for node in ast.walk(ast.Module(body=body_nodes, type_ignores=[])):
        if (isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and isinstance(node.func.value, ast.Name)
                and node.func.value.id == 'self'
                and node.func.attr == method):
            return True
    return False


def _effect_branch_calls_method(tree, effect_name: str, method: str) -> bool:
    """_on_effect の効果分岐 effect_name の中で self.<method>() が呼ばれるか。"""
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == '_on_effect':
            for name, body in _iter_effect_branches(node):
                if name == effect_name:
                    return _branch_calls_self_method(body, method)
    return False


def _reset_autosave_assignments(tree):
    """_reset_autosave_state 本体の `self.<attr> = <定数値>` 代入を (attr, 値) 列で返す。

    RHS がリテラルでない代入は値が None になる。定義が無ければ None を返す。
    """
    funcdef = next((n for n in ast.walk(tree)
                    if isinstance(n, ast.FunctionDef) and n.name == '_reset_autosave_state'),
                   None)
    if funcdef is None:
        return None
    assigns = []
    body = ast.Module(body=funcdef.body, type_ignores=[])
    for stmt in ast.walk(body):
        if isinstance(stmt, ast.Assign):
            const = stmt.value.value if isinstance(stmt.value, ast.Constant) else None
            for target in stmt.targets:
                if (isinstance(target, ast.Attribute)
                        and isinstance(target.value, ast.Name) and target.value.id == 'self'):
                    assigns.append((target.attr, const))
    return assigns


def test_route_recorder_has_reset_autosave_state_definition():
    """WS-9H-2: _reset_autosave_state の定義があること。"""
    tree = _tree(ROUTE_RECORDER)
    assert any(isinstance(n, ast.FunctionDef) and n.name == '_reset_autosave_state'
               for n in ast.walk(tree)), (
        'route_recorder.py に _reset_autosave_state の定義が無い')


def test_start_record_branch_resets_autosave_state():
    """WS-9H-2: start_record 分岐の中で _reset_autosave_state() が呼ばれること。

    変異チェック 1: この呼び出しを消すとテストが落ちる。
    """
    assert _effect_branch_calls_method(
        _tree(ROUTE_RECORDER), 'start_record', '_reset_autosave_state') is True, (
        'start_record 分岐の中で _reset_autosave_state() が呼ばれていない')


def test_resume_record_branch_does_not_reset_autosave_state():
    """WS-9H-2: resume_record 分岐では呼ばれないこと（同じ recorder を使い続ける）。

    変異チェック 2: resume_record にも足すとテストが落ちる。
    """
    assert _effect_branch_calls_method(
        _tree(ROUTE_RECORDER), 'resume_record', '_reset_autosave_state') is False, (
        'resume_record 分岐でも _reset_autosave_state() が呼ばれている（点数比較が壊れる）')


def test_reset_autosave_state_body_resets_all_fields():
    """WS-9H-2（追補）: _reset_autosave_state 本体が 3 状態すべてをまっさらに戻す。

    既存のテストは「ヘルパが呼ばれているか」しか見ておらず、本体が何を戻すかを
    固定していない。`self._last_autosaved_points = 0` の 1 行だけ消す変異では
    2 本目の教示で自動保存が抑止される欠陥がそのまま復活するのに全テストが緑の
    ままであった。ここでは代入先 3 つが揃っていることと、_last_autosaved_points
    の代入値が 0 であることを固定する。
    """
    assigns = _reset_autosave_assignments(_tree(ROUTE_RECORDER))
    assert assigns is not None, 'route_recorder.py に _reset_autosave_state の定義が無い'
    by_attr = {attr: const for attr, const in assigns}
    assert set(by_attr) == {'_last_autosave_s', '_last_autosaved_points', '_autosave_logged'}, (
        f'_reset_autosave_state 本体が 3 状態を戻し切れていない: {sorted(by_attr)}')
    assert by_attr['_last_autosaved_points'] == 0, (
        '_last_autosaved_points の代入値が 0 でない（欠陥が復活し得る）')
