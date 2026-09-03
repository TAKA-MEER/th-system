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
REPLAY_RUNNER = os.path.join(_SCRIPTS, 'replay_runner.py')

# ── パスを通す（colcon build 前にも直接 pytest できるように）
sys.path.insert(0, os.path.join(
    os.path.dirname(__file__),
    '..', '..', 'th_planning', 'th_planning'))

from route_record_core import (
    autosave_path, previous_path, finalized_path,
    save_route_atomic, finalize_route_file, list_finalized_route_files,
    should_autofinalize,
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


# ============================================================================
# WS-9I: 経路名に `/` が入ると保存できるのに再生できない。
#
# route_recorder（保存側）は finalized_path() で `/` `\` を _ に置換して書き、
# replay_runner（再生側）は生の id を `os.path.join(..., f'{route_id}.json')`
# で開いていた。`9/3_koushakukou` のような名前は保存先 <dir>/9_3_koushakukou.json
# にあるのに、再生は <dir>/9/3_koushakukou.json を開こうとして見つからない。
# WS-9H で保存側だけサニタイズした結果の食い違い（保存は通るのに再生で動かない）。
#
# 「保存先と読み込み先が必ず一致する」ことを、純関数・ast・往復の 3 方向で固定する。
# ============================================================================


def test_finalized_path_stays_in_dir_and_is_json(tmp_path):
    """WS-9I: finalized_path(dir, name) が dir 直下に収まり .json で終わる。

    変異チェック 2: _safe_id の置換をやめると、`9/3_...` や `../evil` の dirname が
    dir と一致しなくなり（/ を _ に替えないため）、このテストが落ちる。
    """
    routes_dir = _mkroutes(tmp_path)
    names = ['kousha_01', '校舎1周', '9/3_koushakukou', 'a\\b', '../evil', '']
    for name in names:
        p = finalized_path(routes_dir, name)
        assert os.path.dirname(p) == routes_dir, f'name={name!r}: 先が dir 直下でない: {p!r}'
        assert p.endswith('.json'), f'name={name!r}: .json で終わらない: {p!r}'


def test_replay_runner_uses_finalized_path_and_no_raw_json_fstring(tmp_path):
    """WS-9I: replay_runner が生の f-string でパスを組んでいないこと。

    - finalized_path() の呼び出しが存在すること
    - `f'...{...}.json'` 相当（JoinedStr の一部に '.json' を含む文字列リテラルを
      持つもの）が無いこと

    変異チェック 1: finalized_path(...) を元の os.path.join(..., f'{route_id}.json')
    に戻すと、JoinedStr('.json') が現れこのテストが落ちる。
    """
    tree = _tree(REPLAY_RUNNER)

    has_call = any(
        isinstance(n, ast.Call)
        and isinstance(n.func, ast.Name)
        and n.func.id == 'finalized_path'
        for n in ast.walk(tree))
    assert has_call, 'replay_runner のソースに finalized_path() の呼び出しが見つからない'

    offending = None
    for n in ast.walk(tree):
        if not isinstance(n, ast.JoinedStr):
            continue
        for v in n.values:
            if isinstance(v, ast.Constant) and isinstance(v.value, str) and '.json' in v.value:
                offending = n
                break
        if offending is not None:
            break
    assert offending is None, (
        'replay_runner が生の f-string で .json を組み立てている箇所が残っている: '
        f'{ast.dump(offending)}')


def test_finalize_then_read_back_round_trip(tmp_path):
    """WS-9I: 保存先 (finalized_path) と読み込み先が必ず一致する往復テスト。

    finalize_route_file(dir, name, ...) で保存したファイルを
    finalized_path(dir, name) で必ず開けること。すべての名前で confirm。
    """
    routes_dir = _mkroutes(tmp_path)
    names = ['kousha_01', '校舎1周', '9/3_koushakukou', 'a\\b', '../evil', '']
    for name in names:
        dest = finalize_route_file(routes_dir, name, {'id': name, 'points': []})
        assert dest == finalized_path(routes_dir, name), f'name={name!r}: 保存先が食い違う'
        with open(finalized_path(routes_dir, name), encoding='utf-8') as f:
            assert json.load(f)['id'] == name, f'name={name!r}: 読み戻せない'


# ============================================================================
# WS-9K-D: 重大フォルト（C-06a）で教示モードを弾き飛ばされると記録が孤児になる。
#
# transitions.yaml の C-06a（`mode=* state=* --fault.critical--> ESTOP/NONE`）は、
# 重大フォルトが 1 回出ただけで TEACH_MANUAL から即 ESTOP へ飛ぶ。ESTOP からの
# 復帰は C-09b（IDLE）だけで、元の TEACH_MANUAL へは戻れない。つまり FSM はもう
# finalize_route を発行できず、route_recorder は self._recorder を持ったまま点列と
# .wip を保持し続ける → 記録は永久に保存できない（2026-09-03 の校舎 1 周 185 m も
# .wip だけ残って .json が無かった。SLAM 有効で CPU 負荷が増え LIMITER_DEAD
# (CRITICAL) が出るようになったのが引き金）。
#
# 対策: 記録中にモードが教示系（TEACH_MANUAL / TEACH_FOLLOW）から出たことを
# 検出したら、その場で finalize_route_file して記録を閉じる（self._recorder=None）。
# 判定は純関数 should_autofinalize(prev_mode, new_mode, recording) としてコアに置く。
# ============================================================================


# ── 判定 (should_autofinalize) の純関数テスト ──────────────────────────────

def test_autofinalize_teach_manual_to_estop_while_recording():
    assert should_autofinalize('TEACH_MANUAL', 'ESTOP', True) is True


def test_autofinalize_teach_manual_to_idle_while_recording():
    assert should_autofinalize('TEACH_MANUAL', 'IDLE', True) is True


def test_autofinalize_same_teaching_mode_while_recording():
    """TEACH_MANUAL → TEACH_MANUAL は遷移でないので保存しない。"""
    assert should_autofinalize('TEACH_MANUAL', 'TEACH_MANUAL', True) is False


def test_autofinalize_teach_follow_to_estop_while_recording():
    assert should_autofinalize('TEACH_FOLLOW', 'ESTOP', True) is True


def test_autofinalize_not_recording_is_always_false():
    """記録していなければモードが何でも保存しない（既知の穴）。"""
    assert should_autofinalize('TEACH_MANUAL', 'ESTOP', False) is False
    assert should_autofinalize('TEACH_MANUAL', 'IDLE', False) is False


def test_autofinalize_entering_teaching_mode_is_false():
    """IDLE → TEACH_MANUAL は（記録前提でも）「教示から出た」ではない。"""
    assert should_autofinalize('IDLE', 'TEACH_MANUAL', True) is False


# ── ast: route_recorder.py がモード逸脱で finalize する・finalize が記録を閉じる ──


def _method_def(tree, name):
    return next(
        (n for n in ast.walk(tree)
         if isinstance(n, ast.FunctionDef) and n.name == name), None)


def _method_calls(tree, method, callee):
    node = _method_def(tree, method)
    if node is None:
        return False
    return any(
        isinstance(n, ast.Call)
        and isinstance(n.func, ast.Attribute)
        and isinstance(n.func.value, ast.Name) and n.func.value.id == 'self'
        and n.func.attr == callee
        for n in ast.walk(ast.Module(body=node.body, type_ignores=[])))


def test_route_recorder_mode_lost_triggers_autofinalize():
    """記録中モード逸脱の自動保存処理が、終端的に finalize_route_file を呼ぶ。

    経路: _on_state → _autofinalize_mode_left → _finalize_and_close →
    finalize_route_file。モード遷移を見る関数（_on_state）が存在し、その先で
    finalize_route_file が呼ばれることを固定する。

    変異チェック D1: _finalize_and_close（または _autofinalize_mode_left）が
    finalize_route_file を呼ばなくなると赤くなる。
    """
    tree = _tree(ROUTE_RECORDER)
    assert _method_def(tree, '_on_state') is not None, '_on_state が無い'
    # _on_state がモード逸脱の自動保存へ進む（should_autofinalize を使う）
    assert _method_calls(tree, '_autofinalize_mode_left', '_finalize_and_close') is True, (
        '_autofinalize_mode_left が _finalize_and_close を呼ばない')
    # _finalize_and_close が finalize_route_file（モジュール関数）を呼ぶ
    fclose = _method_def(tree, '_finalize_and_close')
    assert fclose is not None, '_finalize_and_close が無い'
    calls_finalize_file = any(
        isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
        and n.func.id == 'finalize_route_file'
        for n in ast.walk(ast.Module(body=fclose.body, type_ignores=[])))
    assert calls_finalize_file, '_finalize_and_close が finalize_route_file を呼ばない'


def test_route_recorder_closes_recording_on_finalize():
    """finalize（保存）で self._recorder = None にして記録を閉じる。

    変異チェック D2: _close_recording から self._recorder = None を消すと赤くなる。
    （消すと SAVED 後に resume_record で保存済み記録が再開する「既知の落とし穴」が
      復活する）
    """
    tree = _tree(ROUTE_RECORDER)
    close = _method_def(tree, '_close_recording')
    assert close is not None, '_close_recording が無い'
    assigns_none = [
        n for n in ast.walk(ast.Module(body=close.body, type_ignores=[]))
        if isinstance(n, ast.Assign)
        and len(n.targets) == 1 and isinstance(n.targets[0], ast.Attribute)
        and isinstance(n.targets[0].value, ast.Name) and n.targets[0].value.id == 'self'
        and n.targets[0].attr == '_recorder'
        and isinstance(n.value, ast.Constant) and n.value.value is None
    ]
    assert len(assigns_none) >= 1, '_close_recording が self._recorder = None にしない'
    # 通常の保存分岐（finalize_route）とモード逸脱の両方がこの _close_recording を
    # 通る。finalize_route 分岐と _finalize_and_close が閉じる経路を持つこと。
    assert _method_calls(tree, '_finalize_and_close', '_close_recording') is True, (
        '_finalize_and_close が _close_recording を呼ばない')
    assert _method_calls(tree, '_autofinalize_mode_left', '_finalize_and_close') is True


# ============================================================================
# WS-9K-E2: 「保存しました」が嘘をつく問題のノード側対策。
#
# S13TeachManual.jsx の `const saved = st?.state === 'SAVED'` は
# /route/status.state（＝ FSM の状態のコピー）を見ているだけで、route_recorder が
# 実際にファイルを書いたかどうかを一切見ていない。そのため FSM が SAVED になれば、
# 保存が失敗していても「保存しました」と出る（実機 2026-09-03: SAVED は出ていた
# のに保存されていなかった）。
#
# 対策: RouteStatus.msg に `bool saved` を追加し、route_recorder が
# finalize_route_file に成功したときだけ true にする（FSM の SAVED とは無関係）。
# WebUI 側（impl2 担当）はこの saved を見て「保存しました」を出す。
# ============================================================================


def _method_sets_attr_true(tree, method, attr):
    """メソッド本体で `self.<attr> = True` の代入があるか。"""
    node = _method_def(tree, method)
    if node is None:
        return False
    return any(
        isinstance(n, ast.Assign)
        and len(n.targets) == 1 and isinstance(n.targets[0], ast.Attribute)
        and isinstance(n.targets[0].value, ast.Name) and n.targets[0].value.id == 'self'
        and n.targets[0].attr == attr
        and isinstance(n.value, ast.Constant) and n.value.value is True
        for n in ast.walk(ast.Module(body=node.body, type_ignores=[])))


def test_route_recorder_start_record_clears_saved():
    """start_record で saved を false に戻す。

    変異チェック E2a: この代入を消すと、前の教示の saved=true が残って新しい教示の
    「保存しました」が先に嘘を出し得る → 赤くなる。
    """
    tree = _tree(ROUTE_RECORDER)
    # start_record 分岐（if name == 'start_record'）内で self._saved = False
    assert_dim = _effect_branch_sets_attr_false(tree, 'start_record', '_saved')
    assert assert_dim, 'start_record 分岐で self._saved = False にしていない'


def _effect_branch_sets_attr_false(tree, effect_name, attr):
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == '_on_effect':
            for name, body in _iter_effect_branches(node):
                if name == effect_name:
                    return _branch_sets_attr_false(body, attr)
    return False


def _branch_sets_attr_false(body_nodes, attr):
    return any(
        isinstance(n, ast.Assign)
        and len(n.targets) == 1 and isinstance(n.targets[0], ast.Attribute)
        and isinstance(n.targets[0].value, ast.Name) and n.targets[0].value.id == 'self'
        and n.targets[0].attr == attr
        and isinstance(n.value, ast.Constant) and n.value.value is False
        for n in ast.walk(ast.Module(body=body_nodes, type_ignores=[])))


def test_route_recorder_sets_saved_true_only_on_successful_save():
    """finalize_route_file に成功したときだけ self._saved = True にする。

    変異チェック E2b: _finalize_and_close の成功経路で saved=true を消す／あるいは
    FSM の SAVED（状態）を見て true にすると赤くなる。
    """
    tree = _tree(ROUTE_RECORDER)
    # _finalize_and_close は finalize_route_file を呼んで成功したら saved=true。
    assert _method_sets_attr_true(tree, '_finalize_and_close', '_saved'), (
        '_finalize_and_close が self._saved = True にしない')
    fclose = _method_def(tree, '_finalize_and_close')
    calls_finalize_file = any(
        isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
        and n.func.id == 'finalize_route_file'
        for n in ast.walk(ast.Module(body=fclose.body, type_ignores=[])))
    assert calls_finalize_file, '_finalize_and_close が finalize_route_file を呼ばない'


def test_route_recorder_status_publishes_saved():
    """/route/status publish が msg.saved に _saved を載せる。"""
    tree = _tree(ROUTE_RECORDER)
    node = _method_def(tree, '_status_timer')
    assert node is not None, '_status_timer が無い'
    assigns = [
        n for n in ast.walk(ast.Module(body=node.body, type_ignores=[]))
        if isinstance(n, ast.Assign)
        and len(n.targets) == 1 and isinstance(n.targets[0], ast.Attribute)
        and n.targets[0].attr == 'saved'
        and isinstance(n.targets[0].value, ast.Name)
        and n.targets[0].value.id == 'msg'
    ]
    assert assigns, '_status_timer が msg.saved を set していない'
