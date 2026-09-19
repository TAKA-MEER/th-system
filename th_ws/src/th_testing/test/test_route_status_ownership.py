"""WS-9Q: /route/status を出すのは自分のモードのノードだけ、を固定する。

実機フィードバック（2026-09-04）「再生を押したが反応しなかった」。

`/route/status` には route_recorder と replay_runner の**両方**が publisher を
持つ。どちらも無条件に 2Hz で publish していたため、再生中も記録側が
`points=0` を交互に流していた。`state_manager._on_route_status` は届いた
メッセージごとに `route_loaded = points > 0 and current.id` を更新するので、
このフラグが 4Hz で真偽を往復する。`T-REPLAY-07`（PAUSE --ui.run--> RUN）の
ガードは `route_loaded` なので、「再生」を押した瞬間がどちらのメッセージの
直後かで受理／拒否が変わっていた。

実測（2026-09-04・実機）:
    /route/status  publisher 2 個・4.002 Hz（各ノード status_period_ms=500）
    /route/preview publisher 2 個・無音（WS-6.4 で記録側をゲート済み）

`/route/preview` は同じ理由（表示の点滅）で既に直っており、`/route/status`
だけが残っていた。同じ穴を二度開けないようにここで固定する。
"""
import ast
import os
import sys

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.abspath(os.path.join(_HERE, '..', '..', '..', '..'))
sys.path.insert(0, os.path.join(_ROOT, 'th_ws', 'src', 'th_planning'))

from th_planning.route_record_core import owns_route_status  # noqa: E402

_RECORDER = os.path.join(
    _ROOT, 'th_ws', 'src', 'th_planning', 'scripts', 'route_recorder.py')
_REPLAY = os.path.join(
    _ROOT, 'th_ws', 'src', 'th_planning', 'scripts', 'replay_runner.py')


def _read(p: str) -> str:
    with open(p, encoding='utf-8') as f:
        return f.read()


# ── 純関数 ────────────────────────────────────────────────────

@pytest.mark.parametrize('mode', ['TEACH_MANUAL', 'TEACH_FOLLOW'])
def test_recorder_owns_status_in_teach_modes(mode):
    assert owns_route_status(mode, recorder=True) is True
    assert owns_route_status(mode, recorder=False) is False, (
        '教示中に replay_runner が出すと、記録側の実データを空で上書きする')


def test_replay_owns_status_in_replay_mode():
    assert owns_route_status('REPLAY', recorder=False) is True
    assert owns_route_status('REPLAY', recorder=True) is False, (
        '再生中に route_recorder が points=0 を出すと route_loaded が往復し、'
        '「再生」が押したタイミング次第で拒否される（実機 2026-09-04）')


@pytest.mark.parametrize('mode', ['IDLE', 'ESTOP', 'MANUAL', ''])
def test_nobody_owns_status_outside_those_modes(mode):
    assert owns_route_status(mode, recorder=True) is False
    assert owns_route_status(mode, recorder=False) is False


def test_the_two_never_publish_at_the_same_time():
    """どのモードでも同時に真にならないこと（交互配信の再発防止）。"""
    for mode in ('TEACH_MANUAL', 'TEACH_FOLLOW', 'REPLAY', 'IDLE', 'ESTOP',
                  'MANUAL', 'CARRY', 'SUMMON', ''):
        both = (owns_route_status(mode, recorder=True)
                and owns_route_status(mode, recorder=False))
        assert not both, f'mode={mode!r} で両方が publish しようとしている'


# ── ノード側の配線 ────────────────────────────────────────────

def _status_timer_body(path: str) -> str:
    tree = ast.parse(_read(path))
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == '_status_timer':
            return '\n'.join(ast.unparse(n) for n in node.body)
    raise AssertionError(f'{path} に _status_timer が無い')


def test_replay_runner_status_timer_is_gated():
    body = _status_timer_body(_REPLAY)
    assert 'owns_route_status' in body, (
        'replay_runner._status_timer がモードでゲートされていない（WS-9Q）')
    assert 'recorder=False' in body, (
        'replay_runner が recorder=True で問い合わせている（引数の取り違え）')


def test_route_recorder_status_publish_is_gated():
    body = _status_timer_body(_RECORDER)
    assert 'owns_route_status' in body, (
        'route_recorder._status_timer がモードでゲートされていない（WS-9Q）')
    assert 'recorder=True' in body, (
        'route_recorder が recorder=False で問い合わせている（引数の取り違え）')
    # publish がゲートの中に入っていること（外に出ていたら意味が無い）。
    tree = ast.parse(_read(_RECORDER))
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == '_status_timer':
            for stmt in node.body:
                assert not (isinstance(stmt, ast.Expr)
                            and '_pub_status.publish' in ast.unparse(stmt)), (
                    '_pub_status.publish が if の外にある。ゲートが効かない（WS-9Q）')
            return
    pytest.fail('_status_timer が見つからない')


if __name__ == '__main__':
    sys.exit(pytest.main([__file__, '-v']))
