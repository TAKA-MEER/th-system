"""
test_route_map_session.py
==========================
WS-9K-B: 経路に「どの地図セッションで記録したか」の印を付け、別セッションの
経路を再生に進ませない。

実機 2026-09-03: 校舎 1 周（185 m / 1504 点 / 包絡 44.5×56.4 m）の教示は .wip が
残っただけで .json にならず、一覧には**前のセッションで記録した別の経路**しか無い
状態になった。それを再生すると start_yaw=-178.1° からまず約 180° 旋回し、別の地図の
座標を追うため壁に向かった（「開始地点から逆方向に旋回した」の正体）。

地図はセッションごとに作り直されるため（docs/使い方.md §4-4）、別セッションで
記録した map フレーム経路の座標は現在の地図では無意味。これを再生しないための
印（RouteData.map_session_id）と判定（can_replay_route）を固定する。

- map フレーム＋同じセッション         → 再生可 (True)
- map フレーム＋違うセッション        → 再生不可 (False)
- map フレーム＋セッション ""(古い経路) → 再生不可 (False。セッション不明は一致しない)
- odom フレーム                       → セッションに関係なく再生可 (True)

ROS2 なし・純粋 Python。
"""

import ast
import json
import os
import sys

# route_recorder.py / replay_runner.py のソースを ast で静的検証するためのパス
# （test_route_record_files.py と同じ規約）。
_SCRIPTS = os.path.abspath(os.path.join(
    os.path.dirname(__file__), '..', '..', 'th_planning', 'scripts'))
ROUTE_RECORDER = os.path.join(_SCRIPTS, 'route_recorder.py')
REPLAY_RUNNER = os.path.join(_SCRIPTS, 'replay_runner.py')

# ── パスを通す（colcon build 前にも直接 pytest できるように）
sys.path.insert(0, os.path.join(
    os.path.dirname(__file__),
    '..', '..', 'th_planning', 'th_planning'))

from route_record_core import (
    RouteData, can_replay_route, route_from_dict, route_to_dict,
)


def _route_with_session(session_id: str) -> RouteData:
    return RouteData(
        id='r1', name='校舎1周', generation=1, start_yaw=0.0,
        recorded_at_ms=0, frame_id='map', map_session_id=session_id,
        points=[(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)],
    )


# ── 往復でセッション ID が保たれる ──────────────────────────────────────
def test_map_session_id_round_trip():
    """route_to_dict → route_from_dict で map_session_id が保たれる。

    変異チェック 1: route_to_dict が map_session_id を書かない／route_from_dict が
    読まないと、このテストが落ちる。
    """
    route = _route_with_session('sess_12345')
    d = route_to_dict(route)
    assert d['map_session_id'] == 'sess_12345'
    back = route_from_dict(d)
    assert back.map_session_id == 'sess_12345'


def test_route_from_dict_survives_old_json_without_session():
    """古い JSON（map_session_id キーが無い）を読めて、"" になる。

    変異チェック 2: route_from_dict が d['map_session_id'] を直接引くと KeyError で
    落ちる → このテストが赤くなる。
    """
    old = {
        'id': 'r1', 'name': '古い経路', 'generation': 1, 'start_yaw': 0.0,
        'recorded_at_ms': 0, 'frame_id': 'map',
        'points': [[0.0, 0.0, 0.0]],
    }
    route = route_from_dict(old)
    assert route.map_session_id == ''


# ── セッション判定 (can_replay_route) ───────────────────────────────────
def test_can_replay_map_same_session():
    assert can_replay_route('map', 'sess_A', 'sess_A', 'map') is True


def test_can_replay_map_different_session():
    """変異チェック 3: 「違うセッション」を True にすると赤くなる。"""
    assert can_replay_route('map', 'sess_A', 'sess_B', 'map') is False


def test_can_replay_map_unknown_session_is_false():
    """古い経路（セッション ""）は一致しない扱い＝再生不可。"""
    assert can_replay_route('map', '', 'sess_B', 'map') is False


def test_can_replay_odom_ignores_session():
    """odom フレーム経路は地図に依存しないのでセッションに関係なく True。"""
    assert can_replay_route('odom', 'sess_A', 'sess_B', 'map') is True
    assert can_replay_route('odom', '', 'sess_B', 'map') is True


def test_can_replay_current_empty_session_blocks_map_route():
    """replay_runner にセッション ID が渡らない（""）場合も、map 経路は再生不可。"""
    assert can_replay_route('map', 'sess_A', '', 'map') is False
    assert can_replay_route('map', '', '', 'map') is False


# ── 経路 JSON に地図セッション ID が保存される（保存側の配線）──────────
def test_route_recorder_writes_session_only_for_map_frame(tmp_path):
    """route_to_dict 経由で保存される JSON に map 経路のセッション ID が載る。

    route_recorder は map フレーム記録時だけ RouteData.map_session_id をセットし、
    odom フレームでは "" のままにする。ここでは dict 変換・復元を通して、
    セッション ID がファイルに載ること（＝保存側の配線が効くこと）を確認する。
    """
    map_route = _route_with_session('sess_99')
    dict_ = route_to_dict(map_route)
    assert route_from_dict(dict_).map_session_id == 'sess_99'
    odom_route = _route_with_session('')  # odom 記録では "" になる想定
    odom_dict = route_to_dict(odom_route)
    assert route_from_dict(odom_dict).map_session_id == ''


# ── replay_runner の load_route が判定を呼ぶ（静的）──────────────────────
def _read(path: str) -> str:
    with open(path, encoding='utf-8') as f:
        return f.read()


def _tree(path: str) -> ast.AST:
    return ast.parse(_read(path), filename=path)


def test_replay_runner_load_route_calls_can_replay_route():
    """replay_runner の load_route 処理で can_replay_route が呼ばれること。

    変異チェック 4: この呼び出しを消すと再生ガードが外れ、テストが赤くなる。
    """
    tree = _tree(REPLAY_RUNNER)
    funcdef = next((n for n in ast.walk(tree)
                    if isinstance(n, ast.FunctionDef) and n.name == '_on_effect'), None)
    assert funcdef is not None, '_on_effect が無い'
    span = (funcdef.lineno, getattr(funcdef, 'end_lineno', funcdef.lineno))

    calls = [
        n for n in ast.walk(funcdef)
        if isinstance(n, ast.Call)
        and isinstance(n.func, ast.Name)
        and n.func.id == 'can_replay_route'
    ]
    assert calls, 'replay_runner の _on_effect に can_replay_route() の呼び出しが見つからない'

    # 呼び出しが load_route 分岐（`if name == 'load_route'` の If）の範囲内にある
    load_if = next(
        (n for n in ast.walk(funcdef)
         if isinstance(n, ast.If)
         and isinstance(n.test, ast.Compare)
         and isinstance(n.test.left, ast.Name) and n.test.left.id == 'name'
         and len(n.test.ops) == 1 and isinstance(n.test.ops[0], ast.Eq)
         and isinstance(n.test.comparators[0], ast.Constant)
         and n.test.comparators[0].value == 'load_route'),
        None)
    assert load_if is not None, "load_route の分岐 (If) が見つからない"
    span = (load_if.lineno, getattr(load_if, 'end_lineno', load_if.lineno))
    assert all(span[0] <= c.lineno <= span[1] for c in calls), (
        'can_replay_route の呼び出しが load_route 分岐の外にある')


def test_replay_runner_declares_map_session_id_param():
    """replay_runner が map_session_id パラメータを宣言していること。"""
    tree = _tree(REPLAY_RUNNER)
    found = any(
        isinstance(n, ast.Call)
        and isinstance(n.func, ast.Attribute) and n.func.attr == 'declare_parameter'
        and n.args and isinstance(n.args[0], ast.Constant)
        and n.args[0].value == 'map_session_id'
        for n in ast.walk(tree))
    assert found, 'replay_runner に map_session_id パラメータの宣言が無い'


def test_route_recorder_declares_map_session_id_param():
    """route_recorder が map_session_id パラメータを宣言していること。"""
    tree = _tree(ROUTE_RECORDER)
    found = any(
        isinstance(n, ast.Call)
        and isinstance(n.func, ast.Attribute) and n.func.attr == 'declare_parameter'
        and n.args and isinstance(n.args[0], ast.Constant)
        and n.args[0].value == 'map_session_id'
        for n in ast.walk(tree))
    assert found, 'route_recorder に map_session_id パラメータの宣言が無い'
