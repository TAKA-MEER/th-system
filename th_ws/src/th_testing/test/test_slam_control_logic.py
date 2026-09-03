"""
test_slam_control_logic.py
============================
WS-9L: 地図の凍結・保存・再読込の純ロジック（slam_control_logic.py）と、
配線（slam_control / route_recorder / replay_runner）の ast 静的検証。

詳しくは本家 Docstring を参照。ROS2 なし・純粋 Python。
"""

import ast
import os
import sys

# slam_control.py のソースを ast で静的検証するためのパス
_CONF_SCRIPTS = os.path.abspath(os.path.join(
    os.path.dirname(__file__), '..', '..', 'th_config_manager', 'scripts'))
SLAM_CONTROL = os.path.join(_CONF_SCRIPTS, 'slam_control.py')

# slam_control_logic.py（純ロジック本体）のソースを検証するためのパス
_CONF_PKG = os.path.abspath(os.path.join(
    os.path.dirname(__file__), '..', '..', 'th_config_manager',
    'th_config_manager'))
SLAM_CONTROL_LOGIC = os.path.join(_CONF_PKG, 'slam_control_logic.py')

# route_recorder.py / replay_runner.py のソースを ast で静的検証するためのパス
_PLAN_SCRIPTS = os.path.abspath(os.path.join(
    os.path.dirname(__file__), '..', '..', 'th_planning', 'scripts'))
ROUTE_RECORDER = os.path.join(_PLAN_SCRIPTS, 'route_recorder.py')
REPLAY_RUNNER = os.path.join(_PLAN_SCRIPTS, 'replay_runner.py')

# ── パスを通す（colcon build 前にも直接 pytest できるように）────────────
# slam_control_logic は th_config_manager パッケージのみに依存する。送る側の
# 正規化（route_record_core._safe_id）を検証するために th_planning のパスも足す
# （import はしない：slam_control_logic が th_planning を import しないことを
# ast で確認するテストがある）。
sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), '..', '..', 'th_planning', 'th_planning'))
sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), '..', '..', 'th_config_manager',
    'th_config_manager'))

from slam_control_logic import (   # noqa: E402
    converge_pause, map_session_filename, open_session_error,
)
from route_record_core import _safe_id, finalized_path   # noqa: E402


# ── 1. Pause トグルの収束ロジック ───────────────────────────────────────
def test_pause_converged_on_first_status():
    """望みの状態と 1 回目の status が一致 → 追加の呼び出し不要 (None)。"""
    assert converge_pause(True, [True]) is None
    assert converge_pause(False, [False]) is None


def test_pause_retry_after_first_mismatch():
    """1 回目が不一致 → もう 1 回叩く（'retry'、失敗扱いにはしない）。

    変異チェック: 「1 回叩いて終わり」にする（1 回目の不一致で即失敗メッセージを
    返す）と、ここが赤くなる。
    """
    assert converge_pause(True, [False]) == 'retry'
    assert converge_pause(False, [True]) == 'retry'


def test_pause_converged_on_second_status():
    """1 回目不一致 → 2 回目で一致 → 収束 (None)。"""
    assert converge_pause(True, [False, True]) is None
    assert converge_pause(False, [True, False]) is None


def test_pause_fails_after_two_mismatches():
    """2 回叩いても望みの状態にならない → 失敗（文字列）。"""
    assert converge_pause(True, [False, False]) is not None
    assert converge_pause(False, [True, True]) is not None
    # 失敗時も「再試行して」ではなく明示的な失敗メッセージ
    assert 'retry' != converge_pause(True, [False, False])


def test_pause_empty_statuses_means_retry():
    """まだ 1 回も叩いていない → 呼び出し元が status を足してくる前提で retry。"""
    assert converge_pause(True, []) == 'retry'


# ── 2. /map_session/open の引数検証 ────────────────────────────────────
def test_open_session_slot_route_ok():
    assert open_session_error('ROUTE', 'save', 'r1') is None
    assert open_session_error('ROUTE', 'reload', 'r1') is None


def test_open_session_rejects_non_route_slot():
    """変異チェック 1: slot 検証を外すと赤くなる。"""
    assert open_session_error('MAP', 'save', 'r1') is not None


def test_open_session_rejects_bad_mode():
    assert open_session_error('ROUTE', 'delete', 'r1') is not None
    assert open_session_error('ROUTE', '', 'r1') is not None


def test_open_session_rejects_empty_session_id():
    assert open_session_error('ROUTE', 'save', '') is not None


def test_open_session_rejects_unsafe_session_id():
    """session_id に `/` `\\` `..`（未正規化・不正）が含まれると拒否。

    変異チェック 1: 未正規化 id の検証を外す（そのまま通す）と赤くなる。
    受ける側は「送る側が _safe_id で正規化済みの id だけ」を受け取る前提で、
    変換はせず拒否する（サニタイズを 2 か所に持たない）。
    """
    assert open_session_error('ROUTE', 'save', '9/3_kousha') is not None
    assert open_session_error('ROUTE', 'save', 'a\\b') is not None
    assert open_session_error('ROUTE', 'save', '../evil') is not None


def test_open_session_rejects_traversal_session_id():
    """`.` `.`（`..`) を含む session_id を拒否する。

    `/` `\\` を含む id は _UNSAFE_ID_RE（それより前の分岐）で弾かれるため、
    `..` だけを見る _TRAVERSAL_RE の分岐は、`/` も `\\` も含まないケースで
    初めて到達する。ここを固定しないと、`..` だけを弾く分岐が黙って消えても
    `test_open_session_rejects_unsafe_session_id`（`../evil` 含む）は、`/` で
    先に弾かれてしまうので検知できない。`..` だけでディレクトリを抜けることは
    できないが（_safe_id が `/` `\\` を `_` に正規化済み）、検証の分岐が消えるのを
    防ぐためにここで固定する。

    変異チェック: _TRAVERSAL_RE の分岐を `if False:` に潰すと赤くなる。
    """
    assert open_session_error('ROUTE', 'save', '..') is not None
    assert open_session_error('ROUTE', 'save', '..evil') is not None
    assert open_session_error('ROUTE', 'save', 'a..b') is not None
    assert open_session_error('ROUTE', 'reload', 'a..b') is not None


# ── 3. 受ける側は変換しない（検証済み id → そのままファイル名）──────────
def test_map_session_filename_passes_validated_id_through():
    """検証済み（安全な）id はそのままファイル名にする（変換しない）。

    変異チェック 1: ここで `/` や `\\` を `_` に置換する（受ける側で再サニタイズ）と
    赤くなる。サニタイズは送る側（_safe_id）が一律に行い、受ける側は不正なら
    拒否する、という役割分担に固定する。
    """
    assert map_session_filename('r1') == 'r1'
    assert map_session_filename('9_3_kousha') == '9_3_kousha'
    assert map_session_filename('a_b') == 'a_b'


# ── 4. ast ヘルパー ────────────────────────────────────────────────────
def _read(path: str) -> str:
    with open(path, encoding='utf-8') as f:
        return f.read()


def _tree(path: str) -> ast.AST:
    return ast.parse(_read(path), filename=path)


def _funcdef(tree: ast.AST, name: str):
    return next((n for n in ast.walk(tree)
                 if isinstance(n, ast.FunctionDef) and n.name == name), None)


def _calls(node, method_names):
    """method_names のいずれかを呼ぶ ast.Call の一覧（ソース順）。"""
    result = []
    for n in ast.walk(node):
        if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) \
                and n.func.attr in method_names:
            result.append(n)
    result.sort(key=lambda n: n.lineno)
    return result


def _name_calls(node, name):
    """bare 名（ast.Name）で name を呼ぶ ast.Call の一覧（属性呼び出しを除く）。"""
    return [n for n in ast.walk(node)
            if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
            and n.func.id == name]


# ── 5. ast: slam_control に set_localization_mode が残らない ────────────
def test_slam_control_has_no_set_localization_mode_client():
    """slam_control.py に set_localization_mode が転送先に残っていないこと。

    async_slam_toolbox_node には存在しないサービス。クライアント生成やサービス
    転送に使われていたら（＝存在しないサービスを待ち続けて起動が遅れる）赤になる。
    """
    src = _read(SLAM_CONTROL)
    tree = _tree(SLAM_CONTROL)
    for n in ast.walk(tree):
        if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) \
                and n.func.attr in ('create_client', 'create_service'):
            arg_str = ast.get_source_segment(src, n) or ''
            assert 'set_localization_mode' not in arg_str, \
                f'set_localization_mode が転送先に残っている: {arg_str}'


def test_slam_control_has_pause_and_deserialize_clients():
    """slam_control が pause_new_measurements / deserialize_map クライアントを持つ。"""
    src = _read(SLAM_CONTROL)
    assert 'pause_new_measurements' in src
    assert 'deserialize_map' in src
    # クライアント生成が create_client(..., '/slam_toolbox/...') で行われている
    tree = _tree(SLAM_CONTROL)
    clients = [ast.get_source_segment(src, n) or ''
               for n in ast.walk(tree)
               if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
               and n.func.attr == 'create_client']
    assert any('open_session' not in c for c in clients)
    assert any('/slam_toolbox/pause_new_measurements' in c for c in clients), \
        'pause_new_measurements クライアントが無い'


# ── 6. ast: replay_runner の localize_done が reload 成功の分岐の中 ─────
def test_replay_runner_localize_done_guarded_by_reload_success():
    """replay_runner の map 経路で evt.localize_done が reload 成功の分岐内だけ。

    変異チェック: reload 失敗時にも evt.localize_done を出す（or _localize_pending
    を立てる）と赤くなる。
    """
    tree = _tree(REPLAY_RUNNER)
    funcdef = _funcdef(tree, '_on_effect')
    assert funcdef is not None, '_on_effect が無い'

    # map 経路の分岐: `if map_route:` のうち _reload_map 呼び出しを含むもの
    map_ifs = [n for n in ast.walk(funcdef)
               if isinstance(n, ast.If)
               and isinstance(n.test, ast.Name) and n.test.id == 'map_route']
    target = None
    for mif in map_ifs:
        if _calls(mif, {'_reload_map'}):
            target = mif
            break
    assert target is not None, 'map_route 分岐に _reload_map() の呼び出しが無い'

    # reload 成功判定: `if err is None:`（map 分岐内）
    success_if = next(
        (n for n in ast.walk(target)
         if isinstance(n, ast.If)
         and isinstance(n.test, ast.Compare)
         and isinstance(n.test.left, ast.Name) and n.test.left.id == 'err'
         and any(isinstance(o, ast.Is) or isinstance(o, ast.IsNot)
                 for o in n.test.ops)
         and any(isinstance(c, ast.Constant) and c.value is None
                 for c in n.test.comparators)), None)
    assert success_if is not None, 'reload 成功判定 (if err is None) が見つからない'

    def _assigns_pending(node) -> bool:
        if node is None:
            return False
        for n in ast.walk(node):
            if isinstance(n, ast.Assign):
                for t in n.targets:
                    if isinstance(t, ast.Attribute) \
                            and t.attr == '_localize_pending':
                        return True
        return False

    # 成功側 body で _localize_pending = True すること（localize_done の前提）
    assert success_if.body and _assigns_pending(success_if.body[0]), (
        'reload 成功側で _localize_pending が立てられていない')

    # 失敗側（else）には localize_done の発行経路（_localize_pending 代入 /
    # evt.localize_done）が無いこと
    if success_if.orelse:
        assert not _assigns_pending(success_if.orelse[0]), (
            'reload 失敗側でも _localize_pending が立てられている')
        for stmt in success_if.orelse:
            for n in ast.walk(stmt):
                if _is_emit_localize_done(n):
                    raise AssertionError(
                        'reload 失敗側で evt.localize_done が発行されている')

    # 成功側 body に evt.localize_done の直接発行が無いこと（map 経路の
    # localize_done は _tick_localize_pending 経由で TF 到着後に出る）
    for n in ast.walk(success_if):
        if _is_emit_localize_done(n):
            raise AssertionError(
                'map 経路で reload 成功直後に evt.localize_done が直接発行されている'
                '（map TF 待ちが無い）。_tick_localize_pending 経由にすること')


def test_replay_runner_localize_done_still_emitted_for_odom_routes():
    """odom 経路（map 経路でない）では従来どおり即 evt.localize_done を出す。"""
    src = _read(REPLAY_RUNNER)
    assert 'evt.localize_done' in src
    # 最小限: map 経路の else（odom）で必ず発行される文脈を静的に確認する代わりに、
    # ソースに evt.localize_done の発行が残っていることだけ保証（過剰に縛らない）。


# ── 7. ast: route_recorder の save が finalize_route_file の後 ─────────
def _is_emit_localize_done(n) -> bool:
    return (isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
            and n.func.attr == '_emit_event' and n.args
            and isinstance(n.args[0], ast.Constant)
            and n.args[0].value == 'evt.localize_done')


def test_route_recorder_save_map_call_after_finalize_route_file():
    """route_recorder の地図保存呼び出しが finalize_route_file の後にあること。

    変異チェック: save 呼び出しを消す（または finalize_route_file より前に移す）と
    赤くなる。
    """
    tree = _tree(ROUTE_RECORDER)
    funcdef = _funcdef(tree, '_finalize_and_close')
    assert funcdef is not None, '_finalize_and_close が無い'

    fcall = next(
        (n for n in ast.walk(funcdef)
         if isinstance(n, ast.Call)
         and isinstance(n.func, ast.Name)
         and n.func.id == 'finalize_route_file'), None)
    assert fcall is not None, 'finalize_route_file の呼び出しが無い'

    scall = next(
        (n for n in ast.walk(funcdef)
         if isinstance(n, ast.Call)
         and isinstance(n.func, ast.Attribute)
         and n.func.attr == '_save_map_for_route'), None)
    assert scall is not None, '_save_map_for_route の呼び出しが無い'

    assert scall.lineno > fcall.lineno, (
        '地図保存の呼び出しが finalize_route_file より前にある')


def test_route_recorder_map_save_guarded_by_map_frame():
    """route_recorder の地図保存が map フレーム記録時だけ呼ばれること。

    _save_map_for_route 呼び出しが map フレーム判定（self._frame_id ==
    self._map_frame）の条件内にあること。
    """
    src = _read(ROUTE_RECORDER)
    tree = _tree(ROUTE_RECORDER)
    funcdef = _funcdef(tree, '_finalize_and_close')
    assert funcdef is not None

    map_if = next(
        (n for n in ast.walk(funcdef)
         if isinstance(n, ast.If)
         and isinstance(n.test, ast.Compare)
         and isinstance(n.test.left, ast.Attribute)
         and n.test.left.attr == '_frame_id'
         and any(isinstance(c, ast.Attribute) and c.attr == '_map_frame'
                 for c in ast.walk(n.test))), None)
    assert map_if is not None, 'map フレーム判定 (if self._frame_id == self._map_frame) が無い'

    assigns_before = [a.lineno for a in _calls(map_if, {'_save_map_for_route'})]
    assert assigns_before, '_save_map_for_route 呼び出しが map フレーム判定の内側に無い'


def test_route_recorder_and_replay_use_multithreaded_executor():
    """route_recorder / replay_runner が MultiThreadedExecutor を使うこと。

    サービスコールバック内からの同期サービス呼び出しは、単一スレッド executor だと
    応答コールバックが動けずデッドロックする（ブリーフ C「スレッドの落とし穴」）。
    """
    for path, label in [(ROUTE_RECORDER, 'route_recorder'),
                        (REPLAY_RUNNER, 'replay_runner')]:
        src = _read(path)
        assert 'MultiThreadedExecutor' in src, (
            f'{label} に MultiThreadedExecutor が見つからない')
        assert 'ReentrantCallbackGroup' in src, (
            f'{label} に ReentrantCallbackGroup が見つからない')


# ── 8. ast: 依存を一方向（th_config_manager から th_planning を import しない）──
def test_slam_control_logic_does_not_import_th_planning():
    """slam_control_logic.py が th_planning を import していないこと。

    循環依存の根絶。これが破れると th_config_manager <-> th_planning の循環になり
    colcon が受け付けない。ソースに 'th_planning' の文字列が無いことでも confirm
    （import 文はもちろん、コメント・docstring に書いても余計な束縛が増えるので
    書かないことにする）。
    """
    src = _read(SLAM_CONTROL_LOGIC)
    assert 'th_planning' not in src, \
        'slam_control_logic.py が th_planning を参照している（循環依存）'


def test_senders_normalize_session_id_with_safe_id():
    """route_recorder / replay_runner が /map_session/open を呼ぶとき _safe_id を通す。

    「送る側が正規化する」役割を固定。これが外れると受ける側は未正規化 id を拒否
    するため、経路名に `/` などが入ったときに地図だけ見つからなくなる。
    OpenMapSession.Request(...) の呼び出しと同一関数内に _safe_id の呼び出しが
    あること（＝正規化された session_id を渡している）。

    変異チェック 2: 送る側の _safe_id を外す（route_id をそのまま渡す）と赤くなる。
    """
    src = _read(ROUTE_RECORDER)
    tree = _tree(ROUTE_RECORDER)
    funcdef = _funcdef(tree, '_save_map_for_route')
    assert funcdef is not None, '_save_map_for_route が無い'
    assert _name_calls(funcdef, '_safe_id'), \
        'route_recorder が /map_session/open 呼び出しで _safe_id を使っていない'

    src2 = _read(REPLAY_RUNNER)
    tree2 = _tree(REPLAY_RUNNER)
    funcdef2 = _funcdef(tree2, '_reload_map')
    assert funcdef2 is not None, '_reload_map が無い'
    assert _name_calls(funcdef2, '_safe_id'), \
        'replay_runner が /map_session/open 呼び出しで _safe_id を使っていない'

