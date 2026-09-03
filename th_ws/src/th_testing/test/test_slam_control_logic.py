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
    deserialize_match_type, map_session_filename, open_session_error,
)
from route_record_core import _safe_id, finalized_path   # noqa: E402


# ── 1. deserialize_match_type の判定ロジック（WS-9N）──────────────────
def test_deserialize_match_type_with_initial_pose():
    """has_initial_pose=True なら LOCALIZE_AT_POSE (3) を返す。"""
    assert deserialize_match_type(True) == 3


def test_deserialize_match_type_without_initial_pose():
    """has_initial_pose=False なら START_AT_FIRST_NODE (1) を返す。"""
    assert deserialize_match_type(False) == 1


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


# ── 5. ast: slam_control は pause_new_measurements を使わず、reload 時に凍結してから deserialize する (WS-9N) ──
def test_slam_control_has_no_pause_new_measurements():
    """WS-9N: slam_control.py に pause_new_measurements が現れないこと。

    pause_new_measurements は自己位置推定まで止めてしまう旧ノードの代用であり、
    新ノード map_and_localization_slam_toolbox_node では set_localization_mode を
    使うため一切使わない。
    """
    src = _read(SLAM_CONTROL)
    assert 'pause_new_measurements' not in src, (
        'slam_control.py に pause_new_measurements が残っている（WS-9N）')


def test_slam_control_has_localization_and_deserialize_clients():
    """slam_control が set_localization_mode / deserialize_map クライアントを持つ。"""
    src = _read(SLAM_CONTROL)
    assert 'set_localization_mode' in src
    assert 'deserialize_map' in src
    tree = _tree(SLAM_CONTROL)
    clients = [ast.get_source_segment(src, n) or ''
               for n in ast.walk(tree)
               if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
               and n.func.attr == 'create_client']
    assert any('/slam_toolbox/set_localization_mode' in c for c in clients), (
        'set_localization_mode クライアントが無い')
    assert any('/slam_toolbox/deserialize_map' in c for c in clients), (
        'deserialize_map クライアントが無い')


def test_handle_map_reload_calls_localization_before_deserialize():
    """_handle_map_reload の中で localization 切替呼び出しが deserialize の呼び出しより前に現れること。

    再生中に地図作成へ戻す一行が復活したり、deserialize の後に凍結したりすると、
    再生中に地図作成が走って自己位置が発散する（2026-09-03 実機で 1.52m / 41.8° ずれ）。
    ROS ノードなので実行時に捕まえられないため、静的 AST で順序を固定する。
    """
    tree = _tree(SLAM_CONTROL)
    funcdef = _funcdef(tree, '_handle_map_reload')
    assert funcdef is not None, '_handle_map_reload が無い'

    loc_call_lineno = None
    deser_call_lineno = None

    for n in ast.walk(funcdef):
        if isinstance(n, ast.Call):
            # _set_localization(...) または set_localization_mode を含む呼び出し
            if isinstance(n.func, ast.Attribute) and n.func.attr == '_set_localization':
                loc_call_lineno = n.lineno
            # call_and_wait(self, self._cli_deserialize, ...)
            if isinstance(n.func, ast.Name) and n.func.id == 'call_and_wait':
                if len(n.args) >= 2 and isinstance(n.args[1], ast.Attribute) \
                        and n.args[1].attr == '_cli_deserialize':
                    deser_call_lineno = n.lineno

    assert loc_call_lineno is not None, (
        '_handle_map_reload に _set_localization の呼び出しが見つからない')
    assert deser_call_lineno is not None, (
        '_handle_map_reload に _cli_deserialize の call_and_wait が見つからない')
    assert loc_call_lineno < deser_call_lineno, (
        f'_set_localization (line {loc_call_lineno}) が '
        f'deserialize (line {deser_call_lineno}) より後に呼ばれている（順序不正）')


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


def test_replay_runner_reload_map_sends_initial_pose_and_checks_success():
    """WS-9N: replay_runner._reload_map が初期姿勢を載せて呼び、success を確認していること。

    始点 pose（has_initial_pose, initial_x, initial_y, initial_yaw）を渡さないと
    slam_toolbox が match_type 3 (LOCALIZE_AT_POSE) で自己位置を合わせられない。
    また応答の success を見ないと読み直し失敗を検知できず READY に進んでしまう。
    """
    tree = _tree(REPLAY_RUNNER)
    funcdef = _funcdef(tree, '_reload_map')
    assert funcdef is not None, '_reload_map が見つからない'

    # OpenMapSession.Request(...) のキーワード引数に initial pose 関連があること
    req_call = None
    for n in ast.walk(funcdef):
        if isinstance(n, ast.Call):
            if (isinstance(n.func, ast.Attribute) and n.func.attr == 'Request') or \
               (isinstance(n.func, ast.Name) and n.func.id == 'OpenMapSession'):
                req_call = n
                break
    assert req_call is not None, 'OpenMapSession.Request の呼び出しが見つからない'

    kw_names = {kw.arg for kw in req_call.keywords}
    assert 'has_initial_pose' in kw_names, 'has_initial_pose が渡されていない'
    assert 'initial_x' in kw_names, 'initial_x が渡されていない'
    assert 'initial_y' in kw_names, 'initial_y が渡されていない'
    assert 'initial_yaw' in kw_names, 'initial_yaw が渡されていない'

    # resp.success を見ていること
    has_success_check = any(
        isinstance(n, ast.Attribute) and n.attr == 'success'
        for n in ast.walk(funcdef))
    assert has_success_check, '_reload_map で応答の success を確認していない'


# ── 9. ast: deserialize の filename に .posegraph を足していない（WS-9M）──
def test_deserialize_filename_does_not_append_posegraph():
    """_handle_map_reload が deserialize_map に渡す filename が拡張子なしの
    `base` そのものであること。

    slam_toolbox は渡された filename に `.posegraph` を自分で付けるため、
    呼び出し側が付けると `<base>.posegraph.posegraph` を探して必ず失敗する
    （2026-09-03 実機ログ:
      serialization::Read: Failed to open requested file: .../crash_check.posegraph.
      DeserializePoseGraph: Failed to read file: .../crash_check.posegraph.）。
    しかも DeserializePoseGraph の応答は空なので call は成功として返り、
    「読み直した」と誤報告していた。

    変異チェック: `req.filename = base + '.posegraph'` や `= graph_path` に
    戻すと赤くなる。
    """
    tree = _tree(SLAM_CONTROL)
    funcdef = _funcdef(tree, '_handle_map_reload')
    assert funcdef is not None, '_handle_map_reload が無い'

    filename_assigns = [
        n for n in ast.walk(funcdef)
        if isinstance(n, ast.Assign)
        and any(isinstance(t, ast.Attribute) and t.attr == 'filename'
                for t in n.targets)
    ]
    assert filename_assigns, 'req.filename への代入が見つからない'
    for n in filename_assigns:
        # 値は拡張子なしの `base`（bare 名）そのものであること。
        #   - `base + '.posegraph'` は BinOp なので ast.Name ではない → 赤
        #   - `graph_path`（= base + '.posegraph'）は id が base でない → 赤
        assert isinstance(n.value, ast.Name) and n.value.id == 'base', (
            'req.filename は拡張子なしの base を渡すこと（現在: '
            f'{ast.get_source_segment(_read(SLAM_CONTROL), n.value)}）')


# ── 10. ast: _set_pause がトグルをループせず最大 1 回だけ叩く（WS-9M）───
# ── 10. ast: _set_localization が応答の success を確認（WS-9N）──────────
def test_set_localization_checks_response_success():
    """_set_localization が SetBool の応答の success を確認していること。

    応答を見ずに success=True を仮定すると、切替失敗を検知できず
    再生中に地図作成のまま走り出す（2026-09-03 の欠陥）。
    """
    tree = _tree(SLAM_CONTROL)
    funcdef = _funcdef(tree, '_set_localization')
    assert funcdef is not None, '_set_localization が無い'

    has_success_check = any(
        isinstance(n, ast.Attribute) and n.attr == 'success'
        for n in ast.walk(funcdef))
    assert has_success_check, '_set_localization が応答の success を見ていない'


# ── 12. ast: deserialize 前に .posegraph と .data の両方を確認（WS-9M）──
def test_handle_map_reload_checks_both_files():
    """_handle_map_reload が deserialize_map を呼ぶ前に `.posegraph` と `.data`
    の両方の存在を確認し、欠けていれば success=false で返すこと。

    DeserializePoseGraph の応答は空でフィールドが 1 つも無く、読み込み失敗でも
    call_and_wait は成功として返る（実機ログ「DeserializePoseGraph: Failed to
    read file」でも例外にならない）。戻り値で成否が分からないので、呼ぶ前に
    両方のファイルが揃っていることを実質的な成否判定にする。いまは無条件に
    成功を返しており、replay_runner が evt.localize_done を出して READY まで
    進んでしまっていた。

    変異チェック: `.data` の存在確認を消すと赤くなる。
    """
    tree = _tree(SLAM_CONTROL)
    funcdef = _funcdef(tree, '_handle_map_reload')
    assert funcdef is not None

    src = _read(SLAM_CONTROL)
    # `<var> = base + '<ext>'` で作る拡張子つきパス変数 → 拡張子の対応表
    ext_of_var = {}
    for n in ast.walk(funcdef):
        if isinstance(n, ast.Assign) and isinstance(n.value, ast.BinOp) \
                and isinstance(n.value.op, ast.Add):
            for sub in ast.walk(n.value):
                if isinstance(sub, ast.Constant) and isinstance(sub.value, str):
                    for t in n.targets:
                        if isinstance(t, ast.Name):
                            ext_of_var[t.id] = sub.value

    # os.path.exists(...) に渡している拡張子を集める
    checked_exts = set()
    for n in ast.walk(funcdef):
        if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) \
                and n.func.attr == 'exists' and n.args:
            arg = n.args[0]
            if isinstance(arg, ast.Name) and arg.id in ext_of_var:
                checked_exts.add(ext_of_var[arg.id])
            seg = ast.get_source_segment(src, arg) or ''
            for ext in ('.posegraph', '.data'):
                if ext in seg:
                    checked_exts.add(ext)

    assert '.posegraph' in checked_exts, (
        f'_handle_map_reload に .posegraph の存在確認が無い: {checked_exts}')
    assert '.data' in checked_exts, (
        f'_handle_map_reload に .data の存在確認が無い: {checked_exts}')
    # 存在確認に失敗したらエラー文字列つき _finish（= success=false）で返すこと
    body_src = ast.get_source_segment(src, funcdef) or ''
    assert 'return self._finish(' in body_src
