"""
test_route_preview_bandwidth.py
==================================
WS-9F の静的テスト（ソースを ast で読む方式。ROS2 なし・純 Python で実行可能）。

背景: `/route/preview`（nav_msgs/Path）は記録済みの全点を毎回まるごと 2Hz で
publish しており、100m 級の経路（1000 点前後）になると数 Mbps の帯域を食い、
直したばかりのロボット無線を圧迫する。WS-9F では

  - 記録側 (`route_recorder.py`) は表示専用なので `decimate_polyline` で間引いて
    帯域を落とす（`preview_max_points`、既定 400）
  - 再生側 (`replay_runner.py`) は `RoutePreview` が `preview[targetIndex]` で点の
    添字を参照するため**間引かず**、点列が変わったとき＋ハートビート間隔だけ
    publish 回数を減らす（`preview_heartbeat_ms`、既定 5000）

ここではノードを一切起動せず、ソース文字列を ast で解析して上記の設計判断が
コードとして固定されていることを検証する。`test_params_launch.py` の
`_read` / `ast.parse` の書き方に倣う。
"""
import ast
import os

_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'th_planning', 'scripts'))
ROUTE_RECORDER = os.path.join(_SRC, 'route_recorder.py')
REPLAY_RUNNER = os.path.join(_SRC, 'replay_runner.py')


def _read(path: str) -> str:
    with open(path, encoding='utf-8') as f:
        return f.read()


def _tree(path: str) -> ast.AST:
    return ast.parse(_read(path), filename=path)


def _decalres_param(tree, param: str) -> bool:
    """declare_parameter('<param>', ...) の呼び出しが存在するか。"""
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == 'declare_parameter'
                and node.args
                and isinstance(node.args[0], ast.Constant)
                and node.args[0].value == param):
            return True
    return False


def _calls_function(tree, name: str) -> bool:
    """name を関数名（Name で直接呼ぶ）として呼び出している箇所があるか。"""
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call)
                and isinstance(node.func, ast.Name)
                and node.func.id == name):
            return True
    return False


def _function_calls(tree, fn_name: str, callee: str) -> bool:
    """関数 fn_name の本体の中で callee を呼び出しているか。"""
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == fn_name:
            return _calls_function(node, callee)
    return False


# ============================================================================
# 記録側 (route_recorder.py)
# ============================================================================


def test_recorder_declares_preview_max_points():
    assert _decalres_param(_tree(ROUTE_RECORDER), 'preview_max_points') is True, (
        'route_recorder.py に preview_max_points の declare_parameter が無い')


def test_recorder_preview_publish_path_calls_decimate_polyline():
    # 表示用の /route/preview だけを間引く（保存 JSON の points は触らない）。
    assert _function_calls(_tree(ROUTE_RECORDER), '_status_timer', 'decimate_polyline') is True, (
        'route_recorder.py の _status_timer で decimate_polyline が呼ばれていない')


# ============================================================================
# 再生側 (replay_runner.py)
# ============================================================================


def test_replay_runner_does_NOT_decode_but_declares_heartbeat():
    tree = _tree(REPLAY_RUNNER)
    # 再生側は target_index が点の添字を参照するため間引かない（WS-9F の設計判断）。
    assert _calls_function(tree, 'decimate_polyline') is False, (
        'replay_runner.py が decimate_polyline を呼んでいる（target_index の添字がずれる）')
    assert _decalres_param(tree, 'preview_heartbeat_ms') is True, (
        'replay_runner.py に preview_heartbeat_ms の declare_parameter が無い')
