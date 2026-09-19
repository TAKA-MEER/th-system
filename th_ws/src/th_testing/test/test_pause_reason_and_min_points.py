"""WS-9P: ノード側の配線を静的に固定する。

実機フィードバック 2 件（2026-09-04。VISION.md §2）への対処のうち、
C++ の gtest でも playwright でも届かない「ノードがパラメータ／フィールドを
実際に読み書きしているか」だけをここで押さえる。

- obstacle_limiter.cpp が obstacle_min_points を読んで params_ に入れているか
  （core の gtest は params を直接組み立てるので、ノードが読み落としても緑のまま。
   実際に変異チェックで素通りすることを確認した）
- replay_runner.py が RouteStatus.arrived を publish し、resume_path で到着ラッチを
  解除しているか（解除しないと終端で再生を押したとき RUN のまま固まる）
"""
import ast
import os
import re

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.abspath(os.path.join(_HERE, '..', '..', '..', '..'))
_LIMITER = os.path.join(
    _ROOT, 'th_ws', 'src', 'th_safety', 'src', 'obstacle_limiter.cpp')
_REPLAY = os.path.join(
    _ROOT, 'th_ws', 'src', 'th_planning', 'scripts', 'replay_runner.py')
_MSG = os.path.join(
    _ROOT, 'th_ws', 'src', 'th_system_msgs', 'msg', 'RouteStatus.msg')
_REGISTRY = os.path.join(
    _ROOT, 'th_ws', 'src', 'th_params', 'config', 'registry.yaml')


def _read(path: str) -> str:
    with open(path, encoding='utf-8') as f:
        return f.read()


# ── 1. 障害物とみなす点数（ノイズ 1 点で止まらない） ─────────────────

def test_obstacle_limiter_declares_min_points():
    src = _read(_LIMITER)
    assert 'declare_parameter("obstacle_min_points"' in src, (
        'obstacle_limiter.cpp が obstacle_min_points を宣言していない（WS-9P）')


def test_obstacle_limiter_reads_min_points_into_params():
    """宣言しただけで params_ に入れ忘れると、core は既定 1（＝最小値そのもの）で
    動き続ける。gtest は params を直接組み立てるのでこれを検知できない。"""
    src = _read(_LIMITER)
    m = re.search(r'params_\.obstacle_min_points\s*=\s*([^;]+);', src, re.S)
    assert m, 'params_.obstacle_min_points への代入が無い（WS-9P）'
    rhs = m.group(1)
    assert 'get_parameter("obstacle_min_points")' in rhs, (
        f'obstacle_min_points をパラメータから読んでいない: {rhs.strip()!r}')


def test_obstacle_min_points_is_in_registry():
    reg = _read(_REGISTRY)
    assert re.search(r'^- name: obstacle_min_points$', reg, re.M), (
        'registry.yaml に obstacle_min_points が無い（Spec-params が唯一の置き場）')


# ── 2. 一時停止の理由（終端到達かどうか） ───────────────────────────

def test_route_status_has_arrived_field():
    msg = _read(_MSG)
    assert re.search(r'^bool\s+arrived\b', msg, re.M), (
        'RouteStatus.msg に arrived が無い（WS-9P）。'
        'UI が「終端に達した」と「フォルトで止まった」を区別できない')


def test_replay_runner_publishes_arrived():
    src = _read(_REPLAY)
    m = re.search(r'msg\.arrived\s*=\s*([^\n]+)', src)
    assert m, 'replay_runner が RouteStatus.arrived を載せていない（WS-9P）'
    assert '_arrived_sent' in m.group(1), (
        f'arrived が到着ラッチ由来でない: {m.group(1).strip()!r}')


def test_resume_path_clears_arrived_latch():
    """終端に達したあとに再生を押した場合、ラッチを解除しないと evt.arrived を
    出し直せず RUN のまま機体が動かない状態で固まる。"""
    src = _read(_REPLAY)
    tree = ast.parse(src)

    # resume_path を処理している分岐を探す（elif name == 'resume_path':）
    found = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.If):
            continue
        test = node.test
        if (isinstance(test, ast.Compare)
                and isinstance(test.comparators[0], ast.Constant)
                and test.comparators[0].value == 'resume_path'):
            found.append(node)
    assert found, "replay_runner に resume_path の分岐が無い"

    body_src = '\n'.join(ast.unparse(n) for n in found[0].body)
    assert '_arrived_sent = False' in body_src, (
        'resume_path で到着ラッチ（_arrived_sent）を解除していない（WS-9P）。'
        f' 分岐の中身: {body_src!r}')


def test_load_route_does_not_reset_from_index_on_resume():
    """再開は「途中から」でなければならない。resume_path が _from_index を
    0 に戻していたら、止まった場所ではなく先頭から走り直してしまう。"""
    src = _read(_REPLAY)
    tree = ast.parse(src)
    for node in ast.walk(tree):
        if (isinstance(node, ast.If) and isinstance(node.test, ast.Compare)
                and isinstance(node.test.comparators[0], ast.Constant)
                and node.test.comparators[0].value == 'resume_path'):
            body_src = '\n'.join(ast.unparse(n) for n in node.body)
            assert '_from_index' not in body_src, (
                'resume_path が _from_index を触っている。'
                f'途中からの再開にならない: {body_src!r}')
            return
    pytest.fail("resume_path の分岐が見つからない")


if __name__ == '__main__':
    import sys
    sys.exit(pytest.main([__file__, '-v']))
