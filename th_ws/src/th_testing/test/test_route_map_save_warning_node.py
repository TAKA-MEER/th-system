"""
test_route_map_save_warning_node.py — A-21: 地図の画像を残せなかったことを画面へ伝える。

route_recorder は教示の「保存」で /map_session/open（ROUTE/save）を呼ぶ。slam_control が
占有格子（pgm/yaml）の書き出しに最後まで失敗しても、経路（posegraph/JSON）は保存される。
そのとき黙って成功にせず、RouteStatus.map_save_failed を立てて S-13 に警告を出させる。

RouteRecorder を rclpy で生成し（インプロセス）、map_session のサービスだけ代役にする。
  a. 保存後に pgm/yaml が揃っていれば _save_map_for_route は True
  b. 揃っていなければ False（slam_control の応答が成功でも）
  c. 保存（_finalize_and_close）が b の結果を _map_save_failed に反映し、
     RouteStatus.map_save_failed に載る

変異: _map_save_failed を立てない → c が赤 / 画像の有無を見ずに常に True → b が赤。
rclpy が要るためホストでは走らない。
"""
import os
import sys
from types import SimpleNamespace

import pytest
import rclpy

_TEST_DIR = os.path.dirname(os.path.abspath(__file__))
_PLAN = os.path.join(_TEST_DIR, '..', '..', 'th_planning')
sys.path.insert(0, _PLAN)
sys.path.insert(0, os.path.join(_PLAN, 'scripts'))

import route_recorder
from route_recorder import RouteRecorder


@pytest.fixture(scope='module', autouse=True)
def _rclpy():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture()
def env(monkeypatch, tmp_path):
    node = RouteRecorder()
    node._routes_dir = str(tmp_path)
    node._map_frame = 'map'
    node._frame_id = 'map'
    node._map_session_id = 'sess'
    node._route_id = 'r1'
    node._name = '試験ブロックC'
    # /map_session/open の代役。応答は成功（slam_control は画像の失敗を message に
    # 載せるだけで success は真のまま）。
    node._map_session_client = SimpleNamespace(wait_for_service=lambda timeout_sec: True)
    monkeypatch.setattr(
        route_recorder, 'call_and_wait',
        lambda *a: (SimpleNamespace(success=True, message='地図を保存しました'), None))
    yield SimpleNamespace(node=node, dir=str(tmp_path))
    node.destroy_node()


def _write_map(directory, route_id):
    for ext in ('.pgm', '.yaml'):
        with open(os.path.join(directory, route_id + ext), 'w', encoding='utf-8') as f:
            f.write('x')


def test_map_image_present_is_true(env):
    _write_map(env.dir, 'r1')
    assert env.node._save_map_for_route('r1') is True


def test_map_image_missing_is_false(env):
    assert env.node._save_map_for_route('r1') is False, (
        '応答が成功でも pgm/yaml が無ければ警告の対象')


def test_finalize_sets_map_save_failed_into_status(env, monkeypatch):
    monkeypatch.setattr(env.node, '_save_map_for_route', lambda _rid: False)
    monkeypatch.setattr(env.node, '_close_recording', lambda: None)
    monkeypatch.setattr(env.node, '_publish_status_snapshot', lambda: None)
    monkeypatch.setattr(route_recorder, 'route_to_dict', lambda _route: {})
    env.node._recorder = SimpleNamespace(finalize=lambda *a, **k: object())
    env.node._saved = False
    env.node._map_save_failed = False
    env.node._finalize_and_close()
    assert env.node._saved is True
    assert env.node._map_save_failed is True, (
        '画像を残せなかったことが保存の結果に反映されていない')
    # 実機では _close_recording が記録を閉じる（ここでは no-op にした）ので、
    # 閉じたあとの状態組み立てと同じにする。
    env.node._recorder = None
    msg, _preview, _stamp = env.node._build_status_msg()
    assert msg.map_save_failed is True, (
        'RouteStatus.map_save_failed に載っていない（S-13 が警告を出せない）')


def test_status_field_false_when_map_saved(env):
    env.node._map_save_failed = False
    msg, _preview, _stamp = env.node._build_status_msg()
    assert msg.map_save_failed is False
