"""
test_slam_control_grid_retry_node.py — A-21: 占有格子（pgm/yaml）の書き出しのやり直し。

2026-11-13 実機 T-21: 通信が一瞬切れた瞬間の保存で map_saver が /map を 2 秒で
受け取れず失敗した。slam_control は警告ログだけ出して成功扱いにしていたため、
pgm の無い経路が「保存しました」になり、再生で位置探しが黙って効かなかった。

SlamControl を rclpy で生成し（インプロセス）、SaveMap だけ代役にする。
  a. 途中まで失敗しても MAP_SAVE_ATTEMPTS 回までやり直して成功する
  b. 最後まで失敗したら経路の保存（serialize/凍結）は成功のまま、
     応答の message に警告（MAP_SAVE_WARNING_TEXT）が載る
  c. effect 経路（_commit_route_patch。S-14 の地図更新の「保存」）も同じ回数やり直し、
     応答が無いので成功のまま警告ログに残す

変異: やり直しを 1 回にする → a が赤 / 失敗しても警告を message に載せない → b が赤。
rclpy が要るためホストでは走らない。
"""
import os
import sys
from types import SimpleNamespace

import pytest
import rclpy
from slam_toolbox.srv import SaveMap

_TEST_DIR = os.path.dirname(os.path.abspath(__file__))
_CFG = os.path.join(_TEST_DIR, '..', '..', 'th_config_manager')
sys.path.insert(0, _CFG)
sys.path.insert(0, os.path.join(_CFG, 'scripts'))

import slam_control
from slam_control import MAP_SAVE_ATTEMPTS, MAP_SAVE_WARNING_TEXT, SlamControl


@pytest.fixture(scope='module', autouse=True)
def _rclpy():
    rclpy.init()
    yield
    rclpy.shutdown()


class _SaveStub:
    """SaveMap の代役。fail_until 回までは失敗、その後は成功を返す。"""

    def __init__(self):
        self.calls = 0
        self.fail_until = 0

    def wait_for_service(self, timeout_sec=0.0):   # noqa: ARG002
        return True

    def __call__(self, _node, _client, _req, _timeout):
        self.calls += 1
        if self.calls <= self.fail_until:
            return SimpleNamespace(result=SaveMap.Response.RESULT_NO_MAP_RECEIEVD), None
        return SimpleNamespace(result=SaveMap.Response.RESULT_SUCCESS), None


@pytest.fixture()
def env(monkeypatch, tmp_path):
    slam = SlamControl()
    stub = _SaveStub()
    route_dir = tmp_path / 'routes'
    route_dir.mkdir()
    slam._map_dir = str(route_dir)
    slam._venue_map_dir = str(tmp_path / 'venue')
    slam._cli_save = stub
    monkeypatch.setattr(slam_control, 'call_and_wait', stub)
    # やり直しの待ちを飛ばす（間隔そのものは test_interval_is_positive）。
    monkeypatch.setattr(slam_control, 'MAP_SAVE_RETRY_INTERVAL_SEC', 0.0)
    monkeypatch.setattr(slam, '_serialize', lambda _base: None)
    monkeypatch.setattr(slam, '_set_localization', lambda _v: None)
    monkeypatch.setattr(slam, '_set_active', lambda _v: None)
    monkeypatch.setattr(slam, '_publish_anchor', lambda _e: None)
    monkeypatch.setattr(slam, '_open_route_base', '', raising=False)
    yield SimpleNamespace(slam=slam, stub=stub, base=str(route_dir / 'r1'))
    slam.destroy_node()


def test_retries_until_success(env):
    env.stub.fail_until = 2
    resp = env.slam._handle_map_save(
        SimpleNamespace(success=None, message=''), env.base)
    assert resp.success is True
    assert env.stub.calls == 3, '2 回失敗したら 3 回目で成功するはず'
    assert MAP_SAVE_WARNING_TEXT not in resp.message


def test_all_attempts_fail_warns_but_keeps_route(env):
    env.stub.fail_until = 99
    resp = env.slam._handle_map_save(
        SimpleNamespace(success=None, message=''), env.base)
    assert env.stub.calls == MAP_SAVE_ATTEMPTS, 'やり直しの回数が定数と違う'
    assert resp.success is True, '経路の保存は成功のまま（A-21）'
    assert MAP_SAVE_WARNING_TEXT in resp.message, '地図の画像の警告が応答に載っていない'


def test_commit_route_patch_also_retries(env):
    env.stub.fail_until = 99
    err = env.slam._commit_route_patch(env.base)
    assert env.stub.calls == MAP_SAVE_ATTEMPTS, 'effect 経路も同じ回数やり直す'
    assert err is None, '経路の保存は成功のまま（A-21）'


def test_interval_is_positive():
    """やり直しの間隔は 0 より大きい（一過性の失敗を拾うための待ち）。"""
    assert slam_control.MAP_SAVE_RETRY_INTERVAL_SEC > 0
    assert MAP_SAVE_ATTEMPTS >= 2


def test_total_budget_fits_route_save_timeout():
    """やり直し全体が /map_session/open の応答待ち（route_recorder の 30 秒）に
    収まること（A-21。serialize と set_localization の各 5 秒も足す）。"""
    attempts = MAP_SAVE_ATTEMPTS
    total = (attempts * (1.0 + slam_control.MAP_SAVE_CALL_TIMEOUT_SEC)
             + (attempts - 1) * slam_control.MAP_SAVE_RETRY_INTERVAL_SEC)
    assert total + 5.0 + 5.0 < 30.0, f'やり直しの合計が長すぎる: {total:.1f}s'
