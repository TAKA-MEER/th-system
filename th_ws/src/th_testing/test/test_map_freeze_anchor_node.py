"""
test_map_freeze_anchor_node.py — slam_control が map_freezer へ「どの地図を写しに取るか」の
契機（/map_session/freeze_anchor。Spec-onsite §4.0.2）を実際に発行する。

SlamControl を rclpy で生成し（インプロセス。手本は test_venue_loaded_node.py）、発行された
契機を記録して順序を見る。kill・待機・slam_toolbox への呼び出しだけを差し替える。

  a. 起動直後は unloaded
  b. 会場地図の読み直し成功は opening → opened（deserialize の前後）
  c. 読み直しの失敗は opened を出さず unloaded
  d. 地図作成の開始・破棄・slam_toolbox の再起動・教示の地図リセットは unloaded
  e. 経路地図の保存は saving → saved、失敗は saving → save_failed
  f. 書き足しの確定（commit_map_patch）も saving → saved
  g. 経路地図の読み直し（再生）も opening → opened

rclpy が要るためホストでは走らない（Docker のみ）。
変異: opened の発行を消す → b・g が赤 / 失敗側の unloaded を消す → c が赤 /
saved を serialize の前へ動かす → e・f が赤。
"""
import os
import sys
from types import SimpleNamespace

import pytest
import rclpy

_TEST_DIR = os.path.dirname(os.path.abspath(__file__))
_CFG = os.path.join(_TEST_DIR, '..', '..', 'th_config_manager')
sys.path.insert(0, _CFG)
sys.path.insert(0, os.path.join(_CFG, 'scripts'))

import slam_control
from slam_control import SlamControl
from th_config_manager.map_freezer_core import parse_anchor


@pytest.fixture(scope='module', autouse=True)
def _rclpy():
    rclpy.init()
    yield
    rclpy.shutdown()


class _Rec:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg.data)

    def events(self):
        return [parse_anchor(m)[2] for m in self.msgs]

    def clear(self):
        self.msgs.clear()


@pytest.fixture()
def slam():
    node = SlamControl()
    yield node
    node.destroy_node()


@pytest.fixture()
def rec(slam):
    r = _Rec()
    slam._pub_anchor = r
    return r


def _patch(slam, monkeypatch, tmp_path):
    base = os.path.join(str(tmp_path), 'map')
    for ext in ('.posegraph', '.data'):
        with open(base + ext, 'w') as f:
            f.write('x')
    monkeypatch.setattr(slam, '_map_session_base', lambda slot, sid: base)
    monkeypatch.setattr(slam, '_venue_open_reject_reason', lambda: None)
    monkeypatch.setattr(slam, '_stop_only_reject_reason', lambda: None)
    monkeypatch.setattr(slam, '_teach_reset_reject_reason', lambda: None)
    monkeypatch.setattr(slam, '_kill_slam_toolbox', lambda: [424242])
    monkeypatch.setattr(slam, '_wait_for_slam_restart', lambda *a: None)
    monkeypatch.setattr(slam, '_set_localization', lambda *a: None)
    monkeypatch.setattr(slam, '_save_occupancy_grid', lambda b: None)
    monkeypatch.setattr(slam, '_serialize', lambda b: None)
    monkeypatch.setattr(slam, '_apply_mapping',
                        lambda active: (slam._set_active(active), None)[1])
    monkeypatch.setattr(slam_control, 'call_and_wait',
                        lambda *a: (SimpleNamespace(), None))
    monkeypatch.setattr(slam_control, '_find_slam_toolbox_pids', lambda: [424242])
    return base


def _open(slam, slot='VENUE', mode='reload'):
    req = SimpleNamespace(slot=slot, session_id='venue' if slot == 'VENUE' else 'r1',
                          mode=mode, has_initial_pose=False,
                          initial_x=0.0, initial_y=0.0, initial_yaw=0.0)
    return slam._cb_map_session_open(req, SimpleNamespace(success=None, message=''))


def _resp():
    return SimpleNamespace(success=None, message='')


def test_a_startup_publishes_unloaded(monkeypatch):
    published = []
    orig = SlamControl._publish_anchor
    monkeypatch.setattr(SlamControl, '_publish_anchor',
                        lambda self, e: (published.append(e), orig(self, e))[1])
    node = SlamControl()
    try:
        assert published[:1] == ['unloaded']
    finally:
        node.destroy_node()


def test_b_venue_reload_is_opening_then_opened(slam, rec, monkeypatch, tmp_path):
    _patch(slam, monkeypatch, tmp_path)
    assert _open(slam).success is True
    assert rec.events() == ['opening', 'opened']


def test_c_failed_deserialize_is_opening_then_unloaded(slam, rec, monkeypatch, tmp_path):
    _patch(slam, monkeypatch, tmp_path)
    monkeypatch.setattr(slam_control, 'call_and_wait', lambda *a: (None, 'timeout'))
    assert _open(slam).success is False
    assert rec.events() == ['opening', 'unloaded']


def test_c_failed_respawn_wait_is_unloaded(slam, rec, monkeypatch, tmp_path):
    _patch(slam, monkeypatch, tmp_path)
    monkeypatch.setattr(slam, '_wait_for_slam_restart', lambda *a: 'timeout')
    assert _open(slam).success is False
    assert rec.events() == ['unloaded']


def test_d_mapping_start_discard_restart_reset_are_unloaded(
        slam, rec, monkeypatch, tmp_path):
    _patch(slam, monkeypatch, tmp_path)
    monkeypatch.setattr(slam, '_reject_if_mode_disallows', lambda r: None)
    slam._cb_set_mapping(SimpleNamespace(data=True), _resp())
    assert rec.events() == ['unloaded']
    rec.clear()
    slam._cb_discard_map(SimpleNamespace(), _resp())
    assert rec.events() == ['unloaded']
    rec.clear()
    assert _open(slam, slot='ROUTE', mode='reset').success is True
    assert rec.events() == ['unloaded']
    rec.clear()
    ready = {'v': False}
    monkeypatch.setattr(slam._cli_localization, 'service_is_ready', lambda: ready['v'])
    slam._slam_ready = True
    slam._check_slam_restart()
    ready['v'] = True
    slam._check_slam_restart()
    assert 'unloaded' in rec.events()


def test_e_route_save_is_saving_then_saved(slam, rec, monkeypatch, tmp_path):
    _patch(slam, monkeypatch, tmp_path)
    res = _open(slam, slot='ROUTE', mode='save')
    assert res.success is True, res.message
    assert rec.events() == ['saving', 'saved']


def test_e_route_save_failure_is_save_failed(slam, rec, monkeypatch, tmp_path):
    _patch(slam, monkeypatch, tmp_path)
    monkeypatch.setattr(slam, '_serialize', lambda b: 'disk full')
    res = _open(slam, slot='ROUTE', mode='save')
    assert res.success is False
    assert rec.events() == ['saving', 'save_failed']


def test_f_commit_map_patch_is_saving_then_saved(slam, rec, monkeypatch, tmp_path):
    base = _patch(slam, monkeypatch, tmp_path)
    assert slam._commit_route_patch(base) is None
    assert rec.events() == ['saving', 'saved']


def test_f_commit_map_patch_failure_is_save_failed(slam, rec, monkeypatch, tmp_path):
    base = _patch(slam, monkeypatch, tmp_path)
    monkeypatch.setattr(slam, '_set_localization', lambda *a: 'busy')
    assert slam._commit_route_patch(base) is not None
    assert rec.events() == ['saving', 'save_failed']


def test_g_route_reload_is_opening_then_opened(slam, rec, monkeypatch, tmp_path):
    _patch(slam, monkeypatch, tmp_path)
    res = _open(slam, slot='ROUTE')
    assert res.success is True, res.message
    assert rec.events() == ['opening', 'opened']
