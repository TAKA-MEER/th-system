"""
test_venue_map_patch_node.py — 地図更新 ON の「保存」（commit_map_patch）の保存先。
S-21（会場地図を開いている間）は venue/map、S-14（経路地図）は経路地図へ書く。

SlamControl を rclpy で生成し（インプロセス。手本は test_map_freeze_anchor_node.py）、
_on_effect に commit_map_patch を流して、どのファイルへ serialize したかを見る。

  a. 経路地図を開いたあとに会場地図を開いて保存すると、書き先は venue/map（経路地図でない）
  b. 経路地図だけ開いていれば経路地図へ（従来どおり）
  c. 会場地図を開いたあとに経路地図を開き直すと経路地図へ（会場地図は書かない）
  d. どちらも開いていなければ拒否（serialize しない）
  e. 会場への保存で instance_id は変えず、.instance_id へ書く。契機は saving → saved、失敗は save_failed

rclpy が要るためホストでは走らない。
変異: 会場の reload 成功で _open_route_base を消す行を外す → a が赤 /
_commit_venue_patch を _commit_route_patch に戻す → a・e が赤。
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


@pytest.fixture()
def env(monkeypatch, tmp_path):
    slam = SlamControl()
    rec = _Rec()
    slam._pub_anchor = rec
    venue_dir = tmp_path / 'venue'
    route_dir = tmp_path / 'routes'
    venue_dir.mkdir()
    route_dir.mkdir()
    slam._venue_map_dir = str(venue_dir)
    slam._map_dir = str(route_dir)
    for d, n in ((venue_dir, 'map'), (route_dir, 'r1')):
        for ext in ('.posegraph', '.data'):
            (d / (n + ext)).write_text('x')
    serialized = []
    monkeypatch.setattr(slam, '_serialize', lambda b: serialized.append(b))
    monkeypatch.setattr(slam, '_venue_open_reject_reason', lambda: None)
    monkeypatch.setattr(slam, '_stop_only_reject_reason', lambda: None)
    monkeypatch.setattr(slam, '_kill_slam_toolbox', lambda: [424242])
    monkeypatch.setattr(slam, '_wait_for_slam_restart', lambda *a: None)
    monkeypatch.setattr(slam, '_set_localization', lambda *a: None)
    monkeypatch.setattr(slam, '_save_occupancy_grid', lambda b: None)
    monkeypatch.setattr(slam_control, 'call_and_wait',
                        lambda *a: (SimpleNamespace(), None))
    monkeypatch.setattr(slam_control, '_find_slam_toolbox_pids', lambda: [424242])
    yield SimpleNamespace(slam=slam, rec=rec, serialized=serialized,
                          venue=str(venue_dir / 'map'), route=str(route_dir / 'r1'))
    slam.destroy_node()


def _open(slam, slot, mode='reload'):
    req = SimpleNamespace(slot=slot, session_id='venue' if slot == 'VENUE' else 'r1',
                          mode=mode, has_initial_pose=False,
                          initial_x=0.0, initial_y=0.0, initial_yaw=0.0)
    return slam._cb_map_session_open(req, SimpleNamespace(success=None, message=''))


def _patch_effect(slam):
    slam._on_effect(SimpleNamespace(dest='map_session', name='commit_map_patch'))


def test_a_route_then_venue_saves_to_venue(env):
    assert _open(env.slam, 'ROUTE').success is True
    assert _open(env.slam, 'VENUE').success is True
    env.serialized.clear()
    _patch_effect(env.slam)
    assert env.serialized == [env.venue]


def test_b_route_only_saves_to_route(env):
    assert _open(env.slam, 'ROUTE').success is True
    env.serialized.clear()
    _patch_effect(env.slam)
    assert env.serialized == [env.route]


def test_c_venue_then_route_saves_to_route(env):
    assert _open(env.slam, 'VENUE').success is True
    assert _open(env.slam, 'ROUTE').success is True
    env.serialized.clear()
    _patch_effect(env.slam)
    assert env.serialized == [env.route]


def test_c2_venue_only_saves_to_venue(env):
    assert _open(env.slam, 'VENUE').success is True
    env.serialized.clear()
    _patch_effect(env.slam)
    assert env.serialized == [env.venue]


def test_d_nothing_open_is_rejected(env):
    env.rec.msgs.clear()
    _patch_effect(env.slam)
    assert env.serialized == []
    assert env.rec.events() == []   # saving は出ない


def test_e_venue_patch_keeps_instance_id_and_anchors(env):
    assert _open(env.slam, 'VENUE').success is True
    iid = env.slam._instance_id
    env.rec.msgs.clear()
    env.slam._venue_dirty = True
    _patch_effect(env.slam)
    assert env.slam._instance_id == iid
    assert open(env.venue + '.instance_id').read() == iid
    assert env.slam._venue_dirty is False
    assert env.slam._venue_loaded is True
    assert env.rec.events() == ['saving', 'saved']


def test_e_venue_patch_failure_is_save_failed(env, monkeypatch):
    assert _open(env.slam, 'VENUE').success is True
    env.rec.msgs.clear()
    monkeypatch.setattr(env.slam, '_serialize', lambda b: 'disk full')
    _patch_effect(env.slam)
    assert env.rec.events() == ['saving', 'save_failed']
