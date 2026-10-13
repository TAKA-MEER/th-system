"""
test_venue_loaded_node.py — 会場地図を「開いた」かどうかは機体が持つ（Spec-webui §3.11 S-21）。

SlamControl を rclpy で実際に生成し（launch しないインプロセス試験。手本は
test_slam_control_restart.py）、/map_session/status（slot:VENUE）の mode を、
実際に発行された 1 通を購読して見る。kill・待機・slam_toolbox への呼び出しだけを差し替える。

  a. 起動直後は UNLOADED（地図を何も開いていない）
  b. /map_session/open の VENUE 読み直しが成功すると LOCALIZING
  c. /slam_control/discard_map で UNLOADED
  d. 地図作成の開始（set_mapping true）で MAPPING。止めても読み込み済みには戻らない
  e. 読み込み済みのまま slam_toolbox が再起動された（respawn）ら UNLOADED
  f. 読み直しに失敗したら（kill 後に落ちた）読み込み済みにならない
  g. 経路地図（ROUTE）の読み直しは会場地図を載せ替えるので UNLOADED

rclpy が要るためホストでは走らない（Docker のみ）。
変異: 読み直し成功で _venue_loaded を立てる行を消す → b が赤 / 起動直後に True → a が赤 /
discard で下ろす行を消す → c が赤 / 再起動検知で下ろす行を消す → e が赤。
"""
import os
import sys
import time
from types import SimpleNamespace

import pytest
import rclpy
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

_TEST_DIR = os.path.dirname(os.path.abspath(__file__))
_CFG = os.path.join(_TEST_DIR, '..', '..', 'th_config_manager')
sys.path.insert(0, _CFG)
sys.path.insert(0, os.path.join(_CFG, 'scripts'))

import slam_control
from slam_control import SlamControl
from th_system_msgs.msg import MapSessionStatus


@pytest.fixture(scope='module', autouse=True)
def _rclpy():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture()
def slam():
    node = SlamControl()
    yield node
    node.destroy_node()


@pytest.fixture()
def probe():
    node = rclpy.create_node('test_venue_loaded_probe')
    got = []
    qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(MapSessionStatus, '/map_session/status',
                             lambda m: got.append(m), qos)
    yield node, got
    node.destroy_node()


def _spin(node, duration=0.5):
    end = time.time() + duration
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


def _mode(slam, probe):
    """いまの状態を実際に発行させ、購読側に届いた最後の 1 通の mode を返す。"""
    node, got = probe
    got.clear()
    slam._publish_venue_status()
    deadline = time.time() + 3.0
    while time.time() < deadline and not got:
        rclpy.spin_once(node, timeout_sec=0.05)
    assert got, '/map_session/status が届かない'
    assert got[-1].slot == 'VENUE'
    return got[-1].mode


def _patch(slam, monkeypatch, tmp_path):
    base = os.path.join(str(tmp_path), 'venue')
    for ext in ('.posegraph', '.data'):
        with open(base + ext, 'w') as f:
            f.write('x')
    monkeypatch.setattr(slam, '_map_session_base', lambda slot, sid: base)
    monkeypatch.setattr(slam, '_venue_open_reject_reason', lambda: None)
    monkeypatch.setattr(slam, '_stop_only_reject_reason', lambda: None)
    monkeypatch.setattr(slam, '_kill_slam_toolbox', lambda: [424242])
    monkeypatch.setattr(slam, '_wait_for_slam_restart', lambda *a: None)
    monkeypatch.setattr(slam, '_set_localization', lambda *a: None)
    monkeypatch.setattr(slam, '_apply_mapping',
                        lambda active: (slam._set_active(active), None)[1])
    monkeypatch.setattr(slam_control, 'call_and_wait',
                        lambda *a: (SimpleNamespace(), None))
    monkeypatch.setattr(slam_control, '_find_slam_toolbox_pids', lambda: [424242])


def _open(slam, slot='VENUE', mode='reload'):
    req = SimpleNamespace(slot=slot, session_id='venue' if slot == 'VENUE' else 'r1',
                          mode=mode, has_initial_pose=False,
                          initial_x=0.0, initial_y=0.0, initial_yaw=0.0)
    return slam._cb_map_session_open(req, SimpleNamespace(success=None, message=''))


def test_a_fresh_start_is_unloaded(slam, probe):
    assert _mode(slam, probe) == 'UNLOADED'


def test_b_venue_reload_success_is_localizing(slam, probe, monkeypatch, tmp_path):
    _patch(slam, monkeypatch, tmp_path)
    res = _open(slam)
    assert res.success is True, res.message
    assert _mode(slam, probe) == 'LOCALIZING'


def test_c_discard_returns_to_unloaded(slam, probe, monkeypatch, tmp_path):
    _patch(slam, monkeypatch, tmp_path)
    assert _open(slam).success is True
    assert _mode(slam, probe) == 'LOCALIZING'
    res = slam._cb_discard_map(SimpleNamespace(), SimpleNamespace(success=None, message=''))
    assert res.success is True, res.message
    assert _mode(slam, probe) == 'UNLOADED'


def test_d_mapping_start_is_mapping_and_stop_does_not_restore(
        slam, probe, monkeypatch, tmp_path):
    _patch(slam, monkeypatch, tmp_path)
    assert _open(slam).success is True
    res = slam._cb_set_mapping(SimpleNamespace(data=True),
                               SimpleNamespace(success=None, message=''))
    assert res.success is True, res.message
    assert _mode(slam, probe) == 'MAPPING'
    slam._cb_set_mapping(SimpleNamespace(data=False),
                         SimpleNamespace(success=None, message=''))
    assert _mode(slam, probe) == 'UNLOADED'


def test_e_slam_restart_while_loaded_is_unloaded(slam, probe, monkeypatch, tmp_path):
    _patch(slam, monkeypatch, tmp_path)
    assert _open(slam).success is True
    assert _mode(slam, probe) == 'LOCALIZING'
    # respawn: サービスが落ちて戻る（読み直しの最中ではない）
    ready = {'v': False}
    monkeypatch.setattr(slam._cli_localization, 'service_is_ready', lambda: ready['v'])
    slam._slam_ready = True
    slam._check_slam_restart()
    ready['v'] = True
    slam._check_slam_restart()
    assert _mode(slam, probe) == 'UNLOADED'


def test_f_failed_reload_is_not_loaded(slam, probe, monkeypatch, tmp_path):
    _patch(slam, monkeypatch, tmp_path)
    assert _open(slam).success is True
    monkeypatch.setattr(slam_control, 'call_and_wait',
                        lambda *a: (None, 'timeout'))
    res = _open(slam)
    assert res.success is False
    assert _mode(slam, probe) == 'UNLOADED'


def test_g_route_reload_replaces_venue(slam, probe, monkeypatch, tmp_path):
    _patch(slam, monkeypatch, tmp_path)
    assert _open(slam).success is True
    res = _open(slam, slot='ROUTE')
    assert res.success is True, res.message
    assert _mode(slam, probe) == 'UNLOADED'
