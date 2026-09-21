"""
test_slam_control_restart.py — O-e3 の slam_control 側の振る舞い試験。

SlamControl を rclpy で実際に生成し（launch しないインプロセス試験）、
重い部分だけ差し替えて /slam_control/estimator_restarting の受信順を見る:
  g. _handle_map_reload: _kill_slam_toolbox が呼ばれた時点で既に true を
     受信済み（kill 呼び出しの中で受信履歴を見る）。正常終了後は false
  h. _handle_map_reload で途中の関数が例外を投げても最後は false
  i. _cb_discard_map: kill 後に true、サービス復帰後に _check_slam_restart
     で false
  j. クラッシュ（フラグ無し・締切無しでサービスが落ちて戻る）では
     true が出ない

rclpy・slam_toolbox.srv が要るためホストでは走らない（Docker のみ。
CLAUDE.md の除外一覧）。kill・wait・service 呼び出しはすべて差し替え、
実プロセスには触らない（実機稼働中でも安全）。
"""
import os
import sys
import time
from types import SimpleNamespace

import pytest
import rclpy
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool

_TEST_DIR = os.path.dirname(os.path.abspath(__file__))
# slam_control.py は scripts/ にある（パッケージではない）。
# th_config_manager パッケージ（service_call・slam_control_logic）はその親。
_CFG = os.path.join(_TEST_DIR, '..', '..', 'th_config_manager')
sys.path.insert(0, _CFG)
sys.path.insert(0, os.path.join(_CFG, 'scripts'))

import slam_control
from slam_control import SlamControl


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
    """estimator_restarting の受信順を記録する試験用ノード。
    発行側と同じ TRANSIENT_LOCAL で購読する（発行が購読のマッチより
    先でも届く。launch 試験の QoS の罠と同じ）。"""
    node = rclpy.create_node('test_slam_control_restart')
    received = []
    qos = QoSProfile(
        depth=20,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(Bool, '/slam_control/estimator_restarting',
                             lambda msg: received.append(bool(msg.data)), qos)
    yield node, received
    node.destroy_node()


def _spin(node, duration=0.5):
    deadline = time.time() + duration
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)


def _reload_request():
    return SimpleNamespace(has_initial_pose=True, initial_x=1.0,
                           initial_y=2.0, initial_yaw=0.5)


def _reload_base(tmp_path):
    base = os.path.join(str(tmp_path), 'route_map')
    with open(base + '.posegraph', 'w') as f:
        f.write('graph')
    with open(base + '.data', 'w') as f:
        f.write('data')
    return base


def _settle(node, received):
    """発行-購読の discovery を待つ（1 秒）。起動時の latched false
    （SlamControl.__init__ が出す）は試験の対象外なので捨てる。"""
    _spin(node, 1.0)
    received.clear()


def _values(received):
    return list(received)


# ════════════════════════════════════════════════════════
def test_g_reload_true_before_kill_false_after(slam, probe, tmp_path, monkeypatch):
    """g: kill 呼び出し時点で既に true 受信済み。正常終了後は false。"""
    node, received = probe
    _settle(node, received)
    kill_saw = {}

    def fake_kill():
        _spin(node, 0.3)
        kill_saw['true_already'] = True in _values(received)
        return [424242]

    monkeypatch.setattr(slam, '_kill_slam_toolbox', fake_kill)
    monkeypatch.setattr(slam, '_wait_for_slam_restart', lambda *a: None)
    monkeypatch.setattr(slam, '_set_localization', lambda *a: None)
    monkeypatch.setattr(slam_control, 'call_and_wait',
                        lambda *a: (SimpleNamespace(), None))

    resp = SimpleNamespace(success=None, message='')
    out = slam._handle_map_reload(resp, _reload_base(tmp_path), _reload_request())
    assert out.success is True
    assert kill_saw.get('true_already') is True, (
        'kill 呼び出し時点で true を受信していない（kill の前に出していない）')
    _spin(node, 0.5)
    values = _values(received)
    assert True in values and False in values
    assert values.index(True) < values.index(False), (
        'true の前に false が来ている')


def test_h_reload_exception_still_false(slam, probe, tmp_path, monkeypatch):
    """h: 途中で例外を投げても最後は false（true のまま残らない）。"""
    node, received = probe
    _settle(node, received)

    def fake_kill():
        return [424242]

    def boom(*a):
        raise RuntimeError('boom')

    monkeypatch.setattr(slam, '_kill_slam_toolbox', fake_kill)
    monkeypatch.setattr(slam, '_wait_for_slam_restart', lambda *a: None)
    monkeypatch.setattr(slam, '_set_localization', boom)
    monkeypatch.setattr(slam_control, 'call_and_wait',
                        lambda *a: (SimpleNamespace(), None))

    resp = SimpleNamespace(success=None, message='')
    with pytest.raises(RuntimeError):
        slam._handle_map_reload(resp, _reload_base(tmp_path), _reload_request())
    _spin(node, 0.5)
    values = _values(received)
    assert True in values, '例外前に true が出ていない'
    assert values[-1] is False, (
        f'例外後に false に戻っていない: {values}')


def test_i_discard_true_then_false_on_service_back(slam, probe, monkeypatch):
    """i: 破棄は kill 後に true、サービス復帰を _check_slam_restart が
    検知したら false。"""
    node, received = probe
    _settle(node, received)
    monkeypatch.setattr(slam_control, '_find_slam_toolbox_pids',
                        lambda: [424242])
    monkeypatch.setattr(slam, '_kill_slam_toolbox', lambda: [424242])

    resp = SimpleNamespace(success=None, message='')
    out = slam._cb_discard_map(SimpleNamespace(), resp)
    assert out.success is True
    _spin(node, 0.5)
    assert True in _values(received), 'kill 後に true が出ない'

    # サービス復帰を差し替えて _check_slam_restart に検知させる。
    slam._cli_localization = SimpleNamespace(service_is_ready=lambda: True)
    slam._check_slam_restart()
    _spin(node, 0.5)
    values = _values(received)
    assert False in values, 'サービス復帰後に false に戻らない'
    assert values.index(True) < values.index(False)


def test_j_crash_never_true(slam, probe, monkeypatch):
    """j: クラッシュ（フラグ無し・締切無し）では true が出ない。
    サービスが落ちて戻っても同様。"""
    node, received = probe
    _settle(node, received)
    assert slam._reload_in_progress is False
    assert slam._discard_deadline is None
    # モード再適用（_apply_mapping 経路）には入らせない。j の要点は
    # 「true を出さない」ことだけ。
    monkeypatch.setattr(slam, '_set_localization', lambda *a: None)

    slam._cli_localization = SimpleNamespace(service_is_ready=lambda: False)
    slam._check_slam_restart()
    _spin(node, 0.5)
    slam._cli_localization = SimpleNamespace(service_is_ready=lambda: True)
    slam._check_slam_restart()
    _spin(node, 0.5)
    assert True not in _values(received), (
        f'クラッシュで true が出た: {_values(received)}')
