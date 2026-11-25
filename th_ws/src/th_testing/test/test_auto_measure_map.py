"""
test_auto_measure_map.py — 8-g P5a: effect start_measure_map で slam_toolbox を
まっさらな地図作成にし、instance_id が新しくなり dirty が立たないこと。

SlamControl を rclpy でインプロセス生成し、 slam_toolbox のプロセスとサービスを
偽物に差し替える（実プロセスには触らない）。rclpy が要るため Docker のみ。

正本は DetailedDesign-measure.md §6.2（地図 -120・§4.1）。
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

import slam_control  # noqa: E402
from slam_control import SlamControl  # noqa: E402
from th_system_msgs.msg import StateEffect  # noqa: E402


@pytest.fixture(scope='module', autouse=True)
def _rclpy():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture()
def slam(monkeypatch):
    node = SlamControl()
    # _wait_for_slam_restart の待ち条件を実プロセスなしで通す（新旧 PID が違うこと）。
    monkeypatch.setattr(node, '_kill_slam_toolbox', lambda: [1111])
    monkeypatch.setattr(node, '_wait_for_slam_restart', lambda pids, timeout_s: None)
    # localization 切替サービスは無いものとして通す（False = mapping 開始）。
    svc = {'calls': []}
    class _Svc:
        def wait_for_service(self, timeout_sec=0):
            return True
    # _cli_localization は _set_localization の中でもう一度だけ使われるため、
    # 単に _set_localization を置き換える。
    monkeypatch.setattr(
        node, '_set_localization',
        lambda enabled: svc['calls'].append(enabled) or None)
    monkeypatch.setattr(node, '_sync_estimator_restarting', lambda: None)
    node._svc_calls = svc['calls']
    yield node
    node.destroy_node()


def _effect_msg():
    msg = StateEffect()
    msg.name = 'start_measure_map'
    msg.dest = 'map_session'
    msg.args_json = '{}'
    return msg


def test_start_measure_map_fresh_instance_and_not_dirty(slam):
    """effect で instance_id が新しくなり、dirty は立たず、地図作成開始になる。"""
    old_id = slam._instance_id
    slam._venue_dirty = True   # 前の地図が未保存であっても
    slam._venue_loaded = True
    slam._mapping_active = False
    slam._on_effect(_effect_msg())
    assert slam._instance_id != old_id, 'instance_id が変わっていない'
    assert slam._venue_dirty is False, 'dirty が立っている（未保存に数えている）'
    assert slam._venue_loaded is False, 'venue_loaded が残っている'
    assert slam._mapping_active is True, '地図作成が始まっていない'
    assert slam._svc_calls[-1] is False, 'mapping モード切替（False）が呼ばれていない'
    assert slam._open_route_base == '', '前の経路の地図への保存先が残っている'


def test_start_measure_map_ignores_other_dests(slam):
    """宛先が map_session でない effect は無視する（従来どおり）。"""
    old_id = slam._instance_id
    msg = _effect_msg()
    msg.dest = 'something_else'
    slam._on_effect(msg)
    assert slam._instance_id == old_id
