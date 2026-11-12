"""
test_slam_control_restart_apply.py — A-11 ①: slam_toolbox の 1 回の再起動で
モードの再適用は 1 回だけ、望みのモードで終わること。

SlamControl を rclpy でインプロセス生成し、サービスの出入りとロックの
取れない周期を模す（slam_toolbox・実プロセスには触らない）。
rclpy が要るためホストでは走らない（Docker のみ。CLAUDE.md の除外一覧）。

背景（2026-11-11 実機）: 破棄後の再起動で、ロックが取れない周期に
_slam_ready を戻して検知をやり直していたため、1 回の再起動が 3 回の
「再起動」になり、画面の開始と停止のトグルと重なって停止側で終わった。
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

from slam_control import SlamControl


@pytest.fixture(scope='module', autouse=True)
def _rclpy():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture()
def slam(monkeypatch):
    node = SlamControl()
    svc = {'ready': True}
    node._cli_localization = SimpleNamespace(
        service_is_ready=lambda: svc['ready'],
        wait_for_service=lambda timeout_sec=0: svc['ready'])
    calls = []
    monkeypatch.setattr(node, '_set_localization',
                        lambda enabled: calls.append(enabled) or None)
    node._slam_ready = True
    node._svc = svc
    node._calls = calls
    yield node
    node.destroy_node()


def _cycle(node, n=1):
    for _ in range(n):
        node._check_slam_restart()


def test_busy_lock_cycles_apply_once_in_desired_mode(slam):
    """ロックが取れない周期をはさんでも、再適用は 1 回・望みのモードで終わる。"""
    slam._set_active(False)                 # 破棄の直後
    slam._svc['ready'] = False
    _cycle(slam)                            # 落ちた
    slam._svc['ready'] = True
    assert slam._lock.acquire(blocking=False)   # 画面の操作がロックを持っている
    try:
        _cycle(slam, 3)                     # 取れない周期が続く
        assert slam._calls == []
    finally:
        slam._lock.release()
    _cycle(slam, 3)                         # 取れたら 1 回だけ
    assert slam._calls == [True]            # False=停止(localization=True)の 1 回
    assert slam._mapping_active is False


def test_apply_uses_mode_set_while_waiting(slam):
    """待つ間に画面が地図作成を始めたら、その望みのモードで入れ直す。"""
    slam._set_active(False)
    slam._svc['ready'] = False
    _cycle(slam)
    slam._svc['ready'] = True
    assert slam._lock.acquire(blocking=False)
    try:
        _cycle(slam)
    finally:
        slam._lock.release()
    slam._set_active(True)
    _cycle(slam, 3)
    assert slam._calls == [False]           # 地図作成中 = localization オフ
    assert slam._mapping_active is True


def test_toggle_rejected_while_restart_pending(slam):
    """再起動の復帰を再適用が済むまで待つ間はトグルを受けない（重なって逆向きに入るのを防ぐ）。"""
    slam._svc['ready'] = False
    _cycle(slam)
    slam._svc['ready'] = True
    assert slam._lock.acquire(blocking=False)   # 再適用がまだ済まない
    try:
        _cycle(slam)
        assert slam._restart_pending is True
    finally:
        slam._lock.release()
    resp = SimpleNamespace(success=None, message='')
    out = slam._cb_toggle(SimpleNamespace(), resp)
    assert out.success is False
    assert '再起動' in out.message
    assert slam._calls == []
    _cycle(slam)                            # 再適用が済めば受ける
    assert slam._restart_pending is False
    out = slam._cb_toggle(SimpleNamespace(), SimpleNamespace(success=None, message=''))
    assert out.success is True


def test_set_mapping_true_is_idempotent(slam):
    """開始の要求が重なっても地図作成中で終わる（トグルと違い反転しない）。"""
    for _ in range(3):
        out = slam._cb_set_mapping(SimpleNamespace(data=True),
                                   SimpleNamespace(success=None, message=''))
        assert out.success is True
    assert slam._mapping_active is True
