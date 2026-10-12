"""fault_history_core の純コア試験（Spec-safety §3.5.5）。ROS 不要。"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'th_state'))

from th_state.fault_history_core import FaultHistoryCore  # noqa: E402

W = 1.0


def mk():
    c = FaultHistoryCore(W)
    c.on_state(0.0, 'FOLLOW', 'RUN', '')
    return c


def test_instant_fault_is_recorded():
    c = mk()
    c.on_active(10.0, ['LIDAR_LOST'], ['RECOVERABLE'])
    c.on_active(10.1, [], [])
    r = c.records()
    assert len(r) == 1
    assert r[0]['fault_type'] == 'LIDAR_LOST' and not r[0]['ongoing']
    assert abs(r[0]['duration_s'] - 0.1) < 1e-9
    assert (r[0]['mode'], r[0]['state']) == ('FOLLOW', 'RUN')
    assert r[0]['severity'] == 'RECOVERABLE' and r[0]['outcome'] == 'NONE'


def test_two_types_two_records_and_independent_clear():
    c = mk()
    c.on_active(10.0, ['ESP32_DISCONNECTED', 'LIDAR_LOST'], ['RECOVERABLE'] * 2)
    c.on_active(10.5, ['LIDAR_LOST'], ['RECOVERABLE'])
    c.on_active(11.0, [], [])
    d = {r['fault_type']: r for r in c.records()}
    assert len(d) == 2
    assert abs(d['ESP32_DISCONNECTED']['duration_s'] - 0.5) < 1e-9
    assert abs(d['LIDAR_LOST']['duration_s'] - 1.0) < 1e-9


def test_recoverable_during_critical_is_recorded():
    c = mk()
    c.on_active(1.0, ['DRIVE_RUNAWAY'], ['CRITICAL'])
    c.on_active(2.0, ['DRIVE_RUNAWAY', 'LIDAR_LOST'], ['CRITICAL', 'RECOVERABLE'])
    assert {r['fault_type'] for r in c.records()} == {'DRIVE_RUNAWAY', 'LIDAR_LOST'}


def test_ongoing_then_resolved():
    c = mk()
    c.on_active(1.0, ['LIDAR_LOST'], ['RECOVERABLE'])
    r = c.records()[0]
    assert r['ongoing'] and r['duration_s'] == -1.0
    c.on_active(3.0, [], [])
    r = c.records()[0]
    assert not r['ongoing'] and r['duration_s'] == 2.0


def test_outcome_pause_estop_none():
    c = mk()
    c.on_active(1.0, ['LIDAR_LOST'], ['RECOVERABLE'])
    c.on_state(1.05, 'FOLLOW', 'PAUSE', 'fault')
    assert c.records()[0]['outcome'] == 'PAUSE'
    c.on_state(1.2, 'ESTOP', 'NONE', '')
    assert c.records()[0]['outcome'] == 'ESTOP'
    c2 = mk()
    c2.on_active(1.0, ['LIDAR_LOST'], ['RECOVERABLE'])
    c2.on_active(1.1, [], [])
    c2.tick(5.0)
    assert c2.records()[0]['outcome'] == 'NONE'


def test_pause_not_from_fault_is_ignored():
    c = mk()
    c.on_active(1.0, ['LIDAR_LOST'], ['RECOVERABLE'])
    c.on_state(1.05, 'FOLLOW', 'PAUSE', 'jog')
    assert c.records()[0]['outcome'] == 'NONE'


def test_pause_after_clear_within_window_counts_but_not_after():
    c = mk()
    c.on_active(1.0, ['LIDAR_LOST'], ['RECOVERABLE'])
    c.on_active(1.1, [], [])
    c.on_state(1.4, 'FOLLOW', 'PAUSE', 'fault')
    assert c.records()[0]['outcome'] == 'PAUSE'
    c2 = mk()
    c2.on_active(1.0, ['LIDAR_LOST'], ['RECOVERABLE'])
    c2.on_active(1.1, [], [])
    c2.on_state(3.0, 'FOLLOW', 'PAUSE', 'fault')
    assert c2.records()[0]['outcome'] == 'NONE'


def test_state_before_active_order_reversed():
    c = mk()
    c.on_state(1.0, 'FOLLOW', 'PAUSE', 'fault')
    c.on_active(1.02, ['LIDAR_LOST'], ['RECOVERABLE'])
    r = c.records()[0]
    assert r['outcome'] == 'PAUSE'
    assert (r['mode'], r['state']) == ('FOLLOW', 'RUN')   # 発生時はエッジ直前の状態


def test_fault_while_already_stopped_is_none():
    c = mk()
    c.on_state(1.0, 'ESTOP', 'NONE', '')
    c.on_active(30.0, ['LIDAR_LOST'], ['RECOVERABLE'])
    c.on_state(30.1, 'ESTOP', 'NONE', '')
    assert c.records()[0]['outcome'] == 'NONE'


def test_cap_50_newest_first():
    c = mk()
    for i in range(51):
        t = 10.0 + i
        c.on_active(t, ['T%02d' % i], ['RECOVERABLE'])
        c.on_active(t + 0.1, [], [])
    r = c.records()
    assert len(r) == 50
    assert r[0]['fault_type'] == 'T50' and r[-1]['fault_type'] == 'T01'


def test_writes_open_then_final():
    c = mk()
    c.on_active(1.0, ['LIDAR_LOST'], ['RECOVERABLE'])
    w = c.drain_writes()
    assert len(w) == 1 and w[0]['ongoing'] and not w[0]['final']
    c.on_active(1.1, [], [])
    c.on_state(1.2, 'FOLLOW', 'PAUSE', 'fault')
    assert c.drain_writes() == []
    c.tick(1.5)
    assert c.drain_writes() == []
    c.tick(2.2)
    w = c.drain_writes()
    assert len(w) == 1 and w[0]['final'] and w[0]['outcome'] == 'PAUSE' and not w[0]['ongoing']
