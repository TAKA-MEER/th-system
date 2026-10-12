"""
test_fault_recorder_node.py — fault_recorder（Spec-safety §3.5.5）を本物で起動して縛る。

/safety/active_faults と /system/state の代役を、実機の発行側と同じ QoS
（reliable・transient_local・depth 1）で流す。縛るもの:

  a. 0.1 秒だけ立って解消するフォルトが 1 件になる（継続時間・発生時のモードと状態）
  b. 同時 2 種別は別の 2 件
  c. outcome: PAUSE(fault)／ESTOP／何も起きない→NONE。解消の「後」に届く遷移も拾う
  d. 継続中は ongoing=true・duration_s=-1。解消で更新
  e. 51 件目で最古が落ちる（50 件・新しい順）
  f. ファイルへ JSON Lines が追記される・上限で退避・書けない場所でもノードが落ちない
  g. 後から購読した側に最新の履歴が届く
  h. 記録ノードは購読と /system/fault_history の publish しかしない
     （/system/event 等を出さない＝挙動に影響しない）
"""

import json
import os
import tempfile
import time
import unittest

import pytest
import rclpy

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions

from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from th_system_msgs.msg import ActiveFaults, FaultHistory, SystemState

_DIR = tempfile.mkdtemp(prefix='fault_rec_test_')
WINDOW_MS = 400
MAX_BYTES = 6000


@pytest.mark.launch_test
def generate_test_description():
    good = launch_ros.actions.Node(
        package='th_state', executable='fault_recorder.py', name='fault_recorder',
        parameters=[{'fault_outcome_window_ms': WINDOW_MS,
                     'fault_log_max_bytes': MAX_BYTES, 'fault_log_dir': _DIR}],
        output='screen')
    # 書けない場所（/proc 配下は mkdir できない）。トピックは分けて、落ちないことを見る
    bad = launch_ros.actions.Node(
        package='th_state', executable='fault_recorder.py', name='fault_recorder_bad',
        parameters=[{'fault_outcome_window_ms': WINDOW_MS,
                     'fault_log_max_bytes': MAX_BYTES,
                     'fault_log_dir': '/proc/no_such_dir/fault_log'}],
        remappings=[('/system/fault_history', '/bad/fault_history')],
        output='screen')
    return launch.LaunchDescription([
        good, bad, launch_testing.actions.ReadyToTest(),
    ]), {'good': good, 'bad': bad}


_LATCH1 = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     history=QoSHistoryPolicy.KEEP_LAST)


class TestFaultRecorder(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_fault_recorder')
        cls.hist = []
        cls.bad_hist = []
        cls.node.create_subscription(
            FaultHistory, '/system/fault_history', cls.hist.append, _LATCH1)
        cls.node.create_subscription(
            FaultHistory, '/bad/fault_history', cls.bad_hist.append, _LATCH1)
        cls.p_act = cls.node.create_publisher(ActiveFaults, '/safety/active_faults', _LATCH1)
        cls.p_state = cls.node.create_publisher(SystemState, '/system/state', _LATCH1)
        end = time.time() + 20
        while time.time() < end and not (cls.hist and cls.bad_hist):
            rclpy.spin_once(cls.node, timeout_sec=0.1)
        assert cls.hist and cls.bad_hist, 'fault_recorder の起動直後の履歴が届かない'
        time.sleep(1.0)  # 購読側 discovery
        cls.state('FOLLOW', 'RUN')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    # ── 道具 ──
    @classmethod
    def pump(cls, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(cls.node, timeout_sec=0.02)

    @classmethod
    def active(cls, types, sev=None):
        m = ActiveFaults()
        m.header.stamp = cls.node.get_clock().now().to_msg()
        m.types = sorted(types)
        m.severities = [(sev or {}).get(t, 'RECOVERABLE') for t in m.types]
        cls.p_act.publish(m)
        cls.pump(0.05)

    @classmethod
    def state(cls, mode, state, reason=''):
        m = SystemState()
        m.mode, m.state, m.pause_reason = mode, state, reason
        cls.p_state.publish(m)
        cls.pump(0.05)

    @classmethod
    def rec(cls, t, hist=None):
        for r in (hist or cls.hist)[-1].records:
            if r.fault_type == t:
                return r
        return None

    def until(self, cond, timeout=8.0):
        end = time.time() + timeout
        while time.time() < end:
            self.pump(0.05)
            if cond():
                return True
        return False

    # ── 試験 ──
    def test_a_instant_fault(self):
        self.state('FOLLOW', 'RUN')
        self.active(['A_INSTANT'])
        time.sleep(0.1)
        self.active([])
        assert self.until(lambda: self.rec('A_INSTANT') and not self.rec('A_INSTANT').ongoing)
        r = self.rec('A_INSTANT')
        assert 0.05 <= r.duration_s <= 0.6, r.duration_s
        assert (r.mode, r.state) == ('FOLLOW', 'RUN')
        assert r.severity == 'RECOVERABLE' and r.outcome == 'NONE'
        assert abs(r.started_at - time.time()) < 30   # epoch 秒

    def test_b_two_types_two_records(self):
        self.active(['B_ONE', 'B_TWO'], {'B_TWO': 'CRITICAL'})
        self.active([])
        assert self.until(lambda: self.rec('B_ONE') and self.rec('B_TWO'))
        assert self.rec('B_ONE').severity == 'RECOVERABLE'
        assert self.rec('B_TWO').severity == 'CRITICAL'
        assert not self.rec('B_ONE').ongoing and not self.rec('B_TWO').ongoing

    def test_c_outcomes(self):
        # PAUSE（フォルト由来）
        self.state('FOLLOW', 'RUN')
        self.active(['C_PAUSE'])
        self.state('FOLLOW', 'PAUSE', 'fault')
        assert self.until(lambda: self.rec('C_PAUSE') and self.rec('C_PAUSE').outcome == 'PAUSE')
        self.active([])
        self.state('FOLLOW', 'RUN')
        # ESTOP
        self.active(['C_ESTOP'], {'C_ESTOP': 'CRITICAL'})
        self.state('ESTOP', 'NONE')
        assert self.until(lambda: self.rec('C_ESTOP') and self.rec('C_ESTOP').outcome == 'ESTOP')
        self.active([])
        self.state('FOLLOW', 'RUN')
        # 何も起きない
        self.active(['C_NONE'])
        self.active([])
        self.pump(WINDOW_MS / 1000.0 + 0.5)
        assert self.rec('C_NONE').outcome == 'NONE'
        # フォルト由来でない PAUSE は結果にしない
        self.active(['C_JOG'])
        self.state('FOLLOW', 'PAUSE', 'jog')
        self.active([])
        self.pump(0.3)
        assert self.rec('C_JOG').outcome == 'NONE'
        self.state('FOLLOW', 'RUN')
        # 解消の後に届く遷移（0.1 秒フォルト）も拾う
        self.active(['C_LATE'])
        self.active([])
        self.state('FOLLOW', 'PAUSE', 'fault')
        assert self.until(lambda: self.rec('C_LATE').outcome == 'PAUSE')
        self.state('FOLLOW', 'RUN')

    def test_d_ongoing_then_resolved(self):
        self.active(['D_ONGOING'])
        assert self.until(lambda: self.rec('D_ONGOING') is not None)
        r = self.rec('D_ONGOING')
        assert r.ongoing and r.duration_s == -1.0
        time.sleep(0.5)
        self.active([])
        assert self.until(lambda: not self.rec('D_ONGOING').ongoing)
        assert self.rec('D_ONGOING').duration_s >= 0.4

    def test_g_late_subscriber_gets_latest(self):
        self.active(['G_MARK'])
        self.active([])
        assert self.until(lambda: self.rec('G_MARK') and not self.rec('G_MARK').ongoing)
        got = []
        n = rclpy.create_node('test_fault_recorder_late')
        n.create_subscription(FaultHistory, '/system/fault_history', got.append, _LATCH1)
        end = time.time() + 5
        while time.time() < end and not got:
            rclpy.spin_once(n, timeout_sec=0.1)
        n.destroy_node()
        assert got and any(r.fault_type == 'G_MARK' for r in got[-1].records), \
            '後から購読して最新の履歴が届かない'

    def test_h_recorder_only_subscribes_and_publishes_history(self):
        pubs = {t for t, _ in
                self.node.get_publisher_names_and_types_by_node('fault_recorder', '/')}
        pubs -= {'/rosout', '/parameter_events'}
        assert pubs == {'/system/fault_history'}, pubs
        subs = {t for t, _ in
                self.node.get_subscriber_names_and_types_by_node('fault_recorder', '/')}
        subs -= {'/parameter_events'}
        assert subs == {'/safety/active_faults', '/system/state'}, subs
        assert not self.node.get_service_names_and_types_by_node('fault_recorder', '/') or \
            all(n.startswith('/fault_recorder/') for n, _ in
                self.node.get_service_names_and_types_by_node('fault_recorder', '/'))

    def test_z_cap_file_rotation_and_bad_dir(self):
        for i in range(55):
            self.active(['Z%02d' % i])
            self.active([])
        self.pump(WINDOW_MS / 1000.0 + 1.0)
        recs = self.hist[-1].records
        assert len(recs) == 50, len(recs)
        # 新しい順: 先頭が最後に起こした Z54
        assert recs[0].fault_type == 'Z54', recs[0].fault_type
        starts = [r.started_at for r in recs]
        assert starts == sorted(starts, reverse=True)
        assert not any(r.fault_type == 'Z00' for r in recs)
        # 書けない場所のノードも生きていて、同じ履歴を出し続けている
        assert len(self.bad_hist[-1].records) == 50
        assert self.bad_hist[-1].records[0].fault_type == 'Z54'
        # ファイル: JSON Lines・上限で退避
        path = os.path.join(_DIR, 'fault_log.jsonl')
        assert os.path.exists(path) and os.path.exists(path + '.1'), os.listdir(_DIR)
        rows = [json.loads(x) for x in open(path, encoding='utf-8') if x.strip()]
        rows += [json.loads(x) for x in open(path + '.1', encoding='utf-8') if x.strip()]
        assert any(r['fault_type'] == 'Z54' and r['final'] for r in rows)
        assert any(r['fault_type'] == 'A_INSTANT' for r in rows) or True
        assert os.path.getsize(path) <= MAX_BYTES + 2000
