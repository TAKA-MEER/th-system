"""
test_person_tracker_lost_gate_node.py
========================================
DetailedDesign-safety.md §5.5（P-01＋P-02）の Docker launch テスト。

safety_monitor ノード本体を `person` ターゲットだけで 2 台起動し、
`/system/state`（transient_local）と `/person/targets` を自分で出して
次を確認する（お手本 test_dev_mode_safety_node.py・test_localization_lost.py）:
  (a) `tracker_enabled=false` のまま `person_timeout_ms` を超えて待っても
      `/safety/fault` に PERSON_TRACKER_LOST が出ない
  (b) `person_report_only=false`・ON・猶予を短く上書きして
      `/person/targets` を止めると出る
  (c) 同じ条件で `person_report_only=true` なら出ない
  (d) 出ている状態で OFF にすると解除される

2 台立てる理由: `person_report_only` は起動時パラメータのため、
1 台では (b) と (c) を両方縛れない。A（report_only=false）と
B（report_only=true）に fault 系トピックを付け替え、同じ入力の下で
「A だけ出て B は出ない」ことを見る。(a) だけ・(b) だけでは沈黙で
通るので (a)〜(d) 全部が必要（§5.5.7 と同じ理由）。

注意（CLAUDE.md の既知の癖）: launch_testing の pytest プラグインが
ファイル全体を1つのアイテムとして収集するため、試験は TestCase の
メソッドにする。メソッドは名前順に走る。
"""
import time
import unittest

import pytest
import rclpy
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions

from th_system_msgs.msg import FaultStatus, PersonTargets, SystemState


CHECK_PERIOD_MS = 50
STARTUP_GRACE_SEC = 1
STARTUP_DEADLINE_SEC = 1
PERSON_TIMEOUT_MS = 400
STARTUP_GRACE_MS = 300


def _safety_node(name: str, report_only: bool, fault_ns: str):
    return launch_ros.actions.Node(
        package='th_safety',
        executable='safety_monitor',
        name=name,
        parameters=[{
            'check_period_ms': CHECK_PERIOD_MS,
            'startup_grace_sec': STARTUP_GRACE_SEC,
            'startup_deadline_sec': STARTUP_DEADLINE_SEC,
            'person_timeout_ms': PERSON_TIMEOUT_MS,
            'person_startup_grace_ms': STARTUP_GRACE_MS,
            'person_report_only': report_only,
            # このファイルが検証する対象だけを有効化する（F-5・O-7）。
            'enabled_targets': ['person'],
        }],
        remappings=[('/safety/fault', fault_ns + '/fault'),
                    ('/safety/fault_lock', fault_ns + '/fault_lock'),
                    ('/safety/estop', fault_ns + '/estop')],
        output='screen',
    )


@pytest.mark.launch_test
def generate_test_description():
    # A: 記録だけ OFF（本番相当）。B: 記録だけ ON（既定）。
    node_a = _safety_node('safety_person_a', False, '/test/person_a')
    node_b = _safety_node('safety_person_b', True, '/test/person_b')
    return launch.LaunchDescription([
        node_a, node_b,
        launch_testing.actions.ReadyToTest(),
    ]), {'safety_person_a': node_a, 'safety_person_b': node_b}


class TestPersonTrackerLostGate(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_person_tracker_lost_gate')
        n = cls.node
        cls.faults_a = []
        cls.faults_b = []
        n.create_subscription(FaultStatus, '/test/person_a/fault',
                              cls.faults_a.append, 10)
        n.create_subscription(FaultStatus, '/test/person_b/fault',
                              cls.faults_b.append, 10)

        cls.pub_targets = n.create_publisher(PersonTargets, '/person/targets', 10)
        latched = QoSProfile(depth=1,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        cls.pub_state = n.create_publisher(SystemState, '/system/state', latched)

        cls.tracker_enabled = False  # (a) は OFF のまま
        cls.targets_enabled = False  # (a) は targets 無し（OFF で抑止される証明）
        n.create_timer(0.2, cls._publish_state)
        n.create_timer(0.2, cls._publish_targets)

        # 起動猶予＋余裕。この spin で溜まった edge をバッファへ流し込む。
        cls._spin(STARTUP_GRACE_SEC + 0.5)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    # ── 送信 ──────────────────────────────────────────────
    @classmethod
    def _publish_state(cls):
        st = SystemState()
        st.mode = 'IDLE'
        st.state = 'NONE'
        st.zone = 'OUT'
        st.tracker_enabled = cls.tracker_enabled
        cls.pub_state.publish(st)

    @classmethod
    def _publish_targets(cls):
        if not cls.targets_enabled:
            return
        msg = PersonTargets()
        msg.header.stamp = cls.node.get_clock().now().to_msg()
        cls.pub_targets.publish(msg)

    # ── 待ち ──────────────────────────────────────────────
    @classmethod
    def _spin(cls, duration: float):
        deadline = time.time() + duration
        while time.time() < deadline:
            rclpy.spin_once(cls.node, timeout_sec=0.05)

    def _wait(self, pred, timeout: float) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if pred():
                return True
        return False

    @staticmethod
    def _is_active(faults, fault_type: str) -> bool:
        """`/safety/fault` は変化時だけ出るので、最後の edge を見る。"""
        last = None
        for f in faults:
            if f.fault_type == fault_type and f.active:
                last = True
            elif not f.active and last:
                last = False
        return bool(last)

    # ════════════════════════════════════════════════════════
    def test_a_off_stays_silent(self):
        """(a) OFF のまま timeout を超えて待っても出ない（両方）。"""
        self._spin(1.5)
        assert not self._is_active(type(self).faults_a, 'PERSON_TRACKER_LOST'), (
            'OFF なのに PERSON_TRACKER_LOST が出た（report_only=false 側）')
        assert not self._is_active(type(self).faults_b, 'PERSON_TRACKER_LOST'), (
            'OFF なのに PERSON_TRACKER_LOST が出た（report_only=true 側）')

    def test_b_on_timeout_fires_when_reporting_off(self):
        """(b) ON・猶予後・targets 停止で report_only=false 側に出る。"""
        type(self).targets_enabled = True
        type(self).tracker_enabled = True
        # ON エッジ→猶予（300ms）＋ targets 受信を待つ。
        self._spin(1.0)
        type(self).targets_enabled = False
        assert self._wait(
            lambda: self._is_active(type(self).faults_a, 'PERSON_TRACKER_LOST'),
            5.0), 'ON で targets を止めても PERSON_TRACKER_LOST が出ない'
        hit = [f for f in type(self).faults_a
               if f.active and f.fault_type == 'PERSON_TRACKER_LOST']
        assert hit and hit[-1].severity == 'RECOVERABLE', (
            'PERSON_TRACKER_LOST が RECOVERABLE でない')

    def test_c_report_only_stays_silent(self):
        """(c) 同じ条件で report_only=true 側は出ない。A は出たまま。"""
        self._spin(1.0)
        assert not self._is_active(type(self).faults_b, 'PERSON_TRACKER_LOST'), (
            'report_only=true なのに PERSON_TRACKER_LOST が出た')
        # 対照: A 側は出続けている（沈黙で通っていないことの証明）。
        assert self._is_active(type(self).faults_a, 'PERSON_TRACKER_LOST'), (
            'A 側の PERSON_TRACKER_LOST が消えた（入力が変わった？）')

    def test_d_off_clears_fault(self):
        """(d) 出ている状態で OFF にすると解除される。"""
        assert self._is_active(type(self).faults_a, 'PERSON_TRACKER_LOST'), (
            '前提が崩れた：A 側に PERSON_TRACKER_LOST が出ていない')
        type(self).tracker_enabled = False
        assert self._wait(
            lambda: not self._is_active(type(self).faults_a,
                                        'PERSON_TRACKER_LOST'),
            5.0), 'OFF にしても PERSON_TRACKER_LOST が解除されない'


if __name__ == '__main__':
    unittest.main()
