"""SM-3.1.1-19: 実 state_manager で運用入口の回復フォルト拒否を確かめる。

Spec-modes.md §3.1.1-19（2026-11-10 実機試験 T-12）。走行を止める回復フォルト
（LIDAR_LOST・ESP32_DISCONNECTED・必須通信系統の断）が残っている間は、IDLE／MANUAL
からのモード選択と IDLE／AT_HOME からの行き先選択を拒否し、理由と残存フォルト名を
返して W-1 を出す。PERSON_TRACKER_LOST だけなら通す。開発モードで外した項目
（/system/dev_mode の effective の lidar_fault）は数えない。

購読（/safety/active_faults）→ ガード → 拒否理由・W-1 の配線を、本物の
state_manager を launch して縛る（純関数の試験では呼び出し側の数行が抜ける）。
"""
import json
import os
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
from std_msgs.msg import Bool, String
from th_system_msgs.msg import ActiveFaults, FaultStatus, StateEffect, SystemState
from th_system_msgs.srv import UiTrigger

_WS_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))


def _generated_yaml(node: str) -> str:
    # 本番の bringup と同じ生成 yaml を先に読ませる（CLAUDE.md「rclpy の型」）。
    for d in (os.environ.get('TH_GENERATED_DIR', ''), '/root/th_data/generated',
              os.path.join(_WS_ROOT, 'data', 'generated')):
        path = os.path.join(d, f'{node}.yaml') if d else ''
        if path and os.path.exists(path):
            return path
    raise FileNotFoundError(f'生成 yaml {node}.yaml が見つからない')


@pytest.mark.launch_test
def generate_test_description():
    node = launch_ros.actions.Node(
        package='th_state', executable='state_manager.py', name='state_manager',
        parameters=[_generated_yaml('state_manager')],
        output='screen')
    return launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()]), {
        'state_manager': node,
    }


_LATCHED = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                      durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                      history=QoSHistoryPolicy.KEEP_LAST)


class TestRecoverableFaultEntryGuard(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('recoverable_fault_entry_client')
        self.states = []
        self.effects = []
        self.node.create_subscription(SystemState, '/system/state', self.states.append, _LATCHED)
        self.node.create_subscription(StateEffect, '/system/effect', self.effects.append, 10)
        self.pub_faults = self.node.create_publisher(ActiveFaults, '/safety/active_faults', _LATCHED)
        self.pub_fault = self.node.create_publisher(FaultStatus, '/safety/fault', 5)
        self.pub_hw = self.node.create_publisher(Bool, '/safety/estop_hw', 10)
        # state_manager は transient_local で購読するので、合わせないと届かない。
        self.pub_dev = self.node.create_publisher(String, '/system/dev_mode', _LATCHED)
        self.cli = self.node.create_client(UiTrigger, '/system/trigger')
        assert self.cli.wait_for_service(timeout_sec=5.0), 'state_manager が起動していない'
        self.pub_fault.publish(FaultStatus(active=False, fault_type='NONE', severity=''))
        self.pub_hw.publish(Bool(data=False))
        self._spin(0.3)
        # 前の試験がどのモードで終わっても IDLE から始める（INIT からも ui.finish で入れる）。
        # 既に IDLE なら ui.finish は拒否されるので送らない。
        if not self._wait_state('IDLE', 'NONE', timeout=0.5):
            res = self._trigger('ui.finish')
            assert res.accepted, res.reject_reason_key
            assert self._wait_state('IDLE', 'NONE'), (res.mode, res.state)

    def tearDown(self):
        self.node.destroy_node()

    def _spin(self, duration=0.2):
        end = time.time() + duration
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _wait_state(self, mode, state, timeout=5.0):
        end = time.time() + timeout
        while time.time() < end:
            self._spin(0.1)
            if self.states and (self.states[-1].mode, self.states[-1].state) == (mode, state):
                return True
        return False

    def _wait_mode(self, mode, timeout=5.0):
        end = time.time() + timeout
        while time.time() < end:
            self._spin(0.1)
            if self.states and self.states[-1].mode == mode:
                return True
        return False

    def _trigger(self, trigger, arg=None):
        req = UiTrigger.Request()
        req.trigger = trigger
        req.arg_json = json.dumps(arg or {})
        req.requester = 'test'
        future = self.cli.call_async(req)
        deadline = time.time() + 3.0
        while not future.done() and time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        assert future.done(), f'{trigger} の応答がない'
        return future.result()

    def _active(self, types, severities=None):
        severities = severities or ['RECOVERABLE'] * len(types)
        self.pub_faults.publish(ActiveFaults(types=types, severities=severities))
        self._spin(0.3)

    def test_fault_blocks_mode_entry_and_reports_names(self):
        self._active(['ESP32_DISCONNECTED', 'LIDAR_LOST'])
        self.effects.clear()
        res = self._trigger('ui.enter_mode', {'mode': 'FOLLOW'})
        assert not res.accepted, '回復フォルトが残っているのに FOLLOW に入れた'
        assert res.reject_reason_key == 'recoverable_fault_active', res.reject_reason_key
        assert (res.mode, res.state) == ('IDLE', 'NONE'), (res.mode, res.state)
        self._spin(0.3)
        assert self.states[-1].last_reject_reason == 'recoverable_fault_active', \
            '拒否理由が /system/state.last_reject_reason に載っていない'

        # W-1 を開き、残っているフォルト名を添える（画面側の文言は別の作業）。
        windows = [e for e in self.effects if e.name == 'open_window']
        assert windows, '拒否時に W-1 が開かれない'
        args = json.loads(windows[-1].args_json)
        assert args['id'] == 'W-1', args
        assert args['fault_types'] == ['ESP32_DISCONNECTED', 'LIDAR_LOST'], args

    def test_tracker_lost_alone_and_ignored_lidar_do_not_block(self):
        self._active(['PERSON_TRACKER_LOST'])
        res = self._trigger('ui.enter_mode', {'mode': 'FOLLOW'})
        assert res.accepted, res.reject_reason_key
        assert self._wait_mode('FOLLOW')

        assert self._trigger('ui.finish').accepted
        assert self._wait_state('IDLE', 'NONE')

        # 開発モードで lidar_fault を外していれば LIDAR_LOST は数えない（Spec-safety §10）。
        self.pub_dev.publish(String(data=json.dumps({
            'dev_mode': True, 'ignore': ['lidar_fault'], 'effective': {'lidar_fault': True}})))
        self._spin(0.3)
        self._active(['LIDAR_LOST'])
        res = self._trigger('ui.enter_mode', {'mode': 'MANUAL'})
        assert res.accepted, res.reject_reason_key
        assert self._wait_mode('MANUAL')

    def test_carry_resume_keeps_fault_but_entry_stays_blocked_until_cleared(self):
        self._active(['ESP32_DISCONNECTED'])
        self.pub_fault.publish(FaultStatus(active=True, fault_type='ESP32_DISCONNECTED',
                                           severity='RECOVERABLE'))
        self._spin(0.2)
        # IDLE で物理非常停止 → CARRY → 解除 → 再開（SM-3.1.1-14）。
        self.pub_hw.publish(Bool(data=True))
        assert self._wait_state('CARRY', 'NONE')
        self.pub_hw.publish(Bool(data=False))
        self._spin(0.3)
        res = self._trigger('ui.carry_resume')
        assert res.accepted, res.reject_reason_key
        assert (res.mode, res.state) == ('IDLE', 'NONE'), (res.mode, res.state)

        # 戻れたこと自体は仕様どおり。入口はなお塞がっている。
        res = self._trigger('ui.enter_mode', {'mode': 'MANUAL'})
        assert not res.accepted
        assert res.reject_reason_key == 'recoverable_fault_active'

        self.pub_fault.publish(FaultStatus(active=False, fault_type='ESP32_DISCONNECTED',
                                           severity='RECOVERABLE'))
        self._active([])
        res = self._trigger('ui.enter_mode', {'mode': 'MANUAL'})
        assert res.accepted, res.reject_reason_key
        assert self._wait_mode('MANUAL')

    def test_fault_status_clear_alone_unblocks_entry(self):
        """全解消の active=false が先に届いても入口は空く。

        /safety/active_faults と /safety/fault は別トピックで到着順が保証されない。
        """
        self._active(['LIDAR_LOST'])
        res = self._trigger('ui.enter_mode', {'mode': 'MANUAL'})
        assert not res.accepted, res.reject_reason_key
        self.pub_fault.publish(FaultStatus(active=False, fault_type='NONE',
                                           severity='NONE'))
        self._spin(0.3)
        res = self._trigger('ui.enter_mode', {'mode': 'MANUAL'})
        assert res.accepted, res.reject_reason_key
        assert self._wait_mode('MANUAL')

    def test_destination_choice_from_at_home_is_blocked_too(self):
        res = self._trigger('ui.enter_mode', {'mode': 'AT_HOME'})
        assert res.accepted, res.reject_reason_key
        assert self._wait_state('AT_HOME', 'IDLE_H')

        self._active(['LIDAR_LOST'])
        before = (self.states[-1].mode, self.states[-1].state)
        res = self._trigger('ui.goto', {'kind': 'HOME'})
        assert not res.accepted, '回復フォルトが残っているのに行き先を選べた'
        assert res.reject_reason_key == 'recoverable_fault_active', res.reject_reason_key
        self._spin(0.2)
        assert (self.states[-1].mode, self.states[-1].state) == before, \
            f'拒否したのに状態が動いた: {before} -> {(self.states[-1].mode, self.states[-1].state)}'

        self._active([])
        res = self._trigger('ui.goto', {'kind': 'HOME'})
        assert res.accepted, res.reject_reason_key
        assert self._wait_state('HOME_NAV', 'NAV')
