"""
test_working_guard_node.py
============================
1b-9（SG-A9）— 「作業中」をサーバ側（state_manager）で守る。

以前は `guards._goto_allowed` が `ctx.flags["working"]` を見ていたが、そのフラグを立てる者が
いなかった（守っていたのは画面のボタン無効化だけ）。いまは「作業中」の正本を機体の状態
（`AT_PANEL` × `WORKING`）にし、行き先（PANEL／HOME／SUMMON）を **WebUI を介さずに**
`/system/trigger` へ直接投げても拒否される（理由 `working_in_progress`）ことを、
state_manager を実際に起動して縛る。作業終了（`ui.working` をもう一度）後は通る。
"""
import json
import time
import unittest

import pytest
import rclpy

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions

from geometry_msgs.msg import Pose
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                        QoSReliabilityPolicy)
from std_msgs.msg import Bool
from th_system_msgs.msg import FaultStatus, Pin, PinList, StateEvent, SystemState
from th_system_msgs.srv import SetFlag, UiTrigger


@pytest.mark.launch_test
def generate_test_description():
    state_manager = launch_ros.actions.Node(
        package='th_state',
        executable='state_manager.py',
        name='state_manager',
        parameters=[{
            'jog_lease_ms': 300,
            'link_wait_timeout_ms': 60_000,
            'ui_active_window_s': 5,
            'screen_stale_ms': 60_000,
        }],
        output='screen',
    )
    return launch.LaunchDescription([
        state_manager,
        launch_testing.actions.ReadyToTest(),
    ]), {'state_manager': state_manager}


_LATCHED = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST)


class TestWorkingGuardNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_working_guard_client')
        self._hist = []
        self.node.create_subscription(SystemState, '/system/state', self._hist.append, _LATCHED)
        self.pub_event = self.node.create_publisher(StateEvent, '/system/event', 10)
        self.pub_pins = self.node.create_publisher(PinList, '/onsite/pins', _LATCHED)
        self.pub_fault = self.node.create_publisher(FaultStatus, '/safety/fault', 5)
        self.pub_hw = self.node.create_publisher(Bool, '/safety/estop_hw', 10)
        self.cli_trigger = self.node.create_client(UiTrigger, '/system/trigger')
        self.cli_set_flag = self.node.create_client(SetFlag, '/system/set_flag')
        assert self.cli_trigger.wait_for_service(timeout_sec=5.0), 'state_manager が起動していない'
        self.pub_fault.publish(FaultStatus(active=False, fault_type='', severity=''))
        self.pub_hw.publish(Bool(data=False))
        self._spin(0.3)
        self._trigger('ui.finish')
        self._wait_mode('IDLE')

    def tearDown(self):
        self.node.destroy_node()

    # ── ヘルパー ──
    def _spin(self, duration=0.2):
        end = time.time() + duration
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _trigger(self, trigger, arg=None):
        req = UiTrigger.Request()
        req.trigger = trigger
        req.arg_json = json.dumps(arg) if arg else ''
        req.requester = 'test'
        fut = self.cli_trigger.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=3.0)
        return fut.result()

    def _latest(self):
        self._spin(0.2)
        assert self._hist, '/system/state を受信していない'
        return self._hist[-1]

    def _wait_mode(self, mode, state=None, timeout=3.0):
        end = time.time() + timeout
        while time.time() < end:
            self._spin(0.1)
            if self._hist and self._hist[-1].mode == mode and (
                    state is None or self._hist[-1].state == state):
                return True
        return False

    def _event(self, name):
        self.pub_event.publish(StateEvent(event=name, source_node='test', arg_json=''))

    def _go_at_panel(self):
        """IDLE → HOME_NAV → AT_HOME → PANEL_NAV → AT_PANEL / IDLE_P。"""
        res = self._trigger('ui.goto', {'kind': 'HOME'})
        assert res.accepted, res.reject_reason_key
        assert self._wait_mode('HOME_NAV')
        self._event('evt.arrived')
        assert self._wait_mode('AT_HOME')
        pin = Pin(id='p1', name='panel', kind='PANEL', pose=Pose())
        self.pub_pins.publish(PinList(pins=[pin]))
        self._spin(0.4)
        res = self._trigger('ui.goto', {'kind': 'PANEL', 'pin_id': 'p1'})
        assert res.accepted, res.reject_reason_key
        assert self._wait_mode('PANEL_NAV')
        self._event('evt.arrived')
        assert self._wait_mode('PANEL_NAV', 'ALIGN')
        self._event('evt.align_done')
        assert self._wait_mode('AT_PANEL', 'IDLE_P')

    # ── 試験 ──
    def test_goto_rejected_while_working_and_allowed_after(self):
        self._go_at_panel()
        assert self._latest().working is False

        res = self._trigger('ui.working')
        assert res.accepted, res.reject_reason_key
        assert self._wait_mode('AT_PANEL', 'WORKING')
        assert self._latest().working is True, '作業中が /system/state に出ていない'

        for kind in ('PANEL', 'HOME', 'SUMMON'):
            arg = {'kind': kind}
            if kind == 'PANEL':
                arg['pin_id'] = 'p1'
            res = self._trigger('ui.goto', arg)
            assert not res.accepted, f'作業中なのに ui.goto {kind} が通った'
            assert res.reject_reason_key == 'working_in_progress', (
                kind, res.reject_reason_key)
            snap = self._latest()
            assert (snap.mode, snap.state) == ('AT_PANEL', 'WORKING'), (
                f'拒否したのに状態が動いた: {snap.mode}/{snap.state}')

        # 作業終了 → 行き先を選べる
        res = self._trigger('ui.working')
        assert res.accepted
        assert self._wait_mode('AT_PANEL', 'IDLE_P')
        assert self._latest().working is False
        res = self._trigger('ui.goto', {'kind': 'HOME'})
        assert res.accepted, res.reject_reason_key
        assert self._wait_mode('HOME_NAV')

    def test_set_flag_working_is_not_a_back_door(self):
        """working は状態の写しなので /system/set_flag では動かせない。"""
        req = SetFlag.Request()
        req.flag = 'working'
        req.value = True
        fut = self.cli_set_flag.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=3.0)
        assert fut.result().accepted is False
        assert self._latest().working is False
