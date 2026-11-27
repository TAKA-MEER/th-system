"""
test_auto_brake_prep_node.py
============================
R-3（2026-11-27 D73・D75）: 自動ブレーキの既定と試験準備の切替を
state_manager を実際に起動して縛る launch_testing。

  (1) 場外の手動系（MANUAL）の既定は ON（D75。従来は OFF）。
  (2) 試験準備（PREP）は W-6 の心拍（/ui/manual_panel）が無くても OFF にできる。
      ただし PREP の状態が変わったら ON に戻る（D73）。
  (3) 使用中の端末の心拍（/ui/active_screen の在席）が途絶えたら ON に戻り、
      在席が戻っても前の OFF 要求は復活しない（D73）。
  (4) 試験（当日）（PANEL_NAV）は従来どおり W-6 の心拍が無いと OFF にできない。

この試験は /ui/manual_panel を一切送らない（panel_open は偽のまま）。
注意: TestCase のメソッドは名前順に走り状態を引き継ぐので、各メソッドは
冒頭で前提を作る。
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

from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                        QoSReliabilityPolicy)

from std_msgs.msg import String
from th_system_msgs.msg import ActiveScreen, SystemState
from th_system_msgs.srv import SetFlag, UiTrigger

JOG_LEASE_MS = 300
LINK_WAIT_TIMEOUT_MS = 60_000
UI_ACTIVE_WINDOW_S = 5
SCREEN_STALE_MS = 60_000


@pytest.mark.launch_test
def generate_test_description():
    state_manager = launch_ros.actions.Node(
        package='th_state',
        executable='state_manager.py',
        name='state_manager',
        parameters=[{
            'jog_lease_ms': JOG_LEASE_MS,
            'link_wait_timeout_ms': LINK_WAIT_TIMEOUT_MS,
            'ui_active_window_s': UI_ACTIVE_WINDOW_S,
            'screen_stale_ms': SCREEN_STALE_MS,
        }],
        output='screen',
    )
    return launch.LaunchDescription([
        state_manager,
        launch_testing.actions.ReadyToTest(),
    ]), {'state_manager': state_manager}


_STATE_QOS = QoSProfile(
    depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST)


class TestAutoBrakePrep(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_auto_brake_prep_client')
        self.history = []
        self.node.create_subscription(
            SystemState, '/system/state', self.history.append, _STATE_QOS)
        self.pub_screen = self.node.create_publisher(ActiveScreen, '/ui/active_screen', 5)
        # W-6 の心拍は出さない（この試験の前提）。
        self.cli_trigger = self.node.create_client(UiTrigger, '/system/trigger')
        self.cli_set_flag = self.node.create_client(SetFlag, '/system/set_flag')
        assert self.cli_trigger.wait_for_service(timeout_sec=10.0), \
            'state_manager が起動していない'
        self.screen_id = 'S-20'
        self.interacting = True
        self.node.create_timer(0.25, self._publish_screen)
        self._to_idle()

    def tearDown(self):
        self.interacting = True
        self._to_idle()
        self.node.destroy_node()

    # ── ヘルパー ──────────────────────────────────────────
    def _publish_screen(self):
        self.pub_screen.publish(ActiveScreen(
            screen_id=self.screen_id, client_id='tablet-ab-test',
            interacting=self.interacting,
            last_input=self.node.get_clock().now().to_msg()))

    def _spin(self, duration=0.3):
        deadline = time.time() + duration
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _trigger(self, trigger, arg=None):
        req = UiTrigger.Request()
        req.trigger = trigger
        req.arg_json = json.dumps(arg) if arg else ''
        req.requester = 'test'
        fut = self.cli_trigger.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=3.0)
        return fut.result()

    def _set_auto_brake(self, value):
        req = SetFlag.Request()
        req.flag = 'auto_brake'
        req.value = value
        fut = self.cli_set_flag.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=3.0)
        return fut.result()

    def _latest(self):
        self._spin(0.3)
        assert self.history, '/system/state を受信していない'
        return self.history[-1]

    def _wait(self, pred, timeout=3.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._spin(0.1)
            if self.history and pred(self.history[-1]):
                return True
        return False

    def _to_idle(self):
        self._trigger('ui.finish')
        self._wait(lambda s: s.mode == 'IDLE', 3.0)

    def _enter(self, mode):
        self._spin(0.6)  # 画面の申告が届いてゾーンが決まるのを待つ
        res = self._trigger('ui.enter_mode', {'mode': mode})
        assert res.accepted, res.reject_reason_key
        assert self._wait(lambda s: s.mode == mode)

    # ── 試験 ──────────────────────────────────────────────
    def test_1_outdoor_manual_default_on(self):
        self.screen_id = 'S-11'   # 場外
        self._enter('MANUAL')
        assert self._wait(lambda s: s.zone == 'OUT', 3.0)
        self._spin(0.5)
        assert self._latest().auto_brake is True, '場外の手動の既定が ON でない（D75）'

    def test_2_prep_off_without_panel_then_state_change_returns_on(self):
        self._enter('PREP')
        assert self._wait(lambda s: (s.mode, s.state) == ('PREP', 'MAPPING')
                          and s.zone == 'IN', 3.0)
        res = self._set_auto_brake(False)
        assert res.accepted, res.reject_reason_key
        assert self._wait(lambda s: s.auto_brake is False), \
            'PREP で心拍なしに OFF にできない（D73）'
        # PREP の状態が変わったら ON に戻る（MAPPING → EDIT）。
        self._trigger('ui.map_edit')
        assert self._wait(lambda s: s.state == 'EDIT', 3.0)
        self._spin(0.4)
        assert self._latest().auto_brake is True, 'PREP の状態が変わっても OFF のまま'

    def test_3_presence_loss_returns_on_and_does_not_revive(self):
        self._enter('PREP')
        assert self._wait(lambda s: (s.mode, s.state) == ('PREP', 'MAPPING')
                          and s.zone == 'IN', 3.0)
        assert self._set_auto_brake(False).accepted
        assert self._wait(lambda s: s.auto_brake is False)
        self.interacting = False   # 使用中の端末の心拍が途絶えた
        assert self._wait(lambda s: s.auto_brake is True, 3.0), \
            '端末の心拍が途絶えても OFF のまま'
        self.interacting = True    # 在席が戻っても前の要求は復活しない
        self._spin(0.8)
        s = self._latest()
        assert s.auto_brake is True, '在席に戻ったら前の OFF 要求が復活した（D73）'

    def test_4_exam_day_still_needs_panel_heartbeat(self):
        self.screen_id = 'S-21'   # 試験（当日）
        self._enter('PANEL_NAV')
        assert self._wait(lambda s: s.zone == 'IN', 3.0)
        res = self._set_auto_brake(False)
        assert not res.accepted, 'W-6 の心拍なしに試験（当日）で OFF にできた'
        assert res.reject_reason_key == 'auto_brake_locked', res.reject_reason_key
        assert self._latest().auto_brake is True


if __name__ == '__main__':
    unittest.main()
