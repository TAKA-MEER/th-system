"""
test_auto_brake_state_node.py
=============================
1b-15 SG-B10: 自動ブレーキの正本（state_manager の /system/state.auto_brake と
/system/set_flag auto_brake）を、ノード本体を実際に起動して縛る launch_testing。

純関数（test_auto_brake.py）が固くても、モード遷移でのリセット・set_flag の受理条件・
ゾーン途絶時の ON 化は state_manager の呼び出し側の数行で壊れうる。

  a. 手動走行（MANUAL）・場外（S-11）→ 既定 OFF
  b. 手動走行・場内の画面（S-21）→ 既定 ON
  c. 切替要求: 場外で ON にできる／場内の画面でも要求は通る
  d. 手動走行から出て入り直すと既定へ戻る（要求を引きずらない）
  e. 自律系・待機（IDLE）では切替を拒否（auto_brake_locked）し、ON のまま
  f. 画面の申告が途絶（interacting=false）→ ゾーン NA で ON（安全側）。戻ると既定へ
  g. 場内で OFF を要求 → 画面途絶で ON → 復帰すると要求どおり OFF
  h. 手動走行を出た先の自律系に OFF の要求が残らない
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


class TestAutoBrakeState(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_auto_brake_state_client')
        self.history = []
        self.node.create_subscription(
            SystemState, '/system/state', self.history.append, _STATE_QOS)
        self.pub_screen = self.node.create_publisher(ActiveScreen, '/ui/active_screen', 5)
        self.cli_trigger = self.node.create_client(UiTrigger, '/system/trigger')
        self.cli_set_flag = self.node.create_client(SetFlag, '/system/set_flag')
        assert self.cli_trigger.wait_for_service(timeout_sec=10.0), \
            'state_manager が起動していない'
        self.screen_id = 'S-11'
        self.interacting = True
        self.node.create_timer(0.25, self._publish_screen)
        self._to_idle()

    def tearDown(self):
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

    def _enter_manual(self):
        self._spin(0.6)  # 画面の申告が届いてゾーンが決まるのを待つ
        res = self._trigger('ui.enter_mode', {'mode': 'MANUAL'})
        assert res.accepted, res.reject_reason_key
        assert self._wait(lambda s: s.mode == 'MANUAL')

    # ── 試験 ──────────────────────────────────────────────
    def test_a_manual_out_default_off(self):
        self._enter_manual()
        s = self._latest()
        assert s.zone == 'OUT', s.zone
        assert s.auto_brake is False, '場外の手動走行は既定 OFF のはず'

    def test_b_manual_in_default_on(self):
        self.screen_id = 'S-21'
        self._enter_manual()
        s = self._latest()
        assert s.zone == 'IN', s.zone
        assert s.auto_brake is True, '場内の手動は既定 ON のはず'

    def test_c_toggle_on_in_out_zone(self):
        self._enter_manual()
        assert self._latest().auto_brake is False
        res = self._set_auto_brake(True)
        assert res.accepted, res.reject_reason_key
        assert self._wait(lambda s: s.auto_brake is True), '場外で ON にできない'
        res = self._set_auto_brake(False)
        assert res.accepted
        assert self._wait(lambda s: s.auto_brake is False), 'ON から OFF に戻せない'

    def test_d_reentering_manual_restores_default(self):
        self._enter_manual()
        assert self._set_auto_brake(True).accepted
        assert self._wait(lambda s: s.auto_brake is True)
        self._to_idle()
        self._enter_manual()
        assert self._latest().auto_brake is False, '入り直しても前回の要求を引きずった'

    def test_e_locked_mode_rejects_toggle(self):
        s = self._latest()
        assert s.mode == 'IDLE'
        res = self._set_auto_brake(False)
        assert not res.accepted
        assert res.reject_reason_key == 'auto_brake_locked', res.reject_reason_key
        assert self._latest().auto_brake is True, '自律系・待機で OFF になった'

    def test_f_presence_loss_forces_on(self):
        self._enter_manual()
        assert self._latest().auto_brake is False
        self.interacting = False
        assert self._wait(lambda s: s.zone == 'NA', 3.0), '画面の途絶で NA にならない'
        assert self._latest().auto_brake is True, '画面途絶でも OFF のままだった'
        self.interacting = True
        assert self._wait(lambda s: s.zone == 'OUT', 3.0)
        assert self._wait(lambda s: s.auto_brake is False, 3.0), '復帰後に既定へ戻らない'

    def test_g_explicit_off_in_zone_survives_presence_loss(self):
        self.screen_id = 'S-21'
        self._enter_manual()
        assert self._latest().auto_brake is True
        assert self._set_auto_brake(False).accepted
        assert self._wait(lambda s: s.auto_brake is False), '場内で OFF にできない'
        self.interacting = False
        assert self._wait(lambda s: s.zone == 'NA')
        assert self._latest().auto_brake is True, '画面途絶中に OFF のままだった'
        self.interacting = True
        assert self._wait(lambda s: s.zone == 'IN')
        assert self._wait(lambda s: s.auto_brake is False), '要求した OFF が復帰後に失われた'

    def test_h_off_request_does_not_leak_into_autonomous_mode(self):
        self._enter_manual()
        assert self._set_auto_brake(False).accepted
        self._to_idle()
        assert self._latest().auto_brake is True, '手動走行を出ても OFF が残った'
        # 拒否される（IDLE）。要求が静かに残っていないことも合わせて確かめる。
        assert not self._set_auto_brake(False).accepted


if __name__ == '__main__':
    unittest.main()
