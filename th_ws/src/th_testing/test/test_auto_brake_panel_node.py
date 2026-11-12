"""
test_auto_brake_panel_node.py
=============================
A-15（2026-11-12 ユーザー決定。Spec-webui §3.5）: W-6（手動操作パネル）を開いている間は、
スティックに触れていなくても自動ブレーキの OFF を受け付けて保つ。W-6 を閉じた・心拍が途絶えた・
モードが変わったら ON に戻す。state_manager を実際に起動して縛る launch_testing。

開いている知らせは /ui/manual_panel（client_id。閉じるときは空文字）。ジョグのリース
（/ui/jog_lease）とは別で、この試験はリースを一切送らない（＝jog_active は偽のまま）。
自律系（IDLE は auto_brake_default: on_locked）で:
  a. 心拍なし → OFF の切替は拒否（auto_brake_locked）。ON のまま（従来どおり）
  b. 心拍あり → OFF を受理し、リース（jog_lease_ms）の何倍待っても保つ。ON に戻せる
  c. 心拍が途絶える → jog_lease_ms を超えたら ON に戻る。以後の OFF 要求は拒否
  d. 閉じる通知（空文字）→ すぐ ON に戻る
  e. 心拍が続いていても、モードが変わると OFF 要求は捨てられる（ON）
  f. 心拍が途絶えたあと再開しても、前の OFF 要求は復活しない
  g. 画面の途絶（ゾーン NA）→ ON（安全側）
注意: TestCase のメソッドは名前順に走り状態を引き継ぐので、各メソッドは冒頭で前提を作る。
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


class TestAutoBrakePanel(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_auto_brake_panel_client')
        self.history = []
        self.node.create_subscription(
            SystemState, '/system/state', self.history.append, _STATE_QOS)
        self.pub_screen = self.node.create_publisher(ActiveScreen, '/ui/active_screen', 5)
        self.pub_panel = self.node.create_publisher(
            String, '/ui/manual_panel',
            QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT))
        self.panel_open = False   # 真の間は 10 Hz で心拍を出す
        self.node.create_timer(0.1, self._publish_panel)
        self.cli_trigger = self.node.create_client(UiTrigger, '/system/trigger')
        self.cli_set_flag = self.node.create_client(SetFlag, '/system/set_flag')
        assert self.cli_trigger.wait_for_service(timeout_sec=10.0), \
            'state_manager が起動していない'
        self.screen_id = 'S-11'
        self.interacting = True
        self.node.create_timer(0.25, self._publish_screen)
        self._to_idle()

    def tearDown(self):
        self.panel_open = False
        self._to_idle()
        self.node.destroy_node()

    # ── ヘルパー ──────────────────────────────────────────
    def _publish_panel(self):
        if self.panel_open:
            self.pub_panel.publish(String(data='tablet-ab-test'))

    def _open_panel(self):
        self.panel_open = True
        self.pub_panel.publish(String(data='tablet-ab-test'))
        self._spin(0.4)

    def _close_panel(self):
        self.panel_open = False
        self.pub_panel.publish(String(data=''))

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
    def test_a_no_heartbeat_rejects(self):
        s = self._latest()
        assert s.mode == 'IDLE'
        res = self._set_auto_brake(False)
        assert not res.accepted
        assert res.reject_reason_key == 'auto_brake_locked', res.reject_reason_key
        assert self._latest().auto_brake is True

    def test_b_heartbeat_accepts_and_keeps_without_lease(self):
        self.screen_id = 'S-21'
        self._open_panel()
        assert self._wait(lambda s: s.zone == 'IN', 3.0)
        res = self._set_auto_brake(False)
        assert res.accepted, res.reject_reason_key
        assert self._wait(lambda s: s.auto_brake is False), 'OFF にならない'
        self._spin(JOG_LEASE_MS / 1000.0 * 6)   # リースの 6 倍待っても保つ
        s = self._latest()
        assert s.auto_brake is False, '心拍が続いているのに ON に戻った'
        assert s.jog_active is False, 'リースを送っていないのに jog_active になった'
        assert self._set_auto_brake(True).accepted
        assert self._wait(lambda s: s.auto_brake is True)

    def test_c_heartbeat_loss_returns_to_on(self):
        self.screen_id = 'S-21'
        self._open_panel()
        assert self._set_auto_brake(False).accepted
        assert self._wait(lambda s: s.auto_brake is False)
        self.panel_open = False   # 閉じる通知なしで途絶（タブレットが落ちた）
        t0 = time.time()
        assert self._wait(lambda s: s.auto_brake is True, 3.0), '心拍が途絶えても OFF のまま'
        assert (time.time() - t0) > JOG_LEASE_MS / 1000.0 * 0.5, '猶予なしで即 ON になった'
        assert not self._set_auto_brake(False).accepted, '途絶後に OFF 要求が通った'

    def test_d_close_notice_returns_to_on_immediately(self):
        self.screen_id = 'S-21'
        self._open_panel()
        assert self._set_auto_brake(False).accepted
        assert self._wait(lambda s: s.auto_brake is False)
        # 心拍が新しいまま閉じる通知を出す。失効（jog_lease_ms=300）を待たずに戻ること。
        self.panel_open = False
        self.pub_panel.publish(String(data='tablet-ab-test'))
        self.pub_panel.publish(String(data=''))
        self._spin(0.2)   # _latest() は更に 0.3 s 回すので使わない（失効の 0.3 s を超えてしまう）
        assert self.history[-1].auto_brake is True, '閉じても OFF のまま（失効を待たないと戻らない）'

    def test_e_mode_change_discards_request(self):
        self.screen_id = 'S-21'
        self._open_panel()
        assert self._wait(lambda s: s.zone == 'IN', 3.0)
        assert self._set_auto_brake(False).accepted
        assert self._wait(lambda s: s.auto_brake is False)
        self._enter_manual()   # 心拍は続いている。IDLE → MANUAL（場内の既定は ON）
        self._spin(0.5)
        assert self.panel_open
        assert self._latest().auto_brake is True, 'モードが変わっても OFF 要求が残った'

    def test_f_reopen_does_not_revive_old_request(self):
        self.screen_id = 'S-21'
        self._open_panel()
        assert self._set_auto_brake(False).accepted
        self.panel_open = False
        assert self._wait(lambda s: s.auto_brake is True, 3.0)
        self._open_panel()
        self._spin(0.5)
        assert self._latest().auto_brake is True, '途絶前の OFF 要求が復活した'

    def test_g_presence_loss_forces_on(self):
        self.screen_id = 'S-21'
        self._open_panel()
        assert self._set_auto_brake(False).accepted
        assert self._wait(lambda s: s.auto_brake is False)
        self.interacting = False
        assert self._wait(lambda s: s.zone == 'NA', 3.0)
        assert self._latest().auto_brake is True, 'ゾーン NA で OFF のままだった'


if __name__ == '__main__':
    unittest.main()
