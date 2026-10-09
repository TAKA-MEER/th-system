"""
test_auto_brake_defaults_node.py
================================
SG-C14: S-50 で変える自動ブレーキの既定（`auto_brake_default_outdoor` /
`auto_brake_default_indoor`）が、state_manager の実効値に本当に効くことを、
ノード本体を実際に起動して縛る launch_testing。

純関数（test_auto_brake.py）が S-50 既定の引数を正しく扱えても、
state_manager が起動時パラメータを `effective_auto_brake` に渡し忘れる
（引数を外す）と、S-50 の変更は画面の保存だけで機体に効かない。
ここの 3 件はその配線（宣言→読み→計算への引き渡し）を縛る。

  a. 場外の既定を 'on' で起動 → 手動走行（MANUAL）・場外（S-11）で ON
     （spec 既定 OFF の逆。渡した値が効いている証拠）
  b. 場内の既定を 'off' で起動 → 手動走行・場内（S-21）で OFF
     （spec 既定 ON の逆）
  c. ロックは保たれる: 自律系・待機（IDLE）とゾーン NA（画面途絶）は、
     逆にした既定でも ON のまま
"""
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
from th_system_msgs.srv import UiTrigger

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
            # SG-C14: S-50 で変えた既定（再起動後に生成 yaml 経由で届く値の再現）。
            # spec 既定（場外 off・場内 on）の逆を渡す。
            'auto_brake_default_outdoor': 'on',
            'auto_brake_default_indoor': 'off',
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


class TestAutoBrakeDefaults(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_auto_brake_defaults_client')
        self.history = []
        self.node.create_subscription(
            SystemState, '/system/state', self.history.append, _STATE_QOS)
        self.pub_screen = self.node.create_publisher(ActiveScreen, '/ui/active_screen', 5)
        self.cli_trigger = self.node.create_client(UiTrigger, '/system/trigger')
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
            screen_id=self.screen_id, client_id='tablet-ab-defaults-test',
            interacting=self.interacting,
            last_input=self.node.get_clock().now().to_msg()))

    def _spin(self, duration=0.3):
        deadline = time.time() + duration
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _trigger(self, trigger, arg=None):
        import json
        req = UiTrigger.Request()
        req.trigger = trigger
        req.arg_json = json.dumps(arg) if arg else ''
        req.requester = 'test'
        fut = self.cli_trigger.call_async(req)
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
    def test_a_outdoor_default_on_applies_outside(self):
        """場外の既定 'on' → 手動走行・場外で ON（spec 既定 OFF の逆）。"""
        self._enter_manual()
        s = self._latest()
        assert s.zone == 'OUT', s.zone
        assert s.auto_brake is True, \
            'auto_brake_default_outdoor=on が効いていない（場外なのに OFF）'

    def test_b_indoor_default_off_applies_inside(self):
        """場内の既定 'off' → 手動走行・場内で OFF（spec 既定 ON の逆）。"""
        self.screen_id = 'S-21'
        self._enter_manual()
        s = self._latest()
        assert s.zone == 'IN', s.zone
        assert s.auto_brake is False, \
            'auto_brake_default_indoor=off が効いていない（場内なのに ON）'

    def test_c_locks_hold_despite_inverted_defaults(self):
        """逆にした既定でも、自律系・待機とゾーン NA は ON のまま。"""
        s = self._latest()
        assert s.mode == 'IDLE'
        assert s.auto_brake is True, '待機（IDLE）で既定の影響を受けた'
        self._enter_manual()
        self.interacting = False
        assert self._wait(lambda s: s.zone == 'NA', 3.0), '画面の途絶で NA にならない'
        assert self._latest().auto_brake is True, '画面途絶で既定の影響を受けた'
        self.interacting = True
        assert self._wait(lambda s: s.zone == 'OUT', 3.0)


if __name__ == '__main__':
    unittest.main()
