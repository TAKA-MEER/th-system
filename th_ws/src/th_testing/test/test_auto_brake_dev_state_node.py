"""
test_auto_brake_dev_state_node.py
=================================
開発モードの項目 auto_brake（Spec-safety.md §10。2026-10-08 改定）の機体側の正本
（state_manager の /system/set_flag auto_brake と /system/state.auto_brake）を、
ノード本体を実際に起動して縛る launch_testing。

純関数（test_auto_brake.py の dev_item_effective／override_allowed）が固くても、
/system/dev_mode の購読・受信時刻・鮮度・set_flag の受理条件への受け渡しは
state_manager の呼び出し側の数行で壊れうる。

自律系（IDLE は auto_brake_default: on_locked）で:
  a. 開発モードなし → OFF の切替は拒否（auto_brake_locked）。ON のまま
  b. effective.auto_brake=true → 場外・場内どちらでも OFF の切替を受け付け、
     /system/state.auto_brake が OFF になる。ON に戻すこともできる
  c. 項目を選んだだけ（切り替えていない）→ ON のまま（既定は通常と同じ）
  d. OFF にしたあと effective から項目が消える（新しい JSON）→ すぐ ON に戻る。
     その後の OFF 要求は拒否される
  e. OFF にしたあと /system/dev_mode が途絶える → 3 秒を超えたら ON に戻る
  f. ignore（選択）だけ・dev_mode=false・別の項目だけ → OFF の切替は拒否
  g. 項目ありで OFF にしたあと画面が途絶（ゾーン NA）→ ON（安全側）
  h. 項目ありで OFF にしたあと、モードが変わると要求は捨てられる（既定へ戻る）
  i. 壊れた JSON → 何も外さない

注意（CLAUDE.md の既知の癖）: 試験は TestCase のメソッドにする。
メソッドは名前順に走り、前の試験の状態を引き継ぐので、各メソッドは冒頭で自分の前提を作る。
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


ITEMS = ('link', 'lidar_fault', 'scan_stop', 'battery', 'opcheck', 'auto_brake')
DEV_STALE_S = 3.0


def _dev_json(dev_mode, ignore=(), effective=()):
    return json.dumps({
        'dev_mode': dev_mode,
        'ignore': {k: (k in ignore) for k in ITEMS},
        'effective': {k: (k in effective) for k in ITEMS},
        'estop_hw_known': False,
        'estop_hw_pressed': False,
    }, sort_keys=True)


ON = _dev_json(True, ('auto_brake',), ('auto_brake',))


_STATE_QOS = QoSProfile(
    depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST)


class TestAutoBrakeDevState(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_auto_brake_dev_state_client')
        self.history = []
        self.node.create_subscription(
            SystemState, '/system/state', self.history.append, _STATE_QOS)
        self.pub_screen = self.node.create_publisher(ActiveScreen, '/ui/active_screen', 5)
        self.pub_dev = self.node.create_publisher(String, '/system/dev_mode', _STATE_QOS)
        self.dev_payload = None   # None の間は /system/dev_mode を出さない
        self.node.create_timer(1.0, self._publish_dev)
        self.cli_trigger = self.node.create_client(UiTrigger, '/system/trigger')
        self.cli_set_flag = self.node.create_client(SetFlag, '/system/set_flag')
        assert self.cli_trigger.wait_for_service(timeout_sec=10.0), \
            'state_manager が起動していない'
        self.screen_id = 'S-11'
        self.interacting = True
        self.node.create_timer(0.25, self._publish_screen)
        self._to_idle()

    def tearDown(self):
        self.dev_payload = None
        self._to_idle()
        self.node.destroy_node()

    # ── ヘルパー ──────────────────────────────────────────
    def _publish_dev(self):
        if self.dev_payload is not None:
            self.pub_dev.publish(String(data=self.dev_payload))

    def _dev(self, payload):
        """/system/dev_mode を即時に 1 通出し、以後 1 Hz で出し続ける（None で止める）。"""
        self.dev_payload = payload
        if payload is not None:
            self.pub_dev.publish(String(data=payload))
        self._spin(0.4)

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
    def test_a_no_dev_rejects_autonomous_off(self):
        s = self._latest()
        assert s.mode == 'IDLE'
        res = self._set_auto_brake(False)
        assert not res.accepted
        assert res.reject_reason_key == 'auto_brake_locked', res.reject_reason_key
        assert self._latest().auto_brake is True

    def test_b_dev_item_accepts_autonomous_off_out_and_in(self):
        for screen, zone in (('S-11', 'OUT'), ('S-21', 'IN')):
            self.screen_id = screen
            self._dev(ON)
            assert self._wait(lambda s, z=zone: s.zone == z, 3.0), zone
            assert self._latest().mode == 'IDLE'
            res = self._set_auto_brake(False)
            assert res.accepted, (zone, res.reject_reason_key)
            assert self._wait(lambda s: s.auto_brake is False), f'{zone}: OFF にならない'
            assert self._set_auto_brake(True).accepted
            assert self._wait(lambda s: s.auto_brake is True), f'{zone}: ON に戻せない'

    def test_c_dev_item_alone_does_not_turn_off(self):
        self._dev(ON)
        self._spin(1.0)
        s = self._latest()
        assert s.mode == 'IDLE' and s.zone == 'OUT', (s.mode, s.zone)
        assert s.auto_brake is True, '項目を選んだだけで OFF になった（既定は通常と同じ）'

    def test_d_item_removed_returns_to_on_immediately(self):
        self._dev(ON)
        assert self._set_auto_brake(False).accepted
        assert self._wait(lambda s: s.auto_brake is False)
        self._dev(_dev_json(True, ('auto_brake',), ()))   # 項目を外した
        assert self._wait(lambda s: s.auto_brake is True, 1.5), '項目を外しても OFF のまま'
        res = self._set_auto_brake(False)
        assert not res.accepted and res.reject_reason_key == 'auto_brake_locked'

    def test_e_dev_mode_expiry_returns_to_on(self):
        self._dev(ON)
        assert self._set_auto_brake(False).accepted
        assert self._wait(lambda s: s.auto_brake is False)
        self.dev_payload = None   # 発行者が居なくなった
        t0 = time.time()
        assert self._wait(lambda s: s.auto_brake is True, DEV_STALE_S + 3.0), \
            '/system/dev_mode が途絶えても OFF のまま'
        assert (time.time() - t0) > 1.0, '失効の猶予なしで即 ON になった（別の理由で戻った疑い）'
        assert not self._set_auto_brake(False).accepted, '失効後に OFF 要求が通った'
        # 戻ったあと発行が再開しても、要求は残っていない（ON のまま）。
        self._dev(ON)
        self._spin(0.5)
        assert self._latest().auto_brake is True, '失効前の OFF 要求が復活した'

    def test_f_not_effective_rejects(self):
        for payload, why in (
                (_dev_json(True, ('auto_brake',), ()), 'ignore だけ'),
                (_dev_json(False, ('auto_brake',), ('auto_brake',)), 'dev_mode=false'),
                (_dev_json(True, ('scan_stop',), ('scan_stop',)), '別の項目だけ')):
            self._dev(payload)
            res = self._set_auto_brake(False)
            assert not res.accepted, f'{why}: OFF 要求が通った'
            assert res.reject_reason_key == 'auto_brake_locked', why
            assert self._latest().auto_brake is True, why

    def test_g_presence_loss_forces_on(self):
        self._dev(ON)
        assert self._set_auto_brake(False).accepted
        assert self._wait(lambda s: s.auto_brake is False)
        self.interacting = False
        assert self._wait(lambda s: s.zone == 'NA', 3.0)
        assert self._latest().auto_brake is True, 'ゾーン NA で OFF のままだった'

    def test_h_mode_change_discards_request(self):
        self.screen_id = 'S-21'
        self._dev(ON)
        assert self._wait(lambda s: s.zone == 'IN', 3.0)
        assert self._set_auto_brake(False).accepted
        assert self._wait(lambda s: s.auto_brake is False)
        self._enter_manual()   # IDLE → MANUAL（場内の既定は ON）
        self._spin(0.5)
        assert self._latest().auto_brake is True, 'モードが変わっても OFF 要求が残った'

    def test_i_broken_json_does_nothing(self):
        for raw in ('{not json', '[]', '{"dev_mode": true}',
                    '{"dev_mode": true, "effective": {"auto_brake": "true"}}',
                    '{"dev_mode": true, "effective": {"auto_brake": 1}}'):
            self._dev(raw)
            assert not self._set_auto_brake(False).accepted, raw


if __name__ == '__main__':
    unittest.main()
