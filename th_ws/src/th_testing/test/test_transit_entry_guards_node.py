"""
test_transit_entry_guards_node.py
==================================
1b-11（SG-B13・SG-B14）— state_manager を実際に起動し、方式選択の前提と
地図更新の保存が **WebUI を介さずに** サーバ側で縛られることを見る。
ホスト側の step() 試験（test_transit_entry_guards_1b11.py）は純関数だけなので、
ここでは「state_manager が値を渡す数行」（経路の本数・leash_present・
map_update_available）を壊したら赤くなるように、本番の経路で見る。

  a. /route/catalog が 0 本 → 教示再生は no_route_recorded で拒否
     （状態は動かない）／1 本載せると通る
  b. evt.leash_present 前 → 電子リードは device_not_connected で拒否／
     evt.leash_present 後は通り、/system/state の leash_present が真
  c. REPLAY/PAUSE で地図更新 OFF → 保存は map_update_off で拒否（地図の有無によらず）。
     ON は機体が拒否するので保存が通る経路は当面ない（2026-10-22）
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
from std_msgs.msg import Bool
from th_system_msgs.msg import (FaultStatus, RouteInfo, RouteList, RouteStatus,
                                StateEffect, StateEvent, SystemState)
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
_RELIABLE = QoSProfile(
    depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST)


class TestTransitEntryGuardsNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_transit_entry_guards_client')
        self._hist = []
        self.node.create_subscription(SystemState, '/system/state', self._hist.append, _LATCHED)
        self._effects = []
        self.node.create_subscription(StateEffect, '/system/effect', self._effects.append, 10)
        self.pub_event = self.node.create_publisher(StateEvent, '/system/event', 10)
        self.pub_catalog = self.node.create_publisher(RouteList, '/route/catalog', _LATCHED)
        self.pub_status = self.node.create_publisher(RouteStatus, '/route/status', _RELIABLE)
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
        # 次の試験のため IDLE に戻す
        try:
            self._trigger('ui.finish')
        except Exception:
            pass
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

    def _set_flag(self, flag, value):
        req = SetFlag.Request()
        req.flag = flag
        req.value = value
        fut = self.cli_set_flag.call_async(req)
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

    def _publish_catalog(self, ids):
        msg = RouteList()
        for rid in ids:
            info = RouteInfo()
            info.id = rid
            info.name = rid
            info.point_count = 10
            msg.routes.append(info)
        self.pub_catalog.publish(msg)
        self._spin(0.4)

    def _publish_status(self, has_map):
        msg = RouteStatus()
        msg.state = 'PAUSE'
        msg.points = 10
        msg.current = RouteInfo(id='R1', name='R1', point_count=10)
        msg.has_map = has_map
        for _ in range(3):
            self.pub_status.publish(msg)
            self._spin(0.1)
        self._spin(0.3)

    def _go_replay_pause(self):
        """IDLE → REPLAY/ROUTE_SEL → LOCALIZE → READY → RUN → PAUSE。"""
        self._publish_catalog(['R1'])
        res = self._trigger('ui.enter_mode', {'mode': 'REPLAY'})
        assert res.accepted, res.reject_reason_key
        assert self._wait_mode('REPLAY', 'ROUTE_SEL')
        res = self._trigger('ui.route_select', {'id': 'R1', 'reverse': False})
        assert res.accepted, res.reject_reason_key
        assert self._wait_mode('REPLAY', 'LOCALIZE')
        self._event('evt.localize_done')
        assert self._wait_mode('REPLAY', 'READY')
        res = self._trigger('ui.run')
        assert res.accepted, res.reject_reason_key
        assert self._wait_mode('REPLAY', 'RUN')
        res = self._trigger('ui.stop')
        assert res.accepted, res.reject_reason_key
        assert self._wait_mode('REPLAY', 'PAUSE')

    # ── 試験 ──
    def test_a_replay_needs_a_recorded_route(self):
        self._publish_catalog([])
        res = self._trigger('ui.enter_mode', {'mode': 'REPLAY'})
        assert not res.accepted, '経路 0 本なのに教示再生に入れた'
        assert res.reject_reason_key == 'no_route_recorded', res.reject_reason_key
        snap = self._latest()
        assert snap.mode == 'IDLE', f'拒否したのにモードが動いた: {snap.mode}'

        self._publish_catalog(['R1'])
        res = self._trigger('ui.enter_mode', {'mode': 'REPLAY'})
        assert res.accepted, res.reject_reason_key
        assert self._wait_mode('REPLAY', 'ROUTE_SEL')

    def test_b_leash_needs_device(self):
        res = self._trigger('ui.enter_mode', {'mode': 'LEASH'})
        assert not res.accepted, 'デバイス未接続なのに電子リードに入れた'
        assert res.reject_reason_key == 'device_not_connected', res.reject_reason_key
        assert self._latest().leash_present is False

        self._event('evt.leash_present')
        self._spin(0.4)
        assert self._latest().leash_present is True, 'leash_present が /system/state に出ない'
        res = self._trigger('ui.enter_mode', {'mode': 'LEASH'})
        assert res.accepted, res.reject_reason_key
        assert self._wait_mode('LEASH')
        self._trigger('ui.finish')
        self._wait_mode('IDLE')
        self._event('evt.leash_absent')

    def test_c_save_rejected_while_map_update_off(self):
        # 地図更新 ON は当面使えない（2026-10-22）ので、保存は ON 側の経路に入れない。
        # 地図あり・なしのどちらでも、トグル OFF の間は map_update_off で拒否される。
        # ON の拒否そのものは test_map_update_denied_node.py が縛る。
        self._go_replay_pause()
        assert self._set_flag('map_update', False).accepted
        for has_map in (False, True):
            self._publish_status(has_map=has_map)
            res = self._trigger('ui.save')
            assert not res.accepted, f'地図更新 OFF なのに保存が通った (has_map={has_map})'
            assert res.reject_reason_key == 'map_update_off', res.reject_reason_key
