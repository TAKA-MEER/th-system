"""
test_prep_return_home_map_node.py — A-11 ③ / SM-3.1.2-144（本番の経路）。

PREP/MAPPING の「1 ボタンで待機場所へ」は、会場地図を持たず地図作成も始まっていない
（/map_session/status の VENUE が UNLOADED）なら RETURN へ進めず理由つきで拒否する。
地図作成中（MAPPING）なら今までどおり RETURN へ進む。本物の state_manager を起動し、
/map_session/status と HOME ピンを試験側から出す。

変異: T-PREP-06x を消す → a が赤 / state_manager が VENUE の mode を ctx.venue_unloaded へ
渡さない → a が赤。
"""
import json
import time
import unittest

import pytest
import rclpy
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions

from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from th_system_msgs.msg import FaultStatus, MapSessionStatus, Pin, PinList, SystemState
from th_system_msgs.srv import UiTrigger

_LATCHED = QoSProfile(
    depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST)


@pytest.mark.launch_test
def generate_test_description():
    state_manager = launch_ros.actions.Node(
        package='th_state', executable='state_manager.py', name='state_manager',
        parameters=[{'link_wait_timeout_ms': 600_000, 'screen_stale_ms': 600_000,
                     'ui_active_window_s': 5, 'jog_lease_ms': 1200}],
        output='screen')
    return launch.LaunchDescription([
        state_manager, launch_testing.actions.ReadyToTest(),
    ]), {}


class TestPrepReturnHomeMap(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_prep_return_home_map_client')
        self._states = []
        self.node.create_subscription(SystemState, '/system/state', self._states.append, _LATCHED)
        self.pub_map = self.node.create_publisher(MapSessionStatus, '/map_session/status', _LATCHED)
        self.pub_pins = self.node.create_publisher(PinList, '/onsite/pins', _LATCHED)
        self.pub_fault = self.node.create_publisher(FaultStatus, '/safety/fault', 5)
        self.pub_hw = self.node.create_publisher(Bool, '/safety/estop_hw', 10)
        self.cli = self.node.create_client(UiTrigger, '/system/trigger')
        assert self.cli.wait_for_service(timeout_sec=15.0), 'state_manager 未起動'
        self.pub_fault.publish(FaultStatus(active=False, fault_type='', severity=''))
        self.pub_hw.publish(Bool(data=False))
        pin = Pin(id='home', name='home', kind='HOME', pose=Pose())
        self.pub_pins.publish(PinList(pins=[pin]))
        self._spin(0.5)
        self._trigger('ui.finish')
        assert self._wait('IDLE', 'NONE')
        res = self._trigger('ui.enter_mode', {'mode': 'PREP'})
        assert res.accepted, res.reject_reason_key
        assert self._wait('PREP', 'MAPPING')

    def tearDown(self):
        self._trigger('ui.finish')
        self.node.destroy_node()

    def _spin(self, d=0.2):
        end = time.time() + d
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _trigger(self, trigger, arg=None):
        req = UiTrigger.Request()
        req.trigger = trigger
        req.arg_json = json.dumps(arg) if arg else ''
        req.requester = 'test'
        fut = self.cli.call_async(req)
        end = time.time() + 6.0
        while time.time() < end and not fut.done():
            self._spin(0.02)
        assert fut.done()
        return fut.result()

    def _ms(self):
        self._spin(0.2)
        return (self._states[-1].mode, self._states[-1].state) if self._states else None

    def _wait(self, mode, state, timeout=8.0):
        end = time.time() + timeout
        while time.time() < end:
            if self._ms() == (mode, state):
                return True
        return False

    def _venue(self, mode):
        self.pub_map.publish(MapSessionStatus(slot='VENUE', mode=mode))
        self._spin(0.4)

    def test_a_unloaded_rejects_with_reason(self):
        self._venue('UNLOADED')
        res = self._trigger('ui.return_home')
        assert not res.accepted, '地図が無いのに戻れてしまう'
        assert res.reject_reason_key == 'prep_map_not_started', res.reject_reason_key
        assert self._ms() == ('PREP', 'MAPPING'), f'拒否したのに状態が動いた ({self._ms()})'

    def test_b_mapping_goes_to_return(self):
        self._venue('MAPPING')
        res = self._trigger('ui.return_home')
        assert res.accepted, res.reject_reason_key
        assert self._wait('PREP', 'RETURN'), f'RETURN へ進まない ({self._ms()})'

    def test_c_loaded_goes_to_return(self):
        self._venue('LOCALIZING')
        res = self._trigger('ui.return_home')
        assert res.accepted, res.reject_reason_key
        assert self._wait('PREP', 'RETURN')
