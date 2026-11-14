"""
test_venue_save_result_node.py — A-11 ② / SM-3.1.2-052（本番の経路）。

PREP の「保存」は、保存が成功したときだけ SAVED へ進む。失敗したら PREP は元の状態のまま、
理由つきの案内（guide{venue_save_failed, detail}）が /system/effect に出る。

`state_manager`（本物）と `slam_control`（本物）を同時に起動し、slam_toolbox の
`set_localization_mode`・`serialize_map` だけ代役にする。
  a. serialize がファイルを書く → commit_venue_map 成功 → PREP/SAVED
  b. serialize が何も書かない → 失敗 → PREP/MAPPING のまま・理由つきの guide
  c. 失敗のあと成功すれば SAVED へ進む

変異: T-PREP-12 の to_state を SAVED に戻す → b が赤 / slam_control が失敗でも
evt.venue_map_saved を出す → b が赤 / 成功の通知を出さない → a・c が赤。
"""
import json
import os
import shutil
import tempfile
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

from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from slam_toolbox.srv import SerializePoseGraph
from th_system_msgs.msg import FaultStatus, StateEffect, SystemState
from th_system_msgs.srv import UiTrigger

MAP_DIR = tempfile.mkdtemp(prefix='venue_save_maps_')
VENUE_DIR = tempfile.mkdtemp(prefix='venue_save_venue_')

_STATE_QOS = QoSProfile(
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
    slam_control = launch_ros.actions.Node(
        package='th_config_manager', executable='slam_control.py', name='slam_control',
        parameters=[{'map_dir': MAP_DIR, 'venue_map_dir': VENUE_DIR}],
        output='screen')
    return launch.LaunchDescription([
        state_manager, slam_control, launch_testing.actions.ReadyToTest(),
    ]), {}


class TestVenueSaveResult(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()
        shutil.rmtree(MAP_DIR, ignore_errors=True)
        shutil.rmtree(VENUE_DIR, ignore_errors=True)

    def setUp(self):
        self.node = rclpy.create_node('test_venue_save_result_client')
        self.write_graph = True
        self.node.create_service(
            SetBool, '/slam_toolbox/set_localization_mode', self._localization_ok)
        self.node.create_service(
            SerializePoseGraph, '/slam_toolbox/serialize_map', self._serialize)
        self._states = []
        self._effects = []
        self.node.create_subscription(
            SystemState, '/system/state', self._states.append, _STATE_QOS)
        self.node.create_subscription(
            StateEffect, '/system/effect', self._effects.append,
            QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE))
        self.pub_fault = self.node.create_publisher(FaultStatus, '/safety/fault', 5)
        self.pub_hw = self.node.create_publisher(Bool, '/safety/estop_hw', 10)
        self.pub_ui_estop = self.node.create_publisher(Bool, '/safety/estop_ui', 10)
        self.cli_trigger = self.node.create_client(UiTrigger, '/system/trigger')
        assert self.cli_trigger.wait_for_service(timeout_sec=15.0), 'state_manager 未起動'
        self._spin(1.0)
        self.pub_fault.publish(FaultStatus(active=False, fault_type='', severity=''))
        self.pub_hw.publish(Bool(data=False))
        self.pub_ui_estop.publish(Bool(data=False))
        self._spin(0.3)
        self._trigger('ui.finish')
        assert self._wait('IDLE', 'NONE'), f'IDLE に戻らない ({self._ms()})'
        res = self._trigger('ui.enter_mode', {'mode': 'PREP'})
        assert res.accepted, res.reject_reason_key
        assert self._wait('PREP', 'MAPPING'), f'PREP に入らない ({self._ms()})'
        self._effects.clear()

    def tearDown(self):
        self._trigger('ui.finish')
        self.node.destroy_node()

    def _localization_ok(self, req, resp):
        resp.success = True
        resp.message = ''
        return resp

    def _serialize(self, req, resp):
        """slam_toolbox の代役。write_graph が真なら .posegraph を（更新して）書く。"""
        if self.write_graph:
            with open(req.filename + '.posegraph', 'w') as f:
                f.write(str(time.time()))
            os.utime(req.filename + '.posegraph', (time.time() + 5, time.time() + 5))
        return resp

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
        end = time.time() + 6.0
        while time.time() < end and not fut.done():
            self._spin(0.02)
        assert fut.done(), 'ui trigger の応答が無い'
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

    def _guides(self):
        out = []
        for e in self._effects:
            if e.name == 'guide':
                out.append(json.loads(e.args_json))
        return out

    def test_a_success_moves_to_saved(self):
        self.write_graph = True
        res = self._trigger('ui.save')
        assert res.accepted, res.reject_reason_key
        assert self._wait('PREP', 'SAVED'), f'成功したのに SAVED へ進まない ({self._ms()})'
        assert not self._guides(), f'成功なのに案内が出た: {self._guides()}'

    def test_b_failure_stays_and_guides_with_reason(self):
        self.write_graph = False
        res = self._trigger('ui.save')
        assert res.accepted, res.reject_reason_key
        end = time.time() + 6.0
        while time.time() < end and not self._guides():
            self._spin(0.1)
        guides = self._guides()
        assert guides, '失敗の案内が /system/effect に出ない'
        assert guides[-1].get('key') == 'venue_save_failed', guides
        assert guides[-1].get('detail'), f'理由が付いていない: {guides}'
        self._spin(0.5)
        assert self._ms() == ('PREP', 'MAPPING'), (
            f'失敗したのに状態が動いた ({self._ms()})')

    def test_c_retry_after_failure_succeeds(self):
        self.write_graph = False
        self._trigger('ui.save')
        self._spin(1.5)
        assert self._ms() == ('PREP', 'MAPPING')
        self.write_graph = True
        self._trigger('ui.save')
        assert self._wait('PREP', 'SAVED'), f'やり直しで SAVED へ進まない ({self._ms()})'
