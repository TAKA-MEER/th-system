"""SM-3.1.1-19: 実 state_manager で運用入口の回復フォルト拒否を確かめる。"""
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
from th_system_msgs.msg import ActiveFaults, FaultStatus, StateEffect, SystemState
from th_system_msgs.srv import UiTrigger


@pytest.mark.launch_test
def generate_test_description():
    node = launch_ros.actions.Node(
        package='th_state', executable='state_manager.py', name='state_manager',
        parameters=[{'jog_lease_ms': 300, 'link_wait_timeout_ms': 60_000,
                     'ui_active_window_s': 5, 'screen_stale_ms': 60_000}],
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
        self.cli = self.node.create_client(UiTrigger, '/system/trigger')
        assert self.cli.wait_for_service(timeout_sec=5.0)
        self._trigger('ui.enter_mode', {'mode': 'IDLE'})
        self._spin(0.2)

    def tearDown(self):
        self.node.destroy_node()

    def _spin(self, duration=0.2):
        end = time.time() + duration
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

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

    def test_fault_blocks_entry_reports_names_but_tracker_fault_does_not(self):
        self._active(['ESP32_DISCONNECTED', 'LIDAR_LOST'])
        self.effects.clear()
        res = self._trigger('ui.enter_mode', {'mode': 'FOLLOW'})
        assert not res.accepted
        assert res.reject_reason_key == 'recoverable_fault_active'
        assert (res.mode, res.state) == ('IDLE', 'NONE')
        self._spin(0.2)
        windows = [e for e in self.effects if e.name == 'open_window']
        assert windows, '拒否時に W-1 が開かれない'
        args = json.loads(windows[-1].args_json)
        assert args['id'] == 'W-1'
        assert args['fault_types'] == ['ESP32_DISCONNECTED', 'LIDAR_LOST']

        self._active(['PERSON_TRACKER_LOST'])
        res = self._trigger('ui.enter_mode', {'mode': 'FOLLOW'})
        assert res.accepted, res.reject_reason_key
        assert res.mode == 'FOLLOW'

    def test_clear_fault_allows_mode_entry_and_carry_resume_keeps_fault(self):
        self._active(['ESP32_DISCONNECTED'])
        # IDLE を押下前モードにして、SM-3.1.1-14 の CARRY 復帰が既存どおり通ることも確認。
        self.pub_fault.publish(FaultStatus(active=True, fault_type='ESP32_DISCONNECTED',
                                           severity='RECOVERABLE'))
        self._spin(0.2)
        self.pub_hw.publish(Bool(data=True))
        self._spin(0.2)
        self.pub_hw.publish(Bool(data=False))
        self._spin(0.2)
        res = self._trigger('ui.carry_resume')
        assert res.accepted, res.reject_reason_key
        assert (res.mode, res.state) == ('IDLE', 'NONE')
        res = self._trigger('ui.enter_mode', {'mode': 'MANUAL'})
        assert not res.accepted
        assert res.reject_reason_key == 'recoverable_fault_active'

        self.pub_fault.publish(FaultStatus(active=False, fault_type='ESP32_DISCONNECTED',
                                           severity='RECOVERABLE'))
        self._active([])
        self._spin(0.2)
        res = self._trigger('ui.enter_mode', {'mode': 'MANUAL'})
        assert res.accepted, res.reject_reason_key
        assert res.mode == 'MANUAL'
