"""起動時の /system/state 未受信猶予と、受信後の途絶検知を本物のノードで確認する。"""

import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from th_system_msgs.msg import ActiveFaults, SystemState


STARTUP_DEADLINE_SEC = 4
STATE_STALE_MS = 300


def generate_test_description():
    safety = launch_ros.actions.Node(
        package='th_safety',
        executable='safety_monitor',
        name='safety_monitor',
        parameters=[{
            'enabled_targets': ['state'],
            'startup_grace_sec': 0,
            'startup_deadline_sec': STARTUP_DEADLINE_SEC,
            'state_stale_ms': STATE_STALE_MS,
            'check_period_ms': 20,
            'critical_fault_hold_ms': 80,
        }],
        output='screen',
    )
    return launch.LaunchDescription([
        safety,
        launch_testing.actions.ReadyToTest(),
    ]), {'safety_monitor': safety}


class TestStateStartupDeadline(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_state_startup_deadline')
        self.active_faults = []
        self.node.create_subscription(
            ActiveFaults, '/safety/active_faults', self.active_faults.append, 10)
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub_state = self.node.create_publisher(SystemState, '/system/state', state_qos)

    def tearDown(self):
        self.node.destroy_node()

    def _pump(self, seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def _state_fault_active(self):
        return bool(self.active_faults and
                    'STATE_INCONSISTENT' in self.active_faults[-1].types)

    def _wait_for_state_fault(self, active, timeout):
        until = time.monotonic() + timeout
        while time.monotonic() < until:
            self._pump(0.05)
            if self._state_fault_active() == active:
                return True
        return False

    def test_unreceived_state_waits_for_deadline_but_later_outage_is_detected(self):
        # 未受信中、state_stale_ms を超えても startup_deadline までは発報しない。
        self._pump(2.0)
        self.assertFalse(self._state_fault_active(), 'deadline 前に STATE_INCONSISTENT が出た')

        # deadline 超過後も未受信なら、従来どおり重大フォルトを発報する。
        self.assertTrue(self._wait_for_state_fault(True, 4.0), 'deadline 後の未受信を検知しなかった')

        # 有効な状態を受け取れば解除し、受信後に途絶した場合は再び発報する。
        state = SystemState()
        state.mode = 'INIT'
        state.state = 'CHECK'
        self.pub_state.publish(state)
        self.assertTrue(self._wait_for_state_fault(False, 2.0), '状態受信後にフォルトが解除されなかった')
        self.assertTrue(self._wait_for_state_fault(True, 2.0), '受信後の途絶を検知しなかった')
