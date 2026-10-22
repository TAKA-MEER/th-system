"""
test_map_update_denied_node.py
==============================
地図更新 ON は当面使えない（2026-10-22。Spec-webui S-14・S-21、Spec-onsite §4.0.1）。
state_manager を実際に起動して、/system/set_flag の map_update を縛る launch_testing。

純関数（test_map_update_policy.py）が固くても、set_flag の受け口で呼ぶ数行が
外れれば ON が通ってしまう。本番の経路で見る。

  a. map_update を ON にする要求は拒否（map_update_unavailable）し、
     /system/state.map_update は偽のまま
  b. OFF にする要求は受ける
  c. 他のフラグ（tracker_enabled）の set_flag は影響を受けない
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

from th_system_msgs.msg import SystemState
from th_system_msgs.srv import SetFlag


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


_STATE_QOS = QoSProfile(
    depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST)


class TestMapUpdateDenied(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_map_update_denied_client')
        self.history = []
        self.node.create_subscription(
            SystemState, '/system/state', self.history.append, _STATE_QOS)
        self.cli = self.node.create_client(SetFlag, '/system/set_flag')
        assert self.cli.wait_for_service(timeout_sec=10.0), 'state_manager が起動していない'

    def tearDown(self):
        self.node.destroy_node()

    def _set_flag(self, flag, value):
        req = SetFlag.Request()
        req.flag = flag
        req.value = value
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=3.0)
        return fut.result()

    def _latest(self):
        deadline = time.time() + 0.4
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        assert self.history, '/system/state を受信していない'
        return self.history[-1]

    def test_a_on_is_rejected(self):
        res = self._set_flag('map_update', True)
        assert not res.accepted, '地図更新 ON が通った'
        assert res.reject_reason_key == 'map_update_unavailable', res.reject_reason_key
        assert self._latest().map_update is False, 'ON が拒否されたのに state が真になった'

    def test_b_off_is_accepted(self):
        res = self._set_flag('map_update', False)
        assert res.accepted, res.reject_reason_key
        assert self._latest().map_update is False

    def test_c_other_flags_unaffected(self):
        res = self._set_flag('tracker_enabled', True)
        assert res.accepted, res.reject_reason_key


if __name__ == '__main__':
    unittest.main()
