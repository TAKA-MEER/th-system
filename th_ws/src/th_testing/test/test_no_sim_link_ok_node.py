"""
test_no_sim_link_ok_node.py
====================================
8-g P6（DetailedDesign-measure.md §6.7）: sim=false（通常）のときは
`/safety/estop_hw` を受けていなければ `evt.link_ok` が出ないこと
（従来どおり。P6 の sim 分岐が通常の判定を壊していないことの縛り）。

本物の connectivity_checker を sim: False で起動し、疎通の 4 項目をすべて
揃えたうえで、estop_hw 未受信では出ず、受信（false）して初めて出ることを
確かめる。1 つのメソッドで順に確かめるため、メソッド同士の状態に頼らない。
"""
import time
import unittest

import pytest
import rclpy

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from th_system_msgs.msg import StateEvent, WheelFeedback


SCAN_EXPECTED_POINTS = 8
ESP32_ALIVE_TIMEOUT_MS = 3000
SCAN_PUBLISH_PERIOD_S = 0.2


@pytest.mark.launch_test
def generate_test_description():
    connectivity_checker = launch_ros.actions.Node(
        package='th_state',
        executable='connectivity_checker.py',
        name='connectivity_checker',
        parameters=[{
            'esp32_alive_timeout_ms': ESP32_ALIVE_TIMEOUT_MS,
            'scan_expected_points': SCAN_EXPECTED_POINTS,
            # sim=false なので required_nodes が判定に入る。自分自身の名前で揃える。
            'required_nodes': ['connectivity_checker'],
            'restart_max_count': 3,
            'restart_wait_ms': 5000,
            'sim': False,
        }],
        output='screen',
    )
    return launch.LaunchDescription([
        connectivity_checker,
        launch_testing.actions.ReadyToTest(),
    ]), {'connectivity_checker': connectivity_checker}


class TestNoSimLinkOkNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_no_sim_link_ok_client')
        self._event_history = []
        self.node.create_subscription(
            StateEvent, '/system/event', self._event_history.append, 10)
        self.pub_scan = self.node.create_publisher(LaserScan, '/scan', 5)
        self.pub_fb = self.node.create_publisher(WheelFeedback, '/esp32/wheel_feedback', 5)
        self.pub_alive = self.node.create_publisher(Bool, '/esp32/cmd_alive', 10)
        self.pub_estop_hw = self.node.create_publisher(Bool, '/safety/estop_hw', 10)
        self._timers = []

    def tearDown(self):
        for t in self._timers:
            self.node.destroy_timer(t)
        self.node.destroy_node()

    def _spin(self, duration: float = 0.3):
        deadline = time.time() + duration
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _link_ok_events(self):
        return [e for e in self._event_history if e.event == 'evt.link_ok']

    def test_no_sim_requires_estop_hw_seen(self):
        """疎通 4 項目を揃えても estop_hw 未受信なら出ない。false を受けて初めて出る。"""
        # 疎通の 4 項目を揃える: /scan・wheel_feedback・cmd_alive・required_nodes。
        def keepalive():
            scan = LaserScan()
            scan.ranges = [1.0] * SCAN_EXPECTED_POINTS
            self.pub_scan.publish(scan)
            self.pub_fb.publish(WheelFeedback())
            self.pub_alive.publish(Bool(data=True))
        keepalive()
        self._timers.append(self.node.create_timer(SCAN_PUBLISH_PERIOD_S, keepalive))
        self._spin(1.0)  # 疎通が揃うのを待つ（評価は 1 Hz）

        # estop_hw を一度も出さない。2 秒待っても出ないこと。
        self._event_history.clear()
        # さらに 2 秒待つ（評価タイマが複数回まわる）。
        deadline = time.time() + 2.5
        while time.time() < deadline:
            self._spin(0.2)
        assert self._link_ok_events() == [], (
            'sim=false で /safety/estop_hw 未受信のまま evt.link_ok が出た'
            '（通常の判定が壊れている。estop_seen を求めていない）')

        # estop_hw=false を受けて初めて出る。
        self._estop_value = False
        t = self.node.create_timer(
            0.1, lambda: self.pub_estop_hw.publish(Bool(data=self._estop_value)))
        self._timers.append(t)
        self.pub_estop_hw.publish(Bool(data=False))
        deadline = time.time() + 4.0
        while time.time() < deadline and not self._link_ok_events():
            self._spin(0.2)
        assert self._link_ok_events(), 'estop_hw=false の受信後に evt.link_ok が出ない'


if __name__ == '__main__':
    unittest.main()
