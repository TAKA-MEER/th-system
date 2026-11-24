"""
test_sim_link_ok_node.py
====================================
8-g P6（DetailedDesign-measure.md §6.7）: sim=true のときは
`/safety/estop_hw` を一度も受けていなくても `evt.link_ok` が出ること
（publisher の `esp32_bridge` は sim に居ないため、求めると IDLE に入れない）。
押されていると分かっている間は sim でも出さない。

本物の connectivity_checker を sim: True＋生成 yaml 相当のパラメータで起動する
（`launch_testing`）。1 つのメソッドで順に確かめるため、メソッド同士の状態に
頼らない（/safety/estop_hw を出すのはフェーズ 2 から）。
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
            'required_nodes': ['unused_in_sim'],  # sim=True のため判定には使われない
            'restart_max_count': 3,
            'restart_wait_ms': 5000,
            'sim': True,
        }],
        output='screen',
    )
    return launch.LaunchDescription([
        connectivity_checker,
        launch_testing.actions.ReadyToTest(),
    ]), {'connectivity_checker': connectivity_checker}


class TestSimLinkOkNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_sim_link_ok_client')
        self._event_history = []
        self.node.create_subscription(
            StateEvent, '/system/event', self._event_history.append, 10)
        self.pub_scan = self.node.create_publisher(LaserScan, '/scan', 5)
        self.pub_estop_hw = self.node.create_publisher(Bool, '/safety/estop_hw', 10)
        self._scan_timer = None
        self._estop_timer = None
        self._estop_value = True

    def tearDown(self):
        for t in (self._scan_timer, self._estop_timer):
            if t is not None:
                self.node.destroy_timer(t)
        self.node.destroy_node()

    def _spin(self, duration: float = 0.3):
        deadline = time.time() + duration
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _publish_scan(self):
        msg = LaserScan()
        msg.ranges = [1.0] * SCAN_EXPECTED_POINTS
        self.pub_scan.publish(msg)

    def _start_scan_keepalive(self):
        self._publish_scan()
        self._scan_timer = self.node.create_timer(SCAN_PUBLISH_PERIOD_S, self._publish_scan)

    def _link_ok_events(self):
        return [e for e in self._event_history if e.event == 'evt.link_ok']

    def test_sim_link_ok_without_estop_hw_and_pressed_blocks(self):
        """フェーズ 1: /safety/estop_hw を一度も出さずに evt.link_ok が出る。
        フェーズ 2: true を出し続けると新しい evt.link_ok は出ない（押下中は出さない）。
        フェーズ 3: false に戻すと再び出る（gate が下がったあとの立ち上がり）。"""
        # フェーズ 1: estop_hw を一切出さない。疎通（/scan）だけ整える。
        self._start_scan_keepalive()
        deadline = time.time() + 6.0
        while time.time() < deadline and not self._link_ok_events():
            self._spin(0.2)
        assert self._link_ok_events(), (
            'sim=true で /safety/estop_hw 未受信のまま evt.link_ok が出ない'
            '（estop_seen を求め続けている。P6 の分岐が効いていない）')

        # フェーズ 2: 押下中は出さない（既出の 1 件だけが残る）。
        self._event_history.clear()
        self._estop_timer = self.node.create_timer(
            0.1, lambda: self.pub_estop_hw.publish(Bool(data=self._estop_value)))
        self._spin(2.0)
        assert self._link_ok_events() == [], (
            'sim=true で押下中（/safety/estop_hw=true）に evt.link_ok が出た')

        # フェーズ 3: 解除すれば再び出る。
        self._estop_value = False
        self.pub_estop_hw.publish(Bool(data=False))
        deadline = time.time() + 4.0
        while time.time() < deadline and not self._link_ok_events():
            self._spin(0.2)
        assert self._link_ok_events(), '押下解除後に evt.link_ok が出ない'


if __name__ == '__main__':
    unittest.main()
