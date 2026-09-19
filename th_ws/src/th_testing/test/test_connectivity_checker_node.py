"""
test_connectivity_checker_node.py
====================================
DetailedDesign-wp1.md `WP-STATE-03` §7 — connectivity_checker ノードの単体試験。

【実行に関する注意】このファイルはノードを起動する（`ros2 test` 経由）。
`connectivity_core.evaluate()` 自体の判定網羅は test_connectivity_core.py の対象。
ここでは connectivity_checker.py 自体の責務（L-2: 物理 E-Stop 押下中は出さない、
L-3: 立ち上がりで1回だけ）だけを検証する。

`sim=True` にして ESP32 の2項目（esp32_feedback/esp32_loopback）と required_nodes を
除外し（§8）、`/scan` の疎通と `/safety/estop_hw` だけで gate を制御できるようにする
（テストの単純化。実機の ESP32/必須ノード群を用意せずに L-2/L-3 だけを閉じたテストにできる）。
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
from th_system_msgs.msg import StateEvent


SCAN_EXPECTED_POINTS = 8
ESP32_ALIVE_TIMEOUT_MS = 3000
SCAN_PUBLISH_PERIOD_S = 0.2   # esp32_alive_timeout_ms よりずっと短く保つ（疎通を維持する）


# 注意（CLAUDE.md の既知の癖）: このファイルは @pytest.mark.launch_test を持つため、
# launch_testing の pytest プラグインがファイル全体を1つの pytest アイテムとして収集し、
# その中の unittest.TestCase だけを実行する。モジュール直下の素の pytest 関数は
# 収集されず黙って実行されないので、全部 TestConnectivityCheckerNode のメソッドにする。

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


class TestConnectivityCheckerNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_connectivity_checker_client')

        self._event_history = []
        self.node.create_subscription(
            StateEvent, '/system/event', self._event_history.append, 10)

        self.pub_scan = self.node.create_publisher(LaserScan, '/scan', 5)
        self.pub_estop_hw = self.node.create_publisher(Bool, '/safety/estop_hw', 10)
        self._scan_timer = None

        # 前のテストの残留 gate をリセットする: 物理 E-Stop を押して gate を
        # 確実に False へ落としてから各テストを始める（state_manager_node テストの
        # _reset_to_idle() と同じ考え方）。
        #
        # ★ 1 回だけ publish してはいけない。publisher を作った直後は購読側との
        #   マッチングが終わっておらず、RELIABLE でも volatile なので届かずに消える
        #   （実際にこれで L-2 のテストが誤って失敗した）。実機の /safety/estop_hw は
        #   10 Hz で流れ続けるので、テストでも流し続けて実機と同じ状況にする。
        self._estop_value = True
        self._estop_timer = self.node.create_timer(
            0.1, lambda: self.pub_estop_hw.publish(Bool(data=self._estop_value)))
        self._spin(0.5)
        self._event_history.clear()

    def tearDown(self):
        if getattr(self, '_estop_timer', None) is not None:
            self._estop_timer.cancel()
            self.node.destroy_timer(self._estop_timer)
            self._estop_timer = None
        if self._scan_timer is not None:
            self._scan_timer.cancel()
        self.node.destroy_node()

    # ── ヘルパー ──────────────────────────────────────────────
    def _spin(self, duration: float = 0.3):
        deadline = time.time() + duration
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _publish_scan(self):
        msg = LaserScan()
        msg.ranges = [1.0] * SCAN_EXPECTED_POINTS
        self.pub_scan.publish(msg)

    def _start_scan_keepalive(self):
        """/scan を esp32_alive_timeout_ms より十分短い周期で送り続ける。"""
        self._publish_scan()
        self._scan_timer = self.node.create_timer(SCAN_PUBLISH_PERIOD_S, self._publish_scan)

    def _link_ok_events(self):
        return [e for e in self._event_history if e.event == 'evt.link_ok']

    # ════════════════════════════════════════════════════════
    def test_estop_blocks_link_ok(self):
        """L-2（CL-B-6）: 物理 E-Stop 押下中は疎通条件（/scan）が揃っていても
        evt.link_ok を出さない。解除すれば出る。"""
        self._start_scan_keepalive()
        self._spin(0.5)   # setUp で estop_hw=True にした状態のまま、疎通条件だけ整える

        self._event_history.clear()
        self._spin(2.0)
        assert self._link_ok_events() == [], \
            '物理 E-Stop 押下中に evt.link_ok が出た（L-2 違反）'

        self._estop_value = False   # 継続送信の値を切り替える（単発 publish にしない）
        self.pub_estop_hw.publish(Bool(data=False))
        deadline = time.time() + 3.0
        while time.time() < deadline and not self._link_ok_events():
            self._spin(0.2)
        assert self._link_ok_events(), 'E-Stop 解除後に evt.link_ok が出ない'

    def test_link_ok_once(self):
        """L-3: evt.link_ok は立ち上がりで1回だけ。1Hz の評価タイマで毎回撃たない。"""
        self._start_scan_keepalive()
        self._spin(0.5)   # estop_hw=True のまま疎通条件だけ先に整える

        self._estop_value = False                     # ここが唯一の立ち上がり
        self.pub_estop_hw.publish(Bool(data=False))
        deadline = time.time() + 3.5   # 1Hz 評価が複数回まわる長さ
        while time.time() < deadline and not self._link_ok_events():
            self._spin(0.2)
        assert self._link_ok_events(), 'evt.link_ok が一度も出ない'

        # さらに待っても増えない（gate が真のまま続いても、追加の evt.link_ok は出ない）。
        self._spin(2.5)
        count = len(self._link_ok_events())
        assert count == 1, f'evt.link_ok が立ち上がり以外でも出ている: {count} 件'


if __name__ == '__main__':
    unittest.main()
