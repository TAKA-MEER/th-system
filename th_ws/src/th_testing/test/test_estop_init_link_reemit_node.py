"""
test_estop_init_link_reemit_node.py
=====================================
1b-2（SG-A5 の境界）— state_manager と connectivity_checker を同時起動する試験。

ESTOP に入る前に gate が真になっていた（`evt.link_ok` を出し終えた）場合でも、
`INIT/CHECK` に戻ったあと `link_ok` が再送されて `IDLE` まで進む。
`connectivity_checker` は `/system/state` で `INIT` への到着を見て
立ち上がり検出のラッチをリセットする（L-3 の例外。`INIT` 以外ではリセットしない）。

別ファイル（`test_estop_init_link_resume_node.py`）とは launch を分ける。
同じ launch では state_manager が一度 `IDLE` に出ると `INIT` に戻れず、
起動直後の前提が崩れるため。

注意（launch_testing の挙動）: このファイルは @pytest.mark.launch_test を持つため、
launch_testing の pytest プラグインがファイル全体を1つの pytest アイテムとして
収集し、その中の unittest.TestCase だけを実行する。
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
from th_system_msgs.msg import StateEvent, SystemState
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)


SCAN_EXPECTED_POINTS = 8
ESP32_ALIVE_TIMEOUT_MS = 3000

JOG_LEASE_MS = 300
LINK_WAIT_TIMEOUT_MS = 60_000   # sys.link_timeout をテスト中に発火させない
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
        state_manager,
        connectivity_checker,
        launch_testing.actions.ReadyToTest(),
    ]), {'state_manager': state_manager,
         'connectivity_checker': connectivity_checker}


_STATE_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST)


class TestEstopInitLinkReemitNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_estop_init_link_reemit_client')

        self._state_history = []
        self.node.create_subscription(
            SystemState, '/system/state', self._state_history.append, _STATE_QOS)
        self._event_history = []
        self.node.create_subscription(
            StateEvent, '/system/event', self._event_history.append, 10)

        self.pub_scan = self.node.create_publisher(LaserScan, '/scan', 5)
        self.pub_estop_hw = self.node.create_publisher(Bool, '/safety/estop_hw', 10)
        self.pub_ui_estop = self.node.create_publisher(Bool, '/safety/estop_ui', 10)
        self._scan_timer = None
        self._hw_timer = None

    def tearDown(self):
        for t in (self._scan_timer, self._hw_timer):
            if t is not None:
                t.cancel()
                self.node.destroy_timer(t)
        self._scan_timer = None
        self._hw_timer = None
        self.node.destroy_node()

    # ── ヘルパー ──────────────────────────────────────────────
    def _spin(self, duration: float = 0.2):
        deadline = time.time() + duration
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _latest(self) -> SystemState:
        self._spin(0.2)
        assert self._state_history, '/system/state を1件も受信していない'
        return self._state_history[-1]

    def _wait_state(self, mode: str, state: str, timeout: float = 5.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._spin(0.1)
            if self._state_history:
                cur = self._state_history[-1]
                if cur.mode == mode and cur.state == state:
                    return True
        return False

    def _link_ok_events(self):
        return [e for e in self._event_history if e.event == 'evt.link_ok']

    def _start_hw_keepalive(self, value: bool = False):
        """実機の /safety/estop_hw は流れ続けるので、テストでも流し続ける。
        単発 publish はマッチング前に消えて届かないことがある
        （test_connectivity_checker_node.py の既知の癖）。"""
        self.pub_estop_hw.publish(Bool(data=value))
        self._hw_timer = self.node.create_timer(
            0.1, lambda: self.pub_estop_hw.publish(Bool(data=value)))

    def _start_scan_keepalive(self):
        def _pub():
            msg = LaserScan()
            msg.ranges = [1.0] * SCAN_EXPECTED_POINTS
            self.pub_scan.publish(msg)
        _pub()
        self._scan_timer = self.node.create_timer(0.2, _pub)

    # ════════════════════════════════════════════════════════
    def test_link_reemitted_after_return_to_init(self):
        """SG-A5 の境界: ESTOP に入る前に gate が真になっていた場合でも、
        `INIT/CHECK` に戻ったあと `evt.link_ok` が再送されて `IDLE` まで進む。

        `connectivity_checker` の立ち上がり検出（L-3）は gate が真のままでは
        再送しない。`/system/state` で `INIT` への到着を見てリセットする
        （L-3 の例外。`INIT` 以外の遷移ではリセットしない）。
        """
        assert self._wait_state('INIT', 'CHECK'), \
            '起動直後が INIT/CHECK ではない（前提が崩れている）'

        # 1) UI 非常停止 → ESTOP（まだ疎通は揃っていない）。
        self._start_hw_keepalive(value=False)
        self.pub_ui_estop.publish(Bool(data=True))
        assert self._wait_state('ESTOP', 'NONE'), 'UI 非常停止で ESTOP に入らない'

        # 2) ESTOP のあいだに疎通が揃う → evt.link_ok は ESTOP 中に捨てられる。
        self._start_scan_keepalive()
        deadline = time.time() + 5.0
        while time.time() < deadline and not self._link_ok_events():
            self._spin(0.2)
        assert self._link_ok_events(), \
            'ESTOP 中に evt.link_ok が一度も出ない（gate 自体が立たない）'
        assert self._latest().mode == 'ESTOP', \
            'ESTOP 中の evt.link_ok で ESTOP を離れてしまった'
        first_count = len(self._link_ok_events())

        # 3) 解除 → INIT/CHECK に戻る（C-09b-init）。
        self.pub_ui_estop.publish(Bool(data=False))
        assert self._wait_state('INIT', 'CHECK'), \
            '解除で INIT/CHECK に戻っていない（SG-A5）'

        # 4) 疎通は揃ったまま → link_ok が再送されて IDLE まで進む。
        assert self._wait_state('IDLE', 'NONE', timeout=8.0), \
            'INIT に戻ったあと IDLE まで進まない（evt.link_ok が再送されない）'
        assert len(self._link_ok_events()) > first_count, \
            'evt.link_ok の再送が無い（立ち上がりラッチがリセットされていない）'
