"""
test_estop_init_link_resume_node.py
=====================================
1b-2（SG-A5）— state_manager と connectivity_checker を同時起動する試験。

起動直後（INIT/CHECK）に UI 非常停止を押して離すと INIT/CHECK に戻り、
疎通が揃えば evt.link_ok で IDLE まで進む（C-09b-init → T-INIT-01）。

connectivity_checker は `sim=True`（ESP32・必須ノードを除外し、`/scan` と
`/safety/estop_hw` だけで gate を制御する。test_connectivity_checker_node.py と
同じ単純化。問いは link_ok の再送タイミングであり ESP32 の実機条件ではない）。

注意（launch_testing の挙動）: このファイルは @pytest.mark.launch_test を持つため、
launch_testing の pytest プラグインがファイル全体を1つの pytest アイテムとして
収集し、その中の unittest.TestCase だけを実行する。モジュール直下の素の pytest
関数は収集されず黙って実行されないので、全部メソッドにする。
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


class TestEstopInitLinkResumeNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_estop_init_link_client')

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
    def test_init_estop_release_returns_to_check_then_idle(self):
        """SG-A5: INIT/CHECK で UI 非常停止 → 解除で INIT/CHECK に戻り、
        疎通が揃えば evt.link_ok で IDLE まで進む。

        疎通は「戻ったあと」に揃える（戻る前は gate が偽）。この順番なら
        立ち上がり検出（L-3）が必ず発火する。
        """
        assert self._wait_state('INIT', 'CHECK'), \
            '起動直後が INIT/CHECK ではない（前提が崩れている）'

        # 物理 E-Stop の状態だけ先に知らせる（押されていない）。/scan はまだ無い。
        self._start_hw_keepalive(value=False)
        self._spin(0.5)
        assert self._link_ok_events() == [], \
            '疎通が揃っていないのに evt.link_ok が出た'

        # 1) UI 非常停止 → ESTOP（押下前 INIT/CHECK をラッチ）。
        self.pub_ui_estop.publish(Bool(data=True))
        assert self._wait_state('ESTOP', 'NONE'), 'UI 非常停止で ESTOP に入らない'
        snap = self._latest()
        assert (snap.prev_mode, snap.prev_state) == ('INIT', 'CHECK'), \
            f'押下前がラッチされていない: {snap.prev_mode}/{snap.prev_state}'

        # 2) 解除 → INIT/CHECK に戻る（C-09b-init。IDLE に行かない）。
        self.pub_ui_estop.publish(Bool(data=False))
        assert self._wait_state('INIT', 'CHECK'), \
            '解除で INIT/CHECK に戻っていない（SG-A5。IDLE に入った）'

        # 3) 疎通が揃う → evt.link_ok → IDLE（T-INIT-01）。
        self._start_scan_keepalive()
        assert self._wait_state('IDLE', 'NONE', timeout=8.0), \
            'INIT に戻ったあと疎通が揃っても IDLE まで進まない'
        assert self._link_ok_events(), 'evt.link_ok が一度も出ていない'

    def test_init_estop_with_link_already_ok(self):
        """SG-A5 の境界: ESTOP に入る前に gate が真になっていた（evt.link_ok を
        出し終えていて二度と来ない）場合でも、INIT に戻ったあと IDLE まで進む。

        ブリーフの指示（「来ないなら止まって報告」）の検証そのもの。
        connectivity_checker の立ち上がり検出（L-3）は gate が真のままでは
        再送しないため、このテストは現実装では赤くなるはず。
        赤いままなら connectivity_checker 側の対処（spec 判断が要る）をせず、
        報告して止まる。
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

        # 3) 解除 → INIT/CHECK に戻る。
        self.pub_ui_estop.publish(Bool(data=False))
        assert self._wait_state('INIT', 'CHECK'), \
            '解除で INIT/CHECK に戻っていない（SG-A5）'

        # 4) 疎通は揃ったまま → IDLE まで進む（link_ok の再送が要る）。
        assert self._wait_state('IDLE', 'NONE', timeout=8.0), \
            'INIT に戻ったあと IDLE まで進まない（evt.link_ok が再送されない）'
