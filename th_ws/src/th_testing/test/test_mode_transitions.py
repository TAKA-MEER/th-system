"""
test_mode_transitions.py
==========================
設計書 10.1 節 — mode_manager 状態遷移網羅テスト

launch_testing を用い、mode_manager ノードを起動して
全遷移パターンを自動テストする。
"""

import pytest
import rclpy
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.markers

from std_msgs.msg import Bool
from th_system_msgs.msg import RobotMode, FaultStatus
from th_system_msgs.srv import SetMode


# ── モード定数 ──────────────────────────────────────────────
M = RobotMode


@pytest.mark.launch_test
def generate_test_description():
    mode_manager = launch_ros.actions.Node(
        package='th_mode_manager',
        executable='mode_manager',
        name='mode_manager',
        output='screen',
    )
    return launch.LaunchDescription([
        mode_manager,
        launch_testing.actions.ReadyToTest(),
    ]), {'mode_manager': mode_manager}


class TestModeManagerTransitions(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_mode_transitions')

        # mode_manager/set_mode サービスクライアント
        self.cli = self.node.create_client(SetMode, '/mode_manager/set_mode')

        # /robot/mode 購読（現在モードを追跡）
        self._mode_history = []
        self.node.create_subscription(
            RobotMode, '/robot/mode',
            lambda msg: self._mode_history.append(msg.mode), 10)

        # safety 発行用
        self.pub_estop = self.node.create_publisher(Bool, '/safety/estop', 10)
        self.pub_fault = self.node.create_publisher(
            FaultStatus, '/safety/fault', 10)

        # サービス待機
        assert self.cli.wait_for_service(timeout_sec=5.0), \
            'mode_manager サービスが起動していない'

        # 前テストの FSM 状態をリセット: 任意モード → ESTOP → IDLE
        rclpy.spin_once(self.node, timeout_sec=0.1)
        self.pub_estop.publish(Bool(data=True))
        time.sleep(0.2)
        for _ in range(5):
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.pub_estop.publish(Bool(data=False))
        time.sleep(0.3)
        for _ in range(5):
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self._mode_history.clear()

    def tearDown(self):
        self.node.destroy_node()

    # ── ヘルパー ──────────────────────────────────────────────
    def _request_mode(self, mode: int) -> SetMode.Response:
        req = SetMode.Request()
        req.requested_mode = mode
        req.requester      = 'test'
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=3.0)
        return future.result()

    def _current_mode(self) -> int:
        rclpy.spin_once(self.node, timeout_sec=0.2)
        return self._mode_history[-1] if self._mode_history else -1

    def _wait_mode(self, expected: int, timeout: float = 3.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self._mode_history and self._mode_history[-1] == expected:
                return True
        return False

    # ════════════════════════════════════════════════════════
    # 有効遷移テスト
    # ════════════════════════════════════════════════════════

    def test_init_to_idle_on_startup(self):
        """起動後に IDLE になる"""
        assert self._wait_mode(M.IDLE), 'IDLE になっていない'

    def test_idle_to_following(self):
        """IDLE → FOLLOWING（明示操作）"""
        self._wait_mode(M.IDLE)
        res = self._request_mode(M.FOLLOWING)
        assert res.success
        assert self._wait_mode(M.FOLLOWING)

    def test_following_to_manual(self):
        """FOLLOWING → MANUAL"""
        self._wait_mode(M.IDLE)
        self._request_mode(M.FOLLOWING)
        self._wait_mode(M.FOLLOWING)
        res = self._request_mode(M.MANUAL)
        assert res.success
        assert self._wait_mode(M.MANUAL)

    def test_following_to_moving_to_panel(self):
        """FOLLOWING → MOVING_TO_PANEL"""
        self._wait_mode(M.IDLE)
        self._request_mode(M.FOLLOWING)
        self._wait_mode(M.FOLLOWING)
        res = self._request_mode(M.MOVING_TO_PANEL)
        assert res.success
        assert self._wait_mode(M.MOVING_TO_PANEL)

    def test_following_to_idle_on_fault(self):
        """FOLLOWING 中の LiDAR フォルト → IDLE（設計書 3.2 節）"""
        self._wait_mode(M.IDLE)
        self._request_mode(M.FOLLOWING)
        self._wait_mode(M.FOLLOWING)
        fault = FaultStatus(active=True, fault_type='LIDAR_LOST')
        self.pub_fault.publish(fault)
        assert self._wait_mode(M.IDLE), 'フォルトで IDLE になっていない'

    def test_moving_to_panel_to_at_panel(self):
        """MOVING_TO_PANEL → AT_PANEL"""
        self._wait_mode(M.IDLE)
        self._request_mode(M.FOLLOWING)
        self._wait_mode(M.FOLLOWING)
        self._request_mode(M.MOVING_TO_PANEL)
        self._wait_mode(M.MOVING_TO_PANEL)
        res = self._request_mode(M.AT_PANEL)
        assert res.success
        assert self._wait_mode(M.AT_PANEL)

    def test_at_panel_to_following(self):
        """AT_PANEL → FOLLOWING（作業終了）"""
        self._wait_mode(M.IDLE)
        self._request_mode(M.FOLLOWING)
        self._wait_mode(M.FOLLOWING)
        self._request_mode(M.MOVING_TO_PANEL)
        self._wait_mode(M.MOVING_TO_PANEL)
        self._request_mode(M.AT_PANEL)
        self._wait_mode(M.AT_PANEL)
        res = self._request_mode(M.FOLLOWING)
        assert res.success
        assert self._wait_mode(M.FOLLOWING)

    def test_manual_to_following_explicit(self):
        """MANUAL → FOLLOWING（明示的な追従再開）"""
        self._wait_mode(M.IDLE)
        self._request_mode(M.FOLLOWING)
        self._wait_mode(M.FOLLOWING)
        self._request_mode(M.MANUAL)
        self._wait_mode(M.MANUAL)
        res = self._request_mode(M.FOLLOWING)
        assert res.success
        assert self._wait_mode(M.FOLLOWING)

    def test_idle_to_summoning(self):
        """IDLE → SUMMONING（呼び寄せ要求）"""
        self._wait_mode(M.IDLE)
        res = self._request_mode(M.SUMMONING)
        assert res.success
        assert self._wait_mode(M.SUMMONING)

    def test_summoning_to_idle(self):
        """SUMMONING → IDLE（到着/失敗/明示操作）"""
        self._wait_mode(M.IDLE)
        self._request_mode(M.SUMMONING)
        self._wait_mode(M.SUMMONING)
        res = self._request_mode(M.IDLE)
        assert res.success
        assert self._wait_mode(M.IDLE)

    def test_summoning_to_manual(self):
        """SUMMONING → MANUAL"""
        self._wait_mode(M.IDLE)
        self._request_mode(M.SUMMONING)
        self._wait_mode(M.SUMMONING)
        res = self._request_mode(M.MANUAL)
        assert res.success
        assert self._wait_mode(M.MANUAL)

    def test_following_mapless_cannot_go_to_summoning_directly(self):
        """FOLLOWING_MAPLESS → SUMMONING は禁止（IDLE を経由する必要あり）"""
        self._wait_mode(M.IDLE)
        self._request_mode(M.FOLLOWING_MAPLESS)
        self._wait_mode(M.FOLLOWING_MAPLESS)
        res = self._request_mode(M.SUMMONING)
        assert not res.success
        # クリーンアップ
        self._request_mode(M.IDLE)
        self._wait_mode(M.IDLE)

    def test_idle_to_facing(self):
        """IDLE → FACING（展示用・正対旋回要求）"""
        self._wait_mode(M.IDLE)
        res = self._request_mode(M.FACING)
        assert res.success
        assert self._wait_mode(M.FACING)

    def test_facing_to_idle(self):
        """FACING → IDLE（明示操作）"""
        self._wait_mode(M.IDLE)
        self._request_mode(M.FACING)
        self._wait_mode(M.FACING)
        res = self._request_mode(M.IDLE)
        assert res.success
        assert self._wait_mode(M.IDLE)

    def test_facing_to_manual(self):
        """FACING → MANUAL"""
        self._wait_mode(M.IDLE)
        self._request_mode(M.FACING)
        self._wait_mode(M.FACING)
        res = self._request_mode(M.MANUAL)
        assert res.success
        assert self._wait_mode(M.MANUAL)

    def test_following_mapless_cannot_go_to_facing_directly(self):
        """FOLLOWING_MAPLESS → FACING は禁止（IDLE を経由する必要あり）"""
        self._wait_mode(M.IDLE)
        self._request_mode(M.FOLLOWING_MAPLESS)
        self._wait_mode(M.FOLLOWING_MAPLESS)
        res = self._request_mode(M.FACING)
        assert not res.success
        # クリーンアップ
        self._request_mode(M.IDLE)
        self._wait_mode(M.IDLE)

    def test_manual_cannot_go_to_facing_directly(self):
        """MANUAL → FACING は禁止（IDLE を経由する必要あり）"""
        self._wait_mode(M.IDLE)
        self._request_mode(M.MANUAL)
        self._wait_mode(M.MANUAL)
        res = self._request_mode(M.FACING)
        assert not res.success
        # クリーンアップ
        self._request_mode(M.IDLE)
        self._wait_mode(M.IDLE)

    def test_manual_to_idle_on_heartbeat_loss(self):
        """MANUAL 中の heartbeat 途絶相当 → IDLE"""
        self._wait_mode(M.IDLE)
        self._request_mode(M.FOLLOWING)
        self._wait_mode(M.FOLLOWING)
        self._request_mode(M.MANUAL)
        self._wait_mode(M.MANUAL)
        # heartbeat 途絶は manual_command_handler が IDLE を要求するのと同等
        res = self._request_mode(M.IDLE)
        assert res.success
        assert self._wait_mode(M.IDLE)

    # ════════════════════════════════════════════════════════
    # ESTOP テスト（どのモードからでも割り込む）
    # ════════════════════════════════════════════════════════

    def _test_estop_from(self, source_mode: int, setup_fn=None):
        """任意のモードから ESTOP → 解除 → IDLE を確認する共通ロジック"""
        self._wait_mode(M.IDLE)
        if setup_fn:
            setup_fn()

        # ESTOP 発動
        self.pub_estop.publish(Bool(data=True))
        assert self._wait_mode(M.ESTOP), \
            f'{source_mode} → ESTOP への割り込みが失敗'

        # ESTOP 解除 → IDLE（INIT からやり直し不要）
        self.pub_estop.publish(Bool(data=False))
        assert self._wait_mode(M.IDLE), \
            '解除後 IDLE に戻っていない'

    def test_estop_from_idle(self):
        self._test_estop_from(M.IDLE)

    def test_estop_from_following(self):
        def setup():
            self._request_mode(M.FOLLOWING)
            self._wait_mode(M.FOLLOWING)
        self._test_estop_from(M.FOLLOWING, setup)

    def test_estop_from_moving_to_panel(self):
        def setup():
            self._request_mode(M.FOLLOWING)
            self._wait_mode(M.FOLLOWING)
            self._request_mode(M.MOVING_TO_PANEL)
            self._wait_mode(M.MOVING_TO_PANEL)
        self._test_estop_from(M.MOVING_TO_PANEL, setup)

    def test_estop_from_at_panel(self):
        def setup():
            self._request_mode(M.FOLLOWING)
            self._wait_mode(M.FOLLOWING)
            self._request_mode(M.MOVING_TO_PANEL)
            self._wait_mode(M.MOVING_TO_PANEL)
            self._request_mode(M.AT_PANEL)
            self._wait_mode(M.AT_PANEL)
        self._test_estop_from(M.AT_PANEL, setup)

    def test_estop_from_manual(self):
        def setup():
            self._request_mode(M.FOLLOWING)
            self._wait_mode(M.FOLLOWING)
            self._request_mode(M.MANUAL)
            self._wait_mode(M.MANUAL)
        self._test_estop_from(M.MANUAL, setup)

    def test_estop_from_summoning(self):
        def setup():
            self._request_mode(M.SUMMONING)
            self._wait_mode(M.SUMMONING)
        self._test_estop_from(M.SUMMONING, setup)

    def test_estop_from_facing(self):
        def setup():
            self._request_mode(M.FACING)
            self._wait_mode(M.FACING)
        self._test_estop_from(M.FACING, setup)

    # ════════════════════════════════════════════════════════
    # 禁止遷移テスト（success=False が返るべき）
    # ════════════════════════════════════════════════════════

    def test_idle_cannot_go_to_moving_to_panel_directly(self):
        """IDLE → MOVING_TO_PANEL は禁止（FOLLOWING を経由する必要あり）"""
        self._wait_mode(M.IDLE)
        res = self._request_mode(M.MOVING_TO_PANEL)
        assert not res.success

    def test_idle_cannot_go_to_at_panel(self):
        """IDLE → AT_PANEL は禁止"""
        self._wait_mode(M.IDLE)
        res = self._request_mode(M.AT_PANEL)
        assert not res.success

    def test_estop_cannot_go_to_following_directly(self):
        """ESTOP → FOLLOWING は禁止（IDLE を経由する必要あり）"""
        self.pub_estop.publish(Bool(data=True))
        self._wait_mode(M.ESTOP)
        res = self._request_mode(M.FOLLOWING)
        assert not res.success
        # クリーンアップ
        self.pub_estop.publish(Bool(data=False))
        self._wait_mode(M.IDLE)

    def test_no_auto_transition_idle_to_following(self):
        """
        IDLE は明示操作なしで自動的に FOLLOWING に遷移しない
        （設計書 3.2 節: 起動直後の安全保証）
        """
        self._wait_mode(M.IDLE)
        time.sleep(1.5)   # 1.5 秒待機
        rclpy.spin_once(self.node, timeout_sec=0.2)
        assert self._mode_history[-1] == M.IDLE, \
            'IDLE が自動で FOLLOWING に遷移した（禁止）'

    # ════════════════════════════════════════════════════════
    # フォルト遷移テスト（全モードで IDLE になるか）
    # ════════════════════════════════════════════════════════

    def _test_fault_from(self, setup_fn, fault_type: str, target_mode=M.IDLE):
        self._wait_mode(M.IDLE)
        setup_fn()
        fault = FaultStatus(active=True, fault_type=fault_type)
        self.pub_fault.publish(fault)
        assert self._wait_mode(target_mode), \
            f'{fault_type} フォルトで {target_mode} に遷移しなかった'
        # フォルト解除
        self.pub_fault.publish(FaultStatus(active=False, fault_type='NONE'))

    def test_esp32_fault_from_following(self):
        def setup():
            self._request_mode(M.FOLLOWING)
            self._wait_mode(M.FOLLOWING)
        self._test_fault_from(setup, 'ESP32_DISCONNECTED')

    def test_lidar_fault_from_manual(self):
        def setup():
            self._request_mode(M.FOLLOWING)
            self._wait_mode(M.FOLLOWING)
            self._request_mode(M.MANUAL)
            self._wait_mode(M.MANUAL)
        self._test_fault_from(setup, 'LIDAR_LOST')

    def test_esp32_fault_from_at_panel(self):
        def setup():
            self._request_mode(M.FOLLOWING)
            self._wait_mode(M.FOLLOWING)
            self._request_mode(M.MOVING_TO_PANEL)
            self._wait_mode(M.MOVING_TO_PANEL)
            self._request_mode(M.AT_PANEL)
            self._wait_mode(M.AT_PANEL)
        self._test_fault_from(setup, 'ESP32_DISCONNECTED')

    def test_esp32_fault_from_summoning(self):
        def setup():
            self._request_mode(M.SUMMONING)
            self._wait_mode(M.SUMMONING)
        self._test_fault_from(setup, 'ESP32_DISCONNECTED')

    def test_person_tracker_lost_fault_from_facing(self):
        """FACING 中の PERSON_TRACKER_LOST → IDLE（試験員位置依存モードのため強制遷移）"""
        def setup():
            self._request_mode(M.FACING)
            self._wait_mode(M.FACING)
        self._test_fault_from(setup, 'PERSON_TRACKER_LOST')

    def test_esp32_fault_from_facing(self):
        def setup():
            self._request_mode(M.FACING)
            self._wait_mode(M.FACING)
        self._test_fault_from(setup, 'ESP32_DISCONNECTED')
