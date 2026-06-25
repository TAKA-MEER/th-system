"""
test_fault_detection.py
=========================
設計書 10.1 節 § 5 項（追加）—
safety_monitor + mode_manager の連携タイミングテスト

確認事項:
  - LiDAR / ESP32 途絶で /safety/fault が発行される「後に」
    mode_manager が IDLE へ遷移する（フォルト → ロック → モード遷移の順序）
  - /safety/fault が発行される前に /cmd_vel がゼロになることは
    test_twist_mux_priority.py 側で検証済み（twist_mux lock）
"""

import pytest
import rclpy
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions

from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from th_system_msgs.msg import FaultStatus, RobotMode, WheelFeedback
from th_system_msgs.srv import SetMode


LIDAR_TIMEOUT_MS = 300   # テスト用に短く設定
ESP32_TIMEOUT_MS = 300


@pytest.mark.launch_test
def generate_test_description():
    safety = launch_ros.actions.Node(
        package='th_safety',
        executable='safety_monitor',
        name='safety_monitor',
        parameters=[{
            'lidar_timeout_ms':  LIDAR_TIMEOUT_MS,
            'esp32_timeout_ms':  ESP32_TIMEOUT_MS,
            'person_timeout_ms': 5000,   # 人物追跡は今回テスト対象外
            'check_period_ms':   50,
        }],
        output='screen',
    )
    mm = launch_ros.actions.Node(
        package='th_mode_manager',
        executable='mode_manager',
        name='mode_manager',
        output='screen',
    )
    return launch.LaunchDescription([
        safety, mm,
        launch_testing.actions.ReadyToTest(),
    ]), {'safety_monitor': safety, 'mode_manager': mm}


class TestFaultDetectionTiming(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_fault_detection')

        self._fault_times: list[tuple[float, FaultStatus]] = []
        self._mode_times:  list[tuple[float, int]]         = []
        self._estop_times: list[tuple[float, bool]]        = []

        def on_fault(m):
            self._fault_times.append((time.time(), m))
        def on_mode(m):
            self._mode_times.append((time.time(), m.mode))
        def on_estop(m):
            self._estop_times.append((time.time(), m.data))

        self.node.create_subscription(FaultStatus, '/safety/fault',  on_fault, 10)
        self.node.create_subscription(RobotMode,   '/robot/mode',    on_mode,  10)
        self.node.create_subscription(Bool,        '/safety/estop',  on_estop, 10)

        self.pub_scan   = self.node.create_publisher(LaserScan,     '/scan',                10)
        self.pub_wf     = self.node.create_publisher(WheelFeedback, '/esp32/wheel_feedback', 10)
        self.pub_estop  = self.node.create_publisher(Bool,          '/safety/estop',         10)

        self.cli_mode = self.node.create_client(SetMode, '/mode_manager/set_mode')

        assert self.cli_mode.wait_for_service(timeout_sec=5.0)
        time.sleep(3.5)   # grace period

        self._fault_times.clear()
        self._mode_times.clear()
        self._estop_times.clear()

    def tearDown(self):
        self.node.destroy_node()

    # ── ヘルパー ──────────────────────────────────────────────
    def _spin(self, sec: float):
        deadline = time.time() + sec
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _set_mode(self, mode: int):
        req = SetMode.Request()
        req.requested_mode = mode
        req.requester      = 'test'
        future = self.cli_mode.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=3.0)
        return future.result()

    def _wait_mode(self, mode: int, timeout: float = 3.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._spin(0.05)
            if self._mode_times and self._mode_times[-1][1] == mode:
                return True
        return False

    def _pub_scan_once(self):
        msg = LaserScan()
        msg.header.stamp    = self.node.get_clock().now().to_msg()
        msg.angle_min       = -3.14
        msg.angle_max       =  3.14
        msg.angle_increment = 0.01
        msg.range_min       = 0.1
        msg.range_max       = 12.0
        msg.ranges          = [1.0] * 628
        self.pub_scan.publish(msg)

    def _pub_wf_once(self):
        msg = WheelFeedback()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        self.pub_wf.publish(msg)

    # ════════════════════════════════════════════════════════
    # タイミング: フォルト → モード遷移の順序確認
    # ════════════════════════════════════════════════════════

    def test_fault_precedes_idle_transition_on_lidar_loss(self):
        """
        LiDAR 途絶: /safety/fault (LIDAR_LOST) が発行された「後に」
        mode_manager が IDLE へ遷移する。

        設計書 7.1 節・7.2 節:
          twist_mux ロック（物理停止）→ mode_manager への通知（モード遷移）
          の順序を保証する。
        """
        # FOLLOWING 状態に移行
        self._wait_mode(RobotMode.IDLE)
        self._set_mode(RobotMode.FOLLOWING)
        self._wait_mode(RobotMode.FOLLOWING)

        # LiDAR を生かし alive 状態にする
        for _ in range(5):
            self._pub_scan_once()
            self._spin(0.05)

        self._fault_times.clear()
        self._mode_times.clear()

        # スキャン停止 → タイムアウト待ち
        timeout_sec = LIDAR_TIMEOUT_MS / 1000.0 + 0.3
        self._spin(timeout_sec)

        # フォルト発行を待つ
        deadline = time.time() + 3.0
        while time.time() < deadline:
            self._spin(0.05)
            active_faults = [t for t, f in self._fault_times if f.active]
            idle_entries  = [t for t, m in self._mode_times
                             if m == RobotMode.IDLE]
            if active_faults and idle_entries:
                break

        active_faults = [t for t, f in self._fault_times if f.active]
        idle_entries  = [t for t, m in self._mode_times if m == RobotMode.IDLE]

        assert active_faults, 'LIDAR_LOST フォルトが発行されなかった'
        assert idle_entries,  'フォルト後に IDLE 遷移が起きなかった'

        t_fault = active_faults[0]
        t_idle  = idle_entries[0]
        assert t_fault <= t_idle, (
            f'フォルト ({t_fault:.3f}s) が IDLE 遷移 ({t_idle:.3f}s) より後になった。'
            '順序が逆転している（設計書 2.4 節違反）')

    def test_fault_precedes_idle_transition_on_esp32_loss(self):
        """
        ESP32 途絶: ESP32_DISCONNECTED の後に IDLE 遷移する。
        """
        self._wait_mode(RobotMode.IDLE)
        self._set_mode(RobotMode.FOLLOWING)
        self._wait_mode(RobotMode.FOLLOWING)

        for _ in range(5):
            self._pub_wf_once()
            self._spin(0.05)

        self._fault_times.clear()
        self._mode_times.clear()

        timeout_sec = ESP32_TIMEOUT_MS / 1000.0 + 0.3
        self._spin(timeout_sec)

        deadline = time.time() + 3.0
        while time.time() < deadline:
            self._spin(0.05)
            if (any(f.active for _, f in self._fault_times) and
                    any(m == RobotMode.IDLE for _, m in self._mode_times)):
                break

        active_faults = [t for t, f in self._fault_times if f.active]
        idle_entries  = [t for t, m in self._mode_times if m == RobotMode.IDLE]

        assert active_faults, 'ESP32_DISCONNECTED フォルトが発行されなかった'
        assert idle_entries,  'フォルト後に IDLE 遷移が起きなかった'
        assert active_faults[0] <= idle_entries[0], '順序が逆転している'

    def test_fault_does_not_trigger_in_idle(self):
        """
        IDLE 中に LiDAR が途絶しても IDLE のまま（遷移は不要）。
        モード変化が起きないことを確認する。
        """
        self._wait_mode(RobotMode.IDLE)
        self._mode_times.clear()

        # スキャンなしで十分待つ
        self._spin(LIDAR_TIMEOUT_MS / 1000.0 + 0.5)
        self._spin(0.5)

        # IDLE からの離脱がないことを確認
        non_idle = [m for _, m in self._mode_times
                    if m != RobotMode.IDLE]
        assert len(non_idle) == 0, (
            f'IDLE 中に想定外のモード遷移が発生した: {non_idle}')

    def test_recovery_requires_explicit_operation(self):
        """
        フォルト→IDLE 遷移後、FOLLOWING への自動復帰は禁止。
        明示的な操作なしでは FOLLOWING に戻らない。
        （設計書 7.2 節・3.2 節）
        """
        self._wait_mode(RobotMode.IDLE)
        self._set_mode(RobotMode.FOLLOWING)
        self._wait_mode(RobotMode.FOLLOWING)

        # フォルト誘発（スキャン停止）
        self._spin(LIDAR_TIMEOUT_MS / 1000.0 + 0.3)
        self._wait_mode(RobotMode.IDLE, timeout=3.0)

        # フォルト解除（スキャン再開）
        for _ in range(10):
            self._pub_scan_once()
            self._spin(0.05)

        # 1 秒待って FOLLOWING への自動復帰がないことを確認
        self._mode_times.clear()
        self._spin(1.0)

        auto_following = [m for _, m in self._mode_times
                          if m == RobotMode.FOLLOWING]
        assert len(auto_following) == 0, (
            '明示操作なしに FOLLOWING へ自動復帰した（設計書 3.2 節違反）')
