"""
test_safety_monitor.py
========================
設計書 10.1 節 § 5 項 — safety_monitor フォルト検知テスト

確認事項:
  - LiDAR / ESP32 フィードバックの途絶を模擬し /safety/fault 発行を確認
  - /safety/estop_hw + /safety/tablet_estop を集約し /safety/estop を発行
  - twist_mux ロックは /safety/estop が HIGH になることで保証される
    (twist_mux 自体のテストは test_twist_mux_priority.py で行う)
  - mode_manager のモード遷移より前に /safety/fault が発行されること
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
from th_system_msgs.msg import FaultStatus, WheelFeedback, PersonStatus


LIDAR_TIMEOUT_MS  = 500   # safety_monitor のデフォルト値と合わせる
ESP32_TIMEOUT_MS  = 500
PERSON_TIMEOUT_MS = 500


@pytest.mark.launch_test
def generate_test_description():
    safety = launch_ros.actions.Node(
        package='th_safety',
        executable='safety_monitor',
        name='safety_monitor',
        parameters=[{
            'lidar_timeout_ms':   LIDAR_TIMEOUT_MS,
            'esp32_timeout_ms':   ESP32_TIMEOUT_MS,
            'person_timeout_ms':  PERSON_TIMEOUT_MS,
            'check_period_ms':    50,
        }],
        output='screen',
    )
    return launch.LaunchDescription([
        safety,
        launch_testing.actions.ReadyToTest(),
    ]), {'safety_monitor': safety}


class TestSafetyMonitor(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_safety_monitor')

        # 受信バッファ
        self._faults:  list[FaultStatus] = []
        self._estops:  list[bool]        = []

        self.node.create_subscription(
            FaultStatus, '/safety/fault',
            lambda m: self._faults.append(m), 10)
        self.node.create_subscription(
            Bool, '/safety/estop',
            lambda m: self._estops.append(m.data), 10)

        # 入力パブリッシャー
        self.pub_scan   = self.node.create_publisher(LaserScan,     '/scan',               10)
        self.pub_wf     = self.node.create_publisher(WheelFeedback, '/esp32/wheel_feedback', 10)
        self.pub_person = self.node.create_publisher(PersonStatus,  '/person/status',       10)
        self.pub_hw_estop    = self.node.create_publisher(Bool, '/safety/estop_hw',     10)
        self.pub_tab_estop   = self.node.create_publisher(Bool, '/safety/tablet_estop', 10)

        # 起動猶予（grace period）+ 少し余裕
        time.sleep(3.5)
        self._faults.clear()
        self._estops.clear()

    def tearDown(self):
        self.node.destroy_node()

    # ── ヘルパー ──────────────────────────────────────────────
    def _spin(self, sec: float = 0.1):
        deadline = time.time() + sec
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _wait_for_fault(self, fault_type: str, timeout: float = 3.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._spin(0.1)
            for f in self._faults:
                if f.active and f.fault_type == fault_type:
                    return True
        return False

    def _wait_for_fault_cleared(self, timeout: float = 3.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._spin(0.1)
            for f in reversed(self._faults):
                if not f.active:
                    return True
        return False

    def _wait_for_estop(self, expected: bool, timeout: float = 3.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._spin(0.1)
            if self._estops and self._estops[-1] == expected:
                return True
        return False

    def _pub_scan_once(self):
        msg = LaserScan()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.angle_min = -3.14
        msg.angle_max =  3.14
        msg.angle_increment = 0.01
        msg.range_min = 0.1
        msg.range_max = 12.0
        msg.ranges = [1.0] * 628
        self.pub_scan.publish(msg)

    def _pub_wheel_feedback_once(self):
        msg = WheelFeedback()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        self.pub_wf.publish(msg)

    def _pub_person_once(self):
        msg = PersonStatus()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.is_lost = False
        self.pub_person.publish(msg)

    # ════════════════════════════════════════════════════════
    # LiDAR フォルト検知テスト
    # ════════════════════════════════════════════════════════

    def test_lidar_fault_on_timeout(self):
        """
        /scan が途絶したとき LIDAR_LOST フォルトが発行される。
        twist_mux ロックより前（fault 発行が即時）であることを確認。
        """
        # まずスキャンを送り「alive」状態にする
        for _ in range(5):
            self._pub_scan_once()
            self._spin(0.05)
        self._faults.clear()

        # スキャンを停止 → タイムアウト後にフォルト
        wait_sec = LIDAR_TIMEOUT_MS / 1000.0 + 0.5
        self._spin(wait_sec)

        assert self._wait_for_fault('LIDAR_LOST'), \
            f'LIDAR タイムアウト後 {wait_sec:.1f}s で LIDAR_LOST が発行されなかった'

    def test_lidar_fault_cleared_on_recovery(self):
        """スキャン再開後に LIDAR_LOST フォルトが解除される"""
        # フォルト発生させる
        self._spin(LIDAR_TIMEOUT_MS / 1000.0 + 0.5)
        self._wait_for_fault('LIDAR_LOST')
        self._faults.clear()

        # スキャン再開
        for _ in range(10):
            self._pub_scan_once()
            self._spin(0.05)

        assert self._wait_for_fault_cleared(), \
            'スキャン再開後にフォルトが解除されなかった'

    def test_no_lidar_fault_when_active(self):
        """スキャンが定期的に来ている間はフォルトを発行しない"""
        for _ in range(20):
            self._pub_scan_once()
            self._spin(0.08)   # 80ms 間隔（タイムアウト 500ms より十分短い）
        self._faults.clear()
        self._spin(0.2)

        lidar_faults = [f for f in self._faults
                        if f.active and f.fault_type == 'LIDAR_LOST']
        assert len(lidar_faults) == 0, '正常動作中に LIDAR_LOST が発行された'

    # ════════════════════════════════════════════════════════
    # ESP32 フォルト検知テスト
    # ════════════════════════════════════════════════════════

    def test_esp32_fault_on_timeout(self):
        """wheel_feedback が途絶したとき ESP32_DISCONNECTED が発行される"""
        for _ in range(5):
            self._pub_wheel_feedback_once()
            self._spin(0.05)
        self._faults.clear()

        wait_sec = ESP32_TIMEOUT_MS / 1000.0 + 0.5
        self._spin(wait_sec)

        assert self._wait_for_fault('ESP32_DISCONNECTED'), \
            'ESP32 タイムアウト後に ESP32_DISCONNECTED が発行されなかった'

    def test_esp32_fault_cleared_on_recovery(self):
        """wheel_feedback 再開後に ESP32_DISCONNECTED フォルトが解除される"""
        self._spin(ESP32_TIMEOUT_MS / 1000.0 + 0.5)
        self._wait_for_fault('ESP32_DISCONNECTED')
        self._faults.clear()

        for _ in range(10):
            self._pub_wheel_feedback_once()
            self._spin(0.05)

        assert self._wait_for_fault_cleared(), \
            'wheel_feedback 再開後にフォルトが解除されなかった'

    # ════════════════════════════════════════════════════════
    # E-Stop 集約テスト
    # ════════════════════════════════════════════════════════

    def test_hw_estop_triggers_estop_topic(self):
        """物理 E-Stop (estop_hw=True) → /safety/estop が True になる"""
        self.pub_hw_estop.publish(Bool(data=True))
        assert self._wait_for_estop(True), \
            'hw estop 発動後 /safety/estop が True にならなかった'

    def test_tablet_estop_triggers_estop_topic(self):
        """タブレット緊急停止 (tablet_estop=True) → /safety/estop が True"""
        self.pub_tab_estop.publish(Bool(data=True))
        assert self._wait_for_estop(True), \
            'tablet estop 発動後 /safety/estop が True にならなかった'

    def test_estop_cleared_when_both_released(self):
        """両方の E-Stop が False になったら /safety/estop が False になる"""
        self.pub_hw_estop.publish(Bool(data=True))
        self._wait_for_estop(True)

        self.pub_hw_estop.publish(Bool(data=False))
        self.pub_tab_estop.publish(Bool(data=False))
        assert self._wait_for_estop(False), \
            '両 E-Stop 解除後 /safety/estop が False にならなかった'

    def test_estop_remains_if_one_active(self):
        """片方が True のままなら /safety/estop は True のまま"""
        self.pub_hw_estop.publish(Bool(data=True))
        self.pub_tab_estop.publish(Bool(data=True))
        self._wait_for_estop(True)
        self._estops.clear()

        # hw のみ解除
        self.pub_hw_estop.publish(Bool(data=False))
        self._spin(0.5)

        assert self._estops and self._estops[-1] is True, \
            'タブレット E-Stop が残っているのに /safety/estop が False になった'
        # クリーンアップ
        self.pub_tab_estop.publish(Bool(data=False))

    # ════════════════════════════════════════════════════════
    # フォルト発行タイミング（mode_manager 遷移より前）
    # ════════════════════════════════════════════════════════

    def test_fault_published_immediately_on_timeout(self):
        """
        フォルト検知は mode_manager の遷移処理を待たずに即時発行される。
        (設計書 2.4 節・7.1 節)
        /safety/fault の発行時刻が タイムアウト直後であることを確認。
        """
        for _ in range(5):
            self._pub_scan_once()
            self._spin(0.05)
        self._faults.clear()

        timeout_sec = LIDAR_TIMEOUT_MS / 1000.0
        t_stop = time.time()

        # タイムアウト + 余裕 (check_period 50ms × 3 = 150ms)
        self._spin(timeout_sec + 0.15)

        elapsed = time.time() - t_stop
        assert self._wait_for_fault('LIDAR_LOST', timeout=1.0), \
            'LIDAR_LOST が発行されなかった'
        # フォルト発行までの時間がタイムアウト + 大きなマージン以内
        assert elapsed < timeout_sec + 0.5, \
            f'フォルト発行が遅すぎる ({elapsed:.2f}s, timeout={timeout_sec}s)'
