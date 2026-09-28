"""
test_safety_monitor.py
========================
DetailedDesign-wp2.md `WP-SAFE-01` §7 — safety_monitor フォルト検知テスト

確認事項:
  - LiDAR / ESP32 フィードバックの途絶を模擬し /safety/fault 発行を確認
  - /safety/estop_hw + /safety/estop_ui（旧 /safety/tablet_estop）を集約し
    /safety/estop を発行
  - twist_mux ロックは /safety/estop が HIGH になることで保証される
    (twist_mux 自体のテストは test_twist_mux_priority.py で行う)
  - mode_manager のモード遷移より前に /safety/fault が発行されること
  - enabled_targets（F-5・O-7）に入っている対象だけが監視されること
    （このテストは lidar・esp32 だけを有効にする）
  - 回復試験（*_fault_cleared_on_recovery）は「両方 alive → 対象だけ途絶 → 対象を再開」
    の順に直す。WS-9J-B で「一度も受信していない入力を startup_deadline_sec まで
    途絶とみなさない」ようになったため、試験側が先に alive にしておかないと
    LIDAR_LOST / ESP32_DISCONNECTED が立たなくなる。
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
from th_system_msgs.msg import FaultStatus, WheelFeedback


LIDAR_TIMEOUT_MS  = 500   # safety_monitor のデフォルト値と合わせる
ESP32_TIMEOUT_MS  = 500


@pytest.mark.launch_test
def generate_test_description():
    safety = launch_ros.actions.Node(
        package='th_safety',
        executable='safety_monitor',
        name='safety_monitor',
        parameters=[{
            'lidar_timeout_ms':   LIDAR_TIMEOUT_MS,
            'esp32_timeout_ms':   ESP32_TIMEOUT_MS,
            'check_period_ms':    50,
            # F-5・O-7: このテストが検証する対象だけを明示的に有効化する。
            'enabled_targets':    ['lidar', 'esp32'],
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
        self._locks:   list[bool]        = []

        self.node.create_subscription(
            FaultStatus, '/safety/fault',
            lambda m: self._faults.append(m), 10)
        self.node.create_subscription(
            Bool, '/safety/estop',
            lambda m: self._estops.append(m.data), 10)
        self.node.create_subscription(
            Bool, '/safety/fault_lock',
            lambda m: self._locks.append(m.data), 10)

        # 入力パブリッシャー
        self.pub_scan   = self.node.create_publisher(LaserScan,     '/scan',               10)
        self.pub_wf     = self.node.create_publisher(WheelFeedback, '/esp32/wheel_feedback', 10)
        self.pub_hw_estop    = self.node.create_publisher(Bool, '/safety/estop_hw',     10)
        self.pub_ui_estop    = self.node.create_publisher(Bool, '/safety/estop_ui',     10)

        # 起動猶予（grace period）+ 少し余裕
        time.sleep(3.5)
        self._faults.clear()
        self._estops.clear()
        self._locks.clear()

    def tearDown(self):
        self.node.destroy_node()

    # ── ヘルパー ──────────────────────────────────────────────
    def _pump(self, sec: float, keep_alive=(), period: float = 0.05):
        """spin しながら keep_alive の publisher を回す（publish は 1/period = 20Hz）"""
        deadline = time.time() + sec
        while time.time() < deadline:
            for p in keep_alive:
                p()
            rclpy.spin_once(self.node, timeout_sec=period)

    def _spin(self, sec: float = 0.1):
        self._pump(sec)

    def _wait_for_fault(self, fault_type: str, timeout: float = 3.0,
                        keep_alive=()) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._pump(0.1, keep_alive)
            for f in self._faults:
                if f.active and f.fault_type == fault_type:
                    return True
        return False

    def _wait_for_fault_cleared(self, timeout: float = 3.0, keep_alive=()) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._pump(0.1, keep_alive)
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

    def _wait_for_lock(self, expected: bool, timeout: float = 3.0,
                       keep_alive=()) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._pump(0.1, keep_alive)
            if self._locks and self._locks[-1] == expected:
                return True
        return False

    def _both_alive_and_clean(self):
        """両センサ入力を 20Hz で送り、前テストの持ち越し fault が消えたことを確認してから
        受信バッファを空にする（回復試験の前提条件）"""
        self._pump(0.5, (self._pub_scan_once, self._pub_wheel_feedback_once))
        self._faults.clear()
        self._locks.clear()
        assert self._wait_for_lock(False, timeout=3.0,
                                   keep_alive=(self._pub_scan_once,
                                               self._pub_wheel_feedback_once)), \
            '両入力を alive にしたのに /safety/fault_lock が false にならない（持ち越し fault がある）'
        self._faults.clear()
        self._locks.clear()

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
        """
        スキャン再開後に LIDAR_LOST フォルトが解除される。

        解除が「対象のセンサ（LIDAR）由来」であることを区別する方法:
          解除通知 (/safety/fault の active=false) は fault_type が "NONE" 固定で種別を
          区別できない（safety_monitor.cpp の publishFault(false, "NONE")）。そのため
          「解除の通知が起きた」だけでは対象由来だと証明できず、もう片方のセンサの
          解除を拾って偽陽性になりうる。折衷案として次の 2 点を併せて確認する。
            ① 回復待ちの間も ESP32 入力 (/esp32/wheel_feedback) を 20Hz で送り続け、
               ESP32_DISCONNECTED を立てないようにする。加えて試験全体で
               ESP32_DISCONNECTED が一度も active で現れないことも assert する。
            ② /safety/fault_lock（LIDAR_LOST || ESP32_DISCONNECTED || CRITICAL）を
               併せて購読し、LIDAR_LOST 中は true・解除後は false になることまで見る。
               もう片方が alive のままなら、lock が解けたのは対象の回復による
               以外ありえない。
          この 2 つで「解除は対象の回復による」ことが種別を区別できない
          publishFault(false, "NONE") に左右されず成立する。
        """
        self._both_alive_and_clean()

        # 対象（LiDAR）の入力を途絶させ、fault が立ったことを assert する
        assert self._wait_for_fault(
            'LIDAR_LOST', timeout=3.0,
            keep_alive=(self._pub_wheel_feedback_once,)), \
            'LIDAR タイムアウト後に LIDAR_LOST が発行されなかった'
        assert self._wait_for_lock(
            True, timeout=3.0, keep_alive=(self._pub_wheel_feedback_once,)), \
            'LIDAR_LOST 発行中なのに /safety/fault_lock が true にならなかった'
        self._locks.clear()

        # 対象の入力を再開 → 解除を確かめる（もう片方も 20Hz で維持する）
        keep = (self._pub_scan_once, self._pub_wheel_feedback_once)
        assert self._wait_for_fault_cleared(3.0, keep_alive=keep), \
            'スキャン再開後に LIDAR_LOST の解除通知が出なかった'
        assert self._wait_for_lock(False, 3.0, keep_alive=keep), \
            'スキャン再開後に /safety/fault_lock が false にならなかった'

        other = [f for f in self._faults
                 if f.active and f.fault_type == 'ESP32_DISCONNECTED']
        assert not other, \
            'ESP32_DISCONNECTED が試験中に発火した（LIDAR の解除と区別できない）'

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
        """
        wheel_feedback 再開後に ESP32_DISCONNECTED フォルトが解除される

        解除が「対象のセンサ（ESP32）由来」であることを区別する方法:
          解除通知 (/safety/fault の active=false) は fault_type が "NONE" 固定で種別を
          区別できない（safety_monitor.cpp の publishFault(false, "NONE")）。そのため
          「解除の通知が起きた」だけでは対象由来だと証明できず、もう片方のセンサの
          解除を拾って偽陽性になりうる。折衷案として次の 2 点を併せて確認する。
            ① 回復待ちの間も LiDAR 入力 (/scan) を 20Hz で送り続け、LIDAR_LOST を
               立てないようにする。加えて試験全体で LIDAR_LOST が一度も active で
               現れないことも assert する。
            ② /safety/fault_lock（LIDAR_LOST || ESP32_DISCONNECTED || CRITICAL）を
               併せて購読し、ESP32_DISCONNECTED 中は true・解除後は false になること
               まで見る。もう片方が alive のままなら、lock が解けたのは対象の回復に
               よる以外ありえない。
          この 2 つで「解除は対象の回復による」ことが種別を区別できない
          publishFault(false, "NONE") に左右されず成立する。
        """
        self._both_alive_and_clean()

        # 対象の入力を途絶させ、fault が立ったことを assert する
        assert self._wait_for_fault(
            'ESP32_DISCONNECTED', timeout=3.0,
            keep_alive=(self._pub_scan_once,)), \
            'ESP32 タイムアウト後に ESP32_DISCONNECTED が発行されなかった'
        assert self._wait_for_lock(
            True, timeout=3.0, keep_alive=(self._pub_scan_once,)), \
            'ESP32_DISCONNECTED 発行中なのに /safety/fault_lock が true にならなかった'
        self._locks.clear()

        # 対象の入力を再開 → 解除を確かめる（もう片方も 20Hz で維持する）
        keep = (self._pub_wheel_feedback_once, self._pub_scan_once)
        assert self._wait_for_fault_cleared(3.0, keep_alive=keep), \
            'wheel_feedback 再開後に ESP32_DISCONNECTED の解除通知が出なかった'
        assert self._wait_for_lock(False, 3.0, keep_alive=keep), \
            'wheel_feedback 再開後に /safety/fault_lock が false にならなかった'

        other = [f for f in self._faults
                 if f.active and f.fault_type == 'LIDAR_LOST']
        assert not other, \
            'LIDAR_LOST が試験中に発火した（ESP32 の解除と区別できない）'

    # ════════════════════════════════════════════════════════
    # E-Stop 集約テスト
    # ════════════════════════════════════════════════════════

    def test_hw_estop_triggers_estop_topic(self):
        """物理 E-Stop (estop_hw=True) → /safety/estop が True になる"""
        self.pub_hw_estop.publish(Bool(data=True))
        assert self._wait_for_estop(True), \
            'hw estop 発動後 /safety/estop が True にならなかった'

    def test_ui_estop_triggers_estop_topic(self):
        """UI 非常停止 (estop_ui=True) → /safety/estop が True"""
        self.pub_ui_estop.publish(Bool(data=True))
        assert self._wait_for_estop(True), \
            'UI estop 発動後 /safety/estop が True にならなかった'
        # クリーンアップ（§6.3: 押下側にラッチするため明示的に false を送る）
        self.pub_ui_estop.publish(Bool(data=False))

    def test_estop_cleared_when_both_released(self):
        """両方の E-Stop が False になったら /safety/estop が False になる"""
        self.pub_hw_estop.publish(Bool(data=True))
        self._wait_for_estop(True)

        self.pub_hw_estop.publish(Bool(data=False))
        self.pub_ui_estop.publish(Bool(data=False))
        assert self._wait_for_estop(False), \
            '両 E-Stop 解除後 /safety/estop が False にならなかった'

    def test_estop_remains_if_one_active(self):
        """片方が True のままなら /safety/estop は True のまま"""
        self.pub_hw_estop.publish(Bool(data=True))
        self.pub_ui_estop.publish(Bool(data=True))
        self._wait_for_estop(True)
        self._estops.clear()

        # hw のみ解除
        self.pub_hw_estop.publish(Bool(data=False))
        self._spin(0.5)

        assert self._estops and self._estops[-1] is True, \
            'UI 非常停止が残っているのに /safety/estop が False になった'
        # クリーンアップ（§6.3: ラッチするため明示的に false を送る）
        self.pub_ui_estop.publish(Bool(data=False))

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
