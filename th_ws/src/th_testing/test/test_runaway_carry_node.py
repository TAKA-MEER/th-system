"""
test_runaway_carry_node.py
==========================
物理非常停止が押されている間（手押し）は DRIVE_RUNAWAY を判定しない
（Spec-safety.md §3.5.3 末尾の小節。実機 2026-10-13: 手押しで DRIVE_RUNAWAY が出て
ESTOP に落ちた）。本物の safety_monitor を `runaway` だけで launch_testing 起動する。
入力は実機と同じ形: /cmd_vel はゼロ、/esp32/wheel_feedback は約 15 Hz で動いている
（人が押した）、/safety/estop_hw は std_msgs/Bool を 10 Hz で流し続ける。

  a. estop_hw=true のまま保持時間より十分長く流す → 出ない（fault にも active_faults にも）
  b. estop_hw=false に戻した直後は出ず、保持時間ぶん続いて初めて出る（0 から数え直し）
  c. すでに立っているフォルトは estop_hw=true にしても解除されない
  d. 通常（estop_hw=false）では出る。UI 非常停止（/safety/estop_ui）では止まらない

メソッドは名前順に実行され、b→c は状態を引き継ぐ（test_runaway_freshness_node.py と同じ流儀）。
launch_testing のため TestCase のメソッドにする。
"""
import time
import unittest

import pytest
import rclpy
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions

from th_system_msgs.msg import ActiveFaults, FaultStatus, WheelFeedback


CHECK_PERIOD_MS = 50
RUNAWAY_HOLD_MS = 1000
STARTUP_GRACE_SEC = 1
WHEEL_HZ = 15.0
CMD_HZ = 20.0
ESTOP_HZ = 10.0
PUSH_SPEED = 0.3  # 手押しの速さ（runaway_zero_threshold 0.08 を超える）


@pytest.mark.launch_test
def generate_test_description():
    safety = launch_ros.actions.Node(
        package='th_safety',
        executable='safety_monitor',
        name='safety_monitor',
        parameters=[{
            'check_period_ms': CHECK_PERIOD_MS,
            'startup_grace_sec': STARTUP_GRACE_SEC,
            'startup_deadline_sec': 1,
            'runaway_hold_ms': RUNAWAY_HOLD_MS,
            'runaway_feedback_stale_ms': 250,
            'enabled_targets': ['runaway'],
        }],
        output='screen',
    )
    return launch.LaunchDescription([
        safety,
        launch_testing.actions.ReadyToTest(),
    ]), {'safety_monitor': safety}


class TestRunawayCarryNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_runaway_carry_node')
        self._faults: list[FaultStatus] = []
        self.node.create_subscription(
            FaultStatus, '/safety/fault', self._faults.append, 10)
        self._active: list[ActiveFaults] = []
        self.node.create_subscription(
            ActiveFaults, '/safety/active_faults', self._active.append,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self.pub_cmd = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_wheel = self.node.create_publisher(
            WheelFeedback, '/esp32/wheel_feedback', 10)
        self.pub_hw = self.node.create_publisher(Bool, '/safety/estop_hw', 10)
        self.pub_ui = self.node.create_publisher(Bool, '/safety/estop_ui', 10)

        self._hw = True
        self._ui = False
        self._speed = PUSH_SPEED
        self._timers = [
            self.node.create_timer(1.0 / CMD_HZ, self._publish_cmd),
            self.node.create_timer(1.0 / WHEEL_HZ, self._publish_wheel),
            self.node.create_timer(1.0 / ESTOP_HZ, self._publish_estop),
        ]
        self._spin(STARTUP_GRACE_SEC + 0.5)

    def tearDown(self):
        for t in self._timers:
            t.cancel()
            self.node.destroy_timer(t)
        self.node.destroy_node()

    def _spin(self, duration: float):
        deadline = time.time() + duration
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def _publish_cmd(self):
        self.pub_cmd.publish(Twist())  # 指令ゼロ

    def _publish_wheel(self):
        msg = WheelFeedback()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.left_speed = self._speed
        msg.right_speed = self._speed
        self.pub_wheel.publish(msg)

    def _publish_estop(self):
        self.pub_hw.publish(Bool(data=self._hw))
        if self._ui:
            self.pub_ui.publish(Bool(data=True))

    def _fired(self):
        return [f for f in self._faults
                if f.active and f.fault_type == 'DRIVE_RUNAWAY']

    def _in_active_faults(self):
        # 最新の集合（transient_local で setUp でも最後の値が届く）
        return bool(self._active) and 'DRIVE_RUNAWAY' in self._active[-1].types

    def _wait_fired(self, timeout):
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._spin(0.05)
            if self._fired():
                return True
        return False

    # ════════════════════════════════════════════════════════
    def test_a_carry_never_fires(self):
        """物理非常停止のまま手押し（指令ゼロ・実測あり）→ 保持時間の 3 倍流しても出ない。"""
        self._hw = True
        self._spin(RUNAWAY_HOLD_MS / 1000.0 * 3)
        assert self._fired() == [], '手押し中に DRIVE_RUNAWAY が出た'
        assert not self._in_active_faults(), '手押し中に active_faults へ載った'

    def test_b_release_restarts_hold(self):
        """離した直後は出ず、保持時間ぶん続いて初めて出る（0 から数え直し）。"""
        self._hw = True
        self._spin(2.0)  # 押している間に十分溜まりうる時間
        self._faults.clear()
        self._hw = False
        t0 = time.time()
        # 保持時間の半分（500ms）までは出ない。保持が戻っていないと直ちに出る。
        self._spin(RUNAWAY_HOLD_MS / 1000.0 * 0.5)
        assert self._fired() == [], (
            f'離した {time.time() - t0:.2f}s 後に出た（保持時間が 0 に戻っていない）')
        assert self._wait_fired(timeout=2.0), '離したあと乖離が続いても出ない（判定が再開しない）'
        assert time.time() - t0 >= RUNAWAY_HOLD_MS / 1000.0 * 0.8

    def test_c_existing_fault_not_cleared_by_estop(self):
        """立っているフォルトは estop_hw=true にしても解除されない。"""
        # test_b で立った状態を引き継ぐ。setUp の spin 中も含め inactive が来ないこと。
        self._faults.clear()
        self._hw = True
        self._spin(2.0)
        cleared = [f for f in self._faults if not f.active]
        assert cleared == [], f'物理非常停止でフォルトが解除された: {cleared}'
        assert self._in_active_faults(), 'active_faults から DRIVE_RUNAWAY が消えた'

    def test_d_ui_estop_does_not_suspend(self):
        """通常（estop_hw=false）では出る。UI 非常停止だけでは判定は止まらない。"""
        # まず乖離を止めて解除する
        self._hw = False
        self._speed = 0.0
        self._spin(1.0)
        assert not self._in_active_faults(), '乖離を止めてもフォルトが解除されない'
        self._faults.clear()
        self._ui = True
        self._speed = PUSH_SPEED
        assert self._wait_fired(timeout=3.0), 'UI 非常停止中なのに判定が止まった'

    def test_e_hold_charged_before_press_is_reset(self):
        """押す前に溜めた保持時間は、押している間に 0 へ戻る（reset の効きを縛る）。

        乖離を保持時間の 75% ぶん流す（まだ出ない）→ 物理非常停止を押して保持時間の
        3 倍流し続ける → 離す。離した直後〜保持時間の半分は出ず、保持時間ぶん続いて
        初めて出る。押している間に reset しない実装だと、離した直後（残り 25% = 0.25 秒）
        で出てしまう。
        """
        hold = RUNAWAY_HOLD_MS / 1000.0
        # 前の試験の状態を片付ける（UI 非常停止の解除・乖離を止めて DRIVE_RUNAWAY を解除）
        self._ui = False
        self._hw = False
        self._speed = 0.0
        for _ in range(3):
            self.pub_ui.publish(Bool(data=False))
            self._spin(0.1)
        self._spin(1.0)
        assert not self._in_active_faults(), '前準備: DRIVE_RUNAWAY が解除されない'
        self._faults.clear()
        # 押す前に保持時間の 75% ぶん乖離を溜める
        self._speed = PUSH_SPEED
        self._spin(hold * 0.75)
        assert self._fired() == [], '前準備: 保持時間に達する前に出た'
        # 押している間（保持時間の 3 倍）も乖離を流し続ける
        self._hw = True
        self._spin(hold * 3)
        assert self._fired() == [], '押している間に出た'
        # 離す
        self._hw = False
        t0 = time.time()
        self._spin(hold * 0.5)
        assert self._fired() == [], (
            f'離した {time.time() - t0:.2f}s 後に出た（押す前に溜めた保持時間が戻っていない）')
        assert self._wait_fired(timeout=2.0), '離したあと乖離が続いても出ない'
        assert time.time() - t0 >= hold * 0.8, '保持時間ぶん続く前に出た'


if __name__ == '__main__':
    unittest.main()
