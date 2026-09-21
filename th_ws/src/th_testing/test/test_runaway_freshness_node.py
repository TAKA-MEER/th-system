"""
test_runaway_freshness_node.py
==============================
W-06 の② — DRIVE_RUNAWAY の鮮度ゲート（Spec-safety.md §3.5.3）の Docker launch テスト。

safety_monitor を `runaway` ターゲットだけで起動し、テスト側が /cmd_vel と
/esp32/wheel_feedback を流して次を確認する（静的な配線検査は
test_w06_runaway_freshness.py が持ち、ここではノード本体の振る舞いを見る。
純関数だけでは凍結中の解除（else 節の false 報告）を見逃すため）:
  a. 一度も実測が来ない → 発火しない（W-06 が起動のたびに誤発火した状況そのもの）
  b. 新鮮な実測で乖離が続く → 発火する（CRITICAL）
  c. 発火後に実測が止まる → 解除されない（凍結）

お手本は test_localization_lost.py（WP-SAFE-05。起動方法・パラメータ・
test_a_ 順序制御・setup-clear 競合の避け方）。

注意（CLAUDE.md の既知の癖）: launch_testing の pytest プラグインが
ファイル全体を1つのアイテムとして収集するため、試験は TestCase の
メソッドにする。モジュール直下の素の関数は収集されない。

メソッドの実行順は名前順に固定する（fault は safety 側にラッチされるため、
各試験は自分の前後始末を自分で行う。test_c は test_b で発火した状態を
引き継ぐ前提）。
"""
import time
import unittest

import pytest
import rclpy
from geometry_msgs.msg import Twist

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions

from th_system_msgs.msg import FaultStatus, WheelFeedback


CHECK_PERIOD_MS = 50
STARTUP_GRACE_SEC = 1
STARTUP_DEADLINE_SEC = 1
RUNAWAY_FEEDBACK_STALE_MS = 250
RUNAWAY_HOLD_MS = 500

CMD_LINEAR_X = 0.3
CMD_HZ = 20.0
WHEEL_HZ = 10.0


@pytest.mark.launch_test
def generate_test_description():
    safety = launch_ros.actions.Node(
        package='th_safety',
        executable='safety_monitor',
        name='safety_monitor',
        parameters=[{
            'check_period_ms': CHECK_PERIOD_MS,
            'startup_grace_sec': STARTUP_GRACE_SEC,
            'startup_deadline_sec': STARTUP_DEADLINE_SEC,
            'runaway_feedback_stale_ms': RUNAWAY_FEEDBACK_STALE_MS,
            'runaway_hold_ms': RUNAWAY_HOLD_MS,
            # このファイルが検証する対象だけを有効化する（F-5・O-7）。
            # runaway を戻すのは走行日だが、ここでは検知器単体を直接起動する。
            'enabled_targets': ['runaway'],
        }],
        output='screen',
    )
    return launch.LaunchDescription([
        safety,
        launch_testing.actions.ReadyToTest(),
    ]), {'safety_monitor': safety}


class TestRunawayFreshnessNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_runaway_freshness_node')

        self._faults: list[FaultStatus] = []
        self.node.create_subscription(
            FaultStatus, '/safety/fault', self._faults.append, 10)

        self.pub_cmd = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_wheel = self.node.create_publisher(
            WheelFeedback, '/esp32/wheel_feedback', 10)
        # test_a（未受信）は「wheel をまだ一度も出さない」が前提なので、
        # キープアライブは各試験が明示的に有効化する。cmd は最初から流す。
        self._cmd_enabled = True
        self._wheel_enabled = False
        self._cmd_timer = self.node.create_timer(1.0 / CMD_HZ, self._publish_cmd)
        self._wheel_timer = self.node.create_timer(1.0 / WHEEL_HZ, self._publish_wheel)

        # 起動猶予 + 少し余裕（test_localization_lost.py と同じ考え方）。
        # この spin で溜まった edge をバッファへ流し込む。
        self._spin(STARTUP_GRACE_SEC + 0.5)
        # blind clear はしない。setup 中に既に edge 発火している可能性があり
        # （/safety/fault は変化時だけ出る）、消すと二度と観測できない。
        # 各試験は自分の前後始末を持つ。

    def tearDown(self):
        for name in ('_cmd_timer', '_wheel_timer'):
            timer = getattr(self, name, None)
            if timer is not None:
                timer.cancel()
                self.node.destroy_timer(timer)
                setattr(self, name, None)
        self.node.destroy_node()

    # ── ヘルパー ──────────────────────────────────────────────
    def _spin(self, duration: float = 0.1):
        deadline = time.time() + duration
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _publish_cmd(self):
        if not self._cmd_enabled:
            return
        msg = Twist()
        msg.linear.x = CMD_LINEAR_X
        self.pub_cmd.publish(msg)

    def _publish_wheel(self):
        if not self._wheel_enabled:
            return
        msg = WheelFeedback()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.left_speed = 0.0
        msg.right_speed = 0.0
        self.pub_wheel.publish(msg)

    def _active_faults(self, fault_type: str):
        return [f for f in self._faults
                if f.active and f.fault_type == fault_type]

    def _wait_for_active(self, fault_type: str, timeout: float) -> list:
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._spin(0.1)
            hit = self._active_faults(fault_type)
            if hit:
                return hit
        return []

    # ════════════════════════════════════════════════════════
    def test_a_no_feedback_never_fires(self):
        """一度も実測が来ない → 発火しない。

        W-06 が起動のたびに誤発火した状況そのもの（新鮮な指令 0.3 対
        未受信の実測）。鮮度ゲートが無い実装・常時新鮮扱いの変異なら
        500ms 後に発火して赤くなる。
        """
        # setUp の 1.5s に加えて 2.0s。cmd だけが 3.5s 以上流れている。
        self._spin(2.0)
        assert self._active_faults('DRIVE_RUNAWAY') == [], (
            '実測未受信で DRIVE_RUNAWAY が出た（鮮度ゲートが効いていない）')

    def test_b_fresh_divergence_fires(self):
        """新鮮な実測で乖離が続く → 発火する（CRITICAL）。"""
        self._wheel_enabled = True
        self._faults.clear()
        hit = self._wait_for_active('DRIVE_RUNAWAY', timeout=1.5)
        assert hit, '新鮮な実測の乖離で DRIVE_RUNAWAY が出ない'
        assert hit[0].severity == 'CRITICAL', (
            f'DRIVE_RUNAWAY が CRITICAL でない: {hit[0].severity}')

    def test_c_stale_after_fire_stays_fired(self):
        """発火後に実測が止まる → 解除されない（凍結）。

        test_b で発火した状態を引き継ぐ。wheel は test_c では一度も
        出さない。setUp の spin 中も含めて inactive の FaultStatus が
        一度も来ないこと。凍結中の解除変異（else 節の false 報告）は
        setUp の spin 中（実測途絶から 250ms 後）に解除を publish する
        ため、ここで捕まる。blind clear はしない（setup-clear 競合:
        消すと解除の証拠ごと消えて変異を見逃す。test_localization_lost.py
        の setUp コメントと同じ理由）。
        """
        self._wheel_enabled = False
        deadline = time.time() + 2.0
        while time.time() < deadline:
            self._spin(0.1)
        cleared = [f for f in self._faults if not f.active]
        assert cleared == [], (
            f'発火後の実測途絶でフォルトが解除された: {cleared}')


if __name__ == '__main__':
    unittest.main()
