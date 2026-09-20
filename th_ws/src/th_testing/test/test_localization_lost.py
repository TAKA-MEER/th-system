"""
test_localization_lost.py
============================
WP-SAFE-05 — LOCALIZATION_LOST の Docker launch テスト。

safety_monitor を `localization` ターゲットだけで起動し、偽の
/safety/localization_health を流して次を確認する:
  1. ok==false が続いたら LOCALIZATION_LOST (CRITICAL) が出る
  2. 単発の ok==false では出ない（保持時間）
  3. トピックが途絶えても出る

完了条件 4（ターゲット無効時は出ない）は起動条件が launch 側にあるため、
ホストの AST 試験（test_localization_health_launch.py）と Docker の
bringup 実起動で確認し、ここでは扱わない。

注意（CLAUDE.md の既知の癖）: launch_testing の pytest プラグインが
ファイル全体を1つのアイテムとして収集するため、試験は TestCase の
メソッドにする。モジュール直下の素の関数は収集されない。

メソッドの実行順は名前順に固定する（fault は safety 側にラッチされるため、
各試験は自分の前後始末を自分で行う）。
"""
import time
import unittest

import pytest
import rclpy

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions

from th_system_msgs.msg import FaultStatus, LocalizationHealth


CHECK_PERIOD_MS = 50
STARTUP_GRACE_SEC = 1
STARTUP_DEADLINE_SEC = 1
CRITICAL_HOLD_MS = 200
TOPIC_TIMEOUT_MS = 400


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
            'critical_fault_hold_ms': CRITICAL_HOLD_MS,
            'localization_topic_timeout_ms': TOPIC_TIMEOUT_MS,
            # このファイルが検証する対象だけを有効化する（F-5・O-7）。
            'enabled_targets': ['localization'],
        }],
        output='screen',
    )
    return launch.LaunchDescription([
        safety,
        launch_testing.actions.ReadyToTest(),
    ]), {'safety_monitor': safety}


class TestLocalizationLost(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_localization_lost')

        self._faults: list[FaultStatus] = []
        self.node.create_subscription(
            FaultStatus, '/safety/fault', self._faults.append, 10)

        self.pub_health = self.node.create_publisher(
            LocalizationHealth, '/safety/localization_health', 10)
        self._health_ok = True
        # test_a（途絶）は「何も流さない」が前提なので、キープアライブは
        # 各試験が明示的に有効化する。setUp では流さない。
        self._health_enabled = False
        self._health_timer = self.node.create_timer(0.2, self._publish_health)

        # 起動猶予 + 少し余裕（test_safety_monitor.py と同じ考え方）。
        # この spin で溜まった edge をバッファへ流し込む。
        self._spin(STARTUP_GRACE_SEC + 0.5)
        # blind clear はしない。test_a の fault はこの setup 中に既に edge
        # 発火している可能性があり（/safety/fault は変化時だけ出る）、
        # 消すと二度と観測できない。各試験は自分の前後始末を持つ。

    def tearDown(self):
        if getattr(self, '_health_timer', None) is not None:
            self._health_timer.cancel()
            self.node.destroy_timer(self._health_timer)
            self._health_timer = None
        self.node.destroy_node()

    # ── ヘルパー ──────────────────────────────────────────────
    def _spin(self, duration: float = 0.1):
        deadline = time.time() + duration
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _publish_health(self):
        if not self._health_enabled:
            return
        msg = LocalizationHealth()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.ok = self._health_ok
        msg.reason = '' if self._health_ok else 'stale'
        msg.transform_age_sec = 0.1 if self._health_ok else 10.0
        msg.node_present = True
        self.pub_health.publish(msg)

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

    def _wait_for_cleared(self, timeout: float = 5.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._spin(0.1)
            if any(not f.active for f in self._faults):
                return True
        return False

    # ════════════════════════════════════════════════════════
    def test_a_topic_dead_fires(self):
        """完了条件3: 何も publish しない（起動直後）→ 途絶で出る。"""
        hit = self._wait_for_active('LOCALIZATION_LOST', timeout=8.0)
        assert hit, 'トピック途絶で LOCALIZATION_LOST が出ない'
        assert hit[0].severity == 'CRITICAL', (
            f'LOCALIZATION_LOST が CRITICAL でない: {hit[0].severity}')

    def test_b_sustained_ng_fires(self):
        """完了条件1: ok==false が続いたら出る（CRITICAL）。"""
        # test_a のラッチを消す: ok=true を流して cleared を待つ。
        self._health_enabled = True
        self._health_ok = True
        assert self._wait_for_cleared(), 'fault がクリアされない'
        self._faults.clear()

        self._health_ok = False
        hit = self._wait_for_active('LOCALIZATION_LOST', timeout=8.0)
        assert hit, 'ok==false 継続で LOCALIZATION_LOST が出ない'
        assert hit[0].severity == 'CRITICAL'

    def test_c_single_blip_does_not_fire(self):
        """完了条件2: 単発の ok==false では出ない（保持時間）。"""
        self._health_enabled = True
        self._health_ok = True
        assert self._wait_for_cleared(), 'fault がクリアされない'
        self._faults.clear()

        # 1 回だけ ng を流してすぐ戻す（保持 200ms・周期 50ms に対する 100ms）。
        self._health_ok = False
        self._spin(0.1)
        self._health_ok = True
        self._spin(1.5)
        assert self._active_faults('LOCALIZATION_LOST') == [], (
            '単発の ok==false で LOCALIZATION_LOST が出た（保持時間が効いていない）')


if __name__ == '__main__':
    unittest.main()
