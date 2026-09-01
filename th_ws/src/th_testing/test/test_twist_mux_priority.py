"""
test_twist_mux_priority.py
============================
設計書 10.1 節 § 4 項 — twist_mux 優先度設定テスト

WP-SAFE-03 で twist_mux の出力先を /cmd_vel から /cmd_vel_muxed に変更し
（後段に obstacle_limiter が入り、/cmd_vel の publisher は obstacle_limiter だけに
なった）、入力トピック（旧 retreat・優先度20）も /cmd_vel_behavior へ改名した
（th_safety/config/twist_mux.yaml と同じ）。このテストは twist_mux 単体を
起動するため obstacle_limiter は含まない。

確認事項:
  1. /cmd_vel_nav と /cmd_vel_behavior が同時に値を持つ場合、
     /cmd_vel_behavior が /cmd_vel_muxed に出力される（behavior > nav）
  2. /safety/estop が True の時は lock が作動し /cmd_vel_muxed がゼロになる
  3. lock 解除後は挙動系指令が再び通る
"""

import pytest
import rclpy
import time
import math
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from th_system_msgs.msg import FaultStatus


@pytest.mark.launch_test
def generate_test_description():
    # twist_mux パッケージを直接起動
    twist_mux = launch_ros.actions.Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[{
            'topics': {
                'behavior': {
                    'topic':    '/cmd_vel_behavior',
                    'timeout':  0.5,
                    'priority': 20,
                },
                'nav': {
                    'topic':    '/cmd_vel_nav',
                    'timeout':  0.5,
                    'priority': 10,
                },
            },
            'locks': {
                'estop': {
                    'topic':    '/safety/estop',
                    'timeout':  0.5,
                    'priority': 255,
                },
            },
        }],
        remappings=[('cmd_vel_out', '/cmd_vel_muxed')],
        output='screen',
    )
    return launch.LaunchDescription([
        twist_mux,
        launch_testing.actions.ReadyToTest(),
    ]), {'twist_mux': twist_mux}


class TestTwistMuxPriority(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_twist_mux_priority')

        self._cmd_vel_history: list[Twist] = []
        self.node.create_subscription(
            Twist, '/cmd_vel_muxed',
            lambda m: self._cmd_vel_history.append(m), 10)

        self.pub_nav      = self.node.create_publisher(Twist, '/cmd_vel_nav',      10)
        self.pub_behavior = self.node.create_publisher(Twist, '/cmd_vel_behavior', 10)
        self.pub_estop    = self.node.create_publisher(Bool,  '/safety/estop',     10)

        # twist_mux のロックは「ロックトピックが timeout(0.5s) 途絶したら作動」する
        # フェイルセーフ設計。実機では safety_monitor が /safety/estop を 10Hz で
        # 送り続けるためロックは作動しない。テストでもこれを再現するため、
        # _spin() のたびに現在の estop 状態を送り続ける（1回だけだと 0.5s 後に
        # ロックが作動し /cmd_vel_muxed が遮断されてしまう）。
        self._estop_state = False
        time.sleep(1.0)
        self._spin(0.8)
        self._cmd_vel_history.clear()

    def tearDown(self):
        # E-Stop を確実に解除してテスト間干渉を防ぐ
        self._estop_state = False
        self.pub_estop.publish(Bool(data=False))
        time.sleep(0.3)
        self.node.destroy_node()

    # ── ヘルパー ──────────────────────────────────────────────
    def _spin(self, sec: float):
        deadline = time.time() + sec
        while time.time() < deadline:
            # safety_monitor を模して estop ロックを継続的に供給する
            self.pub_estop.publish(Bool(data=self._estop_state))
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _latest_cmd_vel(self) -> Twist | None:
        self._spin(0.2)
        return self._cmd_vel_history[-1] if self._cmd_vel_history else None

    def _is_zero(self, tw: Twist | None) -> bool:
        if tw is None:
            return True
        return (abs(tw.linear.x)  < 1e-6 and
                abs(tw.linear.y)  < 1e-6 and
                abs(tw.angular.z) < 1e-6)

    # ════════════════════════════════════════════════════════
    # 優先度テスト: behavior > nav
    # ════════════════════════════════════════════════════════

    def test_behavior_overrides_nav(self):
        """
        /cmd_vel_nav と /cmd_vel_behavior が同時に来た場合、
        /cmd_vel_muxed には behavior の値が出力される。
        """
        nav_cmd = Twist()
        nav_cmd.linear.x = 0.3

        beh_cmd = Twist()
        beh_cmd.linear.x = -0.15   # 後退 (behavior の識別子)

        # 両方を同時に送る
        for _ in range(5):
            self.pub_nav.publish(nav_cmd)
            self.pub_behavior.publish(beh_cmd)
            self._spin(0.05)

        out = self._latest_cmd_vel()
        assert out is not None, '/cmd_vel_muxed が受信できなかった'
        assert abs(out.linear.x - (-0.15)) < 0.05, (
            f'behavior が優先されていない: /cmd_vel_muxed.linear.x={out.linear.x:.3f}'
            f' (期待値≈-0.15)')

    def test_nav_used_when_no_behavior(self):
        """/cmd_vel_behavior が来ていない場合は /cmd_vel_nav が出力される"""
        nav_cmd = Twist()
        nav_cmd.linear.x = 0.26

        for _ in range(5):
            self.pub_nav.publish(nav_cmd)
            self._spin(0.05)

        out = self._latest_cmd_vel()
        assert out is not None
        assert abs(out.linear.x - 0.26) < 0.05, (
            f'nav が出力されていない: {out.linear.x:.3f}')

    def test_nav_resumes_after_behavior_timeout(self):
        """
        behavior の送信を停止すると twist_mux のタイムアウト(0.5s)で
        nav に自動切替わる。
        """
        nav_cmd = Twist(); nav_cmd.linear.x = 0.26
        beh_cmd = Twist(); beh_cmd.linear.x = -0.15

        # behavior を送り優先させる
        for _ in range(5):
            self.pub_nav.publish(nav_cmd)
            self.pub_behavior.publish(beh_cmd)
            self._spin(0.05)

        # behavior を停止し nav のみ送り続ける
        self._cmd_vel_history.clear()
        for _ in range(15):   # 約 1.5 秒（タイムアウト 0.5s より長く）
            self.pub_nav.publish(nav_cmd)
            self._spin(0.1)

        out = self._latest_cmd_vel()
        assert out is not None
        assert abs(out.linear.x - 0.26) < 0.05, (
            f'behavior タイムアウト後に nav に切り替わっていない: {out.linear.x:.3f}')

    # ════════════════════════════════════════════════════════
    # ロックテスト: estop → ゼロ強制
    # ════════════════════════════════════════════════════════

    def test_estop_lock_zeroes_output(self):
        """
        /safety/estop が True の間は、どの入力があっても
        /cmd_vel_muxed の出力がゼロになる。
        """
        nav_cmd = Twist(); nav_cmd.linear.x = 0.26
        beh_cmd = Twist(); beh_cmd.linear.x = -0.15

        # まず正常に出力されることを確認
        for _ in range(5):
            self.pub_nav.publish(nav_cmd)
            self.pub_behavior.publish(beh_cmd)
            self._spin(0.05)
        assert not self._is_zero(self._latest_cmd_vel()), \
            '正常時にゼロが出てしまっている'

        # E-Stop 発動（以降 _spin が True を送り続ける）
        # ロックが作動すると twist_mux は無出力になるため、作動を待ってから
        # 履歴をクリアする（作動前に漏れた古い速度が残らないように）。
        self._estop_state = True
        self.pub_estop.publish(Bool(data=True))
        self._spin(0.3)
        self._cmd_vel_history.clear()

        # E-Stop 中も両方送り続ける
        for _ in range(10):
            self.pub_nav.publish(nav_cmd)
            self.pub_behavior.publish(beh_cmd)
            self._spin(0.05)

        out = self._latest_cmd_vel()
        assert self._is_zero(out), (
            f'E-Stop 中に非ゼロ速度が出力された: '
            f'linear.x={out.linear.x if out else "None":.3f}')

    def test_output_resumes_after_estop_release(self):
        """E-Stop 解除後に速度出力が再開される"""
        nav_cmd = Twist(); nav_cmd.linear.x = 0.26

        # E-Stop 発動
        self._estop_state = True
        self.pub_estop.publish(Bool(data=True))
        self._spin(0.3)

        # E-Stop 解除
        self._estop_state = False
        self.pub_estop.publish(Bool(data=False))
        self._cmd_vel_history.clear()

        for _ in range(10):
            self.pub_nav.publish(nav_cmd)
            self._spin(0.05)

        out = self._latest_cmd_vel()
        assert not self._is_zero(out), \
            'E-Stop 解除後も速度が出力されない'

    def test_behavior_still_zero_during_estop(self):
        """
        behavior の優先度がどれだけ高くても、
        E-Stop lock 中は /cmd_vel_muxed がゼロになる。
        （lock が全入力より優先される設計を検証）
        """
        beh_cmd = Twist(); beh_cmd.linear.x = -0.15

        self._estop_state = True
        self.pub_estop.publish(Bool(data=True))
        self._spin(0.3)                 # ロックが作動するまで待つ
        self._cmd_vel_history.clear()

        for _ in range(10):
            self.pub_behavior.publish(beh_cmd)
            self._spin(0.05)

        out = self._latest_cmd_vel()
        assert self._is_zero(out), (
            'E-Stop lock 中に behavior が通過してしまった')
