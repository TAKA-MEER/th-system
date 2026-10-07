"""
test_explore_hold_node.py
=========================
W-01 P5 — 全域ローカライズの探索中の保留（Spec-safety.md §3.5.0 の拡張、
options §6）の Docker launch テスト。

localization_health ノードを短い上限・猶予で起動し、テスト側が
/replay_runner/localizing を流して次を確認する（静的な配線検査は
test_localization_health_launch.py が持ち、ここではノード本体の振る舞いを見る）:
  a. 探索の知らせ無し → ok=False, reason='node_down'（従来どおり）
  b. true を publish → ok=True, reason='global_localizing'
  c. true を publish し続けて上限超過 → ok=False, reason='restart_timeout'
  d. 保留中は実機の階段でも jump が出ない（B′ の前回値を捨てている）
  e. 対照: 知らせ無しで同じ階段を与えたら jump が出る

お手本は test_planned_restart_node.py（起動方法・パラメータ・
test_a_ 順序制御・setup-clear 競合の避け方・2 台構成）。
"""
import time
import unittest
import math

import pytest
import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from std_msgs.msg import Bool

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions

from th_system_msgs.msg import LocalizationHealth, SystemState


WARMUP_MS = 1000
STALE_MS = 2000
JUMP_WINDOW_MS = 500
EXPLORE_MAX_MS = 3000
POST_RESTART_GRACE_MS = 1500

# ノードの購読 QoS（発行側 status_qos）に合わせる。合わせないと届かない
# （test_planned_restart_node.py と同じ罠）。
_EXPLORE_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

# /system/state を監視するモード（REPLAY/RUN）で出し続ける。出さないと
# mode gate が inactive を配信し、a〜e の検査が黙って何も検査しなくなる。
_STATE_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST)


@pytest.mark.launch_test
def generate_test_description():
    health = launch_ros.actions.Node(
        package='th_state',
        executable='localization_health.py',
        name='explore_hold_health',
        parameters=[{
            'localization_stale_ms': STALE_MS,
            'localization_warmup_ms': WARMUP_MS,
            # 存在しないノード名。test_a で node_down を出すため。
            'localization_expected_nodes': ['__no_such_estimator__'],
            'jump_window_ms': JUMP_WINDOW_MS,
            'jump_translation_m': 1.0,
            'jump_rotation_rad': 0.5,
            'localization_restart_max_ms': EXPLORE_MAX_MS,
            'localization_post_restart_grace_ms': POST_RESTART_GRACE_MS,
        }],
        remappings=[('/safety/localization_health', '/safety/explore_health')],
        output='screen',
    )
    # d/e 用の 2 台目。expected_nodes と TF の要否が違うため別インスタンス。
    health_tf = launch_ros.actions.Node(
        package='th_state',
        executable='localization_health.py',
        name='explore_hold_health_tf',
        parameters=[{
            'localization_stale_ms': 5000,
            'localization_warmup_ms': WARMUP_MS,
            # 1 台目のノード名。起動直後から在席が確定する。
            'localization_expected_nodes': ['explore_hold_health'],
            'jump_window_ms': 200,
            'jump_translation_m': 1.0,
            'jump_rotation_rad': 0.5,
            'localization_restart_max_ms': EXPLORE_MAX_MS,
            'localization_post_restart_grace_ms': POST_RESTART_GRACE_MS,
        }],
        remappings=[('/safety/localization_health', '/safety/explore_health_tf')],
        output='screen',
    )
    return launch.LaunchDescription([
        health,
        health_tf,
        launch_testing.actions.ReadyToTest(),
    ]), {'explore_hold_health': health, 'explore_hold_health_tf': health_tf}


class TestExploreHoldNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_explore_hold_node')

        self._health: list[LocalizationHealth] = []
        self.node.create_subscription(
            LocalizationHealth, '/safety/explore_health',
            self._health.append, 10)
        self._health2: list[LocalizationHealth] = []
        self.node.create_subscription(
            LocalizationHealth, '/safety/explore_health_tf',
            self._health2.append, 10)

        self.pub_explore = self.node.create_publisher(
            Bool, '/replay_runner/localizing', _EXPLORE_QOS)
        self._explore_enabled = False
        self._explore_value = True
        self._explore_timer = self.node.create_timer(0.5, self._publish_explore)

        self.pub_state = self.node.create_publisher(
            SystemState, '/system/state', _STATE_QOS)
        self._state_timer = self.node.create_timer(0.5, self._publish_state)

        # d/e 用の TF 階段。map→odom を動かす。10Hz で現在値を出し続ける。
        self._tf_x = 0.0
        self._tf_yaw = 0.0
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self.node)
        self._tf_timer = self.node.create_timer(0.1, self._publish_tf)

        self._spin(WARMUP_MS / 1000.0 + 0.5)

    def tearDown(self):
        if getattr(self, '_explore_timer', None) is not None:
            self._explore_timer.cancel()
            self.node.destroy_timer(self._explore_timer)
            self._explore_timer = None
        if getattr(self, '_state_timer', None) is not None:
            self._state_timer.cancel()
            self.node.destroy_timer(self._state_timer)
            self._state_timer = None
        if getattr(self, '_tf_timer', None) is not None:
            self._tf_timer.cancel()
            self.node.destroy_timer(self._tf_timer)
            self._tf_timer = None
        self.node.destroy_node()

    # ── ヘルパー ──────────────────────────────────────────────
    def _spin(self, duration: float = 0.1):
        deadline = time.time() + duration
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _publish_state(self):
        """監視するモード（REPLAY/RUN）を出し続ける。"""
        msg = SystemState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.mode = 'REPLAY'
        msg.state = 'RUN'
        self.pub_state.publish(msg)

    def _publish_explore(self):
        if not self._explore_enabled:
            return
        self.pub_explore.publish(Bool(data=self._explore_value))

    def _wait_for_reason(self, reason: str, ok: bool, timeout: float):
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._spin(0.1)
            hit = [m for m in self._health if m.reason == reason and m.ok == ok]
            if hit:
                return hit
        return []

    def _reset_explore(self):
        """探索状態を false に戻し、通常検知（node_down）に戻った証拠を
        待ってからバッファを捨てる。c の開始合図。"""
        self._health.clear()
        self._explore_value = False
        self._explore_enabled = True
        try:
            hit = self._wait_for_reason('node_down', False, timeout=6.0)
        finally:
            self._explore_enabled = False
        assert hit, 'reset 後に node_down に戻らない'
        self._health.clear()

    # ════════════════════════════════════════════════════════
    def test_a_no_notice_detects_node_down(self):
        """知らせが無いときは従来どおり検知する（node_down が出る）。"""
        self._spin(1.0)
        hit = [m for m in self._health
               if m.reason == 'node_down' and not m.ok]
        assert hit, '推定ノード不在で node_down が出ない'

    def test_b_exploring_while_notified(self):
        """true を publish → ok=True, reason='global_localizing'。"""
        self._explore_value = True
        self._explore_enabled = True
        try:
            hit = self._wait_for_reason('global_localizing', True, timeout=3.0)
        finally:
            self._explore_enabled = False
        assert hit, '探索通知中に global_localizing が出ない'

    def test_c_timeout_while_true_continues(self):
        """true を publish し続けて上限超過 → restart_timeout。"""
        self._reset_explore()
        self._explore_value = True
        self._explore_enabled = True
        try:
            hit = self._wait_for_reason('restart_timeout', False, timeout=7.0)
        finally:
            self._explore_enabled = False
        assert hit, '上限超過で restart_timeout が出ない'

    # ── d/e 用の TF 階段 ─────────────────────────────────────
    def _publish_tf(self):
        """map→odom の現在値（self._tf_x, self._tf_yaw）を 10Hz で出す。"""
        t = TransformStamped()
        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = float(self._tf_x)
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        half = float(self._tf_yaw) / 2.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(half)
        t.transform.rotation.w = math.cos(half)
        self._tf_broadcaster.sendTransform(t)

    def _wait_for_reason2(self, reason: str, ok: bool, timeout: float):
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._spin(0.1)
            hit = [m for m in self._health2 if m.reason == reason and m.ok == ok]
            if hit:
                return hit
        return []

    def _reset_explore2(self):
        """2 台目を「探索の知らせなし・通常評価の ok」に戻す。
        c は true のまま終わる（2 台目も同じ知らせを受けて上限超過の
        まま残る）ため、false を出して edge を受理させ、通常評価の
        ok（reason 空）に戻ったことを待つ。"""
        self._health2.clear()
        self._explore_value = False
        self._explore_enabled = True
        try:
            deadline = time.time() + 8.0
            hit = []
            while time.time() < deadline and not hit:
                self._spin(0.1)
                hit = [m for m in self._health2
                       if m.ok and m.reason == '' and m.transform_age_sec < 1.0]
        finally:
            self._explore_enabled = False
        assert hit, '2 台目が通常の ok（reason 空）に戻らない'
        self._health2.clear()

    # ════════════════════════════════════════════════════════
    def test_d_jump_suppressed_while_exploring(self):
        """保留中は実機の階段（0.241 m・2.545 rad）でも jump が出ない。
        解除後も jump 無し（前回値を捨てている）。TF が生きていること
        （transform_age_sec の小ささ）も併せて確認し、空振りを防ぐ。"""
        self._reset_explore2()
        self._tf_x, self._tf_yaw = 0.0, 0.0
        self._spin(1.0)
        self._health2.clear()
        self._explore_value = True
        self._explore_enabled = True
        hit = self._wait_for_reason2('global_localizing', True, timeout=3.0)
        assert hit, '2 台目が global_localizing にならない'
        # 保留中に階段（実機の地図読み直しの規模）。
        self._tf_x, self._tf_yaw = 0.241, 2.545
        self._spin(1.0)
        # 解除して猶予（探索に猶予は無い）＋2 tick 分まで見る。
        self._explore_value = False
        self._spin(2.5)
        self._explore_enabled = False
        jumps = [m for m in self._health2 if m.reason == 'jump']
        assert jumps == [], f'保留中・解除後に jump が出た: {jumps}'
        live = [m for m in self._health2 if m.transform_age_sec < 1.0]
        assert live, 'TF が 2 台目に届いていない（空振りの可能性）'

    def test_e_jump_fires_without_explore(self):
        """対照: 知らせ無しで同じ階段を与えたら jump が出る。
        保留が『いつでも』効いているのではない証拠。"""
        self._reset_explore2()
        self._tf_x, self._tf_yaw = 0.0, 0.0
        self._spin(1.0)
        self._health2.clear()
        self._tf_x, self._tf_yaw = 0.241, 2.545
        hit = self._wait_for_reason2('jump', False, timeout=3.0)
        assert hit, '知らせ無しで階段を与えても jump が出ない'


if __name__ == '__main__':
    unittest.main()
