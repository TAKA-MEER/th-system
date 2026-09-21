"""
test_planned_restart_node.py
==============================
O-e3 — 計画的な自己位置推定の再起動の保留（Spec-safety.md §3.5.0）の
Docker launch テスト。

localization_health ノードを短い上限・猶予で起動し、テスト側が
/slam_control/estimator_restarting を流して次を確認する（静的な配線検査は
test_localization_health_launch.py が持ち、ここではノード本体の振る舞いを見る）:
  a. 推定ノード不在で知らせ無し → ok=False, reason='node_down'（従来どおり）
  b. true を publish → ok=True, reason='restarting'
  c. true を publish し続けて上限超過 → ok=False, reason='restart_timeout'
  d. true を 1 回 publish したあと止める（publisher 死亡想定）→
     上限後に restart_timeout（永遠に保留しない）

お手本は test_runaway_freshness_node.py（起動方法・パラメータ・
test_a_ 順序制御・setup-clear 競合の避け方）。

注意（CLAUDE.md の既知の癖）:
- launch_testing の pytest プラグインがファイル全体を1つのアイテムとして
  収集するため、試験は TestCase のメソッドにする。
- 試験側の publisher も QoS を発行側（RELIABLE + TRANSIENT_LOCAL）に
  合わせる。素の create_publisher(..., 10)（VOLATILE）はノード側の
  TRANSIENT_LOCAL 購読と非互換で1件も届かない（conftest.py の _clock_qos
  と同じ罠）。
- メソッドの実行順は名前順に固定する。各試験は自分の前後始末を持つ
  （c・d は開始時に false で状態を戻してから始める。setup-clear 競合を
  避けるため、証拠を消す clear は「新しい証拠を待つ前」にだけ行う）。

試験用に上限・猶予を短くする（restart_max 3 秒・grace 1.5 秒・warmup 1 秒）。

e/f（B′ が再起動をまたがない）用に 2 台目のノードを立てる。a〜d 用の
1 台目は expected_nodes が不在前提だが、e/f は base が ok になること
（ノード在席＋新鮮な TF＋猶予明け）が要るため、1 台では両立しない。
2 台目は expected_nodes=['localization_health']（1 台目自身。起動直後から
在席が確定する）＋ TF 階段用。health トピックが混ざらないよう
/safety/localization_health_tf へ remap する。
"""
import time
import unittest
import math

import pytest
import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions

from th_system_msgs.msg import LocalizationHealth


WARMUP_MS = 1000
STALE_MS = 2000
JUMP_WINDOW_MS = 500
RESTART_MAX_MS = 3000
POST_RESTART_GRACE_MS = 1500

# ノードの購読 QoS（発行側 status_qos）に合わせる。合わせないと届かない
# （上の docstring 参照）。
_RESTART_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


@pytest.mark.launch_test
def generate_test_description():
    health = launch_ros.actions.Node(
        package='th_state',
        # Python スクリプトノードは拡張子つきで指定する（C++ の bare 名と違う。
        # 素の 'localization_health' では libexec に無いと起動失敗する）。
        executable='localization_health.py',
        name='localization_health',
        parameters=[{
            'localization_stale_ms': STALE_MS,
            'localization_warmup_ms': WARMUP_MS,
            # 存在しないノード名。test_a で node_down を出すため。
            'localization_expected_nodes': ['__no_such_estimator__'],
            'jump_window_ms': JUMP_WINDOW_MS,
            'jump_translation_m': 1.0,
            'jump_rotation_rad': 0.5,
            'localization_restart_max_ms': RESTART_MAX_MS,
            'localization_post_restart_grace_ms': POST_RESTART_GRACE_MS,
        }],
        output='screen',
    )
    # e/f 用の 2 台目。a〜d 用とは expected_nodes と TF の要否が違うため別
    # インスタンスにする（モジュール docstring 参照）。health は別トピックへ
    # remap して混ざらないようにする。
    health_tf = launch_ros.actions.Node(
        package='th_state',
        executable='localization_health.py',
        name='localization_health_tf',
        parameters=[{
            'localization_stale_ms': 5000,
            'localization_warmup_ms': WARMUP_MS,
            # 1 台目のノード名。起動直後から在席が確定する。
            'localization_expected_nodes': ['localization_health'],
            'jump_window_ms': 200,
            'jump_translation_m': 1.0,
            'jump_rotation_rad': 0.5,
            'localization_restart_max_ms': RESTART_MAX_MS,
            'localization_post_restart_grace_ms': POST_RESTART_GRACE_MS,
        }],
        remappings=[('/safety/localization_health', '/safety/localization_health_tf')],
        output='screen',
    )
    return launch.LaunchDescription([
        health,
        health_tf,
        launch_testing.actions.ReadyToTest(),
    ]), {'localization_health': health, 'localization_health_tf': health_tf}


class TestPlannedRestartNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_planned_restart_node')

        self._health: list[LocalizationHealth] = []
        self.node.create_subscription(
            LocalizationHealth, '/safety/localization_health',
            self._health.append, 10)
        # e/f 用。2 台目（remap 先）の health。
        self._health2: list[LocalizationHealth] = []
        self.node.create_subscription(
            LocalizationHealth, '/safety/localization_health_tf',
            self._health2.append, 10)

        self.pub_restart = self.node.create_publisher(
            Bool, '/slam_control/estimator_restarting', _RESTART_QOS)
        self._restart_enabled = False
        self._restart_value = True
        self._restart_timer = self.node.create_timer(0.5, self._publish_restart)

        # e/f 用の TF 階段。map→odom を動かす。10Hz で現在値を出し続ける。
        self._tf_x = 0.0
        self._tf_yaw = 0.0
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self.node)
        self._tf_timer = self.node.create_timer(0.1, self._publish_tf)

        # 起動猶予 + 少し余裕。この spin で溜まった edge をバッファへ流し込む。
        self._spin(WARMUP_MS / 1000.0 + 0.5)
        # blind clear はしない（setup-clear 競合。各試験は自分の前後始末を持つ）。

    def tearDown(self):
        if getattr(self, '_restart_timer', None) is not None:
            self._restart_timer.cancel()
            self.node.destroy_timer(self._restart_timer)
            self._restart_timer = None
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

    def _publish_restart(self):
        if not self._restart_enabled:
            return
        self.pub_restart.publish(Bool(data=self._restart_value))

    def _wait_for_reason(self, reason: str, ok: bool, timeout: float):
        """指定 reason・ok のメッセージを待つ。見つかればそのリストを返す。"""
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._spin(0.1)
            hit = [m for m in self._health if m.reason == reason and m.ok == ok]
            if hit:
                return hit
        return []

    def _reset_restart(self):
        """再起動状態を false に戻し、通常検知（node_down）に戻った証拠を
        待ってからバッファを捨てる。c・d の開始合図。
        固定秒数ではなく node_down を待つ理由: 前の試験が長引いて上限超過の
        edge が reset 中に出ても、node_down が来た時点で「false edge 受理＋
        猶予明け」が確定する。先に捨ててから新しい証拠を待つ形なら安全。
        """
        self._health.clear()
        self._restart_value = False
        self._restart_enabled = True
        try:
            hit = self._wait_for_reason('node_down', False, timeout=6.0)
        finally:
            self._restart_enabled = False
        assert hit, 'reset 後に node_down に戻らない'
        self._health.clear()

    # ════════════════════════════════════════════════════════
    def test_a_no_notice_detects_node_down(self):
        """知らせが無いときは従来どおり検知する（node_down が出る）。"""
        self._spin(1.0)
        hit = [m for m in self._health
               if m.reason == 'node_down' and not m.ok]
        assert hit, '推定ノード不在で node_down が出ない'

    def test_b_restarting_while_notified(self):
        """true を publish → ok=True, reason='restarting'。"""
        self._restart_value = True
        self._restart_enabled = True
        try:
            hit = self._wait_for_reason('restarting', True, timeout=3.0)
        finally:
            self._restart_enabled = False
        assert hit, '再起動通知中に restarting が出ない'

    def test_c_timeout_while_true_continues(self):
        """true を publish し続けて上限超過 → restart_timeout。"""
        self._reset_restart()
        self._restart_value = True
        self._restart_enabled = True
        try:
            hit = self._wait_for_reason('restart_timeout', False, timeout=7.0)
        finally:
            self._restart_enabled = False
        assert hit, '上限超過で restart_timeout が出ない'

    def test_d_timeout_after_publisher_dies(self):
        """true を 1 回 publish したあと止める → 上限後に restart_timeout。
        publisher が死んで true のまま止まっても永遠に保留しない。"""
        self._reset_restart()
        self._restart_value = True
        self._restart_enabled = True
        self._spin(0.5)
        self._restart_enabled = False  # ここで publisher 死亡を模す
        hit = self._wait_for_reason('restart_timeout', False, timeout=7.0)
        assert hit, 'publisher 死亡後に restart_timeout が出ない'

    # ── e/f 用の TF 階段 ─────────────────────────────────────
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
        """2 台目の health から指定 reason・ok を待つ。"""
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._spin(0.1)
            hit = [m for m in self._health2 if m.reason == reason and m.ok == ok]
            if hit:
                return hit
        return []

    def _reset_restart2(self):
        """2 台目を「再起動の知らせなし・猶予明け・base ok」に戻す。
        c・d は true のまま終わる（2 台目も同じ知らせを受けて上限超過の
        まま残る）ため、そのままでは e/f が保留に入れず jump も評価されない。
        false を出して edge を受理させ、猶予（1.5 秒）が明けて通常評価の
        ok（reason 空）に戻ったことを待つ。"""
        self._health2.clear()
        self._restart_value = False
        self._restart_enabled = True
        try:
            deadline = time.time() + 8.0
            hit = []
            while time.time() < deadline and not hit:
                self._spin(0.1)
                hit = [m for m in self._health2
                       if m.ok and m.reason == '' and m.transform_age_sec < 1.0]
        finally:
            self._restart_enabled = False
        assert hit, '2 台目が通常の ok（reason 空）に戻らない'
        self._health2.clear()

    # ════════════════════════════════════════════════════════
    def test_e_jump_suppressed_while_held(self):
        """保留中は実機の階段（0.241 m・2.545 rad）でも jump が出ない。
        階段の前に前回値を取っておく（捨てられていないと出る状況）。
        解除後・猶予中・猶予明けも含めて jump 無し。TF が生きていること
        （transform_age_sec の小ささ）も併せて確認し、空振りを防ぐ。"""
        self._reset_restart2()
        # 前回値を取らせる（yaw=0 で 1 秒以上＝200ms 周期の複数 tick）。
        self._tf_x, self._tf_yaw = 0.0, 0.0
        self._spin(1.0)
        self._health2.clear()
        # 保留に入れる（2 台目が restarting を出すまで待つ）。
        self._restart_value = True
        self._restart_enabled = True
        hit = self._wait_for_reason2('restarting', True, timeout=3.0)
        assert hit, '2 台目が restarting にならない'
        # 保留中に階段（実機の地図読み直しの規模）。
        self._tf_x, self._tf_yaw = 0.241, 2.545
        self._spin(1.0)
        # 解除して猶予（1.5 秒）＋猶予明け（2 tick 分）まで見る。
        self._restart_value = False
        self._spin(2.5)
        self._restart_enabled = False
        jumps = [m for m in self._health2 if m.reason == 'jump']
        assert jumps == [], f'保留中・猶予中に jump が出た: {jumps}'
        live = [m for m in self._health2 if m.transform_age_sec < 1.0]
        assert live, 'TF が 2 台目に届いていない（空振りの可能性）'

    def test_f_jump_fires_without_hold(self):
        """対照: 知らせ無しで同じ階段を与えたら jump が出る。
        保留が『いつでも』効いているのではない証拠。階段の前に 0 へ戻して
        落ち着かせ、その戻りの飛びを health2 から捨ててから階段を与える
        （捨てないと戻りの jump を拾って空振りで通る）。"""
        self._reset_restart2()
        # 前回値を取らせる。
        self._tf_x, self._tf_yaw = 0.0, 0.0
        self._spin(1.0)
        self._health2.clear()
        # 階段。知らせは出さない（_restart_enabled は False のまま）。
        self._tf_x, self._tf_yaw = 0.241, 2.545
        hit = self._wait_for_reason2('jump', False, timeout=3.0)
        assert hit, '知らせ無しで階段を与えても jump が出ない'


if __name__ == '__main__':
    unittest.main()
