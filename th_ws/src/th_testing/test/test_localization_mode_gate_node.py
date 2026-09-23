"""
test_localization_mode_gate_node.py
==============================
WP-SAFE-05修正 — mode gate（Spec-safety.md §3.5.0「使っていない間は監視しない」）
の Docker launch テスト。

localization_health ノードを1台だけ起動し（TF は出さない＝推定が居ない状態。
expected_nodes は存在しない名前にして actual を node_down に固定）、
テスト側が /system/state を流して次を確認する（ノード本体の振る舞いを見る。
ソースの文字列検査だけでは配線の欠落がすり抜けるため）:
  a. IDLE で出す → ok=true・reason='inactive'、かつ node_present=false が生値のまま
  b. 続けて PANEL_NAV を出す → 1 秒以内に ok=false（node_down か stale）
  c. PREP/MAPPING → inactive
  d. PREP/RETURN → ok=false
  e. ESTOP＋prev REPLAY（TF なし）→ ok=false のまま（inactive にならない。
     非常停止中は止まる直前のモードに従う。Spec-safety.md §3.5.0）
  f. ESTOP＋prev IDLE → inactive
  g. CARRY＋prev REPLAY → inactive（手押し中は監視しない。prev を見ない）

お手本は test_runaway_freshness_node.py と test_planned_restart_node.py
（ノードの起動・パラメータの渡し方・test_a_ 順序制御・setup-clear 競合の避け方）。

注意（CLAUDE.md の既知の癖）:
- launch_testing の pytest プラグインがファイル全体を1つのアイテムとして
  収集するため、試験は TestCase のメソッドにする。
- /system/state を出す側の QoS は state_manager と同じにする
  （TRANSIENT_LOCAL。VOLATILE で出すとノード側の TRANSIENT_LOCAL 購読と
  非互換で1件も届かず、ずっと「未受信＝監視しない」になって全試験が黙って
  通らなくなる。conftest.py の _clock_qos と同じ罠）。
- /safety/localization_health は 200ms 周期で出続ける連続配信なので、
  切り替え後に clear してから新しい証拠を待つ形で安全（edge 配信の
  /safety/fault のような setup-clear 競合は無い）。
"""
import math
import time
import unittest

import pytest
import rclpy
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions

from th_system_msgs.msg import LocalizationHealth, SystemState


WARMUP_MS = 1000
STALE_MS = 2000
# タイマ周期＝比較周期。b の「1 秒以内」を余裕で見るため短め（200ms）。
JUMP_WINDOW_MS = 200
RESTART_MAX_MS = 30000
POST_RESTART_GRACE_MS = 1500

# state_manager.py の state_qos と同じ（depth 1・RELIABLE・TRANSIENT_LOCAL・
# KEEP_LAST）。合わせないと届かない（上の docstring 参照）。
_STATE_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST)


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
            # 存在しないノード名。actual を node_down に固定するため。
            'localization_expected_nodes': ['__no_such_estimator__'],
            'jump_window_ms': JUMP_WINDOW_MS,
            'jump_translation_m': 1.0,
            'jump_rotation_rad': 0.5,
            'localization_restart_max_ms': RESTART_MAX_MS,
            'localization_post_restart_grace_ms': POST_RESTART_GRACE_MS,
        }],
        output='screen',
    )
    return launch.LaunchDescription([
        health,
        launch_testing.actions.ReadyToTest(),
    ]), {'localization_health': health}


class TestLocalizationModeGateNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_localization_mode_gate_node')

        self._health: list[LocalizationHealth] = []
        self.node.create_subscription(
            LocalizationHealth, '/safety/localization_health',
            self._health.append, 10)

        # /system/state を出し続ける（5Hz）。TRANSIENT_LOCAL で state_manager
        # と同じ QoS。後から見るノードにも最新が届く。
        self._mode = 'IDLE'
        self._state = 'NONE'
        self._prev_mode = ''
        self._prev_state = ''
        self.pub_state = self.node.create_publisher(
            SystemState, '/system/state', _STATE_QOS)
        self._state_timer = self.node.create_timer(0.2, self._publish_state)

        # 起動猶予 + 少し余裕。この spin で state がノードに届き、health の
        # 証拠がバッファへ溜まる。health は連続配信なので blind clear の心配は
        # 無い（各試験は切り替え後に clear して新しい証拠を待つ）。
        self._spin(WARMUP_MS / 1000.0 + 0.5)

    def tearDown(self):
        if getattr(self, '_state_timer', None) is not None:
            self._state_timer.cancel()
            self.node.destroy_timer(self._state_timer)
            self._state_timer = None
        self.node.destroy_node()

    # ── ヘルパー ──────────────────────────────────────────────
    def _spin(self, duration: float = 0.1):
        deadline = time.time() + duration
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _publish_state(self):
        msg = SystemState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.mode = self._mode
        msg.state = self._state
        msg.prev_mode = self._prev_mode
        msg.prev_state = self._prev_state
        self.pub_state.publish(msg)

    def _set_mode(self, mode: str, state: str,
                  prev_mode: str = '', prev_state: str = ''):
        """モードを切り替え、切り替え後の証拠だけを見るためバッファを捨てる。
        health は 200ms 周期で出続けるので、捨てた後の証拠は必ず新しいもの
        （setup-clear 競合は無い）。prev の既定 '' は state_manager の初期値
        （ラッチ前）と同じ。"""
        self._mode = mode
        self._state = state
        self._prev_mode = prev_mode
        self._prev_state = prev_state
        self._health.clear()

    def _wait_for(self, pred, timeout: float):
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._spin(0.05)
            hit = [m for m in self._health if pred(m)]
            if hit:
                return hit
        return []

    # ════════════════════════════════════════════════════════
    def test_a_idle_is_inactive_with_raw_values(self):
        """IDLE（TF なし＝推定が居ない）→ ok=true・reason=inactive、
        かつ node_present=false が生値のまま。"""
        hit = [m for m in self._health
               if m.ok and m.reason == 'inactive']
        assert hit, 'IDLE で inactive が出ない（/system/state が届いていない可能性）'
        assert hit[-1].node_present is False, (
            'node_present が生値(false)のまま出ていない')
        assert math.isinf(hit[-1].transform_age_sec), (
            'transform_age_sec が生値(inf)のまま出ていない')

    def test_b_panel_nav_fires_within_1s(self):
        """続けて PANEL_NAV を出す → 1 秒以内に ok=false（node_down か stale）。"""
        self._set_mode('PANEL_NAV', 'NAV')
        hit = self._wait_for(lambda m: not m.ok, timeout=1.0)
        assert hit, 'PANEL_NAV にして 1 秒以内に ok=false にならない'
        assert hit[0].reason in ('node_down', 'stale'), (
            f'想定外の reason: {hit[0].reason}')

    def test_c_prep_mapping_is_inactive(self):
        """PREP/MAPPING → inactive（地図作成中は監視しない）。"""
        self._set_mode('PREP', 'MAPPING')
        hit = self._wait_for(lambda m: m.ok and m.reason == 'inactive',
                             timeout=3.0)
        assert hit, 'PREP/MAPPING で inactive に戻らない'

    def test_d_prep_return_fires(self):
        """PREP/RETURN → ok=false（自動帰還中は監視する）。"""
        self._set_mode('PREP', 'RETURN')
        hit = self._wait_for(lambda m: not m.ok, timeout=3.0)
        assert hit, 'PREP/RETURN で ok=false にならない'
        assert hit[0].reason in ('node_down', 'stale'), (
            f'想定外の reason: {hit[0].reason}')

    def test_e_estop_keeps_monitoring_prev_replay(self):
        """ESTOP＋prev REPLAY（TF なし）→ ok=false のまま（inactive にならない）。
        非常停止中は止まる直前のモードに従う。推定が止まったままフォルトが
        解除されてはいけない（2026-09-23 実機で発覚）。"""
        self._set_mode('ESTOP', 'NONE', prev_mode='REPLAY', prev_state='RUN')
        hit = self._wait_for(lambda m: not m.ok, timeout=3.0)
        assert hit, 'ESTOP/prev=REPLAY で ok=false にならない（inactive になった）'
        assert hit[0].reason in ('node_down', 'stale'), (
            f'想定外の reason: {hit[0].reason}')

    def test_f_estop_idle_prev_is_inactive(self):
        """ESTOP＋prev IDLE → inactive（止まる直前も使っていないので監視しない）。"""
        self._set_mode('ESTOP', 'NONE', prev_mode='IDLE', prev_state='NONE')
        hit = self._wait_for(lambda m: m.ok and m.reason == 'inactive',
                             timeout=3.0)
        assert hit, 'ESTOP/prev=IDLE で inactive にならない'

    def test_g_carry_ignores_prev(self):
        """CARRY＋prev REPLAY → inactive（手押し中は人が動かすので監視しない）。"""
        self._set_mode('CARRY', 'NONE', prev_mode='REPLAY', prev_state='RUN')
        hit = self._wait_for(lambda m: m.ok and m.reason == 'inactive',
                             timeout=3.0)
        assert hit, 'CARRY/prev=REPLAY で inactive にならない'


if __name__ == '__main__':
    unittest.main()
