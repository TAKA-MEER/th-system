"""
test_venue_nav_collision_backoff_node.py — 衝突による打ち切りの走り直し間隔の延長と、
Nav2 TF 停滞の検知からの除外（Spec-onsite §6.3。実機 2026-10-13）の launch_testing 試験。

実機の症状: 狭い隙間で、経路の先は空いているのに Nav2(RPP) が「前方に衝突」で打ち切り、
2 秒ごとに送り直しては打ち切られる往復が続いた。さらに `evt.nav_tf_stall` が
TF 停滞でないのに出た。

本番の `venue_navigator`＋`state_manager`＋`pin_registrar` を起動し（共通部品は
`_blocked_hold_harness.py`）、FollowPath の代役が実機と同じ形で ABORT の直前に
controller_server の ERROR ログ（`... detected collision ahead!`）を /rosout へ流す。
再確認の周期は 0.5 秒に縮め、上限は 4 秒に下げる（倍率 2 は本番の値）。
延長の列は 0.5 → 1 → 2 → 4（頭打ち）秒。

  ① 連続して衝突で打ち切られると、送り直しの間隔が延び、上限で頭打ちになる
  ② そのあいだ evt.nav_tf_stall は出ない（/tf 新・静止・3 回以上。対照として、
     衝突のログが無い同じ打ち切りは出る＝この条件で検知は生きている）
  ③ 機体が進めば間隔は元に戻る／離脱して入り直しても戻る
"""

from __future__ import annotations

import time

import pytest
import rclpy.time
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

from _blocked_hold_harness import BlockedHoldBase, make_description

_PERIOD_S = 0.5
_MAX_S = 4.0
_ABORT_S = 0.3


@pytest.mark.launch_test
def generate_test_description():
    return make_description(_PERIOD_S, {'collision_retry_max_s': _MAX_S})


class TestVenueNavCollisionBackoff(BlockedHoldBase):

    def setUp(self):
        super().setUp()
        # 実機の slam_toolbox と同じ約 +0.7 秒の未来 stamp の map->odom（/tf が新しい
        # ことが TF 停滞の発火条件。これが無いと『数えても出ない』空の試験になる）。
        self._odom_timer = self.node.create_timer(0.1, self._publish_map_odom)

    def tearDown(self):
        try:
            self.node.destroy_timer(self._odom_timer)
        except Exception:
            pass
        super().tearDown()

    def _publish_map_odom(self):
        now_ns = self.node.get_clock().now().nanoseconds + int(0.7e9)
        t = TransformStamped()
        t.header.stamp = rclpy.time.Time(nanoseconds=now_ns).to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.rotation.w = 1.0
        self._pub_tf_dyn.publish(TFMessage(transforms=[t]))

    def _start(self, collision: bool):
        if not hasattr(self, '_pub_tf_dyn'):
            self._pub_tf_dyn = self.node.create_publisher(TFMessage, '/tf', 10)
        self._reset_to_home()
        with self._lock:
            self.compute_mode = 'success'
            self.follow_mode = 'abort'
            self.follow_abort_delay_s = _ABORT_S
            self.follow_collision = collision
            self.follow_times.clear()
            self.event_names.clear()
        self.robot_x = 0.0
        self._publish_tf()
        time.sleep(0.5)
        self.costmap_auto = True
        self.set_obstacle(None)
        self._enter_nav()

    def _times(self):
        with self._lock:
            return list(self.follow_times)

    def _wait_calls(self, n, timeout=60.0):
        return self._wait_count(lambda: len(self._times()), n, timeout)

    def _gaps(self):
        t = self._times()
        return [b - a for a, b in zip(t, t[1:])]

    def test_gap_grows_caps_and_no_tf_stall(self):
        """①②: 間隔が延び（0.5→1→2→4）、上限 4 秒で頭打ち。TF 停滞は出ない。"""
        self._start(collision=True)
        self.assertTrue(self._wait_calls(6), f'FollowPath が 6 回届かない {self._times()}')
        g = self._gaps()
        # g[k] = k+1 回目の打ち切りから次の送り直しまで（＋打ち切りまでの 0.3 秒）。
        # 待ちの下限だけを縛る（周期の量子化は上に足されるだけ）。
        for k, delay in enumerate([0.5, 1.0, 2.0, 4.0, 4.0]):
            self.assertGreaterEqual(
                g[k], _ABORT_S + delay - 0.05,
                f'{k + 1} 回目の打ち切り後の待ちが短い (g={g})')
        self.assertLess(g[4], _ABORT_S + _MAX_S + 1.6,
                        f'上限 {_MAX_S}s で頭打ちになっていない (g={g})')
        self.assertNotIn('evt.nav_tf_stall', self._snap_events(),
                         '衝突による打ち切りが TF 停滞に数えられた')
        self._mark_passed(f'gaps={[round(x, 2) for x in g]}')

    def test_control_non_collision_aborts_still_emit_tf_stall(self):
        """対照: 衝突のログが無い同じ打ち切りは、この条件（/tf 新・静止）で TF 停滞を出す。"""
        self._start(collision=False)
        self.assertTrue(self._wait_calls(5), f'FollowPath が 5 回届かない {self._times()}')
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and \
                'evt.nav_tf_stall' not in self._snap_events():
            time.sleep(0.1)
        self.assertIn('evt.nav_tf_stall', self._snap_events(),
                      '衝突でない打ち切りの連続で検知が出ない（検知を弱めた）')
        # 衝突でないので間隔は延びない（周期＋量子化の範囲）。
        self.assertLess(max(self._gaps()[1:]), 2.2, f'gaps={self._gaps()}')
        self._mark_passed()

    def test_gap_resets_when_robot_moves(self):
        """③: 3 回連続のあと機体が 0.2 m 以上進めば、間隔は元（0.5 秒）に戻る。"""
        self._start(collision=True)
        self.assertTrue(self._wait_calls(3), f'FollowPath が 3 回届かない {self._times()}')
        self.robot_x = 1.0     # 3 回目の呼び出しの直後に機体を 1 m 進める
        self._publish_tf()
        self.assertTrue(self._wait_calls(4), '4 回目が届かない')
        g = self._gaps()
        # 動かなければ 3 回連続＝待ち 2 秒（g[2] >= 2.25）。動いたので 1 回目扱い＝0.5 秒。
        self.assertLess(g[2], _ABORT_S + 0.5 + 1.0, f'進んだのに間隔が戻らない (g={g})')
        self._mark_passed(f'gaps={[round(x, 2) for x in g]}')

    def test_gap_resets_on_new_episode(self):
        """③: 離脱して入り直せば連続数は戻る（新しい目的地・到着・成功も同じ _stall_reset）。"""
        self._start(collision=True)
        self.assertTrue(self._wait_calls(4), f'FollowPath が 4 回届かない {self._times()}')
        self._reset_to_home()
        n0 = len(self._times())
        self.set_obstacle(None)
        self._enter_nav()
        self.assertTrue(self._wait_calls(n0 + 3), '入り直し後の FollowPath が 3 回届かない')
        t = self._times()[n0:]
        # 入り直し後 2 回目の打ち切りの待ちは 1 秒。持ち越しなら上限の 4 秒。
        self.assertLess(t[2] - t[1], _ABORT_S + 1.0 + 1.5,
                        f'入り直しても間隔が戻らない (t={t})')
        self._mark_passed()
