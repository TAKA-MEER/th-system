"""
test_venue_nav_proactive_block_node.py — 走行中に障害物が経路の先に入ったら、Nav2 の
打ち切りを待たず止まって BLOCKED で待つ（Spec-onsite §6.3「走行中に障害物が経路の先に
入ったとき」。ユーザー決定 2026-10-13・実機 2026-10-13）の launch_testing 試験。

実機の症状: 塞がれて停止したあと、Nav2 が旋回指令を出し続け、障害物リミッタは旋回を
通すので、機体が障害物の前でその場旋回した。`BLOCKED` には入っていなかった。

共通部品は `_blocked_hold_harness.py`（本番の `venue_navigator`＋`state_manager`＋
`pin_registrar`。障害物マップの代役は実機と同じ RELIABLE + TRANSIENT_LOCAL）。
FollowPath の代役は `follow_mode='run'`＝**受け付けて走り続ける（自分では打ち切らない）**。
BLOCKED に入るのは venue_navigator の周期確認が取り消したときだけ。

  (a) 経路の前方 1.5 m 以内に障害物が現れる → 約 1〜2.5 秒で FollowPath が取り消され BLOCKED
  (b) 障害物が除かれる → 既存の流れで再送され NAV に戻る
  (c) 障害物が 1.5 m より遠い → 止まらない
  (d) ALIGN 中は止めない
  (e) 障害物マップが未受信・古い → 走行中は止めない
  (f) evt.blocked が二重に出ない・BLOCKED ⇄ NAV のフラップにならない
"""

from __future__ import annotations

import time

import pytest
from geometry_msgs.msg import PoseStamped

from _blocked_hold_harness import (BlockedHoldBase,  # noqa: F401
                                   _BLOCKED_PERIOD_S_DEFAULT, make_description)


@pytest.mark.launch_test
def generate_test_description():
    return make_description(_BLOCKED_PERIOD_S_DEFAULT)


_NEAR = (1.0, 1.2)    # 前方 1.0 m（1.5 m 以内）
_FAR = (2.2, 2.4)     # 前方 2.2 m（1.5 m より先）
_SETTLE_S = 3.0       # NAV 入口の消去のあと、再記録待ちが終わるまで


class TestVenueNavProactiveBlock(BlockedHoldBase):

    def _run_nav(self, obstacle=None, costmap=True):
        """障害物なし（または指定）で PANEL_NAV に入り、FollowPath が走り続ける NAV まで。"""
        self._reset_to_home()
        with self._lock:
            self.compute_mode = 'success'
            self.follow_mode = 'run'
            self.compute_count = 0
            self.follow_goals.clear()
            self.follow_cancel_count = 0
            self.states.clear()
            self.event_names.clear()
        self.costmap_auto = costmap
        self.set_obstacle(obstacle)
        self._enter_nav()
        self.assertTrue(self._wait_count(self._n_follow, 1, timeout=15.0),
                        f'最初の FollowPath が届かない ({self._mode_state()})')
        if not self._wait_mode_state('PANEL_NAV', 'NAV', timeout=10.0):
            self.fail(f'NAV に入らない ({self._mode_state()})')

    def _hold_nav(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.assertEqual(self._mode_state(), ('PANEL_NAV', 'NAV'),
                             f'NAV のはずが {self._mode_state()} になった。')
            self.assertEqual(self._n_cancel(), 0, 'FollowPath を取り消した。')
            time.sleep(0.2)

    # ── (a)(f) ────────────────────────────────────────────
    def test_a_obstacle_appears_cancels_and_blocks(self):
        self._run_nav(None)
        time.sleep(_SETTLE_S)
        self._hold_nav(1.5)  # 障害物が無い間は止めない（陰性対照）
        t0 = time.monotonic()
        self.set_obstacle(_NEAR + (100,))
        self.assertTrue(
            self._wait_count(self._n_cancel, 1, timeout=4.0),
            'FollowPath が打ち切られない代役でも、障害物が現れたら取り消すはず。')
        self.assertTrue(
            self._wait_mode_state('PANEL_NAV', 'BLOCKED', timeout=4.0),
            f'BLOCKED に移らない ({self._mode_state()})')
        self.assertLess(time.monotonic() - t0, 3.0, '止まるのが遅すぎる')
        # (f) 二重 evt.blocked なし・フラップなし（障害物が残る間 BLOCKED のまま）
        f0 = self._n_follow()
        time.sleep(6.0)
        self.assertEqual(self._snap_events().count('evt.blocked'), 1,
                         'evt.blocked が 1 回でない（取り消しの結果で二重に出た）。')
        self.assertEqual(self._mode_state(), ('PANEL_NAV', 'BLOCKED'))
        self.assertEqual(self._n_follow(), f0, '塞がれたまま FollowPath が再送された。')
        self.assertNotIn('evt.unblocked', self._snap_events())
        self._reset_to_home()

    # ── (b) ───────────────────────────────────────────────
    def test_b_cleared_resumes(self):
        self._run_nav(None)
        time.sleep(_SETTLE_S)
        self.set_obstacle(_NEAR + (100,))
        self.assertTrue(self._wait_mode_state('PANEL_NAV', 'BLOCKED', timeout=6.0))
        f0 = self._n_follow()
        time.sleep(3.0)
        self.set_obstacle(None)
        self.assertTrue(
            self._wait_count(self._n_follow, f0 + 1, timeout=8.0),
            '障害物を除いても FollowPath が再送されない。')
        self.assertTrue(self._wait_mode_state('PANEL_NAV', 'NAV', timeout=6.0),
                        f'NAV に戻らない ({self._mode_state()})')
        # (f) 往復しない: 戻ったあと 5 秒 NAV のまま、evt.blocked は最初の 1 回だけ
        c0 = self._n_cancel()
        time.sleep(5.0)
        self.assertEqual(self._mode_state(), ('PANEL_NAV', 'NAV'))
        self.assertEqual(self._n_cancel(), c0)
        self.assertEqual(self._snap_events().count('evt.blocked'), 1)
        self._reset_to_home()

    # ── (c) ───────────────────────────────────────────────
    def test_c_far_obstacle_does_not_stop(self):
        self._run_nav(_FAR + (100,))
        time.sleep(_SETTLE_S)
        self._hold_nav(6.0)
        self._reset_to_home()

    # ── (d) ───────────────────────────────────────────────
    def test_d_align_not_stopped(self):
        """到着して向き合わせ（ALIGN）中は、前方に障害物があっても止めない。"""
        self._reset_to_home()
        with self._lock:
            self.compute_mode = 'success'
            self.follow_mode = 'run'
            self.states.clear()
            self.event_names.clear()
        self.costmap_auto = True
        self.set_obstacle(None)
        # ゴールを機体のすぐそば(0.05 m)に置く → NAV 直後に到着し ALIGN に入る。
        # 向きは 1.5 rad ずらす（TF は固定で機体は回らないので ALIGN が続く）。
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.pose.position.x = 0.05
        pose.pose.orientation.z = 0.7071
        pose.pose.orientation.w = 0.7071
        self.pub_summon.publish(pose)
        self._call_trigger('ui.goto', '{"kind":"SUMMON"}')
        self.assertTrue(self._wait_mode_state('SUMMON', 'POINT', timeout=10.0))
        self._publish_setup_event('evt.two_point_done')
        self.assertTrue(self._wait_mode_state('SUMMON', 'WAIT_CLEAR', timeout=5.0))
        self._publish_setup_event('evt.clear_ok')
        self.assertTrue(self._wait_mode_state('SUMMON', 'ALIGN', timeout=15.0),
                        f'ALIGN に入らない ({self._mode_state()})')
        self.set_obstacle((0.0, 0.5, 100))
        time.sleep(_SETTLE_S + 4.0)
        self.assertEqual(self._mode_state(), ('SUMMON', 'ALIGN'),
                         f'ALIGN 中に止めた ({self._mode_state()})')
        self.assertNotIn('evt.blocked', self._snap_events())
        self._reset_to_home()

    # ── (e) ───────────────────────────────────────────────
    def test_e_no_costmap_does_not_stop(self):
        """障害物マップが（NAV 入口の消去のあと）一度も届かない → 走行中は止めない。"""
        self.costmap_auto = False
        self._run_nav(None, costmap=False)
        self._hold_nav(8.0)
        self._reset_to_home()
