"""
test_venue_nav_resume_check_node.py — 再開のときも、走り出す前に経路の先を確認する
（Spec-onsite §6.3「一時停止・手押し・フォルト解決後からの再開」。ユーザー決定 2026-10-12）。

`BLOCKED` で止まってから「停止→再開」すると、確認なしで障害物へ走り出していた。
再開は、残りの経路の先 1.5 m が空いていなければ送らず `NAV` から `BLOCKED` へ移る
（SM-3.1.2-057）。以降は BLOCKED の既存の流れ（空いたら再送→evt.unblocked）。

本番の `venue_navigator`＋`state_manager`＋`pin_registrar` を起動し、Nav2 の縁だけ
代役にする（共通部品 `_blocked_hold_harness.py`。障害物マップは実機と同じ
RELIABLE + TRANSIENT_LOCAL・`map` 系・1 Hz）。再開の入口は 2 つ:
  ・`ui.run`（resume_follow_path の effect と /system/state(NAV) の両方が届く）
  ・`ui.resume_yes`（フォルト解決の W-1「はい」。effect 無しで state(NAV) のみ）
手押し(CARRY)の復帰は経路が捨てられ最初から計画し直す別の流れで、本試験の対象外
（報告に記載）。

  (a) 停止→障害物を前方 1.5 m 以内に置く→再開: FollowPath を送らず BLOCKED へ。
  (b) 障害物を取り除くと数秒で再送されて NAV に戻る。
  (c) 障害物が無ければ再開で従来どおり即座に送られる。
  (d) フォルト解決の「はい」でも同じ（a の入口違い）。
  (e) 障害物マップが古い: 再開で送られず BLOCKED。
"""

from __future__ import annotations

import time

import pytest

from _blocked_hold_harness import (BlockedHoldBase,  # noqa: F401
                                   _BLOCKED_PERIOD_S_DEFAULT, make_description)


@pytest.mark.launch_test
def generate_test_description():
    return make_description(_BLOCKED_PERIOD_S_DEFAULT)


_NEAR = (1.0, 1.2)   # 前方 1.0 m（1.5 m 以内）


class TestVenueNavResumeCheck(BlockedHoldBase):

    def _nav_then_pause(self, via: str):
        """障害物なしで NAV に入り（FollowPath は走り続ける）、停止して PAUSE にする。
        via: 'stop'（ui.stop）| 'fault'（回復フォルト）。"""
        self._reset_to_home()
        with self._lock:
            self.compute_mode = 'success'
            self.follow_mode = 'run'
            self.compute_count = 0
            self.follow_goals.clear()
            self.follow_cancel_count = 0
            self.states.clear()
            self.event_names.clear()
        self.costmap_auto = True
        self.set_obstacle(None)
        self._enter_nav()
        self.assertTrue(self._wait_count(self._n_follow, 1, timeout=15.0),
                        f'最初の FollowPath が届かない ({self._mode_state()})')
        self.assertTrue(self._wait_mode_state('PANEL_NAV', 'NAV', timeout=5.0))
        c0 = self._n_cancel()
        if via == 'stop':
            self._call_trigger('ui.stop')
        else:
            self._publish_fault(True)
        if not self._wait_mode_state('PANEL_NAV', 'PAUSE', timeout=5.0):
            self.fail(f'PAUSE に入らない ({self._mode_state()})')
        self.assertTrue(self._wait_count(self._n_cancel, c0 + 1, timeout=5.0),
                        'PAUSE で FollowPath を取り消していない。')
        if via == 'fault':
            self._publish_fault(False)
        self._sleep(0.5)

    def _resume(self, via: str):
        if via == 'stop':
            self._call_trigger('ui.run')
        else:
            self._call_trigger('ui.resume_yes')

    def _assert_blocked_not_sent(self, f0, seconds=3.0):
        self.assertTrue(
            self._wait_mode_state('PANEL_NAV', 'BLOCKED', timeout=6.0),
            f'再開したのに BLOCKED へ移らない ({self._mode_state()})。')
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.assertEqual(
                self._n_follow(), f0,
                '経路の先が塞がれているのに再開で FollowPath が送られた。')
            time.sleep(0.2)
        self.assertEqual(self._mode_state(), ('PANEL_NAV', 'BLOCKED'))

    # ── (a) ───────────────────────────────────────────────
    def test_a_resume_with_obstacle_goes_blocked(self):
        self._nav_then_pause('stop')
        f0 = self._n_follow()
        self.set_obstacle((*_NEAR, 100))
        self._sleep(0.3)
        self._resume('stop')
        self._assert_blocked_not_sent(f0)
        self.assertIn('evt.blocked', self._snap_events())
        self.assertEqual(self._n_compute(), 1, '再開で経路を計算し直した。')
        self._reset_to_home()

    # ── (b) ───────────────────────────────────────────────
    def test_b_obstacle_removed_resends(self):
        self._nav_then_pause('stop')
        f0 = self._n_follow()
        self.set_obstacle((*_NEAR, 100))
        self._sleep(0.3)
        self._resume('stop')
        self._assert_blocked_not_sent(f0, 2.0)
        with self._lock:
            mark = len(self.states)
        self.set_obstacle(None)
        self.assertTrue(
            self._wait_count(self._n_follow, f0 + 1, timeout=10.0),
            '障害物を取り除いても再送されない。')
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            if ('PANEL_NAV', 'NAV') in self._snap_states()[mark:]:
                break
            time.sleep(0.1)
        self.assertIn(('PANEL_NAV', 'NAV'), self._snap_states()[mark:])
        self.assertIn('evt.unblocked', self._snap_events())
        self._reset_to_home()

    # ── (c) ───────────────────────────────────────────────
    def test_c_clear_resumes_immediately(self):
        self._nav_then_pause('stop')
        f0 = self._n_follow()
        t0 = time.monotonic()
        self._resume('stop')
        self.assertTrue(
            self._wait_count(self._n_follow, f0 + 1, timeout=5.0),
            '障害物が無いのに再開で送られない。')
        self.assertLess(time.monotonic() - t0, 3.0)
        self.assertNotIn('evt.blocked', self._snap_events())
        self.assertEqual(self._mode_state(), ('PANEL_NAV', 'NAV'))
        self._reset_to_home()

    def test_c2_clear_resume_yes_resumes_immediately(self):
        self._nav_then_pause('fault')
        f0 = self._n_follow()
        self._resume('fault')
        self.assertTrue(
            self._wait_count(self._n_follow, f0 + 1, timeout=5.0),
            '障害物が無いのに「はい」の再開で送られない。')
        self.assertNotIn('evt.blocked', self._snap_events())
        self._reset_to_home()

    # ── (d) ───────────────────────────────────────────────
    def test_d_fault_resume_yes_with_obstacle_goes_blocked(self):
        self._nav_then_pause('fault')
        f0 = self._n_follow()
        self.set_obstacle((*_NEAR, 100))
        self._sleep(0.3)
        self._resume('fault')
        self._assert_blocked_not_sent(f0)
        self.assertIn('evt.blocked', self._snap_events())
        self._reset_to_home()

    # ── (e) ───────────────────────────────────────────────
    def test_e_stale_costmap_goes_blocked(self):
        self._nav_then_pause('stop')
        f0 = self._n_follow()
        self.costmap_auto = False   # 以降マップが届かない
        self._sleep(7.0)            # 古い（約 5 秒以上更新なし）にする
        self._resume('stop')
        self._assert_blocked_not_sent(f0)
        # 陽性対照: 新しい空きのマップが届けば再送される。
        self.costmap_auto = True
        self.set_obstacle(None)
        self.assertTrue(
            self._wait_count(self._n_follow, f0 + 1, timeout=10.0),
            '新しいマップが届いて空いているのに再送されない（陽性対照）。')
        self._reset_to_home()
