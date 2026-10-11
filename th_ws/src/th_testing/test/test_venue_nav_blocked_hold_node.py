"""
test_venue_nav_blocked_hold_node.py — 塞がれた経路は、経路の先が空くまで走り直さない
（Spec-onsite §6.3。ユーザー決定 2026-10-12・実機 2026-10-10 T-2 ④）の
launch_testing 試験。

実機の症状: 経路を段ボールで塞ぐと `BLOCKED` が約 2 秒で自動解除され、機体が
障害物へ向かって走り直す往復を約 12 秒周期で繰り返した（危険で物理非常停止）。
原因は「残りを再送して次の周期に走っていれば通れた」という判定で、障害物の有無を
見ていなかったこと。

共通部品は `_blocked_hold_harness.py`（本番の `venue_navigator`＋`state_manager`＋
`pin_registrar` を起動し、Nav2 の縁だけ代役にする。障害物マップの代役は実機と同じ
RELIABLE + TRANSIENT_LOCAL・`map` 系・1 Hz）。FollowPath の代役は受け付けて
3 秒後に中断（ABORTED）を返す。

試験の対応（ブリーフ 5）:
  (a) 障害物が経路の前方 1.5 m 以内: FollowPath を再送しない・BLOCKED のまま・
      evt.unblocked を出さない（致死 100 と内接 99 の両方）。コストマップの消去は 1 回だけ。
  (b) 障害物を取り除くと再送されて NAV に戻る。
  (c) 障害物が 1.5 m より先: 再送して走り直す（範囲の境界）。膨張域(98)は通れる。
  (d) 障害物マップが一度も届かない: 再送せず BLOCKED のまま。
  (e) BLOCKED の間も「再検索」「停止」は受け付けられる。
  （古い障害物マップは `test_venue_nav_blocked_hold_stale_node.py`）
"""

from __future__ import annotations

import time

import pytest

from _blocked_hold_harness import (BlockedHoldBase,  # noqa: F401
                                   _BLOCKED_PERIOD_S_DEFAULT, make_description)


@pytest.mark.launch_test
def generate_test_description():
    return make_description(_BLOCKED_PERIOD_S_DEFAULT)


# ロボットは原点、経路は (0,0)→(3,0)。前方 1.5 m は x <= 1.5。
_NEAR = (1.0, 1.2)    # 前方 1.0 m（1.5 m 以内）
_FAR = (2.2, 2.4)     # 前方 2.2 m（1.5 m より先）
_HOLD_S = 12.0        # BLOCKED が続くことを見る時間（10 秒以上）


class TestVenueNavBlockedHold(BlockedHoldBase):

    def _go_blocked(self, obstacle):
        """障害物を置いて PANEL_NAV に入り、最初の FollowPath の中断で BLOCKED に落ちる
        まで進める。戻り値: (BLOCKED になった時点の follow 数, イベント数)。"""
        self._reset_to_home()
        with self._lock:
            self.compute_mode = 'success'
            self.follow_mode = 'abort'
            self.follow_abort_delay_s = 3.0
            self.compute_count = 0
            self.follow_goals.clear()
            self.states.clear()
            self.event_names.clear()
            self.clear_call_times.clear()
        self.costmap_auto = True
        self.set_obstacle(obstacle)
        self._enter_nav()
        self.assertTrue(
            self._wait_count(self._n_follow, 1, timeout=15.0),
            f'最初の FollowPath が届かない ({self._mode_state()})')
        if not self._wait_mode_state('PANEL_NAV', 'BLOCKED', timeout=15.0):
            self.fail(f'BLOCKED に入らない ({self._mode_state()})')
        return self._n_follow(), len(self._snap_events())

    def _assert_holds(self, f0, ev0, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.assertEqual(
                self._n_follow(), f0,
                f'経路の先が塞がれているのに FollowPath が再送された '
                f'({f0} → {self._n_follow()})。')
            self.assertEqual(
                self._mode_state(), ('PANEL_NAV', 'BLOCKED'),
                f'BLOCKED のはずが {self._mode_state()} になった。')
            time.sleep(0.25)
        self.assertNotIn('evt.unblocked', self._snap_events()[ev0:],
                         '塞がれたままなのに evt.unblocked が出た。')

    # ── (a) ───────────────────────────────────────────────
    def test_a_lethal_ahead_holds(self):
        f0, ev0 = self._go_blocked((*_NEAR, 100))
        # NAV に入るときの消去（§6.1）は BLOCKED の前に済んでいる。ここからの消去は
        # 塞がれた状態に入るごとに 1 回だけ（待っている間は何度も消さない）。
        time.sleep(0.2)
        with self._lock:
            n0 = len(self.clear_call_times)
        self._assert_holds(f0, ev0, _HOLD_S)
        with self._lock:
            n_clear = len(self.clear_call_times) - n0
        self.assertLessEqual(
            n_clear, 1,
            f'待っている間にコストマップを {n_clear} 回消した。塞がれた状態に入る'
            f'ごとに 1 回だけ（Spec-onsite §6.1）。')
        self._reset_to_home()

    def test_a2_inscribed_ahead_holds(self):
        """内接(99)も通れない。閾値を 100 にすると見逃す。"""
        f0, ev0 = self._go_blocked((*_NEAR, 99))
        self._assert_holds(f0, ev0, 8.0)
        self._reset_to_home()

    # ── (b) ───────────────────────────────────────────────
    def test_b_cleared_resends_and_returns_to_nav(self):
        f0, ev0 = self._go_blocked((*_NEAR, 100))
        self._assert_holds(f0, ev0, 3.0)
        with self._lock:
            mark = len(self.states)
        t0 = time.monotonic()
        self.set_obstacle(None)
        self.assertTrue(
            self._wait_count(self._n_follow, f0 + 1, timeout=5.0),
            '障害物を取り除いても FollowPath が再送されない。')
        self.assertLess(time.monotonic() - t0, 4.5, '再送が遅すぎる')
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if ('PANEL_NAV', 'NAV') in self._snap_states()[mark:]:
                break
            time.sleep(0.1)
        self.assertIn(('PANEL_NAV', 'NAV'), self._snap_states()[mark:],
                      '再送したのに NAV に戻らない。')
        self.assertIn('evt.unblocked', self._snap_events()[ev0:])
        self._reset_to_home()

    # ── (c) ───────────────────────────────────────────────
    def test_c_obstacle_beyond_range_resends(self):
        f0, ev0 = self._go_blocked((*_FAR, 100))
        self.assertTrue(
            self._wait_count(self._n_follow, f0 + 1, timeout=10.0),
            '障害物が 1.5 m より先なのに再送されない。')
        self._reset_to_home()

    def test_c2_inflation_value_is_passable(self):
        """膨張域(98)は通れる。致死/内接だけが障害物。"""
        f0, ev0 = self._go_blocked((*_NEAR, 98))
        self.assertTrue(
            self._wait_count(self._n_follow, f0 + 1, timeout=10.0),
            '膨張域(98)を障害物扱いして再送しない。')
        self._reset_to_home()

    # ── (d) ───────────────────────────────────────────────
    def test_d_no_costmap_holds(self):
        """障害物マップが（消去のあとに）一度も届かない → 空いていない扱いで待つ。
        setUp で 1 回流した空きのマップは消去より前なので、確認に使われない。"""
        self.costmap_auto = False
        self._reset_to_home()
        with self._lock:
            self.compute_mode = 'success'
            self.follow_mode = 'abort'
            self.follow_abort_delay_s = 3.0
            self.follow_goals.clear()
            self.states.clear()
            self.event_names.clear()
        self._enter_nav()
        self.assertTrue(self._wait_count(self._n_follow, 1, timeout=15.0))
        if not self._wait_mode_state('PANEL_NAV', 'BLOCKED', timeout=15.0):
            self.fail(f'BLOCKED に入らない ({self._mode_state()})')
        f0, ev0 = self._n_follow(), len(self._snap_events())
        self._assert_holds(f0, ev0, 10.0)
        # 陽性対照: マップが届けば（空きなので）再送される。
        self.set_obstacle(None)
        self.assertTrue(
            self._wait_count(self._n_follow, f0 + 1, timeout=6.0),
            'マップが届いて空いているのに再送されない（陽性対照）。')
        self._reset_to_home()

    # ── (e) ───────────────────────────────────────────────
    def test_e_reroute_and_stop_accepted_while_blocked(self):
        f0, ev0 = self._go_blocked((*_NEAR, 100))
        self._assert_holds(f0, ev0, 3.0)
        c0 = self._n_compute()
        self._call_trigger('ui.reroute')
        self.assertTrue(
            self._wait_count(self._n_compute, c0 + 1, timeout=5.0),
            'BLOCKED 中の「再検索」で経路を取り直さない。')
        self._reset_to_home()

    def test_e2_stop_accepted_while_blocked(self):
        f0, ev0 = self._go_blocked((*_NEAR, 100))
        self._assert_holds(f0, ev0, 3.0)
        self._call_trigger('ui.stop')
        if not self._wait_mode_state('PANEL_NAV', 'PAUSE', timeout=5.0):
            self.fail(f'BLOCKED 中の「停止」で PAUSE にならない ({self._mode_state()})')
        self._reset_to_home()
