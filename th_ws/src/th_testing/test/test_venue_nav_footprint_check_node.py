"""
test_venue_nav_footprint_check_node.py — 引いた経路を機体の形（0.55 m 角）で検査し、
通れない狭い隙間しか無いときは走らず BLOCKED で止まる
（Spec-onsite §6.1.1「引いた経路を機体の形で検査する」。ユーザー決定 2026-10-22・
実機 2026-10-22）。

実機の症状: Nav2 の経路計画は機体を点（内接円 0.275 m 相当）として扱うため、経路が
無いはずの場面で、機体が通れない狭い隙間に経路を引いてそこへ走ろうとした。

本番の `venue_navigator`＋`state_manager`＋`pin_registrar` を起動し、Nav2 の縁だけ
代役にする（共通部品 `_blocked_hold_harness.py`。障害物マップは実機と同じ
RELIABLE + TRANSIENT_LOCAL・`map` 系）。`ComputePathToPose` の代役は原点から行き先
(3, 0) への一直線を返し、障害物マップに y=0 を挟む壁（x 1.0〜2.0）を置いて通路にする。
共通部品は既定で検査を切っている（既存の試験が障害物を経路の上に置くため）。本試験
だけ `path_footprint_check: True` で起動する。

  (a) 壁面の間隔 0.50 m の隙間: FollowPath を送らず BLOCKED（evt.blocked）。
      待っている間も送らない（再検索のたびに同じ検査）。
  (b) 壁面の間隔 1.3 m・1.0 m の通路: 従来どおり FollowPath が送られ NAV。
  (c) 隙間が開く（壁が除かれる）と、再探索で経路を取り直して走り出し NAV に戻る。
  (d) BLOCKED 中の「再検索」（ui.reroute）も同じ検査: 狭いままなら送らず、
      広がれば送る。
"""

from __future__ import annotations

import time

import pytest

from _blocked_hold_harness import (BlockedHoldBase,  # noqa: F401
                                   _BLOCKED_PERIOD_S_DEFAULT, make_description)


@pytest.mark.launch_test
def generate_test_description():
    return make_description(_BLOCKED_PERIOD_S_DEFAULT,
                            {'path_footprint_check': True})


_LINE = [(0.1 * k, 0.0) for k in range(31)]   # 原点 → (3, 0)


def _corridor(gap: float):
    """x 1.0〜2.0 に y=0 を挟む壁。gap = 壁面の間隔 [m]（セル境界に合う値）。
    壁のセル中心は gap/2 + 0.025 以上に置く（セル中心 ≧ 壁面 + 0.025）。"""
    y = gap / 2.0 + 0.01
    return [(1.0, 2.0, y, 5.0, 100), (1.0, 2.0, -5.0, -y, 100)]


class TestVenueNavFootprintCheck(BlockedHoldBase):

    def _start(self, rects):
        """壁を置いて PANEL_NAV に入る（最初の経路計算まで）。"""
        self._reset_to_home()
        with self._lock:
            self.compute_mode = 'success'
            self.follow_mode = 'run'
            self.compute_count = 0
            self.follow_goals.clear()
            self.follow_cancel_count = 0
            self.states.clear()
            self.event_names.clear()
        self.compute_path = list(_LINE)
        self.costmap_auto = True
        self.costmap_rects = list(rects)
        self.set_obstacle(None)    # 壁を含めてすぐ配信する
        self._sleep(0.5)           # venue 側が壁入りのマップを受けてから入る
        self._enter_nav()

    def _assert_blocked_not_sent(self, seconds=3.0):
        self.assertTrue(
            self._wait_count(self._n_compute, 1, timeout=15.0),
            f'経路を計算しない ({self._mode_state()})')
        self.assertTrue(
            self._wait_mode_state('PANEL_NAV', 'BLOCKED', timeout=10.0),
            f'狭い隙間の経路で BLOCKED に入らない ({self._mode_state()})')
        self.assertIn('evt.blocked', self._snap_events())
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.assertEqual(
                self._n_follow(), 0,
                '機体が通れない隙間の経路で FollowPath が送られた。')
            self.assertEqual(self._mode_state(), ('PANEL_NAV', 'BLOCKED'))
            time.sleep(0.2)

    # ── (a) ───────────────────────────────────────────────
    def test_a_narrow_gap_goes_blocked_and_stays(self):
        self._start(_corridor(0.50))
        self._assert_blocked_not_sent(4.0)
        # 待っている間も再検索のたびに検査している（経路を取り直している）
        self.assertGreaterEqual(self._n_compute(), 2,
                                'BLOCKED の再探索で経路を取り直していない。')
        self.assertNotIn('evt.unblocked', self._snap_events())
        self._reset_to_home()

    # ── (b) ───────────────────────────────────────────────
    def test_b_wide_corridor_runs(self):
        for gap in (1.3, 1.0):
            self._start(_corridor(gap))
            self.assertTrue(
                self._wait_count(self._n_follow, 1, timeout=15.0),
                f'幅 {gap} m の通路で FollowPath が送られない '
                f'({self._mode_state()})')
            self.assertTrue(self._wait_mode_state('PANEL_NAV', 'NAV', timeout=5.0))
            self.assertNotIn('evt.blocked', self._snap_events())
            self._reset_to_home()

    # ── (c) ───────────────────────────────────────────────
    def test_c_gap_opens_resumes(self):
        self._start(_corridor(0.50))
        self._assert_blocked_not_sent(1.0)
        self.costmap_rects = []
        self.set_obstacle(None)
        self.assertTrue(
            self._wait_count(self._n_follow, 1, timeout=15.0),
            '隙間が開いたのに再探索で走り出さない（陽性対照）。')
        self.assertTrue(self._wait_mode_state('PANEL_NAV', 'NAV', timeout=8.0))
        self.assertIn('evt.unblocked', self._snap_events())
        self._reset_to_home()

    # ── (d) ───────────────────────────────────────────────
    def test_d_reroute_is_checked_too(self):
        self._start(_corridor(0.50))
        self._assert_blocked_not_sent(1.0)
        c0 = self._n_compute()
        self._call_trigger('ui.reroute')
        self.assertTrue(
            self._wait_count(self._n_compute, c0 + 1, timeout=5.0),
            'BLOCKED 中の「再検索」で経路を取り直さない。')
        self._sleep(1.5)
        self.assertEqual(self._n_follow(), 0,
                         '再検索で引き直した狭い隙間の経路が送られた。')
        # 広げて再検索 → 送られる（陽性対照）
        self.costmap_rects = _corridor(1.3)
        self.set_obstacle(None)
        self._sleep(1.5)
        self._call_trigger('ui.reroute')
        self.assertTrue(
            self._wait_count(self._n_follow, 1, timeout=15.0),
            '隙間を広げて再検索しても走り出さない（陽性対照）。')
        self._reset_to_home()
