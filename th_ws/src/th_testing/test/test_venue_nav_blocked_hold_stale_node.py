"""
test_venue_nav_blocked_hold_stale_node.py — 古い障害物マップでは再送しない
（Spec-onsite §6.3「障害物マップが確認できないとき」。約 5 秒以上更新が無い）。

再探索の周期を 8 秒に伸ばす（`blocked_recheck_period_s`）。消去の直後の周期 T で
消去し、T+1.1 秒に空きのマップを 1 回だけ流す（消去後のマップなので確認に使える）。
次の周期 T+8 ではそのマップの受信から 6.9 秒たっている = 古い。古さの判定が
無ければここで再送される。陽性対照で新しいマップを流すと次の周期で再送される。
共通部品は `_blocked_hold_harness.py`。
"""

from __future__ import annotations

import time

import pytest

from _blocked_hold_harness import BlockedHoldBase, make_description


@pytest.mark.launch_test
def generate_test_description():
    return make_description(8.0)


class TestVenueNavBlockedHoldStale(BlockedHoldBase):

    def test_stale_costmap_holds(self):
        self.costmap_auto = False
        self._reset_to_home()
        with self._lock:
            self.compute_mode = 'success'
            self.follow_mode = 'abort'
            self.follow_abort_delay_s = 3.0
            self.follow_goals.clear()
            self.states.clear()
            self.event_names.clear()
            self.clear_call_times.clear()
        self._enter_nav()
        self.assertTrue(self._wait_count(self._n_follow, 1, timeout=15.0))
        if not self._wait_mode_state('PANEL_NAV', 'BLOCKED', timeout=15.0):
            self.fail(f'BLOCKED に入らない ({self._mode_state()})')
        f0, ev0 = self._n_follow(), len(self._snap_events())
        with self._lock:
            self.clear_call_times.clear()  # NAV 入口の消去は数えない

        # 消去（周期 T）を待ち、その 1.1 秒後に空きのマップを 1 回だけ流す。
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            with self._lock:
                cleared = bool(self.clear_call_times)
            if cleared:
                break
            time.sleep(0.05)
        self.assertTrue(cleared, 'コストマップの消去が呼ばれない')
        with self._lock:
            t_clear = self.clear_call_times[0]
        time.sleep(max(0.0, t_clear + 1.1 - time.monotonic()))
        self.set_obstacle(None)
        t_map = time.monotonic()

        # 周期 T+8 と T+16 を越えるまで（マップ受信から 12 秒）再送されない。
        while time.monotonic() < t_map + 12.0:
            self.assertEqual(
                self._n_follow(), f0,
                f'マップが {time.monotonic() - t_map:.1f} 秒更新されないのに再送された。')
            time.sleep(0.25)
        self.assertEqual(self._mode_state(), ('PANEL_NAV', 'BLOCKED'))
        self.assertNotIn('evt.unblocked', self._snap_events()[ev0:])

        # 陽性対照: 新しいマップを流すと次の周期で再送される。
        self.set_obstacle(None)
        self.assertTrue(
            self._wait_count(self._n_follow, f0 + 1, timeout=10.0),
            '新しいマップが届いて空いているのに再送されない（陽性対照）。')
        self._reset_to_home()
