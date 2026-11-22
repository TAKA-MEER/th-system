"""
test_venue_nav_summon_retry_node.py — 呼び寄せのやり直し（A-12）の
launch_testing 試験。本番の `state_manager`・`pin_registrar`・`venue_navigator`
を起動し、Nav2 の縁だけ代役にする（共通部品は `_blocked_hold_harness.py`）。

縛るもの:
  1. `POINT` に戻ったら 2 点指示の受付が開き直る（`C-09c-summon` の
     `begin_two_point`）。非常停止 → 解除の「戻る」で `POINT` へ戻した後、
     `/onsite/two_point` の P1 が `no_pending` で拒否されないこと。
  2. 前の行き先を捨てる（`venue_navigator._on_state`）。`POINT` 入場前に古い
     `/onsite/summon_goal` があっても、2 点指示をやり直さずに `NAV` へ入ると
     走り出さない（`ComputePathToPose`／`FollowPath` が呼ばれない）。
  3. `POINT` のジョグは `PAUSE` へ落ちない（`T-SUM-18/19`。`SM-3.1.2-143`）。
"""

from __future__ import annotations

import pytest

from th_system_msgs.srv import TwoPointPress

from _blocked_hold_harness import (BlockedHoldBase,  # noqa: F401
                                   _BLOCKED_PERIOD_S_DEFAULT, make_description)


@pytest.mark.launch_test
def generate_test_description():
    return make_description(_BLOCKED_PERIOD_S_DEFAULT)


class TestVenueNavSummonRetry(BlockedHoldBase):

    def _two_point(self, index: int):
        req = TwoPointPress.Request()
        req.purpose = 'SUMMON'
        req.index = index
        return self._call_srv(TwoPointPress, '/onsite/two_point', req)

    def _enter_point(self):
        self._call_trigger('ui.goto', '{"kind":"SUMMON"}')
        if not self._wait_mode_state('SUMMON', 'POINT', timeout=10.0):
            self.fail(f'SUMMON/POINT に入らない ({self._mode_state()})')

    def _reset_counts(self):
        with self._lock:
            self.compute_mode = 'success'
            self.follow_mode = 'run'
            self.compute_count = 0
            self.follow_goals.clear()
            self.follow_cancel_count = 0
            self.event_names.clear()

    def test_stale_goal_is_discarded_on_point_entry(self):
        """POINT 入場で前の行き先を捨てる。捨てなければ古いゴールへ走り出す。"""
        self._reset_to_home()
        self._reset_counts()
        # POINT に入る前に古いゴールを置く（前回の呼び寄せの残り）。
        self._publish_summon_goal(3.0, 0.0)
        self._sleep(0.3)
        self._enter_point()
        self._sleep(1.0)  # 入場時の破棄が届くのを待つ
        self._publish_setup_event('evt.two_point_done')
        if not self._wait_mode_state('SUMMON', 'WAIT_CLEAR', timeout=5.0):
            self.fail(f'SUMMON/WAIT_CLEAR に入らない ({self._mode_state()})')
        self._publish_setup_event('evt.clear_ok')
        # ゴールが無ければ NAV 入口で evt.blocked → BLOCKED（走らない）。
        if not self._wait_mode_state('SUMMON', 'BLOCKED', timeout=10.0):
            self.fail(f'古い行き先が残っていて BLOCKED にならない '
                      f'({self._mode_state()})')
        self._sleep(1.0)
        self.assertEqual(self._n_compute(), 0,
                         '古い行き先で経路を計算した（POINT 入場で捨てていない）')
        self.assertEqual(self._n_follow(), 0,
                         '古い行き先で FollowPath を送った（POINT 入場で捨てていない）')

    def test_point_return_reopens_two_point_and_jog_stays_in_point(self):
        """非常停止から POINT へ戻した後も 2 点指示を受け付け、ジョグでも PAUSE に落ちない。"""
        self._reset_to_home()
        self._reset_counts()
        self._publish_person(1.0, 0.0)
        self._enter_point()
        self.assertTrue(self._two_point(1).accepted, '最初の P1 が拒否された')
        self._publish_person(1.6, 0.0)
        r2 = self._two_point(2)
        self.assertTrue(r2.accepted, f'最初の P2 が拒否された: {r2.reject_reason_key}')
        if not self._wait_mode_state('SUMMON', 'WAIT_CLEAR', timeout=5.0):
            self.fail(f'SUMMON/WAIT_CLEAR に入らない ({self._mode_state()})')

        self._call_trigger('ui.estop.press')
        if not self._wait_mode_state('ESTOP', 'NONE', timeout=5.0):
            self.fail(f'非常停止に入らない ({self._mode_state()})')
        self._call_trigger('ui.estop.release')
        self._call_trigger('ui.resume_yes')
        if not self._wait_mode_state('SUMMON', 'POINT', timeout=10.0):
            self.fail(f'非常停止の解除で POINT に戻らない ({self._mode_state()})')

        # C-09c-summon が begin_two_point を出していなければ no_pending になる。
        self._publish_person(1.0, 0.0)
        r = self._two_point(1)
        self.assertTrue(
            r.accepted,
            f'POINT に戻ったのに 2 点指示が拒否された: {r.reject_reason_key}')

        follow0, compute0 = self._n_follow(), self._n_compute()
        self._call_trigger('ui.jog.hold')
        self._sleep(1.5)  # リース満了（sys.jog_lease_expired）も跨ぐ
        self.assertEqual(
            self._mode_state(), ('SUMMON', 'POINT'),
            f'POINT のジョグで状態が動いた: {self._mode_state()}')
        self.assertEqual(self._n_follow(), follow0,
                         'POINT のジョグで FollowPath が呼ばれた')
        self.assertEqual(self._n_compute(), compute0,
                         'POINT のジョグで経路を計算した')
