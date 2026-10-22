"""
test_venue_nav_goto_while_nav_node.py — 移動中・一時停止中は行き先を選べない
（Spec-modes.md SM-3.1.2-118。2026-10-22 実機 T-10）。

手押し(CARRY)から「再開」で戻った `PANEL_NAV/PAUSE` で画面の行き先を押すと、
`/onsite/select_pin` だけが通って保留中の行き先（`_pending_goal`）が書き換わり、
続く `ui.goto` は FSM に受け付ける遷移が無く黙って捨てられていた。
経路のキャッシュが無い再開（`_start_nav`）は `_pending_goal` へ走るので、
別の行き先へ走り出す恐れがあった。

本番の `venue_navigator`＋`state_manager`＋`pin_registrar` を起動し、Nav2 の縁だけ
代役にする（共通部品 `_blocked_hold_harness.py`）。盤ピン p1=(3,0)・待機場所 home1=(-3,0)。

  (a) PANEL_NAV/PAUSE: select_pin(home1) は success=False（理由は日本語）、
      ui.goto{HOME}/{PANEL} も goto_while_nav で拒否、状態は PANEL_NAV/PAUSE のまま。
  (b) PANEL_NAV/NAV でも同じ（移動中）。
  (c) 拒否したあと ui.abort → AT_HOME/IDLE_H に戻り、select_pin 抜きで ui.goto{PANEL} すると
      保留ゴールは元の p1 のまま（経路の終点が (3,0)。home1 の (-3,0) に書き換わっていない）。
  (d) AT_HOME/IDLE_H では select_pin が受理される（選べる状態を巻き込んでいない）。
"""

from __future__ import annotations

import pytest

from _blocked_hold_harness import (BlockedHoldBase,  # noqa: F401
                                   _BLOCKED_PERIOD_S_DEFAULT, _HOME_ID,
                                   _PIN_ID, make_description)
from th_system_msgs.srv import GoToPanel, UiTrigger


@pytest.mark.launch_test
def generate_test_description():
    return make_description(_BLOCKED_PERIOD_S_DEFAULT)


class TestVenueNavGotoWhileNav(BlockedHoldBase):

    def _enter_nav_running(self):
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

    def _raw_select_pin(self, panel_id: str):
        req = GoToPanel.Request()
        req.panel_id = panel_id
        return self._call_srv(GoToPanel, '/onsite/select_pin', req)

    def _raw_trigger(self, trigger: str, arg_json: str = '{}'):
        """拒否も含めて 1 回だけ送って応答を返す（_call_trigger は受理まで押し直す）。"""
        req = UiTrigger.Request()
        req.trigger = trigger
        req.arg_json = arg_json
        req.requester = 'vnbh_test'
        return self._call_srv(UiTrigger, '/system/trigger', req)

    def _assert_goto_rejected(self, mode_state):
        res = self._raw_select_pin(_HOME_ID)
        self.assertIsNotNone(res)
        self.assertFalse(
            res.success,
            f'{mode_state} で select_pin が通った。保留ゴールが書き換わる。')
        self.assertTrue(res.message, '拒否の理由（日本語）が空')
        for kind in ('HOME', 'PANEL'):
            r = self._raw_trigger('ui.goto', '{"kind":"%s"}' % kind)
            self.assertFalse(r.accepted, f'{mode_state} で ui.goto{{{kind}}} が受理された')
            self.assertEqual(r.reject_reason_key, 'goto_while_nav')
        self.assertEqual(self._mode_state(), mode_state, '拒否したのに状態が変わった')

    # ── (a) PAUSE ─────────────────────────────────────────
    def test_a_pause_rejects_select_pin_and_goto(self):
        self._enter_nav_running()
        c0 = self._n_cancel()
        self._call_trigger('ui.stop')
        self.assertTrue(self._wait_mode_state('PANEL_NAV', 'PAUSE', timeout=5.0),
                        f'PAUSE に入らない ({self._mode_state()})')
        self.assertTrue(self._wait_count(self._n_cancel, c0 + 1, timeout=5.0))
        self._assert_goto_rejected(('PANEL_NAV', 'PAUSE'))
        self._assert_pending_goal_unchanged()

    # ── (b) NAV ───────────────────────────────────────────
    def test_b_nav_rejects_select_pin_and_goto(self):
        self._enter_nav_running()
        self._assert_goto_rejected(('PANEL_NAV', 'NAV'))
        self._assert_pending_goal_unchanged()

    # ── (c) 保留ゴールが p1 のまま ─────────────────────────
    def _assert_pending_goal_unchanged(self):
        """中断して AT_HOME に戻り、select_pin 抜きで ui.goto{PANEL}。保留が home1 に
        書き換わっていれば経路の終点が (-3,0) になる。"""
        self._reset_to_home()
        self.assertEqual(self._mode_state(), ('AT_HOME', 'IDLE_H'))
        n0 = self._n_follow()
        self._call_trigger('ui.goto', '{"kind":"PANEL"}')
        self.assertTrue(self._wait_count(self._n_follow, n0 + 1, timeout=15.0),
                        f'再度の FollowPath が届かない ({self._mode_state()})')
        with self._lock:
            end = self.follow_goals[-1][-1]
        self.assertAlmostEqual(end[0], 3.0, delta=0.05,
                               msg=f'保留ゴールが p1 から書き換わっている（終点 {end}）')
        self.assertAlmostEqual(end[1], 0.0, delta=0.05)
        self._reset_to_home()

    # ── (d) 選べる状態では受理 ─────────────────────────────
    def test_d_at_home_accepts_select_pin(self):
        self._reset_to_home()
        self.assertEqual(self._mode_state(), ('AT_HOME', 'IDLE_H'))
        for pid in (_PIN_ID, _HOME_ID):
            res = self._raw_select_pin(pid)
            self.assertTrue(res.success, f'AT_HOME/IDLE_H で select_pin({pid}) が拒否された')
