"""
test_venue_nav_venue_unloaded_node.py — 会場地図を持たない間は自律移動しない
（Spec-onsite.md §4.0.3・Spec-modes.md SM-3.1.2-119。2026-10-23 実機）。

盤前移動中に自己位置推定を終了させると `LOCALIZATION_LOST` で非常停止し、推定は数秒で
立ち上がり直したが、会場地図を持たない（`/map_session/status` の VENUE が `UNLOADED`）
まっさらな地図からだった。非常停止から戻って「走行」を押すと、別の原点の地図の上で会場座標の
ピンへ走り出す。機体側で塞ぐ（画面だけに頼らない）。

本番の `venue_navigator`＋`state_manager`＋`pin_registrar` を起動し、Nav2 の縁だけ代役にする
（共通部品 `_blocked_hold_harness.py`）。`slam_control` の代役として、この試験が
`/map_session/status`（RELIABLE＋TRANSIENT_LOCAL）を 5 Hz で流す。盤ピン p1=(3,0)・
待機場所 home1=(-3,0)。

  (a) PANEL_NAV/PAUSE で UNLOADED: 「走行」(ui.run) も W-1 の「はい」(ui.resume_yes) も
      venue_not_loaded で拒否、状態は PANEL_NAV/PAUSE のまま、ComputePath／FollowPath は
      1 回も増えない。LOCALIZING に戻すと「走行」が通って FollowPath が送られる。
  (b) AT_HOME/IDLE_H で UNLOADED: select_pin は success=False（日本語の理由）、
      ui.goto{PANEL/HOME/SUMMON} は venue_not_loaded で拒否、状態は変わらず
      ComputePath は増えない。拒否のあいだ保留ゴール(p1)は書き換わらない。
  (c) LOCALIZING に戻すと行き先が通る。MAPPING（前日 PREP の地図作成中）と、状態を一度も
      受け取っていない間は塞がない。
"""

from __future__ import annotations

import time

import pytest

from _blocked_hold_harness import (BlockedHoldBase,  # noqa: F401
                                   _BLOCKED_PERIOD_S_DEFAULT, _HOME_ID,
                                   _PIN_ID, _tl_qos, make_description)
from th_system_msgs.msg import MapSessionStatus
from th_system_msgs.srv import GoToPanel, UiTrigger


@pytest.mark.launch_test
def generate_test_description():
    return make_description(_BLOCKED_PERIOD_S_DEFAULT)


class TestVenueNavVenueUnloaded(BlockedHoldBase):

    def setUp(self):
        super().setUp()
        # slam_control の代役。公開値は self._venue_mode で切り替え、5 Hz で流し続ける
        # （本物は publish タイマで毎周期流す）。試験ごとに必ず LOCALIZING から始める
        # （前の試験が UNLOADED のまま終わっても state_manager／venue_navigator は
        # 最後の値を覚えているため）。
        self._venue_mode = 'LOCALIZING'
        self._pub_status = self.node.create_publisher(
            MapSessionStatus, '/map_session/status', _tl_qos())
        self._status_timer = self.node.create_timer(0.2, self._publish_status)
        self._publish_status()
        time.sleep(1.0)

    def tearDown(self):
        try:
            self.node.destroy_timer(self._status_timer)
        except Exception:
            pass
        # 次の試験の setUp が LOCALIZING を流し直すまでの橋渡し。
        self._set_venue('LOCALIZING')
        super().tearDown()

    def _publish_status(self):
        msg = MapSessionStatus()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.slot = 'VENUE'
        msg.session_id = 'venue'
        msg.mode = self._venue_mode
        msg.instance_id = 'vnvu'
        self._pub_status.publish(msg)

    def _set_venue(self, mode: str):
        self._venue_mode = mode
        self._publish_status()
        time.sleep(1.0)   # state_manager／venue_navigator が受け取る猶予

    def _raw_select_pin(self, panel_id: str):
        req = GoToPanel.Request()
        req.panel_id = panel_id
        return self._call_srv(GoToPanel, '/onsite/select_pin', req)

    def _raw_trigger(self, trigger: str, arg_json: str = '{}'):
        """拒否も含めて 1 回だけ送って応答を返す（_call_trigger は受理まで押し直す）。"""
        req = UiTrigger.Request()
        req.trigger = trigger
        req.arg_json = arg_json
        req.requester = 'vnvu_test'
        return self._call_srv(UiTrigger, '/system/trigger', req)

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

    # ── (a) PAUSE からの「走行」 ───────────────────────────
    def test_a_pause_run_rejected_without_venue_map(self):
        self._enter_nav_running()
        c0 = self._n_cancel()
        self._call_trigger('ui.stop')
        self.assertTrue(self._wait_mode_state('PANEL_NAV', 'PAUSE', timeout=5.0),
                        f'PAUSE に入らない ({self._mode_state()})')
        self.assertTrue(self._wait_count(self._n_cancel, c0 + 1, timeout=5.0))
        n_compute, n_follow = self._n_compute(), self._n_follow()

        self._set_venue('UNLOADED')
        for trig in ('ui.run', 'ui.resume_yes'):
            res = self._raw_trigger(trig)
            self.assertIsNotNone(res)
            self.assertFalse(res.accepted,
                             f'会場地図が無いのに {trig} が受理された（別の原点の地図で走り出す）')
            self.assertEqual(res.reject_reason_key, 'venue_not_loaded', trig)
        self._sleep(2.0)
        self.assertEqual(self._mode_state(), ('PANEL_NAV', 'PAUSE'), '拒否したのに状態が変わった')
        self.assertEqual(self._n_follow(), n_follow, 'FollowPath が送られた')
        self.assertEqual(self._n_compute(), n_compute, 'ComputePath が呼ばれた')

        # 会場地図が戻れば「走行」は通り、残りの経路が送られる。
        self._set_venue('LOCALIZING')
        self._call_trigger('ui.run')
        self.assertTrue(self._wait_mode_state('PANEL_NAV', 'NAV', timeout=5.0),
                        f'読み込み済みなのに走行できない ({self._mode_state()})')
        self.assertTrue(self._wait_count(self._n_follow, n_follow + 1, timeout=10.0),
                        '読み込み済みなのに FollowPath が再送されない')
        self._reset_to_home()

    # ── (b) 行き先を選ぶ ───────────────────────────────────
    def test_b_goto_rejected_without_venue_map(self):
        # AT_HOME に居る状態から始める（単独実行でも INIT/IDLE のままにならないよう、
        # 一度移動を始めて「中断」で戻る）。
        self._enter_nav_running()
        self._reset_to_home()
        self.assertEqual(self._mode_state(), ('AT_HOME', 'IDLE_H'))
        # 保留ゴールを p1 にしておく（読み込み済みのうちに）。
        self.assertTrue(self._raw_select_pin(_PIN_ID).success)
        with self._lock:
            self.compute_count = 0
        n_follow = self._n_follow()

        self._set_venue('UNLOADED')
        res = self._raw_select_pin(_HOME_ID)
        self.assertIsNotNone(res)
        self.assertFalse(res.success, '会場地図が無いのに select_pin が通った（保留ゴールが書き換わる）')
        self.assertTrue(res.message, '拒否の理由（日本語）が空')
        for kind in ('PANEL', 'HOME', 'SUMMON'):
            r = self._raw_trigger('ui.goto', '{"kind":"%s"}' % kind)
            self.assertFalse(r.accepted, f'会場地図が無いのに ui.goto{{{kind}}} が受理された')
            self.assertEqual(r.reject_reason_key, 'venue_not_loaded', kind)
        self._sleep(1.5)
        self.assertEqual(self._mode_state(), ('AT_HOME', 'IDLE_H'), '拒否したのに状態が変わった')
        self.assertEqual(self._n_compute(), 0, 'ComputePath が呼ばれた')
        self.assertEqual(self._n_follow(), n_follow, 'FollowPath が送られた')

        # 会場地図が戻れば行き先は通り、保留ゴールは拒否された home1 でなく p1 のまま。
        self._set_venue('LOCALIZING')
        self._call_trigger('ui.goto', '{"kind":"PANEL"}')
        self.assertTrue(self._wait_count(self._n_follow, n_follow + 1, timeout=15.0),
                        f'読み込み済みなのに FollowPath が届かない ({self._mode_state()})')
        with self._lock:
            end = self.follow_goals[-1][-1]
        self.assertAlmostEqual(end[0], 3.0, delta=0.05,
                               msg=f'保留ゴールが p1 から書き換わっている（終点 {end}）')
        self.assertAlmostEqual(end[1], 0.0, delta=0.05)
        self._reset_to_home()

    # ── (c) 塞いではいけない場面 ───────────────────────────
    def test_c_mapping_does_not_block(self):
        """前日 PREP の地図作成中（MAPPING）は会場地図を「持たない」のではなく作っている最中。"""
        self._reset_to_home()
        self._set_venue('MAPPING')
        self.assertTrue(self._raw_select_pin(_PIN_ID).success)
        r = self._raw_trigger('ui.goto', '{"kind":"PANEL"}')
        self.assertTrue(r.accepted, f'MAPPING で行き先が拒否された ({r.reject_reason_key})')
        self.assertTrue(self._wait_count(self._n_follow, 1, timeout=15.0))
        self._reset_to_home()
