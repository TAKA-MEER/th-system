"""test_wait_clear_core.py — 退避待ちゲートの純ロジックテスト"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'th_onsite'))

from th_onsite.wait_clear_core import (
    WaitClearParams, WaitClearState, remaining_sec, reset, step,
)


def _make_params(**kw):
    defaults = dict(clear_distance_m=1.0, clear_hold_ms=1500, clear_timeout_ms=30000)
    defaults.update(kw)
    return WaitClearParams(**defaults)


class TestClearOk:
    def test_clear_ok_fires_once_after_hold(self):
        """距離十分 (1.2 ≥ 1.0) を clear_hold_ms(1500) ぶん刻む →
        15 回目で evt.clear_ok が1回だけ出る、verdict が "WAITING"→"OK"。"""
        p = _make_params()
        s = reset()
        fired = []
        verdicts = []
        for i in range(15):
            s, v, ev = step(s, 100, 1.2, True, p)
            fired.extend(ev)
            verdicts.append(v)

        assert verdicts[0] == 'WAITING'
        assert verdicts[-1] == 'OK'
        assert fired == ['evt.clear_ok']

    def test_clear_ok_only_once(self):
        """clear_ok は1回だけ。""",
        p = _make_params()
        s = reset()
        fired = []
        for _ in range(30):
            s, _, ev = step(s, 100, 1.2, True, p)
            fired.extend(ev)
        assert fired == ['evt.clear_ok']


class TestDistanceInsufficient:
    def test_not_clear_when_distance_below_threshold(self):
        """距離不足 (0.5) でいくら刻んでも evt.clear_ok は出ず verdict="NOT_CLEAR"、
        hold_accum=0 のまま。"""
        p = _make_params()
        s = reset()
        fired = []
        for _ in range(30):
            s, v, ev = step(s, 100, 0.5, True, p)
            fired.extend(ev)
            assert v == 'NOT_CLEAR'
        assert s.hold_accum_ms == 0
        assert 'evt.clear_ok' not in fired


class TestTargetLostResetsHold:
    def test_hold_resets_on_target_lost(self):
        """target_visible=False を挟むと hold_accum が 0 に戻る。"""
        p = _make_params()
        s = reset()
        # 10 ティック蓄積 (1000ms)
        for _ in range(10):
            s, _, _ = step(s, 100, 1.2, True, p)
        assert s.hold_accum_ms == 1000

        # 見失い → reset
        s, v, _ = step(s, 100, None, False, p)
        assert s.hold_accum_ms == 0
        assert v == 'NOT_CLEAR'


class TestTimeout:
    def test_timeout_fires_once(self):
        """距離不足のまま dt 100ms を 300 回 → elapsed 30000ms で
        evt.clear_timeout が1回だけ。その後さらに刻んでも二重に出ない。"""
        p = _make_params()
        s = reset()
        fired = []
        for _ in range(350):
            s, _, ev = step(s, 100, 0.5, True, p)
            fired.extend(ev)

        timeout_events = [e for e in fired if e == 'evt.clear_timeout']
        assert timeout_events == ['evt.clear_timeout']

    def test_timeout_not_fired_when_clear_ok_first(self):
        """clear_ok が先に出ていれば、elapsed が timeout を超えても
        evt.clear_timeout は出ない。"""
        p = _make_params()
        s = reset()
        fired = []
        for _ in range(15):
            s, _, ev = step(s, 100, 1.2, True, p)
            fired.extend(ev)
        # 15 ティックで clear_ok 発火済み
        assert 'evt.clear_ok' in fired
        # さらに 300 ティック (残り 28500ms + 元の 1500ms = 30000ms)
        for _ in range(350):
            s, _, ev = step(s, 100, 1.2, True, p)
            fired.extend(ev)
        timeout_events = [e for e in fired if e == 'evt.clear_timeout']
        assert timeout_events == []


class TestRemainingSec:
    def test_remaining_sec_basic(self):
        """elapsed 10000ms / timeout 30000ms → 20.0。"""
        p = _make_params()
        s = WaitClearState(hold_accum_ms=0, elapsed_ms=10000,
                           fired_ok=False, fired_timeout=False)
        assert remaining_sec(s, p) == 20.0

    def test_remaining_sec_clamps_at_zero(self):
        p = _make_params()
        s = WaitClearState(hold_accum_ms=0, elapsed_ms=35000,
                           fired_ok=False, fired_timeout=False)
        assert remaining_sec(s, p) == 0.0


class TestVerdictTransitions:
    def test_verdict_ok_only_after_hold(self):
        """距離十分でも hold 達成前は WAITING。"""
        p = _make_params()
        s = reset()
        for _ in range(14):
            s, v, _ = step(s, 100, 1.2, True, p)
            assert v == 'WAITING'
        s, v, _ = step(s, 100, 1.2, True, p)
        assert v == 'OK'
