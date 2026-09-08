"""
test_person_tracker_bridge_logic.py
=====================================
person_tracker_bridge_core の単体テスト（classify / match_selected_index /
auto_select_step）。ROS2 なし・純粋 Python で実行可能。
"""
import pytest

from person_tracker_bridge_core import (
    classify,
    match_selected_index,
    auto_select_step,
    apply_lost_grace,
    AutoSelectState,
    BridgeDecision,
    LostGraceState,
    STATUS_EXISTS_LEG,
    TRACKED_CONFIDENCE,
    GRACE_CONFIDENCE,
)

STATUS_NO_EXISTS = 0


class TestClassify:
    def test_no_exists_is_detection_lost(self):
        out = classify(STATUS_NO_EXISTS, stop_following=False)
        assert out.is_lost is True
        assert out.lost_reason == "DETECTION_LOST"
        assert out.confidence == 0.0

    def test_no_exists_is_detection_lost_regardless_of_stop_following(self):
        out = classify(STATUS_NO_EXISTS, stop_following=True)
        assert out.is_lost is True
        assert out.lost_reason == "DETECTION_LOST"

    def test_exists_leg_without_stop_following_tracks_normally(self):
        out = classify(STATUS_EXISTS_LEG, stop_following=False)
        assert out.is_lost is False
        assert out.lost_reason == ""
        assert out.confidence == pytest.approx(TRACKED_CONFIDENCE)

    def test_exists_leg_with_stop_following_is_target_switched(self):
        """回帰テスト: 対象切替の疑い(stop_following)が EXISTS_LEG のまま
        追従ロジックへ伝わらず、間違った対象を追い続けてしまっていた不具合の修正確認。"""
        out = classify(STATUS_EXISTS_LEG, stop_following=True)
        assert out.is_lost is True
        assert out.lost_reason == "TARGET_SWITCHED"
        assert out.confidence == 0.0


class TestMatchSelectedIndex:
    def test_nearest_candidate_index_0(self):
        cands = [(1.0, 0.0), (2.0, 0.0)]
        assert match_selected_index(cands, (1.05, 0.0), is_lost=False) == 0

    def test_nearest_candidate_index_1(self):
        cands = [(1.0, 0.0), (2.0, 0.0)]
        assert match_selected_index(cands, (1.9, 0.0), is_lost=False) == 1

    def test_lost_returns_minus_1(self):
        cands = [(1.0, 0.0), (2.0, 0.0)]
        assert match_selected_index(cands, (1.05, 0.0), is_lost=True) == -1

    def test_no_followed_xy_returns_minus_1(self):
        cands = [(1.0, 0.0), (2.0, 0.0)]
        assert match_selected_index(cands, None, is_lost=False) == -1

    def test_beyond_tol_returns_minus_1(self):
        """最近傍が (1,0) でも followed=(1,1)（距離1.0 > tol0.35）なら -1"""
        cands = [(1.0, 0.0), (2.0, 0.0)]
        assert match_selected_index(cands, (1.0, 1.0), is_lost=False) == -1

    def test_custom_tol_far_candidate_selected(self):
        """tol を大きくすると遠い候補も選択される（0.35 以外の既定パラメータ確認）"""
        cands = [(1.0, 0.0)]
        assert match_selected_index(cands, (1.0, 0.5), is_lost=False, match_tol_m=0.6) == 0

    def test_empty_candidates_returns_minus_1(self):
        assert match_selected_index([], (1.0, 0.0), is_lost=False) == -1


class TestAutoSelectStep:
    def test_fires_after_hold_and_no_refire(self):
        """n=1 を hold_ms=2000 ぶん刻む → 2000ms 時点で index 0、以後 -1"""
        s = AutoSelectState()
        state, idx = auto_select_step(s, 1, now_ms=0, hold_ms=2000, already_selected=False)
        assert idx == -1          # 開始直後はまだ
        state, idx = auto_select_step(state, 1, now_ms=1000, hold_ms=2000, already_selected=False)
        assert idx == -1
        state, idx = auto_select_step(state, 1, now_ms=2000, hold_ms=2000, already_selected=False)
        assert idx == 0           # hold 到達で発火
        state, idx = auto_select_step(state, 1, now_ms=3000, hold_ms=2000, already_selected=False)
        assert idx == -1          # 1人継続でも再発火しない
        assert state.fired is True
        # fired ガードの検証: 発火後さらに 2*hold_ms 経っても n=1 継続なら二度と発火しない
        # （single_since_ms のリセットだけでは hold_ms 後に再発火してしまう）
        for t in (5000, 7000, 9000):
            state, idx = auto_select_step(state, 1, now_ms=t, hold_ms=2000, already_selected=False)
            assert idx == -1, f'now_ms={t} で再発火した'

    def test_n2_resets_and_restarts_timing(self):
        """途中で n=2 を挟むと single_since_ms がリセットされ、再度 n=1 から数え直し"""
        s = AutoSelectState()
        state, idx = auto_select_step(s, 1, now_ms=0, hold_ms=2000, already_selected=False)
        assert idx == -1
        state, idx = auto_select_step(state, 1, now_ms=1500, hold_ms=2000, already_selected=False)
        assert idx == -1
        state, idx = auto_select_step(state, 2, now_ms=1500, hold_ms=2000, already_selected=False)
        assert idx == -1
        assert state.single_since_ms is None  # リセットされた
        state, idx = auto_select_step(state, 1, now_ms=1600, hold_ms=2000, already_selected=False)
        assert idx == -1
        state, idx = auto_select_step(state, 1, now_ms=3600, hold_ms=2000, already_selected=False)
        assert idx == 0  # n=2 でリセット後、now=1600 から再スタートして 2000ms 到達

    def test_already_selected_blocks(self):
        """already_selected=True なら n=1 でも -1"""
        s = AutoSelectState()
        state, idx = auto_select_step(s, 1, now_ms=0, hold_ms=2000, already_selected=True)
        assert idx == -1
        state, idx = auto_select_step(state, 1, now_ms=2000, hold_ms=2000, already_selected=True)
        assert idx == -1


class TestApplyLostGrace:
    """brief-onsite-ux2 F-4: is_lost のデバウンス（猶予）。実機で確認された
    「脚検出が歩行中に一瞬抜けて evt.target_lost が点滅する」を防ぐ。"""

    def test_tracked_updates_last_seen_and_passes_through(self):
        s = LostGraceState()
        tracked = BridgeDecision(is_lost=False, lost_reason="", confidence=TRACKED_CONFIDENCE)
        state, out = apply_lost_grace(s, tracked, now_ms=1000, grace_ms=1500)
        assert out == tracked
        assert state.last_seen_ms == 1000

    def test_lost_within_grace_is_masked_to_not_lost(self):
        """検出済み(last_seen_ms=1000)から grace_ms=1500 未満(now=2000)の lost は
        is_lost=False に倒す（点滅を吸収する本体）。"""
        s = LostGraceState(last_seen_ms=1000)
        lost = BridgeDecision(is_lost=True, lost_reason="DETECTION_LOST", confidence=0.0)
        state, out = apply_lost_grace(s, lost, now_ms=2000, grace_ms=1500)
        assert out.is_lost is False
        assert out.lost_reason == ""
        assert out.confidence == pytest.approx(GRACE_CONFIDENCE)
        # 猶予中は last_seen_ms を更新しない(実際には見えていないので残り猶予は減り続ける)
        assert state.last_seen_ms == 1000

    def test_lost_past_grace_is_confirmed(self):
        """grace_ms を過ぎたら decision をそのまま通す(confirmed lost)。"""
        s = LostGraceState(last_seen_ms=1000)
        lost = BridgeDecision(is_lost=True, lost_reason="DETECTION_LOST", confidence=0.0)
        state, out = apply_lost_grace(s, lost, now_ms=2500, grace_ms=1500)
        assert out == lost
        assert out.is_lost is True

    def test_lost_exactly_at_grace_boundary_is_confirmed(self):
        """境界(now - last_seen == grace_ms)は「猶予未満」ではないので confirmed。"""
        s = LostGraceState(last_seen_ms=1000)
        lost = BridgeDecision(is_lost=True, lost_reason="DETECTION_LOST", confidence=0.0)
        state, out = apply_lost_grace(s, lost, now_ms=2500, grace_ms=1500)
        assert out.is_lost is True

    def test_never_seen_yet_has_no_grace(self):
        """起動直後、まだ一度も検出できていない(last_seen_ms is None)なら猶予の
        起点が無いのでそのまま lost(誤って「猶予中」を演出しない)。"""
        s = LostGraceState()
        lost = BridgeDecision(is_lost=True, lost_reason="DETECTION_LOST", confidence=0.0)
        state, out = apply_lost_grace(s, lost, now_ms=500, grace_ms=1500)
        assert out.is_lost is True
        assert state.last_seen_ms is None

    def test_recovery_within_grace_never_surfaces_as_lost(self):
        """猶予内に再検出されれば、呼び出し側の edge 検出(is_lost の
        False→True→False)は一度も True を見ない = evt.target_lost は出ない。"""
        s = LostGraceState()
        tracked = BridgeDecision(is_lost=False, lost_reason="", confidence=TRACKED_CONFIDENCE)
        lost = BridgeDecision(is_lost=True, lost_reason="DETECTION_LOST", confidence=0.0)

        seen_is_lost = []
        s, out = apply_lost_grace(s, tracked, now_ms=0, grace_ms=1500)
        seen_is_lost.append(out.is_lost)
        s, out = apply_lost_grace(s, lost, now_ms=200, grace_ms=1500)   # 一瞬抜け
        seen_is_lost.append(out.is_lost)
        s, out = apply_lost_grace(s, tracked, now_ms=400, grace_ms=1500)  # すぐ復帰
        seen_is_lost.append(out.is_lost)

        assert seen_is_lost == [False, False, False]

    def test_grace_expires_then_confirmed_lost_stays_lost(self):
        """猶予切れ後は再検出が無い限り lost のまま(時間が経つほど確定側に倒れる)。"""
        s = LostGraceState(last_seen_ms=0)
        lost = BridgeDecision(is_lost=True, lost_reason="DETECTION_LOST", confidence=0.0)
        s, out1 = apply_lost_grace(s, lost, now_ms=2000, grace_ms=1500)  # 猶予切れ
        assert out1.is_lost is True
        s, out2 = apply_lost_grace(s, lost, now_ms=10000, grace_ms=1500)  # さらに後
        assert out2.is_lost is True
