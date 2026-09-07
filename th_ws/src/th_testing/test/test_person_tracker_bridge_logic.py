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
    AutoSelectState,
    STATUS_EXISTS_LEG,
    TRACKED_CONFIDENCE,
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
