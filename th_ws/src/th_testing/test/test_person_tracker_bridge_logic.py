"""
test_person_tracker_bridge_logic.py
======================================
person_tracker_bridge_core.classify() の単体テスト。
ROS2 なし・純粋 Python で実行可能。
"""
import pytest

from person_tracker_bridge_core import classify, STATUS_EXISTS_LEG, TRACKED_CONFIDENCE

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
