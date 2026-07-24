"""
test_summon_logic.py
======================
summon_navigator のコアロジック単体テスト

ROS2 なし・純粋 Python で実行可能。
pytest で実行: pytest test/test_summon_logic.py -v
"""

import math
import sys
import os
import pytest

sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), '..', '..', 'th_planning'))

from th_planning.summon_navigator_core import (
    SummonParams, is_target_valid, compute_stop_goal,
)


DEFAULT_PARAMS = SummonParams(stop_distance_m=1.0, min_confidence=0.5)


class TestIsTargetValid:
    def test_rejects_lost_target(self):
        ok, reason = is_target_valid(True, 0.9, DEFAULT_PARAMS)
        assert not ok
        assert '見失' in reason

    def test_rejects_low_confidence(self):
        ok, reason = is_target_valid(False, 0.1, DEFAULT_PARAMS)
        assert not ok
        assert '信頼度' in reason

    def test_accepts_confident_tracked_target(self):
        ok, reason = is_target_valid(False, 0.9, DEFAULT_PARAMS)
        assert ok
        assert reason == 'OK'

    def test_boundary_confidence_is_accepted(self):
        ok, _ = is_target_valid(False, DEFAULT_PARAMS.min_confidence, DEFAULT_PARAMS)
        assert ok


class TestComputeStopGoal:
    def test_already_within_stop_distance_returns_none(self):
        assert compute_stop_goal(0.5, 0.0, DEFAULT_PARAMS) is None

    def test_exactly_at_stop_distance_returns_none(self):
        assert compute_stop_goal(DEFAULT_PARAMS.stop_distance_m, 0.0, DEFAULT_PARAMS) is None

    def test_goal_stops_short_along_straight_ahead(self):
        goal = compute_stop_goal(3.0, 0.0, DEFAULT_PARAMS)
        assert goal is not None
        x, y, yaw = goal
        assert x == pytest.approx(2.0)
        assert y == pytest.approx(0.0)
        assert yaw == pytest.approx(0.0)

    def test_goal_faces_the_person(self):
        goal = compute_stop_goal(0.0, 3.0, DEFAULT_PARAMS)
        assert goal is not None
        x, y, yaw = goal
        assert x == pytest.approx(0.0)
        assert y == pytest.approx(2.0)
        assert yaw == pytest.approx(math.pi / 2)

    def test_goal_distance_from_origin_matches_stop_distance(self):
        goal = compute_stop_goal(4.0, 3.0, DEFAULT_PARAMS)
        assert goal is not None
        x, y, _ = goal
        person_dist = math.hypot(4.0, 3.0)
        goal_dist = math.hypot(x, y)
        assert goal_dist == pytest.approx(person_dist - DEFAULT_PARAMS.stop_distance_m)

    def test_goal_direction_matches_person_direction(self):
        goal = compute_stop_goal(4.0, 3.0, DEFAULT_PARAMS)
        assert goal is not None
        x, y, yaw = goal
        assert yaw == pytest.approx(math.atan2(3.0, 4.0))
        assert yaw == pytest.approx(math.atan2(y, x))
