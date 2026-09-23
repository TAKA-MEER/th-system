"""test_dr_spaam_lifecycle_controller_core.py — DR-SPAAM 起動/停止判断の純ロジック

brief-tracker-default-off §3.2: tracker_enabled と lifecycle 状態から
ChangeState に渡す transition_id を決める核心を、ROS2 なしで縛る。
"""
from dr_spaam_lifecycle_controller_core import (
    STATE_ACTIVE,
    STATE_INACTIVE,
    STATE_UNCONFIGURED,
    TRANSITION_ACTIVATE,
    TRANSITION_DEACTIVATE,
    TRANSITION_NONE,
    desired_lifecycle_transition,
)


class TestDesiredLifecycleTransition:
    def test_enabled_and_inactive_activates(self):
        """ON かつ INACTIVE（auto_configure 済み・待機中）→ activate。"""
        assert desired_lifecycle_transition(True, STATE_INACTIVE) == TRANSITION_ACTIVATE

    def test_enabled_and_active_does_nothing(self):
        """ON かつ ACTIVE（起動済み）→ 何もしない。"""
        assert desired_lifecycle_transition(True, STATE_ACTIVE) == TRANSITION_NONE

    def test_disabled_and_active_deactivates(self):
        """OFF かつ ACTIVE → deactivate（推論の停止）。"""
        assert desired_lifecycle_transition(False, STATE_ACTIVE) == TRANSITION_DEACTIVATE

    def test_disabled_and_inactive_does_nothing(self):
        """OFF かつ INACTIVE（既定状態）→ 何もしない。"""
        assert desired_lifecycle_transition(False, STATE_INACTIVE) == TRANSITION_NONE

    def test_transitioning_and_unconfigured_do_nothing(self):
        """遷移中（id が 4〜15 のどこか）や UNCONFIGURED では触らない。"""
        for state_id in (STATE_UNCONFIGURED, 6, 10, 13, 14, 15):
            assert desired_lifecycle_transition(True, state_id) == TRANSITION_NONE, state_id
            assert desired_lifecycle_transition(False, state_id) == TRANSITION_NONE, state_id