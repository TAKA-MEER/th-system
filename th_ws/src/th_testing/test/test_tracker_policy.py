"""test_tracker_policy.py — tracker_enabled の純ロジック（th_state.tracker_policy）

brief-tracker-default-off §3.1 のポリシー（Spec-modes.md §9「動かさない」自動停止・
§5.1 要 person での OFF 拒否）を、モード名の集合と判定関数で縛る。
ROS2 なし・純粋 Python で実行可能。
"""
import pytest

from th_state.tracker_policy import (
    TRACKER_OFF_DENIED_REASON,
    TRACKER_AUTOSTOP_MODES,
    tracker_autostop,
    tracker_off_denied,
)


class TestTrackerAutostop:
    def test_autostop_modes_matches_spec(self):
        """Spec-modes.md §9（E-9）「動かさない」の 9 モードと一致する。"""
        assert TRACKER_AUTOSTOP_MODES == {
            "INIT", "IDLE", "MANUAL", "TEACH_MANUAL", "REPLAY", "LINE", "LEASH",
            "OPCHECK", "CALIB",
        }

    def test_estop_and_carry_are_never_autostopped(self):
        """ESTOP / CARRY は対象外（継続。brief-tracker-default-off §3.1）。"""
        assert not tracker_autostop("ESTOP")
        assert not tracker_autostop("CARRY")

    def test_person_operated_modes_are_not_autostopped(self):
        """「人の操作で起動」のモード（PREP / SUMMON / AT_HOME 等）は対象外。"""
        for mode in ("PREP", "SUMMON", "AT_HOME", "PANEL_NAV", "HOME_NAV", "AT_PANEL"):
            assert not tracker_autostop(mode), mode

    def test_follow_modes_are_not_autostopped(self):
        """FOLLOW / TEACH_FOLLOW（要 person・自動起動。未実装）も対象外。"""
        assert not tracker_autostop("FOLLOW")
        assert not tracker_autostop("TEACH_FOLLOW")


class TestTrackerOffDenied:
    def test_summon_all_states_denied(self):
        """SUMMON はどの状態でも OFF 拒否（Spec-modes.md §5.1-2）。"""
        for state in ("POINT", "WAIT_CLEAR", "NAV", "BLOCKED", "PAUSE", "ALIGN"):
            assert tracker_off_denied("SUMMON", state), state

    def test_prep_register_denied_only(self):
        """PREP は REGISTER（ピン登録中）だけが対象。"""
        assert tracker_off_denied("PREP", "REGISTER")
        for state in ("MAPPING", "RETURN", "EDIT", "SAVED"):
            assert not tracker_off_denied("PREP", state), state

    def test_other_modes_allow_off(self):
        """それ以外のモード・状態では OFF できる。"""
        assert not tracker_off_denied("IDLE", "NONE")
        assert not tracker_off_denied("MANUAL", "PAUSE")
        assert not tracker_off_denied("FOLLOW", "SELECT")

    def test_reason_key_is_tracker_required(self):
        """拒否理由キーは 'tracker_required'（WebUI の reasons.js と同値）。"""
        assert TRACKER_OFF_DENIED_REASON == "tracker_required"