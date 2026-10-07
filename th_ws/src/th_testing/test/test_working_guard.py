"""1b-9 SG-A9 — 「作業中」の判定（純関数）と遷移表の拒否行。ROS 不要。"""
import pytest

from th_state import guards, state_core


def _ctx(kind, flags=None):
    return state_core.Context(
        prev_mode="FOLLOW", prev_state="RUN", prev_sub="MAPPING",
        flags=dict(flags or {}), zone="OUT", candidate_count=0,
        target_selected=False, target_confident=False,
        fault_active=False, fault_severity="", fault_type="",
        hw_estop=False, ui_estop=False,
        route_ids=(), pin_kinds=("PANEL", "HOME"), leash_present=False, leash_taut=False,
        line_visible=False, camera_present=False,
        check_item="", check_result="", calib_item="", calib_preview_sane=False,
        map_update_available=False, now_ms=0, arg={"kind": kind},
    )


def test_is_working_only_at_panel_working():
    assert guards.is_working("AT_PANEL", "WORKING") is True
    for mode, state in [("AT_PANEL", "IDLE_P"), ("AT_PANEL", "PAUSE"), ("AT_HOME", "IDLE_H"),
                        ("IDLE", "NONE"), ("PANEL_NAV", "NAV"), ("SUMMON", "WORKING")]:
        assert guards.is_working(mode, state) is False, (mode, state)


@pytest.mark.parametrize("kind", ["PANEL", "HOME", "SUMMON"])
def test_goto_in_working_is_rejected_with_reason(kind, state_core_bundle):
    core = state_core_bundle[0]
    d = core.step("AT_PANEL", "WORKING", "ui.goto", _ctx(kind))
    assert d.accepted is False
    assert d.reject_reason_key == "working_in_progress"


@pytest.mark.parametrize("kind,mode", [("PANEL", "PANEL_NAV"), ("HOME", "HOME_NAV"),
                                       ("SUMMON", "SUMMON")])
def test_goto_in_idle_p_is_accepted(kind, mode, state_core_bundle):
    core = state_core_bundle[0]
    d = core.step("AT_PANEL", "IDLE_P", "ui.goto", _ctx(kind))
    assert d.accepted is True
    assert d.to_mode == mode


def test_working_flag_is_not_the_source(state_core_bundle):
    """flags["working"] を立てても、状態が WORKING でなければ拒否しない（正本は状態）。"""
    core = state_core_bundle[0]
    d = core.step("AT_PANEL", "IDLE_P", "ui.goto", _ctx("HOME", {"working": True}))
    assert d.accepted is True


def test_toggle_working_roundtrip(state_core_bundle):
    core = state_core_bundle[0]
    d = core.step("AT_PANEL", "IDLE_P", "ui.working", _ctx("HOME"))
    assert (d.to_mode, d.to_state) == ("AT_PANEL", "WORKING")
    d = core.step("AT_PANEL", "WORKING", "ui.working", _ctx("HOME"))
    assert (d.to_mode, d.to_state) == ("AT_PANEL", "IDLE_P")
