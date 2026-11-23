"""test_adjust_fsm.py — 8-g P3 ADJUST の状態機械の純関数試験。ROS 不要。

正本は docs/plan/spec/Spec-modes.md SM-3.1.2-127・-128・-132・-136〜-140 と
docs/plan/detailed/DetailedDesign-measure.md §6.4。
"""
import pytest

from th_state import guards
from th_state.state_core import Context, MODE_STATES
from th_state.zones import mode_speed_limit


def _mk_ctx(**overrides):
    base = dict(
        prev_mode="", prev_state="", prev_sub="",
        flags={}, zone="IN", candidate_count=0,
        target_selected=False, target_confident=False,
        fault_active=False, fault_severity="", fault_type="",
        hw_estop=False, ui_estop=False,
        route_ids=(), pin_kinds=("PANEL",), leash_present=False, leash_taut=False,
        line_visible=False, camera_present=False,
        check_item="", check_result="", calib_item="", calib_preview_sane=False,
        map_update_available=False, now_ms=0, arg={},
    )
    base.update(overrides)
    return Context(**base)


def test_mode_states_include_adjust():
    assert MODE_STATES["AT_PANEL"] == {"IDLE_P", "WORKING", "ADJUST", "PAUSE"}
    assert MODE_STATES["AUTO_MEASURE"] == {
        "WAIT", "PAUSE", "NAV", "ALIGN", "BLOCKED", "ADJUST"}


@pytest.mark.rule("T-MEAS-20")
def test_auto_turn_accepted_to_adjust(state_core_bundle):
    core, _, _, _ = state_core_bundle
    d = core.step("AUTO_MEASURE", "WAIT", "meas.turn",
                  _mk_ctx(arg={"command_id": "t1", "angle_rad": 1.5},
                          measure_corner_clear=True))
    assert d.accepted is True
    assert (d.to_mode, d.to_state) == ("AUTO_MEASURE", "ADJUST")
    eff = [e for e in d.effects if e.name == "measure_adjust"]
    assert len(eff) == 1
    assert eff[0].args == {"command_id": "t1", "kind": "turn",
                           "angle_rad": 1.5, "side": "", "distance_m": 0}


@pytest.mark.rule("T-MEAS-21")
def test_auto_translate_path_obstacle_rejected(state_core_bundle):
    core, _, _, _ = state_core_bundle
    d = core.step("AUTO_MEASURE", "WAIT", "meas.translate",
                  _mk_ctx(arg={"command_id": "t1", "side": "FORWARD",
                               "distance_m": 0.5},
                          measure_corner_clear=True,
                          measure_free_forward_m=0.3))
    assert d.accepted is False
    assert d.reject_reason_key == "path_obstacle"
    assert (d.to_mode, d.to_state) == ("AUTO_MEASURE", "WAIT")


@pytest.mark.rule("T-MEAS-20X")
def test_auto_turn_reentry_first(state_core_bundle):
    """拒否の順: reentry → corner → path。"""
    core, _, _, _ = state_core_bundle
    d = core.step("AUTO_MEASURE", "WAIT", "meas.turn",
                  _mk_ctx(arg={"command_id": "t1", "angle_rad": 1.0},
                          measure_reentry_required=True,
                          measure_corner_clear=False))
    assert d.reject_reason_key == "localization_lost"


@pytest.mark.rule("T-ATP-00")
def test_panel_turn_accepted_and_done(state_core_bundle):
    core, _, _, _ = state_core_bundle
    d = core.step("AT_PANEL", "WORKING", "meas.turn",
                  _mk_ctx(arg={"command_id": "t1", "angle_rad": 1.5},
                          measure_corner_clear=True))
    assert d.accepted is True
    assert (d.to_mode, d.to_state) == ("AT_PANEL", "ADJUST")
    d = core.step("AT_PANEL", "ADJUST", "evt.adjust_done",
                  _mk_ctx(arg={"command_id": "t1"}))
    assert d.accepted is True
    assert (d.to_mode, d.to_state) == ("AT_PANEL", "WORKING")


@pytest.mark.rule("T-ATP-00-adjust-reject")
def test_adjust_rejects_done_and_working_off_and_goto(state_core_bundle):
    core, _, _, _ = state_core_bundle
    for event, arg in (("meas.done", {}),
                       ("ui.working", {"on": False}),
                       ("ui.goto", {"kind": "PANEL"})):
        d = core.step("AT_PANEL", "ADJUST", event, _mk_ctx(arg=arg))
        assert d.accepted is False, event
        assert d.reject_reason_key == "adjust_in_progress", event
        assert (d.to_mode, d.to_state) == ("AT_PANEL", "ADJUST"), event


@pytest.mark.rule("T-MEAS-14")
def test_auto_stop_from_adjust_to_pause(state_core_bundle):
    core, _, _, _ = state_core_bundle
    d = core.step("AUTO_MEASURE", "ADJUST", "meas.stop", _mk_ctx())
    assert d.accepted is True
    assert (d.to_mode, d.to_state) == ("AUTO_MEASURE", "PAUSE")


@pytest.mark.rule("T-MEAS-02B")
def test_auto_abort_from_adjust_to_wait(state_core_bundle):
    core, _, _, _ = state_core_bundle
    d = core.step("AUTO_MEASURE", "ADJUST", "meas.abort", _mk_ctx())
    assert d.accepted is True
    assert (d.to_mode, d.to_state) == ("AUTO_MEASURE", "WAIT")


@pytest.mark.rule("T-ATP-10L")
def test_panel_link_lost_from_adjust_to_working(state_core_bundle):
    """§6.4 停止命令の ADJUST の「回復フォルト・途絶」→ WORKING（-139）。
    測定機能との途絶は WORKING では遷移しないが ADJUST では戻す
    （動きをやめて捨てる。台帳は state_manager が CANCELLED（link_lost））。"""
    core, _, _, _ = state_core_bundle
    d = core.step("AT_PANEL", "ADJUST", "evt.measure_link_lost", _mk_ctx())
    assert d.accepted is True
    assert (d.to_mode, d.to_state) == ("AT_PANEL", "WORKING")


@pytest.mark.rule("T-MEAS-15")
def test_auto_link_lost_from_adjust_to_pause(state_core_bundle):
    core, _, _, _ = state_core_bundle
    d = core.step("AUTO_MEASURE", "ADJUST", "evt.measure_link_lost", _mk_ctx())
    assert d.accepted is True
    assert (d.to_mode, d.to_state) == ("AUTO_MEASURE", "PAUSE")


def test_jog_rejected_in_adjust_and_fault_stays():
    # C-01-working は ADJUST にも効く（腕が起きたままジョグで動かさない）。
    assert guards._working("AT_PANEL", "ADJUST", _mk_ctx()) is True
    assert guards.is_working("AT_PANEL", "ADJUST") is True
    assert guards.published_working("AT_PANEL", "ADJUST", "", "") is True
    # ESTOP/CARRY から戻るときも ADJUST は WORKING へ（-140）。
    assert guards.published_working("ESTOP", "NONE", "AT_PANEL", "ADJUST") is True


def test_adjust_speed_limit_is_jog_panel_in_both_modes():
    # ADJUST は両モードとも v_jog_panel（ジョグ分岐を流用しない）。
    assert mode_speed_limit({"speed_limit": "v_slow", "jog": "allowed",
                             "jog_speed_limit": "v_jog_panel"}, False, "ADJUST") == "v_jog_panel"
    assert mode_speed_limit({"speed_limit": "stop", "jog": "allowed",
                             "jog_speed_limit": "v_jog_panel"}, False, "ADJUST") == "v_jog_panel"
    # ADJUST 以外は従来どおり。
    assert mode_speed_limit({"speed_limit": "v_slow", "jog": "allowed",
                             "jog_speed_limit": "v_jog_panel"}, False, "WAIT") == "v_slow"
