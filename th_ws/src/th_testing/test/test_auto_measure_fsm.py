"""test_auto_measure_fsm.py — 8-g P5a 自律測定（AUTO_MEASURE）の状態機械の純関数試験。

正本は docs/plan/spec/Spec-modes.md（-120・-130・-131・-135・§6）と
docs/plan/spec/Spec-measure.md、詳細は docs/plan/detailed/DetailedDesign-measure.md
§6.2（P5a は状態 WAIT・PAUSE だけ。NAV・ALIGN・BLOCKED は P5b、ADJUST は P3）。

議論（state_core の validate / step を直接叩く）:
- 入口（-120）: ui.goto kind:AUTO_MEASURE、link 拒否の理由キー
  measure_link_lost／measure_version_mismatch、effect start_measure_map
- /measure/command の遷移（-130・-131・-135・set_limits）、reentry 拒否（-142・-130）
- 回復フォルト・ジョグ（-03・-01）で PAUSE、ui.resume_yes では戻らず
  meas.start で戻る（attributes の resume 値）
"""
import pytest

from th_state.state_core import Context


def _mk_ctx(**overrides):
    base = dict(
        prev_mode="", prev_state="", prev_sub="",
        flags={}, zone="IN", candidate_count=0,
        target_selected=False, target_confident=False,
        fault_active=False, fault_severity="", fault_type="",
        hw_estop=False, ui_estop=False,
        route_ids=(), pin_kinds=(), leash_present=False, leash_taut=False,
        line_visible=False, camera_present=False,
        check_item="", check_result="", calib_item="", calib_preview_sane=False,
        map_update_available=False, now_ms=0, arg={},
    )
    base.update(overrides)
    return Context(**base)


@pytest.mark.rule("C-15-auto-measure")
def test_entry_accepted_when_link_ok(state_core_bundle):
    """-120: link OK で ui.goto kind:AUTO_MEASURE → AUTO_MEASURE/WAIT、
    effect start_measure_map。"""
    core, _, _, _ = state_core_bundle
    d = core.step("IDLE", "NONE", "ui.goto",
                  _mk_ctx(arg={"kind": "AUTO_MEASURE"}))
    assert d.accepted is True
    assert (d.to_mode, d.to_state) == ("AUTO_MEASURE", "WAIT")
    assert d.rule_id == "C-15-auto-measure"
    assert any(e.name == "start_measure_map" for e in d.effects)


@pytest.mark.rule("C-15-measure-link-lost")
def test_entry_rejected_link_lost(state_core_bundle):
    """-120: 生存無し → 拒否 measure_link_lost（版の食い違い以外）。"""
    core, _, _, _ = state_core_bundle
    d = core.step("IDLE", "NONE", "ui.goto",
                  _mk_ctx(arg={"kind": "AUTO_MEASURE"},
                          measure_link_ok=False, measure_link_reason="timeout"))
    assert d.accepted is False
    assert d.reject_reason_key == "measure_link_lost"
    assert d.rule_id == "C-15-measure-link-lost"
    assert (d.to_mode, d.to_state) == ("IDLE", "NONE")


@pytest.mark.rule("C-15-measure-link-version")
def test_entry_rejected_version_mismatch(state_core_bundle):
    """-120: 版の食い違い → 拒否 measure_version_mismatch。"""
    core, _, _, _ = state_core_bundle
    d = core.step("IDLE", "NONE", "ui.goto",
                  _mk_ctx(arg={"kind": "AUTO_MEASURE"},
                          measure_link_ok=False,
                          measure_link_reason="version_mismatch"))
    assert d.accepted is False
    assert d.reject_reason_key == "measure_version_mismatch"
    assert d.rule_id == "C-15-measure-link-version"


@pytest.mark.rule("C-15")
def test_other_gotos_unaffected_by_link_guards(state_core_bundle):
    """link 拒否行は kind:AUTO_MEASURE だけに当たる（PANEL／HOME は従来どおり）。"""
    core, _, _, _ = state_core_bundle
    d = core.step("IDLE", "NONE", "ui.goto",
                  _mk_ctx(arg={"kind": "PANEL", "pin_id": "P1"},
                          pin_kinds=("PANEL",),
                          measure_link_ok=False, measure_link_reason="timeout"))
    assert d.accepted is True
    assert (d.to_mode, d.to_state) == ("PANEL_NAV", "NAV")


@pytest.mark.rule("T-MEAS-01")
def test_meas_start_from_pause_to_wait(state_core_bundle):
    """-130: PAUSE → WAIT（止める前が WAIT・それ以外）。"""
    core, _, _, _ = state_core_bundle
    d = core.step("AUTO_MEASURE", "PAUSE", "meas.start", _mk_ctx())
    assert d.accepted is True
    assert (d.to_mode, d.to_state) == ("AUTO_MEASURE", "WAIT")
    assert d.rule_id == "T-MEAS-01"


@pytest.mark.rule("T-MEAS-01N")
def test_meas_start_returns_to_nav_after_nav_or_blocked(state_core_bundle):
    """-130: 止める前が NAV・BLOCKED なら meas.start は NAV へ。"""
    core, _, _, _ = state_core_bundle
    for prev in ("NAV", "BLOCKED"):
        d = core.step("AUTO_MEASURE", "PAUSE", "meas.start",
                      _mk_ctx(measure_prev_state=prev, measure_corner_clear=True))
        assert d.accepted is True, prev
        assert (d.to_mode, d.to_state) == ("AUTO_MEASURE", "NAV"), prev
        assert d.rule_id == "T-MEAS-01N", prev


@pytest.mark.rule("T-MEAS-01A")
def test_meas_start_returns_to_align_after_align(state_core_bundle):
    """-130: 止める前が ALIGN なら meas.start は ALIGN へ。"""
    core, _, _, _ = state_core_bundle
    d = core.step("AUTO_MEASURE", "PAUSE", "meas.start",
                  _mk_ctx(measure_prev_state="ALIGN", measure_corner_clear=True))
    assert d.accepted is True
    assert (d.to_mode, d.to_state) == ("AUTO_MEASURE", "ALIGN")
    assert d.rule_id == "T-MEAS-01A"


@pytest.mark.rule("T-MEAS-01C")
def test_meas_start_rejected_corner_obstacle_when_resuming_drive(state_core_bundle):
    """-130: NAV・ALIGN へ戻るとき角が確認できない／障害物ありは corner_obstacle
    （PAUSE のまま）。止める前が WAIT なら角を見ない。"""
    core, _, _, _ = state_core_bundle
    for prev in ("NAV", "BLOCKED", "ALIGN"):
        d = core.step("AUTO_MEASURE", "PAUSE", "meas.start",
                      _mk_ctx(measure_prev_state=prev, measure_corner_clear=False))
        assert d.accepted is False, prev
        assert d.reject_reason_key == "corner_obstacle", prev
        assert d.rule_id == "T-MEAS-01C", prev
        assert (d.to_mode, d.to_state) == ("AUTO_MEASURE", "PAUSE"), prev
    d = core.step("AUTO_MEASURE", "PAUSE", "meas.start",
                  _mk_ctx(measure_prev_state="WAIT", measure_corner_clear=False))
    assert d.accepted is True
    assert (d.to_mode, d.to_state) == ("AUTO_MEASURE", "WAIT")


@pytest.mark.rule("T-MEAS-01X")
def test_meas_start_rejected_reentry_required(state_core_bundle):
    """-130・-142: reentry_required 中は meas.start を localization_lost で拒否。"""
    core, _, _, _ = state_core_bundle
    d = core.step("AUTO_MEASURE", "PAUSE", "meas.start",
                  _mk_ctx(measure_reentry_required=True))
    assert d.accepted is False
    assert d.reject_reason_key == "localization_lost"
    assert d.rule_id == "T-MEAS-01X"
    assert (d.to_mode, d.to_state) == ("AUTO_MEASURE", "PAUSE")


@pytest.mark.rule("T-MEAS-02")
def test_meas_abort_from_pause_to_wait(state_core_bundle):
    """-131: PAUSE → WAIT。"""
    core, _, _, _ = state_core_bundle
    d = core.step("AUTO_MEASURE", "PAUSE", "meas.abort", _mk_ctx())
    assert d.accepted is True
    assert (d.to_mode, d.to_state) == ("AUTO_MEASURE", "WAIT")
    assert d.rule_id == "T-MEAS-02"


@pytest.mark.rule("T-MEAS-03")
def test_meas_finish_to_idle(state_core_bundle):
    """-135: 任意の状態 → IDLE。"""
    core, _, _, _ = state_core_bundle
    for state in ("WAIT", "PAUSE"):
        d = core.step("AUTO_MEASURE", state, "meas.finish", _mk_ctx())
        assert d.accepted is True, state
        assert (d.to_mode, d.to_state) == ("IDLE", "NONE")


def test_meas_set_limits_not_accepted(state_core_bundle):
    """meas.set_limits は効かせる仕組み（P5c）まで受けない（行が無いので
    state_not_accepting。step では not_allowed）。P5b の決まり。"""
    core, _, _, _ = state_core_bundle
    for state in ("WAIT", "PAUSE", "NAV", "ALIGN", "BLOCKED"):
        d = core.step("AUTO_MEASURE", state, "meas.set_limits", _mk_ctx())
        assert d.accepted is False, state
        assert d.reject_reason_key == "not_allowed", state
        assert (d.to_mode, d.to_state) == ("AUTO_MEASURE", state), state


def test_meas_commands_not_accepted_without_rows(state_core_bundle):
    """行が無い指示（turn・translate・done・ADJUST 以外の reroute/stop の
    受けない状態・WAIT/PAUSE の set_limits）は state_not_accepting
    （step では not_allowed）。"""
    core, _, _, _ = state_core_bundle
    for event in ("meas.turn", "meas.translate", "meas.done"):
        for state in ("WAIT", "PAUSE", "NAV", "ALIGN", "BLOCKED"):
            d = core.step("AUTO_MEASURE", state, event, _mk_ctx())
            assert d.accepted is False, (event, state)
            assert d.reject_reason_key == "not_allowed", (event, state)
            assert (d.to_mode, d.to_state) == ("AUTO_MEASURE", state)
    for event in ("meas.stop", "meas.reroute"):
        for state in ("WAIT", "PAUSE"):
            d = core.step("AUTO_MEASURE", state, event, _mk_ctx())
            assert d.accepted is False, (event, state)
            assert d.reject_reason_key == "not_allowed", (event, state)
            assert (d.to_mode, d.to_state) == ("AUTO_MEASURE", state)


def test_meas_commands_rejected_outside_auto_measure(state_core_bundle):
    """AUTO_MEASURE 以外のモードでは meas.* をすべて拒否する。
    （AT_PANEL の WORKING の meas 行は P2。）"""
    core, _, _, _ = state_core_bundle
    for mode, state in (("IDLE", "NONE"), ("AT_PANEL", "WORKING"),
                        ("MANUAL", "RUN"), ("PREP", "MAPPING")):
        for event in ("meas.start", "meas.abort", "meas.finish",
                      "meas.set_limits", "meas.goal"):
            d = core.step(mode, state, event, _mk_ctx())
            assert d.accepted is False, (mode, state, event)


@pytest.mark.rule("C-01")
def test_jog_from_wait_falls_to_pause(state_core_bundle):
    """-01: WAIT でのジョグ介入 → PAUSE（set_jog 付き）。"""
    core, _, _, _ = state_core_bundle
    d = core.step("AUTO_MEASURE", "WAIT", "ui.jog.hold", _mk_ctx())
    assert d.accepted is True
    assert (d.to_mode, d.to_state) == ("AUTO_MEASURE", "PAUSE")
    assert any(e.name == "set_jog" and e.args.get("on") is True for e in d.effects)


@pytest.mark.rule("C-03")
def test_recoverable_fault_from_wait_falls_to_pause(state_core_bundle):
    """-03: WAIT での回復フォルト → PAUSE（W-1 を開く）。"""
    core, _, _, _ = state_core_bundle
    d = core.step("AUTO_MEASURE", "WAIT", "fault.recoverable",
                  _mk_ctx(fault_active=True, fault_type="LIDAR_LOST"))
    assert d.accepted is True
    assert (d.to_mode, d.to_state) == ("AUTO_MEASURE", "PAUSE")
    assert any(e.name == "open_window" and e.args.get("id") == "W-1" for e in d.effects)


def test_resume_yes_does_not_return_but_ack_stays_in_pause(state_core_bundle):
    """§6: ui.resume_yes では戻らず、ui.resume_ack は PAUSE に留める
    （戻るのは meas.start だけ）。attributes の resume が ack_only であること。"""
    core, _, _, attrs = state_core_bundle
    assert attrs["AUTO_MEASURE"]["resume"] == "ack_only"
    assert attrs["AUTO_MEASURE"]["resume_state"] == "PAUSE"
    # P5b: 走行状態は NAV（run_state）。ui.resume_yes では走らないのは
    # RESUME_RUN_UNAVAILABLE_MODES のまま（C-04 はそのガードで弾く）。
    assert attrs["AUTO_MEASURE"]["run_state"] == "NAV"
    d = core.step("AUTO_MEASURE", "PAUSE", "ui.resume_yes", _mk_ctx())
    assert d.accepted is False
    assert (d.to_mode, d.to_state) == ("AUTO_MEASURE", "PAUSE")
    for event in ("ui.resume_no", "ui.resume_ack"):
        d = core.step("AUTO_MEASURE", "PAUSE", event, _mk_ctx())
        assert d.accepted is True, event
        assert (d.to_mode, d.to_state) == ("AUTO_MEASURE", "PAUSE")


def test_attributes_match_spec_modes_table(state_core_bundle):
    """§6 表: 速度 v_slow、ジョグ可（v_jog_panel）、自動ブレーキ ON 固定。"""
    core, _, _, attrs = state_core_bundle
    a = attrs["AUTO_MEASURE"]
    assert a["initial_state"] == "WAIT"
    assert a["speed_limit"] == "v_slow"
    assert a["jog_speed_limit"] == "v_jog_panel"
    assert a["jog"] == "allowed"
    assert a["auto_brake_default"] == "on_locked"
    assert attrs["AUTO_MEASURE"]["has_record"] is False


def test_auto_measure_mode_states_include_p5b(state_core_bundle):
    """P5b は WAIT・PAUSE に NAV・ALIGN・BLOCKED を足す（ADJUST は P3）。"""
    from th_state.state_core import MODE_STATES
    assert MODE_STATES["AUTO_MEASURE"] == {
        "WAIT", "PAUSE", "NAV", "ALIGN", "BLOCKED"}


@pytest.mark.rule("T-MEAS-08")
def test_meas_goal_accepted_to_nav_with_measure_goal_effect(state_core_bundle):
    """-121: WAIT の meas.goal → NAV、effect measure_goal（command_id・x・y・yaw）。"""
    core, _, _, _ = state_core_bundle
    arg = {"map_instance_id": "m1", "command_id": "g1", "x": 1.5, "y": -2.0,
           "yaw": 0.25}
    d = core.step("AUTO_MEASURE", "WAIT", "meas.goal",
                  _mk_ctx(arg=arg, measure_map_instance_id="m1",
                        measure_corner_clear=True))
    assert d.accepted is True
    assert (d.to_mode, d.to_state) == ("AUTO_MEASURE", "NAV")
    assert d.rule_id == "T-MEAS-08"
    eff = [e for e in d.effects if e.name == "measure_goal"]
    assert len(eff) == 1
    assert eff[0].args == {"command_id": "g1", "x": 1.5, "y": -2.0, "yaw": 0.25}


@pytest.mark.rule("T-MEAS-05")
def test_meas_goal_rejected_reentry_first(state_core_bundle):
    """-121 の拒否の順: reentry → localization_lost（地図・角より先）。"""
    core, _, _, _ = state_core_bundle
    d = core.step("AUTO_MEASURE", "WAIT", "meas.goal",
                  _mk_ctx(arg={"map_instance_id": "wrong"},
                        measure_reentry_required=True,
                        measure_map_instance_id="m1", measure_corner_clear=False))
    assert d.accepted is False
    assert d.reject_reason_key == "localization_lost"
    assert d.rule_id == "T-MEAS-05"


@pytest.mark.rule("T-MEAS-06")
def test_meas_goal_rejected_map_mismatch_before_corner(state_core_bundle):
    """-121 の拒否の順: map_mismatch は角の確認より先。"""
    core, _, _, _ = state_core_bundle
    d = core.step("AUTO_MEASURE", "WAIT", "meas.goal",
                  _mk_ctx(arg={"map_instance_id": "wrong"},
                        measure_map_instance_id="m1", measure_corner_clear=False))
    assert d.accepted is False
    assert d.reject_reason_key == "map_mismatch"
    assert d.rule_id == "T-MEAS-06"


@pytest.mark.rule("T-MEAS-07")
def test_meas_goal_rejected_corner_obstacle(state_core_bundle):
    """-121: 地図が合っていても角が確認できない／障害物ありは corner_obstacle。"""
    core, _, _, _ = state_core_bundle
    d = core.step("AUTO_MEASURE", "WAIT", "meas.goal",
                  _mk_ctx(arg={"map_instance_id": "m1"},
                        measure_map_instance_id="m1", measure_corner_clear=False))
    assert d.accepted is False
    assert d.reject_reason_key == "corner_obstacle"
    assert d.rule_id == "T-MEAS-07"
    assert (d.to_mode, d.to_state) == ("AUTO_MEASURE", "WAIT")


@pytest.mark.rule("T-MEAS-09")
def test_meas_reroute_from_nav_and_blocked_to_nav(state_core_bundle):
    """-122: NAV・BLOCKED の meas.reroute → NAV（replan 効果）。"""
    core, _, _, _ = state_core_bundle
    for state in ("NAV", "BLOCKED"):
        d = core.step("AUTO_MEASURE", state, "meas.reroute", _mk_ctx())
        assert d.accepted is True, state
        assert (d.to_mode, d.to_state) == ("AUTO_MEASURE", "NAV"), state
        assert d.rule_id == "T-MEAS-09", state
        assert any(e.name == "replan" for e in d.effects), state


@pytest.mark.rule("T-MEAS-10")
def test_evt_arrived_nav_to_align(state_core_bundle):
    """-123: NAV の evt.arrived → ALIGN。"""
    core, _, _, _ = state_core_bundle
    d = core.step("AUTO_MEASURE", "NAV", "evt.arrived", _mk_ctx())
    assert d.accepted is True
    assert (d.to_mode, d.to_state) == ("AUTO_MEASURE", "ALIGN")
    assert d.rule_id == "T-MEAS-10"


@pytest.mark.rule("T-MEAS-11")
def test_evt_align_done_to_wait(state_core_bundle):
    """-124: ALIGN の evt.align_done → WAIT（台帳の ARRIVED は state_manager の仕事）。"""
    core, _, _, _ = state_core_bundle
    d = core.step("AUTO_MEASURE", "ALIGN", "evt.align_done", _mk_ctx())
    assert d.accepted is True
    assert (d.to_mode, d.to_state) == ("AUTO_MEASURE", "WAIT")
    assert d.rule_id == "T-MEAS-11"


@pytest.mark.rule("T-MEAS-12")
def test_evt_blocked_nav_to_blocked(state_core_bundle):
    """-125: NAV の evt.blocked → BLOCKED。"""
    core, _, _, _ = state_core_bundle
    d = core.step("AUTO_MEASURE", "NAV", "evt.blocked", _mk_ctx())
    assert d.accepted is True
    assert (d.to_mode, d.to_state) == ("AUTO_MEASURE", "BLOCKED")
    assert d.rule_id == "T-MEAS-12"


@pytest.mark.rule("T-MEAS-13")
def test_evt_unblocked_blocked_to_nav(state_core_bundle):
    """-126: BLOCKED の evt.unblocked → NAV。"""
    core, _, _, _ = state_core_bundle
    d = core.step("AUTO_MEASURE", "BLOCKED", "evt.unblocked", _mk_ctx())
    assert d.accepted is True
    assert (d.to_mode, d.to_state) == ("AUTO_MEASURE", "NAV")
    assert d.rule_id == "T-MEAS-13"


@pytest.mark.rule("T-MEAS-14")
def test_meas_stop_driving_states_to_pause(state_core_bundle):
    """-129: NAV・ALIGN・BLOCKED の meas.stop → PAUSE（走行を止める）。"""
    core, _, _, _ = state_core_bundle
    for state in ("NAV", "ALIGN", "BLOCKED"):
        d = core.step("AUTO_MEASURE", state, "meas.stop", _mk_ctx())
        assert d.accepted is True, state
        assert (d.to_mode, d.to_state) == ("AUTO_MEASURE", "PAUSE"), state
        assert d.rule_id == "T-MEAS-14", state
        assert any(e.name == "cancel_follow_path" for e in d.effects), state


@pytest.mark.rule("T-MEAS-15")
def test_link_lost_driving_states_to_pause(state_core_bundle):
    """-133: NAV・ALIGN・BLOCKED の evt.measure_link_lost → PAUSE。"""
    core, _, _, _ = state_core_bundle
    for state in ("NAV", "ALIGN", "BLOCKED"):
        d = core.step("AUTO_MEASURE", state, "evt.measure_link_lost", _mk_ctx())
        assert d.accepted is True, state
        assert (d.to_mode, d.to_state) == ("AUTO_MEASURE", "PAUSE"), state
        assert d.rule_id == "T-MEAS-15", state
    for state in ("WAIT", "PAUSE"):
        d = core.step("AUTO_MEASURE", state, "evt.measure_link_lost", _mk_ctx())
        assert d.accepted is False, state
        assert (d.to_mode, d.to_state) == ("AUTO_MEASURE", state), state


@pytest.mark.rule("T-MEAS-02B")
def test_meas_abort_driving_states_to_wait(state_core_bundle):
    """-131: NAV・ALIGN・BLOCKED の meas.abort → WAIT（走行を止める）。"""
    core, _, _, _ = state_core_bundle
    for state in ("NAV", "ALIGN", "BLOCKED"):
        d = core.step("AUTO_MEASURE", state, "meas.abort", _mk_ctx())
        assert d.accepted is True, state
        assert (d.to_mode, d.to_state) == ("AUTO_MEASURE", "WAIT"), state
        assert d.rule_id == "T-MEAS-02B", state
        assert any(e.name == "cancel_follow_path" for e in d.effects), state


@pytest.mark.parametrize("estop_mode,estop_state,event", [
    ("ESTOP", "NONE", "ui.resume_yes"),
    ("CARRY", "NONE", "ui.carry_resume"),
])
def test_estop_carry_never_resume_directly_to_driving_states(
        state_core_bundle, estop_mode, estop_state, event):
    """SM-3.1.1-11・-14: 非常停止・手押しから NAV・ALIGN・BLOCKED へ直接戻る道が無い。
    押下前がそれらでも、戻り先は同モードの PAUSE（C-09c-running・C-11-running が
    C-09c-generic・C-11-generic より前に拾う）。"""
    core, _, _, _ = state_core_bundle
    for prev_state in ("NAV", "ALIGN", "BLOCKED"):
        d = core.step(estop_mode, estop_state, event,
                      _mk_ctx(prev_mode="AUTO_MEASURE", prev_state=prev_state))
        assert d.accepted is True, (event, prev_state)
        assert (d.to_mode, d.to_state) == ("AUTO_MEASURE", "PAUSE"), (event, prev_state)
        assert (d.to_mode, d.to_state) != ("AUTO_MEASURE", prev_state)


def test_meas_goal_only_accepted_in_wait(state_core_bundle):
    """-121・-132: NAV・ALIGN・BLOCKED・PAUSE の meas.goal は行が無い
    （state_not_accepting。step では not_allowed）。"""
    core, _, _, _ = state_core_bundle
    arg = {"map_instance_id": "m1", "command_id": "g1"}
    for state in ("NAV", "ALIGN", "BLOCKED", "PAUSE"):
        d = core.step("AUTO_MEASURE", state, "meas.goal",
                      _mk_ctx(arg=arg, measure_map_instance_id="m1",
                            measure_corner_clear=True))
        assert d.accepted is False, state
        assert d.reject_reason_key == "not_allowed", state
        assert (d.to_mode, d.to_state) == ("AUTO_MEASURE", state), state


def test_zone_in_for_s60():
    """画面 S-60 はゾーン IN（Spec-modes.md §6）。"""
    from th_state.zones import SCREEN_ZONE, SCREEN_SPEED_LIMIT
    assert SCREEN_ZONE["S-60"] == "IN"
    assert SCREEN_SPEED_LIMIT["S-60"] == "v_slow"
