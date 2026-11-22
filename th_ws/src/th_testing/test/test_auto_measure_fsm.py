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
    """-130: PAUSE → WAIT（P5a は止める前が WAIT だけ）。"""
    core, _, _, _ = state_core_bundle
    d = core.step("AUTO_MEASURE", "PAUSE", "meas.start", _mk_ctx())
    assert d.accepted is True
    assert (d.to_mode, d.to_state) == ("AUTO_MEASURE", "WAIT")
    assert d.rule_id == "T-MEAS-01"


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


@pytest.mark.rule("T-MEAS-04")
def test_meas_set_limits_keeps_state(state_core_bundle):
    """meas.set_limits: 状態を変えない（WAIT・PAUSE ともに受理）。"""
    core, _, _, _ = state_core_bundle
    for state in ("WAIT", "PAUSE"):
        d = core.step("AUTO_MEASURE", state, "meas.set_limits", _mk_ctx())
        assert d.accepted is True, state
        assert (d.to_mode, d.to_state) == ("AUTO_MEASURE", state)
        assert d.rule_id == "T-MEAS-04"


def test_meas_commands_not_accepted_in_p5a(state_core_bundle):
    """P5a では meas.stop・meas.goal・turn・translate に行が無い
    （state_not_accepting になる。片方は _on_measure_command の写像で扱う）
    → step は accepted=False、reject_reason_key=not_allowed。"""
    core, _, _, _ = state_core_bundle
    for event in ("meas.stop", "meas.goal", "meas.turn", "meas.translate",
                  "meas.done", "meas.reroute"):
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
    assert attrs["AUTO_MEASURE"]["run_state"] is None
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


def test_auto_measure_mode_states_only_wait_pause(state_core_bundle):
    """P5a は WAIT・PAUSE だけ（NAV・ALIGN・BLOCKED は P5b、ADJUST は P3）。"""
    from th_state.state_core import MODE_STATES
    assert MODE_STATES["AUTO_MEASURE"] == {"WAIT", "PAUSE"}


def test_zone_in_for_s60():
    """画面 S-60 はゾーン IN（Spec-modes.md §6）。"""
    from th_state.zones import SCREEN_ZONE, SCREEN_SPEED_LIMIT
    assert SCREEN_ZONE["S-60"] == "IN"
    assert SCREEN_SPEED_LIMIT["S-60"] == "v_slow"
