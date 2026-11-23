"""test_working_stop_fsm.py — 8-g P2 お立会試験の停止命令（AT_PANEL/WORKING）の
状態機械の純関数試験。

正本は docs/plan/spec/Spec-modes.md（SM-3.1.2-062・-063・-134・-139・-140）と
docs/plan/detailed/DetailedDesign-measure.md §6.3（P2 の表）。

議論（state_core の validate / step を直接叩く）:
- meas.done（-063）: WORKING → IDLE_P。WORKING のほかの meas.* は行が無いので
  step では not_allowed（ノード側で state_not_accepting になる）。
- ジョグの拒否（-062）: WORKING の ui.jog.hold は override_common の拒否行で
  C-01 より先に working_in_progress で落とす。
- 回復フォルト（-139）: WORKING の fault.recoverable は WORKING のまま
  （open_window。C-03 の PAUSE には落とさない）。ui.resume_ack は
  fault_cleared のときだけ状態を変えずに受ける。evt.measure_link_lost は
  WORKING では遷移しない。
- 非常停止・手押し（-140）: 押下前が WORKING なら WORKING へ戻す。
  ふつうの AT_PANEL（IDLE_P）は従来どおり IDLE_P。
- /system/state.working の出す値（published_working）: ESTOP・CARRY の間も
  押下前が AT_PANEL/WORKING なら true。行き先拒否のガード is_working は変えない。
"""
import pytest

from th_state import guards
from th_state.state_core import Context


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


@pytest.mark.rule("T-ATP-02-done")
def test_meas_done_from_working_to_idle_p(state_core_bundle):
    """-063: 測定機能の「測定を終えた」（meas.done）で WORKING → IDLE_P。"""
    core, _, _, _ = state_core_bundle
    d = core.step("AT_PANEL", "WORKING", "meas.done", _mk_ctx())
    assert d.accepted is True
    assert (d.to_mode, d.to_state) == ("AT_PANEL", "IDLE_P")
    assert d.rule_id == "T-ATP-02-done"


def test_other_meas_rejected_in_working(state_core_bundle):
    """WORKING で受ける meas.* は meas.done（と P5c の set_limits）だけ。ほかは行が無いので
    step では not_allowed（ノード側で state_not_accepting になる）。"""
    core, _, _, _ = state_core_bundle
    # 8-g P3: WORKING では turn・translate を受けて ADJUST へ（-136）。ここではそれ以外を見る。
    for event in ("meas.goal", "meas.reroute",
                  "meas.start", "meas.stop", "meas.abort", "meas.finish"):
        d = core.step("AT_PANEL", "WORKING", event, _mk_ctx())
        assert d.accepted is False, event
        assert d.reject_reason_key == "not_allowed", event
        assert (d.to_mode, d.to_state) == ("AT_PANEL", "WORKING"), event


def test_meas_done_rejected_in_idle_p_and_pause(state_core_bundle):
    """AT_PANEL の IDLE_P・PAUSE では meas.done も含めすべて拒否。"""
    core, _, _, _ = state_core_bundle
    for state in ("IDLE_P", "PAUSE"):
        d = core.step("AT_PANEL", state, "meas.done", _mk_ctx())
        assert d.accepted is False, state
        assert (d.to_mode, d.to_state) == ("AT_PANEL", state), state


@pytest.mark.rule("C-01-working")
def test_jog_rejected_in_working_before_c01(state_core_bundle):
    """-062: WORKING の ui.jog.hold は override_common の拒否行で
    working_in_progress で落とす（C-01 の受理より先）。"""
    core, _, _, _ = state_core_bundle
    d = core.step("AT_PANEL", "WORKING", "ui.jog.hold", _mk_ctx())
    assert d.accepted is False
    assert d.reject_reason_key == "working_in_progress"
    assert d.rule_id == "C-01-working"
    assert (d.to_mode, d.to_state) == ("AT_PANEL", "WORKING")
    assert not d.effects, "拒否行で set_jog を出さない（jog_active が立たない）"


def test_jog_still_accepted_in_idle_p(state_core_bundle):
    """対照: IDLE_P のジョグは従来どおり受理（PAUSE・set_jog）。"""
    core, _, _, _ = state_core_bundle
    d = core.step("AT_PANEL", "IDLE_P", "ui.jog.hold", _mk_ctx())
    assert d.accepted is True
    assert (d.to_mode, d.to_state) == ("AT_PANEL", "PAUSE")
    assert any(e.name == "set_jog" and e.args.get("on") is True for e in d.effects)


@pytest.mark.rule("T-ATP-08-fault")
def test_recoverable_fault_stays_in_working(state_core_bundle):
    """-139: WORKING の回復フォルトは WORKING のまま（W-1 を開く）。
    C-03 の PAUSE には落とさない（override_common が先に拾う）。"""
    core, _, _, _ = state_core_bundle
    d = core.step("AT_PANEL", "WORKING", "fault.recoverable",
                  _mk_ctx(fault_active=True, fault_type="LIDAR_LOST",
                          fault_severity="RECOVERABLE"))
    assert d.accepted is True
    assert (d.to_mode, d.to_state) == ("AT_PANEL", "WORKING")
    assert d.rule_id == "T-ATP-08-fault"
    assert any(e.name == "open_window" and e.args.get("id") == "W-1"
               for e in d.effects)


@pytest.mark.rule("T-ATP-09-ack")
def test_resume_ack_in_working_stays_when_cleared(state_core_bundle):
    """-139: WORKING の ui.resume_ack（fault_cleared）は状態を変えずに受ける。"""
    core, _, _, _ = state_core_bundle
    d = core.step("AT_PANEL", "WORKING", "ui.resume_ack",
                  _mk_ctx(fault_active=False))
    assert d.accepted is True
    assert (d.to_mode, d.to_state) == ("AT_PANEL", "WORKING")
    assert d.rule_id == "T-ATP-09-ack"
    assert any(e.name == "close_window" and e.args.get("id") == "W-1"
               for e in d.effects)


def test_resume_ack_in_working_rejected_while_fault_active(state_core_bundle):
    """フォルトが残っている間の ui.resume_ack は受けない（ガード fault_cleared）。"""
    core, _, _, _ = state_core_bundle
    d = core.step("AT_PANEL", "WORKING", "ui.resume_ack",
                  _mk_ctx(fault_active=True, fault_type="LIDAR_LOST"))
    assert d.accepted is False
    assert (d.to_mode, d.to_state) == ("AT_PANEL", "WORKING")


def test_measure_link_lost_does_not_transition_in_working(state_core_bundle):
    """-139: 測定機能との途絶（evt.measure_link_lost）は WORKING では遷移しない。"""
    core, _, _, _ = state_core_bundle
    d = core.step("AT_PANEL", "WORKING", "evt.measure_link_lost", _mk_ctx())
    assert d.accepted is False
    assert (d.to_mode, d.to_state) == ("AT_PANEL", "WORKING")


@pytest.mark.rule("C-09c-at-panel-working")
def test_estop_resume_returns_to_working_when_prev_working(state_core_bundle):
    """-140: 押下前が AT_PANEL/WORKING なら ESTOP 解除（W-1 の ui.resume_yes）で
    AT_PANEL/WORKING へ戻る（停止命令のまま）。"""
    core, _, _, _ = state_core_bundle
    d = core.step("ESTOP", "NONE", "ui.resume_yes",
                  _mk_ctx(prev_mode="AT_PANEL", prev_state="WORKING"))
    assert d.accepted is True
    assert (d.to_mode, d.to_state) == ("AT_PANEL", "WORKING")
    assert d.rule_id == "C-09c-at-panel-working"


def test_estop_resume_to_idle_p_when_prev_not_working(state_core_bundle):
    """対照: ふつうの AT_PANEL（IDLE_P）は従来どおり IDLE_P へ戻る。"""
    core, _, _, _ = state_core_bundle
    d = core.step("ESTOP", "NONE", "ui.resume_yes",
                  _mk_ctx(prev_mode="AT_PANEL", prev_state="IDLE_P"))
    assert d.accepted is True
    assert (d.to_mode, d.to_state) == ("AT_PANEL", "IDLE_P")
    assert d.rule_id == "C-09c-at-panel"


@pytest.mark.rule("C-11-at-panel-working")
def test_carry_resume_returns_to_working_when_prev_working(state_core_bundle):
    """-140: 押下前が AT_PANEL/WORKING なら手押し解除（ui.carry_resume）で
    AT_PANEL/WORKING へ戻る。"""
    core, _, _, _ = state_core_bundle
    d = core.step("CARRY", "NONE", "ui.carry_resume",
                  _mk_ctx(prev_mode="AT_PANEL", prev_state="WORKING"))
    assert d.accepted is True
    assert (d.to_mode, d.to_state) == ("AT_PANEL", "WORKING")
    assert d.rule_id == "C-11-at-panel-working"


def test_carry_resume_to_idle_p_when_prev_not_working(state_core_bundle):
    """対照: ふつうの AT_PANEL（IDLE_P）は従来どおり IDLE_P へ戻る。"""
    core, _, _, _ = state_core_bundle
    d = core.step("CARRY", "NONE", "ui.carry_resume",
                  _mk_ctx(prev_mode="AT_PANEL", prev_state="IDLE_P"))
    assert d.accepted is True
    assert (d.to_mode, d.to_state) == ("AT_PANEL", "IDLE_P")
    assert d.rule_id == "C-11-at-panel"


def test_published_working_kept_in_estop_and_carry():
    """出す値（/system/state.working）: ESTOP・CARRY の間も押下前が
    AT_PANEL/WORKING なら true。"""
    assert guards.published_working(
        "ESTOP", "NONE", "AT_PANEL", "WORKING") is True
    assert guards.published_working(
        "CARRY", "NONE", "AT_PANEL", "WORKING") is True
    assert guards.published_working(
        "AT_PANEL", "WORKING", "", "") is True


def test_published_working_false_otherwise():
    assert guards.published_working(
        "ESTOP", "NONE", "AT_PANEL", "IDLE_P") is False
    assert guards.published_working(
        "CARRY", "NONE", "AT_PANEL", "PAUSE") is False
    assert guards.published_working(
        "ESTOP", "NONE", "MANUAL", "RUN") is False
    assert guards.published_working("AT_PANEL", "IDLE_P", "", "") is False
    assert guards.published_working("IDLE", "NONE", "", "") is False


def test_is_working_guard_unchanged():
    """行き先拒否のガード is_working は今の形のまま（モードと状態で見る）。
    ESTOP・CARRY 中は false のまま（出す値だけ published_working が別に決める）。"""
    assert guards.is_working("AT_PANEL", "WORKING") is True
    assert guards.is_working("ESTOP", "NONE") is False
    assert guards.is_working("CARRY", "NONE") is False
