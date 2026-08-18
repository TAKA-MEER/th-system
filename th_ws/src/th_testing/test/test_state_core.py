"""test_state_core.py — StateCore の単体試験（WP-STATE-01）

DetailedDesign-state.md §11 の要件 4・5・6 と、FMEA①〜③ に対応する
（パケット `WP-STATE-01` §7 の必須8試験）。th_state/th_state/state_core.py は
rclpy に依存しないので、この試験も ROS2 を必要としない。
"""
import random

import pytest

from th_state.state_core import Context, MODES, MODE_STATES


def _mk_ctx(**overrides):
    base = dict(
        prev_mode="", prev_state="", prev_sub="",
        flags={}, zone="OUT", candidate_count=0,
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


# ============================================================
# §11-4・5: validate() が空を返す（到達不能な状態・PAUSE欠落が無い）
# ============================================================
@pytest.mark.rule("C-01")
@pytest.mark.rule("C-06a")
@pytest.mark.rule("C-07")
def test_validate_returns_empty(state_core_bundle):
    core, transitions, mode_entry, attributes = state_core_bundle
    errors = core.validate()
    assert errors == [], f"validate() がエラーを返した: {errors}"


# ============================================================
# §11-6: 表に無い (mode, state, event) は必ず accepted=False
# （property test。ランダム10,000通り）
# ============================================================
_EVENT_UNIVERSE = [
    "ui.jog.hold", "ui.stop", "ui.confirm", "ui.run", "ui.save", "ui.finish",
    "ui.select_target", "ui.route_select", "ui.resume_yes", "ui.resume_no",
    "ui.resume_ack", "ui.abort", "ui.working", "ui.goto", "ui.register",
    "ui.return_home", "ui.map_edit", "ui.check_item", "ui.calib_item",
    "ui.calib_next", "ui.reroute", "ui.localize_global", "ui.screen",
    "ui.carry_resume", "ui.estop.press", "ui.estop.release", "ui.enter_mode",
    "evt.link_ok", "evt.arrived", "evt.align_done", "evt.blocked", "evt.unblocked",
    "evt.target_lost", "evt.auto_selected", "evt.clear_ok", "evt.clear_timeout",
    "evt.localize_done", "evt.localize_low", "evt.leash_taut", "evt.leash_slack",
    "evt.leash_absent", "evt.leash_present", "evt.line_lost", "evt.plane_done",
    "evt.two_point_done", "evt.setup_done", "evt.register_ok", "evt.register_rejected",
    "evt.record_broken", "evt.check_result", "evt.calib_step_done", "evt.calib_verify_ng",
    "fault.recoverable", "fault.critical", "hw.estop.press", "hw.estop.release",
    "sys.jog_lease_expired", "sys.link_timeout",
    "bogus.unknown.event",
]


def test_unknown_triple_is_rejected(state_core_bundle):
    """§11-6: 遷移表のどの行にもマッチしない (mode, state, event) は accepted=False。
    マッチする行が実在する三つ組は対象から除外し、純粋に「該当行が無い」ケースだけを見る。"""
    core, transitions, mode_entry, attributes = state_core_bundle

    def any_row_matches(mode, state, event):
        for row in transitions:
            rm, rs, re_ = row["mode"], row["state"], row["event"]
            mode_ok = rm == "*" or (isinstance(rm, list) and mode in rm) or rm == mode
            state_ok = rs == "*" or (isinstance(rs, list) and state in rs) or rs == state
            event_ok = (isinstance(re_, list) and event in re_) or re_ == event
            if mode_ok and state_ok and event_ok:
                return True
        return False

    modes = sorted(MODES)
    rng = random.Random(0)
    trial_count = 0
    checked = 0
    max_trials = 10000
    while trial_count < max_trials:
        trial_count += 1
        mode = rng.choice(modes)
        state = rng.choice(sorted(MODE_STATES[mode]))
        event = rng.choice(_EVENT_UNIVERSE)
        if any_row_matches(mode, state, event):
            continue  # 該当行がある三つ組は対象外
        checked += 1
        decision = core.step(mode, state, event, _mk_ctx())
        assert decision.accepted is False, (
            f"表に無い ({mode}, {state}, {event}) が accepted=True になった")
        assert decision.reject_reason_key, "accepted=False なのに reject_reason_key が空 (S-3)"
    assert checked > 0, "「該当行が無い」ケースが1つも生成されなかった（テストが無意味）"


# ============================================================
# FMEA①: override_common の行でもガード評価を省かない
# T-OPC-05（checking_estop_item）が false のとき、C-07（物理E-Stop→CARRY）が効く。
# MOTOR 点検中に物理E-Stopが効かなくなる、を防ぐ。
# ============================================================
@pytest.mark.rule("T-OPC-05")
@pytest.mark.rule("C-07")
def test_override_evaluates_guard(state_core_bundle):
    core, _, _, _ = state_core_bundle

    # MOTOR 項目を点検中（ESTOP 項目ではない）→ override のガードが false → C-07 に落ちる。
    ctx_motor = _mk_ctx(check_item="MOTOR")
    d1 = core.step("OPCHECK", "RUNNING_CHECK", "hw.estop.press", ctx_motor)
    assert d1.accepted is True
    assert d1.to_mode == "CARRY", (
        "MOTOR 点検中は override が効かず、物理 E-Stop は C-07 どおり CARRY へ落ちるべき")
    assert d1.rule_id == "C-07"

    # ESTOP 項目を点検中 → override のガードが true → RUNNING_CHECK のまま（CARRY に落ちない）。
    ctx_estop_item = _mk_ctx(check_item="ESTOP")
    d2 = core.step("OPCHECK", "RUNNING_CHECK", "hw.estop.press", ctx_estop_item)
    assert d2.accepted is True
    assert d2.to_mode == "OPCHECK" and d2.to_state == "RUNNING_CHECK"
    assert d2.rule_id == "T-OPC-05"


# ============================================================
# FMEA②: latch_prev は ESTOP / CARRY では記録しない（§7 のラッチ表）。
# ============================================================
@pytest.mark.rule("C-06a")
@pytest.mark.rule("C-06b")
@pytest.mark.rule("C-07")
def test_latch_skipped_in_estop_carry(state_core_bundle):
    core, _, _, _ = state_core_bundle

    # 通常モードからの重大フォルト → ESTOP: latch_prev が発火する。
    d_normal = core.step("FOLLOW", "RUN", "fault.critical", _mk_ctx())
    assert d_normal.accepted and d_normal.to_mode == "ESTOP"
    assert "latch_prev" in [e.name for e in d_normal.effects]

    # ESTOP 中に物理押下 → CARRY: latch_prev は発火しない（既にラッチ済みの prev_* を守る）。
    d_estop_to_carry = core.step("ESTOP", "NONE", "hw.estop.press", _mk_ctx())
    assert d_estop_to_carry.accepted and d_estop_to_carry.to_mode == "CARRY"
    assert "latch_prev" not in [e.name for e in d_estop_to_carry.effects]

    # CARRY 中に重大フォルト → ESTOP: latch_prev は発火しない。
    d_carry_to_estop = core.step("CARRY", "NONE", "fault.critical", _mk_ctx())
    assert d_carry_to_estop.accepted and d_carry_to_estop.to_mode == "ESTOP"
    assert "latch_prev" not in [e.name for e in d_carry_to_estop.effects]


# ============================================================
# FMEA③: C-06r（CARRY 中の UI 非常停止は拒否する）を落とさない。
# ============================================================
@pytest.mark.rule("C-06r")
def test_estop_in_carry_rejected(state_core_bundle):
    core, _, _, _ = state_core_bundle
    d = core.step("CARRY", "NONE", "ui.estop.press", _mk_ctx())
    assert d.accepted is False
    assert d.rule_id == "C-06r"
    assert d.reject_reason_key == "estop_disabled_in_carry"


# ============================================================
# C-09f: ESTOP はトラップにならない。
# fault.critical → ESTOP → ui.resume_ack → IDLE
# ============================================================
@pytest.mark.rule("C-06a")
@pytest.mark.rule("C-09f")
def test_estop_is_not_a_trap(state_core_bundle):
    core, _, _, _ = state_core_bundle

    d1 = core.step("FOLLOW", "RUN", "fault.critical", _mk_ctx())
    assert d1.accepted and d1.to_mode == "ESTOP" and d1.to_state == "NONE"

    # フォルトが解消し、UI・物理どちらの非常停止も解放されている状態で「確認」。
    d2 = core.step("ESTOP", "NONE", "ui.resume_ack",
                    _mk_ctx(fault_active=False, ui_estop=False, hw_estop=False))
    assert d2.accepted is True
    assert d2.to_mode == "IDLE" and d2.to_state == "NONE"
    assert d2.rule_id == "C-09f"

    # フォルトが継続中は「確認」しても出られない（トラップにならないことの反証確認）。
    d3 = core.step("ESTOP", "NONE", "ui.resume_ack", _mk_ctx(fault_active=True))
    assert d3.accepted is False


# ============================================================
# §3.5: ui.goto の kind → モード写像。PANEL というモードは存在しない。
# ============================================================
@pytest.mark.rule("C-15")
@pytest.mark.rule("T-ATP-05")
def test_goto_kind_mapping(state_core_bundle):
    core, _, _, _ = state_core_bundle

    d_panel = core.step("IDLE", "NONE", "ui.goto",
                         _mk_ctx(arg={"kind": "PANEL", "pin_id": "P1"}, pin_kinds=("PANEL",)))
    assert d_panel.accepted is True
    assert d_panel.to_mode == "PANEL_NAV"
    assert d_panel.to_mode != "PANEL"
    assert d_panel.to_mode in MODES

    d_home = core.step("IDLE", "NONE", "ui.goto", _mk_ctx(arg={"kind": "HOME"}))
    assert d_home.accepted is True and d_home.to_mode == "HOME_NAV"

    d_summon = core.step("IDLE", "NONE", "ui.goto", _mk_ctx(arg={"kind": "SUMMON"}))
    assert d_summon.accepted is True and d_summon.to_mode == "SUMMON"

    # AT_PANEL からも同じ写像で入れる（T-ATP-05）。
    d_atp = core.step("AT_PANEL", "IDLE_P", "ui.goto", _mk_ctx(arg={"kind": "HOME"}))
    assert d_atp.accepted is True and d_atp.to_mode == "HOME_NAV"
    assert d_atp.rule_id == "T-ATP-05"


# ============================================================
# T-MANUAL-01: RUN 発の自己ループを含む。
# リースは5Hz以上で送り続けるので、PAUSE 発だけにすると RUN 中の毎秒5回の
# ui.jog.hold がすべて not_allowed で拒否されてしまう。
# ============================================================
@pytest.mark.rule("T-MANUAL-01")
def test_manual_run_self_loop(state_core_bundle):
    core, _, _, _ = state_core_bundle

    # PAUSE -> RUN
    d1 = core.step("MANUAL", "PAUSE", "ui.jog.hold", _mk_ctx())
    assert d1.accepted is True and d1.to_state == "RUN" and d1.rule_id == "T-MANUAL-01"

    # RUN -> RUN（自己ループ。ここが無いと繰り返し送られる合図が拒否される）
    d2 = core.step("MANUAL", "RUN", "ui.jog.hold", _mk_ctx())
    assert d2.accepted is True and d2.to_state == "RUN" and d2.rule_id == "T-MANUAL-01"
