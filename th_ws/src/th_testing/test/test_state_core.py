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
    "ui.save", "ui.discard",
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
    """このテストは DetailedDesign-wp1.md §7 が名指しする
    test_critical_in_carry_keeps_prev（C-3）と
    test_hw_press_in_estop_keeps_prev（C-4）を、別テストとして重複させず
    このテストのアサーション 2・3 が兼ねている。対応は以下:

    - d_carry_to_estop（CARRY 中に fault.critical → ESTOP、latch_prev
      発火しない）= C-3。CARRY 中の重大フォルトが prev_* を CARRY 自身で
      上書きしてはいけない、という不変条件。
    - d_estop_to_carry（ESTOP 中に hw.estop.press → CARRY、latch_prev
      発火しない）= C-4。ESTOP 中の物理押下が prev_* を ESTOP 自身で
      上書きしてはいけない、という不変条件。

    d_normal（FOLLOW/RUN 中に fault.critical → ESTOP、latch_prev 発火する）
    は §7 のどの行にも対応しない、通常経路の対照用アサーション。
    """
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
# WP-CARRY-01 §7: C-07 → C-10 → C-11 の一巡（物理E-Stop押下でCARRYへ、
# 解除で表示、「再開」で押下前のモード・状態へ戻る）。
# ============================================================
@pytest.mark.rule("C-07")
@pytest.mark.rule("C-10")
@pytest.mark.rule("C-11")
def test_carry_roundtrip(state_core_bundle):
    core, _, _, _ = state_core_bundle

    # C-07: 通常モード（FOLLOW/RUN）中に物理E-Stop押下 → CARRY。
    # mode=FOLLOW は NO_LATCH_MODES に含まれないので latch_prev が発火する。
    d1 = core.step("FOLLOW", "RUN", "hw.estop.press", _mk_ctx())
    assert d1.accepted is True
    assert d1.to_mode == "CARRY" and d1.to_state == "NONE"
    assert d1.rule_id == "C-07"
    assert "latch_prev" in [e.name for e in d1.effects]
    assert "open_window" in [e.name for e in d1.effects]

    # latch_prev effect の適用（呼び出し側 = state_manager の仕事）を模擬し、
    # 押下前のモード・状態を prev_* として引き継いだ Context を以降で使う。
    ctx_pressed = _mk_ctx(prev_mode="FOLLOW", prev_state="RUN", hw_estop=True)

    # C-10: CARRY 中に物理E-Stop解除 → モード・状態は変わらず、show_resume を出す。
    d2 = core.step("CARRY", "NONE", "hw.estop.release", ctx_pressed)
    assert d2.accepted is True
    assert d2.to_mode == "CARRY" and d2.to_state == "NONE"
    assert d2.rule_id == "C-10"
    assert "show_resume" in [e.name for e in d2.effects]

    # C-11: 解除済み・重大フォルトなしで「再開」→ 押下前のモード（$prev_mode/$prev_state）へ戻る。
    ctx_released = _mk_ctx(prev_mode="FOLLOW", prev_state="RUN", hw_estop=False, fault_severity="")
    d3 = core.step("CARRY", "NONE", "ui.carry_resume", ctx_released)
    assert d3.accepted is True
    assert d3.to_mode == "FOLLOW" and d3.to_state == "RUN"
    assert d3.rule_id == "C-11"
    assert "close_window" in [e.name for e in d3.effects]


# ============================================================
# WP-CARRY-01 §7 FMEA②: hw_released_and_no_critical を hw_released だけにすると、
# 重大フォルトが継続したまま走行モードへ戻ってしまう（Spec-safety.md §3.5 違反）。
# 物理は解除済みでも、重大フォルトが残っている間は ui.carry_resume が通らないこと。
# ============================================================
@pytest.mark.rule("C-11")
def test_carry_resume_blocked_by_critical(state_core_bundle):
    core, _, _, _ = state_core_bundle

    ctx = _mk_ctx(prev_mode="FOLLOW", prev_state="RUN", hw_estop=False,
                  fault_active=True, fault_severity="CRITICAL")
    d = core.step("CARRY", "NONE", "ui.carry_resume", ctx)
    assert d.accepted is False
    assert d.reject_reason_key == "not_allowed"


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
# 2026-09-01 変更: UI ボタン起因の ESTOP（重大フォルト無し）は解除後に
# 「元のモードに戻る／メニューへ」を選べる（SM-3.1.1-11 / -11b / -11f）。
# ============================================================
@pytest.mark.rule("C-06b")
@pytest.mark.rule("C-09")
@pytest.mark.rule("C-09c-running")
@pytest.mark.rule("C-09d")
def test_ui_estop_resume_to_prev_mode(state_core_bundle):
    core, _, _, _ = state_core_bundle

    # UI ボタン → ESTOP。押下前のモード・状態をラッチ。
    d1 = core.step("MANUAL", "RUN", "ui.estop.press", _mk_ctx())
    assert d1.accepted and d1.to_mode == "ESTOP"
    assert "latch_prev" in [e.name for e in d1.effects]

    # 解除（UI 起因・重大フォルト無し・物理も解放・押下前が MANUAL）→ ESTOP のまま show_resume。
    ctx = _mk_ctx(prev_mode="MANUAL", prev_state="RUN", ui_estop=False, hw_estop=False,
                  estop_from_ui=True)
    d2 = core.step("ESTOP", "NONE", "ui.estop.release", ctx)
    assert d2.accepted is True
    assert d2.to_mode == "ESTOP" and d2.to_state == "NONE"
    assert d2.rule_id == "C-09"
    assert "show_resume" in [e.name for e in d2.effects]

    # 「元のモードに戻る」→ 押下前モードの PAUSE へ（勝手に RUN へは戻さない）。
    d3 = core.step("ESTOP", "NONE", "ui.resume_yes", ctx)
    assert d3.accepted is True
    assert d3.to_mode == "MANUAL" and d3.to_state == "PAUSE"
    assert d3.rule_id == "C-09c-running"
    assert "close_window" in [e.name for e in d3.effects]

    # 「メインメニューへ」→ IDLE。
    d4 = core.step("ESTOP", "NONE", "ui.resume_no", ctx)
    assert d4.accepted is True
    assert d4.to_mode == "IDLE" and d4.to_state == "NONE"
    assert d4.rule_id == "C-09d"


@pytest.mark.rule("C-09b")
@pytest.mark.rule("C-09c-running")
def test_ui_estop_release_goes_idle_when_prev_not_resumable_or_critical(state_core_bundle):
    core, _, _, _ = state_core_bundle

    # 押下前が IDLE（＝メニューで押した）→ 解除は IDLE のみ（C-09b）。
    d1 = core.step("ESTOP", "NONE", "ui.estop.release",
                    _mk_ctx(prev_mode="IDLE", ui_estop=False, hw_estop=False,
                            estop_from_ui=True))
    assert d1.accepted is True and d1.to_mode == "IDLE"
    assert d1.rule_id == "C-09b"

    # WS-9O (2026-09-04): フォルト起因の ESTOP（estop_from_ui=False）でも、フォルトが
    # 消えていて押下前が動作系モードなら復帰の選択肢を出す（C-09）。
    # 以前は estop_from_ui を要求していたため必ず IDLE（C-09b）に落ちており、再生中に
    # 一瞬のフォルトで止まると経路選択からやり直しになっていた（Spec-safety.md §3.5.2）。
    d1b = core.step("ESTOP", "NONE", "ui.estop.release",
                     _mk_ctx(prev_mode="MANUAL", prev_state="RUN",
                             ui_estop=False, hw_estop=False, estop_from_ui=False))
    assert d1b.accepted is True and d1b.rule_id == "C-09"
    assert d1b.to_mode == "ESTOP", "C-09 は ESTOP に留まって選択肢を出す"

    # そのまま「元のモードに戻る」を押すと、押下前モードの PAUSE（停止状態）へ。
    # 走り出すには操作者がもう一度「再生」を押す必要がある。
    d1c = core.step("ESTOP", "NONE", "ui.resume_yes",
                     _mk_ctx(prev_mode="MANUAL", prev_state="RUN",
                             ui_estop=False, hw_estop=False, estop_from_ui=False))
    assert d1c.accepted is True and d1c.rule_id == "C-09c-running"
    assert d1c.to_mode == "MANUAL" and d1c.to_state == "PAUSE"

    # 押下前が復帰不能なモードなら、フォルト起因でも IDLE（C-09b）。
    d1d = core.step("ESTOP", "NONE", "ui.estop.release",
                     _mk_ctx(prev_mode="IDLE", ui_estop=False, hw_estop=False,
                             estop_from_ui=False))
    assert d1d.accepted is True and d1d.to_mode == "IDLE"
    assert d1d.rule_id == "C-09b"

    # 物理ボタンが押されたままなら復帰させない（フォルト起因でも同じ）。
    d1e = core.step("ESTOP", "NONE", "ui.resume_yes",
                     _mk_ctx(prev_mode="MANUAL", prev_state="RUN",
                             ui_estop=False, hw_estop=True, estop_from_ui=False))
    assert d1e.accepted is False

    # 重大フォルトが継続中は、UI 起因でも復帰させない（C-09b にフォールバック）。
    d2 = core.step("ESTOP", "NONE", "ui.estop.release",
                    _mk_ctx(prev_mode="MANUAL", prev_state="RUN",
                            ui_estop=False, hw_estop=False, estop_from_ui=True,
                            fault_active=True, fault_severity="CRITICAL"))
    # 重大フォルト継続中は no_critical_fault も false なので C-09b も通らない。
    assert d2.accepted is False

    # 「元のモードに戻る」は重大フォルト中は拒否される。
    d3 = core.step("ESTOP", "NONE", "ui.resume_yes",
                    _mk_ctx(prev_mode="MANUAL", prev_state="RUN", estop_from_ui=True,
                            fault_active=True, fault_severity="CRITICAL"))
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


# ============================================================
# SM-3.1.2-116 / -117（2026-10-04）: 実行中の項目から抜けられる・別の項目へ移れる。
# 結果は記録しない（record_result を出さない）。runner には abort_check を必ず送る
# （送らないと前の項目のモニター・モーター指令が残る）。
# ============================================================
@pytest.mark.rule("T-OPC-09")
def test_opcheck_running_abort_returns_to_list(state_core_bundle):
    core, _, _, _ = state_core_bundle
    d = core.step("OPCHECK", "RUNNING_CHECK", "ui.abort", _mk_ctx(check_item="LIDAR"))
    assert d.accepted is True
    assert (d.to_mode, d.to_state) == ("OPCHECK", "LIST")
    names = [e.name for e in d.effects]
    assert names == ["abort_check"], names


@pytest.mark.rule("T-OPC-10")
def test_opcheck_running_select_other_item_switches(state_core_bundle):
    core, _, _, _ = state_core_bundle
    d = core.step("OPCHECK", "RUNNING_CHECK", "ui.check_item",
                  _mk_ctx(check_item="LIDAR", arg={"item": "MOTOR"}))
    assert d.accepted is True
    assert (d.to_mode, d.to_state) == ("OPCHECK", "RUNNING_CHECK")
    names = [e.name for e in d.effects]
    assert names == ["abort_check", "start_monitor"], names
    assert d.effects[1].args == {"item": "MOTOR"}


# ============================================================
# 1b-8 (SG-B5): NG → LIST →「校正へ」→ CALIB の本番の順番。
# T-OPC-02（NG・校正可 → LIST）→ T-OPC-07（LIST → CALIB）が通ることを
# 実際の順番で回して縛る。画面はこの順番にボタンを出す（e2e 側）。
# ============================================================
@pytest.mark.rule("T-OPC-02")
@pytest.mark.rule("T-OPC-07")
def test_opcheck_ng_lidar_returns_to_list_then_calib(state_core_bundle):
    core, _, _, _ = state_core_bundle
    d1 = core.step("OPCHECK", "LIST", "ui.check_item",
                   _mk_ctx(arg={"item": "LIDAR"}))
    assert d1.accepted is True
    assert (d1.to_mode, d1.to_state) == ("OPCHECK", "RUNNING_CHECK")
    assert [e.name for e in d1.effects] == ["start_monitor"]

    # runner が出す evt.check_result（IMU/LIDAR の NG → LIST＋校正導線）。
    d2 = core.step("OPCHECK", "RUNNING_CHECK", "evt.check_result",
                   _mk_ctx(check_item="LIDAR", check_result="NG",
                           arg={"item": "LIDAR", "result": "NG"}))
    assert d2.accepted is True, "T-OPC-02 が通らない（前提が崩れている）"
    assert (d2.to_mode, d2.to_state) == ("OPCHECK", "LIST")
    assert d2.rule_id == "T-OPC-02"
    assert [e.name for e in d2.effects] == ["offer_calib", "record_result"], \
        [e.name for e in d2.effects]

    # LIST で「校正へ」（T-OPC-07 は LIST 限定。本番の順番どおり）。
    d3 = core.step("OPCHECK", "LIST", "ui.enter_mode",
                   _mk_ctx(arg={"mode": "CALIB"}))
    assert d3.accepted is True, \
        "LIST からの ui.enter_mode{CALIB} が拒否された（T-OPC-07 の標的）"
    assert (d3.to_mode, d3.to_state) == ("CALIB", "LIST")
    assert d3.rule_id == "T-OPC-07"


# ============================================================
# 1b-2 追補: ESTOP／CARRY に入るとき jog を解除する。
# jog 中（jog_active）に非常停止・物理ボタンを押すと、ESTOP/CARRY をまたいで
# jog_active が残り、復帰先で stale のままになる。突入時に set_jog{on:false} で
# 落とす（既存の effect 駆動。state_core 側に特別な処理は無い）。
# 復帰後に触れれば通常どおりリースが始まる。
# ============================================================
@pytest.mark.rule("C-06a")
@pytest.mark.rule("C-06b")
@pytest.mark.rule("C-07")
def test_estop_carry_entry_clears_jog(state_core_bundle):
    core, _, _, _ = state_core_bundle

    d = core.step("FOLLOW", "PAUSE", "fault.critical", _mk_ctx())
    assert d.accepted is True
    assert (d.to_mode, d.to_state) == ("ESTOP", "NONE")
    assert d.rule_id == "C-06a"
    by_name = {e.name: e.args for e in d.effects}
    assert by_name.get("set_jog") == {"on": False}, \
        f"C-06a が jog を解除していない: {[e.name for e in d.effects]}"

    d = core.step("FOLLOW", "PAUSE", "ui.estop.press", _mk_ctx())
    assert d.accepted is True
    assert (d.to_mode, d.to_state) == ("ESTOP", "NONE")
    assert d.rule_id == "C-06b"
    by_name = {e.name: e.args for e in d.effects}
    assert by_name.get("set_jog") == {"on": False}, \
        f"C-06b が jog を解除していない: {[e.name for e in d.effects]}"

    d = core.step("FOLLOW", "PAUSE", "hw.estop.press", _mk_ctx())
    assert d.accepted is True
    assert (d.to_mode, d.to_state) == ("CARRY", "NONE")
    assert d.rule_id == "C-07"
    by_name = {e.name: e.args for e in d.effects}
    assert by_name.get("set_jog") == {"on": False}, \
        f"C-07 が jog を解除していない: {[e.name for e in d.effects]}"


# ============================================================
# 1b-2（SG-A3・SG-D1）: ESTOP からの「戻る」は §6 の復帰先へ。
# 走行中（自律走行・記録中）なら同モードの PAUSE、止まっている状態なら
# 押下前の状態へ入口からやり直す。PAUSE を持たないモードに PAUSE を作らない。
# 本番の経路（StateCore.step）を「押下 → 解除 → 戻る」の順で回す。
# ============================================================
# 走行中（自律走行・記録中）なら同モードの PAUSE、止まっている状態なら
# 押下前の状態へ入口からやり直す。PAUSE を持たないモードに PAUSE を作らない。
# 本番の経路（StateCore.step）を「押下 → 解除 → 戻る」の順で回す。
# ============================================================
def _resume_ctx(prev_mode, prev_state, **over):
    kw = dict(prev_mode=prev_mode, prev_state=prev_state,
              fault_active=False, fault_severity="", fault_type="",
              hw_estop=False, ui_estop=False)
    kw.update(over)
    return _mk_ctx(**kw)


@pytest.mark.rule("C-09c-running")
def test_estop_resume_running_goes_to_pause(state_core_bundle):
    """走行中の 9 モードは同モードの PAUSE へ（勝手に走り出さない）。"""
    core, _, _, _ = state_core_bundle
    for prev_mode, run_state in [
            ("FOLLOW", "RUN"), ("MANUAL", "RUN"),
            ("TEACH_FOLLOW", "REC"), ("TEACH_MANUAL", "REC"),
            ("REPLAY", "RUN"), ("LINE", "RUN"), ("LEASH", "RUN"),
            ("PANEL_NAV", "NAV"), ("HOME_NAV", "NAV")]:
        d = core.step("ESTOP", "NONE", "ui.resume_yes",
                      _resume_ctx(prev_mode, run_state))
        assert d.accepted is True, f"{prev_mode}/{run_state} が拒否された"
        assert (d.to_mode, d.to_state) == (prev_mode, "PAUSE"), \
            f"{prev_mode}/{run_state} の戻り先が PAUSE ではない: {d.to_mode}/{d.to_state}"
        assert d.rule_id == "C-09c-running", f"{prev_mode}: {d.rule_id}"
        assert "clear_prev" in [e.name for e in d.effects]


@pytest.mark.rule("C-09c-generic")
def test_estop_resume_stopped_returns_to_prev_state(state_core_bundle):
    """止まっている状態は押下前の状態へ（入口からやり直す）。effect は出さない
    （通常の入口の遷移も effect を持たない状態だけがここに来る）。"""
    core, _, _, _ = state_core_bundle
    for prev_mode, prev_state in [
            ("FOLLOW", "SELECT"),
            ("TEACH_FOLLOW", "ROUTE_SEL"), ("TEACH_FOLLOW", "PAUSE"),
            ("TEACH_FOLLOW", "SAVED"),
            ("TEACH_MANUAL", "ROUTE_SEL"), ("TEACH_MANUAL", "PAUSE"),
            ("TEACH_MANUAL", "SAVED"),
            ("REPLAY", "ROUTE_SEL"), ("REPLAY", "READY"), ("REPLAY", "PAUSE"),
            ("REPLAY", "SAVED"),
            ("LINE", "SETUP"), ("LINE", "PLANNED"), ("LINE", "PAUSE"),
            ("LINE", "ARRIVED"),
            ("LEASH", "DEV_CHECK"), ("LEASH", "READY"), ("LEASH", "HOLD"),
            ("LEASH", "PAUSE"),
            ("PANEL_NAV", "PAUSE"), ("PANEL_NAV", "ALIGN"),
            ("HOME_NAV", "PAUSE")]:
        # AT_HOME/PAUSE は C-09c-at-home（IDLE_H）が先に拾うためここに含めない。
        d = core.step("ESTOP", "NONE", "ui.resume_yes",
                      _resume_ctx(prev_mode, prev_state))
        assert d.accepted is True, f"{prev_mode}/{prev_state} が拒否された"
        assert (d.to_mode, d.to_state) == (prev_mode, prev_state), \
            f"戻り先が押下前の状態ではない: {d.to_mode}/{d.to_state}"
        assert d.rule_id == "C-09c-generic", f"{prev_mode}/{prev_state}: {d.rule_id}"


@pytest.mark.rule("C-09c-generic")
def test_estop_resume_teach_route_sel_emits_no_start_record(state_core_bundle):
    """SG-A3 の再発防止: TEACH_MANUAL/ROUTE_SEL から戻って REC に入る経路が無い。
    ROUTE_SEL へ戻り、start_record は出さない（選び直したときに出る）。"""
    core, _, _, _ = state_core_bundle
    d = core.step("ESTOP", "NONE", "ui.resume_yes",
                  _resume_ctx("TEACH_MANUAL", "ROUTE_SEL"))
    assert (d.to_mode, d.to_state) == ("TEACH_MANUAL", "ROUTE_SEL")
    assert "start_record" not in [e.name for e in d.effects]
    assert "resume_record" not in [e.name for e in d.effects]


@pytest.mark.rule("C-09c-confirm")
def test_estop_resume_follow_confirm_refaces_target(state_core_bundle):
    """FOLLOW/CONFIRM へ戻り、向き直し（face_target）を再実行する（確認を飛ばさない）。"""
    core, _, _, _ = state_core_bundle
    d = core.step("ESTOP", "NONE", "ui.resume_yes",
                  _resume_ctx("FOLLOW", "CONFIRM"))
    assert d.accepted is True
    assert (d.to_mode, d.to_state) == ("FOLLOW", "CONFIRM")
    assert d.rule_id == "C-09c-confirm"
    assert "face_target" in [e.name for e in d.effects]


@pytest.mark.rule("C-09c-localize")
def test_estop_resume_replay_localize_reloads_route(state_core_bundle):
    """REPLAY/LOCALIZE へ戻り、経路読み込み（load_route）を再実行する。
    route_id / reverse は ui.resume_yes の arg から解決する
    （state_manager がラッチ済みの選択で補完する。本番の経路）。"""
    core, _, _, _ = state_core_bundle
    d = core.step("ESTOP", "NONE", "ui.resume_yes",
                  _resume_ctx("REPLAY", "LOCALIZE",
                              arg={"id": "R1", "reverse": False}))
    assert d.accepted is True
    assert (d.to_mode, d.to_state) == ("REPLAY", "LOCALIZE")
    assert d.rule_id == "C-09c-localize"
    by_name = {e.name: e.args for e in d.effects}
    assert by_name.get("load_route") == {"route_id": "R1", "reverse": False}, \
        f"load_route の引数が違う: {by_name}"


@pytest.mark.rule("C-09c-blocked")
def test_estop_resume_blocked_reopens_window(state_core_bundle):
    """BLOCKED へ戻り、W-5 を開き直す（塞がれ中の表示を復元する）。"""
    core, _, _, _ = state_core_bundle
    for prev_mode in ("PANEL_NAV", "HOME_NAV"):
        d = core.step("ESTOP", "NONE", "ui.resume_yes",
                      _resume_ctx(prev_mode, "BLOCKED"))
        assert d.accepted is True
        assert (d.to_mode, d.to_state) == (prev_mode, "BLOCKED")
        assert d.rule_id == "C-09c-blocked", f"{prev_mode}: {d.rule_id}"
        opened = [e.args.get("id") for e in d.effects if e.name == "open_window"]
        assert "W-5" in opened, f"W-5 を開き直していない: {[e.name for e in d.effects]}"


@pytest.mark.rule("C-09c-summon")
def test_estop_resume_summon_returns_to_point(state_core_bundle):
    """SG-A2: SUMMON はどの状態からでも POINT（退避待ちからやり直す）。
    NAV／PAUSE に戻すと T-SUM-07（ガード無し）で退避待ちを飛ばして発進できる。"""
    core, _, _, _ = state_core_bundle
    for prev_state in ("POINT", "WAIT_CLEAR", "NAV", "BLOCKED", "PAUSE", "ALIGN"):
        d = core.step("ESTOP", "NONE", "ui.resume_yes",
                      _resume_ctx("SUMMON", prev_state))
        assert d.accepted is True, f"SUMMON/{prev_state} が拒否された"
        assert (d.to_mode, d.to_state) == ("SUMMON", "POINT"), \
            f"SUMMON/{prev_state} の戻り先が POINT ではない: {d.to_mode}/{d.to_state}"
        assert d.rule_id == "C-09c-summon", f"SUMMON/{prev_state}: {d.rule_id}"
    names = [e.name for e in d.effects]
    assert "enable_jog_ui" in names, \
        "WAIT_CLEAR で無効化した手動 UI を戻していない"


@pytest.mark.rule("C-09c-at-panel")
@pytest.mark.rule("C-09c-at-home")
def test_estop_resume_at_panel_home(state_core_bundle):
    """AT_PANEL→IDLE_P／AT_HOME→IDLE_H（§6 の右列。確認をやり直す）。"""
    core, _, _, _ = state_core_bundle
    for prev_mode, prev_state, dst in [("AT_PANEL", "WORKING", "IDLE_P"),
                                       ("AT_PANEL", "PAUSE", "IDLE_P"),
                                       ("AT_PANEL", "IDLE_P", "IDLE_P"),
                                       ("AT_HOME", "PAUSE", "IDLE_H"),
                                       ("AT_HOME", "IDLE_H", "IDLE_H")]:
        d = core.step("ESTOP", "NONE", "ui.resume_yes",
                      _resume_ctx(prev_mode, prev_state))
        assert d.accepted is True, f"{prev_mode}/{prev_state} が拒否された"
        assert (d.to_mode, d.to_state) == (prev_mode, dst), \
            f"戻り先が違う: {d.to_mode}/{d.to_state}"


@pytest.mark.rule("C-09c-opcheck")
@pytest.mark.rule("C-09c-calib")
def test_estop_resume_opcheck_calib_returns_to_list(state_core_bundle):
    """SG-A3: OPCHECK／CALIB は LIST（定義外の X/PAUSE に入らない）。
    実行中の項目は通常のフォルト時と同じ effect で畳む。"""
    core, _, _, _ = state_core_bundle
    d = core.step("ESTOP", "NONE", "ui.resume_yes",
                  _resume_ctx("OPCHECK", "RUNNING_CHECK"))
    assert d.accepted is True
    assert (d.to_mode, d.to_state) == ("OPCHECK", "LIST")
    assert d.rule_id == "C-09c-opcheck"
    assert "abort_check" in [e.name for e in d.effects]

    d = core.step("ESTOP", "NONE", "ui.resume_yes",
                  _resume_ctx("CALIB", "S2"))
    assert d.accepted is True
    assert (d.to_mode, d.to_state) == ("CALIB", "LIST")
    assert d.rule_id == "C-09c-calib"
    assert "discard_calib" in [e.name for e in d.effects]


@pytest.mark.rule("C-09c-prep-return")
@pytest.mark.rule("C-09c-prep")
def test_estop_resume_prep_returns_to_prev_state(state_core_bundle):
    """PREP は押下前の状態へ。RETURN だった場合は PREP/PAUSE
    （SG-A7。RETURN の PAUSE を 1b-1 で作った）。"""
    core, _, _, _ = state_core_bundle
    for prev_state, dst in [("MAPPING", "MAPPING"), ("REGISTER", "REGISTER"),
                            ("RETURN", "PAUSE"), ("EDIT", "EDIT"),
                            ("SAVED", "SAVED")]:
        d = core.step("ESTOP", "NONE", "ui.resume_yes",
                      _resume_ctx("PREP", prev_state))
        assert d.accepted is True, f"PREP/{prev_state} が拒否された"
        assert (d.to_mode, d.to_state) == ("PREP", dst), \
            f"PREP/{prev_state} の戻り先が違う: {d.to_mode}/{d.to_state}"
    d = core.step("ESTOP", "NONE", "ui.resume_yes",
                  _resume_ctx("PREP", "RETURN"))
    assert d.rule_id == "C-09c-prep-return"


# ============================================================
# 1b-2（SG-A5）: 押下前が INIT なら INIT/CHECK へ戻り疎通確認からやり直す。
# 「メニューへ」（C-09d）もフォルト解消・両非常停止解放のときだけ。
# ============================================================
@pytest.mark.rule("C-09b-init")
def test_estop_release_from_init_returns_to_check(state_core_bundle):
    core, _, _, _ = state_core_bundle

    # UI 非常停止 → 解除で INIT/CHECK（IDLE へ行かない）。
    d = core.step("ESTOP", "NONE", "ui.estop.release",
                  _mk_ctx(prev_mode="INIT", prev_state="CHECK",
                           fault_active=False, fault_severity="",
                           hw_estop=False, ui_estop=False))
    assert d.accepted is True
    assert (d.to_mode, d.to_state) == ("INIT", "CHECK")
    assert d.rule_id == "C-09b-init"

    # 物理非常停止が押されたままでも IDLE に入れない（T-INIT-03 が留める）。
    d = core.step("ESTOP", "NONE", "ui.estop.release",
                  _mk_ctx(prev_mode="INIT", prev_state="CHECK",
                           fault_active=False, fault_severity="",
                           hw_estop=True, ui_estop=False))
    assert d.accepted is True
    assert (d.to_mode, d.to_state) == ("INIT", "CHECK"), \
        "物理押下中のまま IDLE に入ってしまう（SG-A5）"
    assert d.rule_id == "C-09b-init"


@pytest.mark.rule("C-09d")
def test_estop_menu_requires_fault_cleared_and_released(state_core_bundle):
    """C-09d「メニューへ」は C-09f と同じガード（重大フォルト中・押下中は拒否）。"""
    core, _, _, _ = state_core_bundle
    cleared = dict(prev_mode="FOLLOW", prev_state="RUN",
                   fault_active=False, fault_severity="",
                   hw_estop=False, ui_estop=False)
    d = core.step("ESTOP", "NONE", "ui.resume_no", _mk_ctx(**cleared))
    assert d.accepted is True
    assert (d.to_mode, d.to_state) == ("IDLE", "NONE")
    assert d.rule_id == "C-09d"

    for bad in [dict(fault_active=True, fault_severity="CRITICAL"),
                dict(fault_active=True, fault_severity=""),
                dict(hw_estop=True),
                dict(ui_estop=True)]:
        kw = dict(cleared)
        kw.update(bad)
        d = core.step("ESTOP", "NONE", "ui.resume_no", _mk_ctx(**kw))
        assert d.accepted is False, f"{bad} のまま C-09d が通ってしまった"


# ============================================================
# 1b-2（SG-B22）: C-04／C-05 が state=None を作らない。
# ============================================================
@pytest.mark.rule("C-04")
def test_c04_rejected_where_run_state_is_null(state_core_bundle):
    """run_state が null のモード（AT_PANEL／AT_HOME／OPCHECK／CALIB）では
    ui.resume_yes を遷移させない（None を publish して state_manager を落とす
    SG-B22 の穴）。走行状態を持つモードは従来どおり通る。"""
    core, _, _, attrs = state_core_bundle
    for mode, state in [("AT_PANEL", "PAUSE"), ("AT_HOME", "PAUSE")]:
        assert attrs[mode]["run_state"] is None, "前提（run_state null）が崩れている"
        d = core.step(mode, state, "ui.resume_yes", _mk_ctx(fault_active=False))
        assert d.accepted is False, f"{mode}/{state} で C-04 が通ってしまった"
        assert d.to_state is not None
    d = core.step("FOLLOW", "PAUSE", "ui.resume_yes", _mk_ctx(fault_active=False))
    assert d.accepted is True
    assert (d.to_mode, d.to_state) == ("FOLLOW", "RUN")


@pytest.mark.rule("C-05")
@pytest.mark.rule("T-PREP-17")
def test_c05_rejected_where_resume_state_is_null(state_core_bundle):
    """resume_state が null の PREP では汎用の C-05 は通らない（SG-B22）。
    PREP/PAUSE の「いいえ」は専用の T-PREP-17 が MAPPING へ送る（SG-A7）。
    他のモードは §6 の復帰先へ送る（spec §6.1 の読み方どおり）。"""
    core, _, _, attrs = state_core_bundle
    assert attrs["PREP"]["resume_state"] is None, "前提（resume_state null）が崩れている"
    for event in ("ui.resume_no", "ui.resume_ack"):
        d = core.step("PREP", "PAUSE", event, _mk_ctx(fault_active=False))
        assert d.accepted is True, f"PREP/PAUSE で {event} が拒否された"
        assert (d.to_mode, d.to_state) == ("PREP", "MAPPING"), \
            f"PREP/PAUSE の {event} の行き先が違う: {d.to_mode}/{d.to_state}"
        assert d.rule_id == "T-PREP-17"
    d = core.step("AT_PANEL", "PAUSE", "ui.resume_ack", _mk_ctx(fault_active=False))
    assert d.accepted is True
    assert (d.to_mode, d.to_state) == ("AT_PANEL", "IDLE_P")


def test_c04_c05_have_availability_guards(state_core_bundle):
    """SG-B22 の再発防止: C-04／C-05 のガードが availability ガードであること。
    _validate_combos は None 解決をガードに委ねるため、ガードが差し戻されると
    None が復活する。この test がそれを縛る。"""
    _, transitions, _, _ = state_core_bundle
    by_id = {row["id"]: row for row in transitions}
    assert by_id["C-04"]["guard"] == "resume_run_available", \
        f"C-04 のガードが差し戻されている: {by_id['C-04']['guard']}"
    assert by_id["C-05"]["guard"] == "resume_state_available", \
        f"C-05 のガードが差し戻されている: {by_id['C-05']['guard']}"


def test_estop_resume_run_matches_attributes_run_state(state_core_bundle):
    """「走行中」の判定（ESTOP_RESUME_RUN）が attributes.yaml の run_state と一致する。
    食い違うと、走行中なのに旧状態へ戻って確認を飛ばす／止まっているのに PAUSE に
    落ちて入口を飛ばす。どちらも SG-A3 と同じ型の穴。"""
    from th_state.state_core import ESTOP_RESUME_RUN
    _, _, _, attrs = state_core_bundle
    for mode, run_state in ESTOP_RESUME_RUN.items():
        assert attrs[mode]["run_state"] == run_state, \
            f"{mode}: ESTOP_RESUME_RUN={run_state} が attributes run_state=" \
            f"{attrs[mode]['run_state']} と食い違う"
    # PREP／SUMMON は別行が先に拾うためここに含めない（一貫性の宣言）。
    assert "PREP" not in ESTOP_RESUME_RUN and "SUMMON" not in ESTOP_RESUME_RUN


def test_validate_combos_catches_pause_fixed_regression(state_core_bundle):
    """組み合わせ検査（_validate_combos）の検出力: C-09c 系を PAUSE 固定に戻す変異は
    validate() で赤くなること（SG-A3 の再発を起動時に止める）。"""
    import copy
    core, transitions, mode_entry, attributes = state_core_bundle
    assert core.validate() == [], f"現行表で validate が赤: {core.validate()}"
    mutated = copy.deepcopy(transitions)
    for row in mutated:
        if row["id"].startswith("C-09c-"):
            row["guard"] = "estop_resume_prev"
            row["to_mode"] = "$prev_mode"
            row["to_state"] = "PAUSE"
    from th_state.state_core import StateCore
    from th_state import guards as guards_module
    core2 = StateCore(mutated, mode_entry, attributes,
                      guards_module.build_guards(mode_entry, attributes))
    errors = [e for e in core2.validate() if "組合せ" in e]
    assert errors, "PAUSE 固定に戻す変異を組み合わせ検査が捕まえていない"


# ============================================================
# 1b-1（SG-A7）: PREP/RETURN の一時停止。
# RETURN 中だけジョグ・回復フォルトで PREP/PAUSE に落とす。
# ============================================================
@pytest.mark.rule("C-01")
def test_prep_return_jog_falls_to_pause(state_core_bundle):
    """PREP/RETURN 中のジョグ介入 → PREP/PAUSE（C-01。set_jog 付き）。"""
    core, _, _, _ = state_core_bundle
    d = core.step("PREP", "RETURN", "ui.jog.hold", _mk_ctx())
    assert d.accepted is True
    assert (d.to_mode, d.to_state) == ("PREP", "PAUSE")
    assert d.rule_id == "C-01"
    assert any(e.name == "set_jog" and e.args.get("on") is True for e in d.effects)


@pytest.mark.rule("C-03")
def test_prep_return_recoverable_fault_falls_to_pause(state_core_bundle):
    """PREP/RETURN 中の回復フォルト → PREP/PAUSE（C-03。W-1 を開く）。"""
    core, _, _, _ = state_core_bundle
    d = core.step("PREP", "RETURN", "fault.recoverable",
                  _mk_ctx(fault_active=True, fault_type="LIDAR_LOST"))
    assert d.accepted is True
    assert (d.to_mode, d.to_state) == ("PREP", "PAUSE")
    assert d.rule_id == "C-03"
    assert any(e.name == "open_window" and e.args.get("id") == "W-1" for e in d.effects)


@pytest.mark.rule("C-01")
@pytest.mark.rule("C-03")
def test_prep_non_return_states_hold_on_jog_and_fault(state_core_bundle):
    """MAPPING／REGISTER／EDIT／SAVED ではジョグ・回復フォルトで状態が変わらない
    （地図作成中の連れ回し。既存の $pause_unless_prep の縛り）。"""
    core, _, _, _ = state_core_bundle
    for state in ("MAPPING", "REGISTER", "EDIT", "SAVED"):
        d = core.step("PREP", state, "ui.jog.hold", _mk_ctx())
        assert d.accepted is True
        assert (d.to_mode, d.to_state) == ("PREP", state), \
            f"PREP/{state} がジョグで動いた: {d.to_mode}/{d.to_state}"
        # 回復フォルトは C-03 の対象外（拒否。状態も W-1 も動かさない）。
        d = core.step("PREP", state, "fault.recoverable",
                      _mk_ctx(fault_active=True, fault_type="LIDAR_LOST"))
        assert d.accepted is False, f"PREP/{state} で C-03 が通ってしまった"
        assert (d.to_mode, d.to_state) == ("PREP", state), \
            f"PREP/{state} が回復フォルトで動いた: {d.to_mode}/{d.to_state}"


@pytest.mark.rule("T-PREP-18")
@pytest.mark.rule("T-PREP-10")
def test_prep_return_stop_falls_to_pause_others_inert(state_core_bundle):
    """PREP/RETURN 中の「停止」→ PREP/PAUSE（T-PREP-18。SM-3.1.2-050a）。
    RETURN 以外は inert のまま（T-PREP-10。SM-3.1.2-050）。"""
    core, _, _, _ = state_core_bundle
    d = core.step("PREP", "RETURN", "ui.stop", _mk_ctx())
    assert d.accepted is True
    assert (d.to_mode, d.to_state) == ("PREP", "PAUSE"), \
        f"行き先が違う: {d.to_mode}/{d.to_state}"
    assert d.rule_id == "T-PREP-18"
    for state in ("MAPPING", "REGISTER", "EDIT", "SAVED", "PAUSE"):
        d = core.step("PREP", state, "ui.stop", _mk_ctx())
        assert d.accepted is True, f"PREP/{state} で停止が拒否された"
        assert (d.to_mode, d.to_state) == ("PREP", state), \
            f"PREP/{state} が停止で動いた: {d.to_mode}/{d.to_state}"
        assert d.rule_id == "T-PREP-10"


@pytest.mark.rule("T-PREP-16")
def test_prep_pause_resume_yes_returns_to_return(state_core_bundle):
    """PREP/PAUSE の「はい」→ RETURN（T-PREP-16。汎用 C-04 の MAPPING ではない）。
    venue_navigator が PAUSE 中に取り消した FollowPath の残りを送り直す
    （状態遷移で戻るだけ。effect は close_window のみ）。"""
    core, _, _, _ = state_core_bundle
    d = core.step("PREP", "PAUSE", "ui.resume_yes", _mk_ctx(fault_active=False))
    assert d.accepted is True
    assert (d.to_mode, d.to_state) == ("PREP", "RETURN"), \
        f"行き先が違う: {d.to_mode}/{d.to_state}"
    assert d.rule_id == "T-PREP-16"
    assert any(e.name == "close_window" and e.args.get("id") == "W-1" for e in d.effects)
    # フォルト継続中は「はい」を受け付けない（C-04 と同じ fault_cleared）。
    d = core.step("PREP", "PAUSE", "ui.resume_yes",
                  _mk_ctx(fault_active=True, fault_type="LIDAR_LOST"))
    assert d.accepted is False, "フォルト継続中に PREP/PAUSE の「はい」が通ってしまった"
    assert (d.to_mode, d.to_state) == ("PREP", "PAUSE")


@pytest.mark.rule("C-03")
def test_prep_tracker_lost_never_pauses(state_core_bundle):
    """PREP では PERSON_TRACKER_LOST で PAUSE にしない（RETURN／PAUSE 中も。
    Spec-modes.md §5 の表どおり。登録拒否だけで地図作成は続く）。"""
    core, _, _, _ = state_core_bundle
    for state in ("RETURN", "PAUSE", "MAPPING"):
        d = core.step("PREP", state, "fault.recoverable",
                      _mk_ctx(fault_active=True, fault_type="PERSON_TRACKER_LOST"))
        assert d.accepted is False, \
            f"PREP/{state} が人物追跡ロストで動いた: {d.to_mode}/{d.to_state}"


# ============================================================
# 1b-1（SG-A12）: 「走行中」の判定と sys.presence_lost。
# ============================================================
def test_is_driving_matches_brief_table(state_core_bundle):
    """「走行中」の判定がブリーフの一覧と一致すること。全モード×全状態の表で縛る。
    run_state は attributes.yaml から読むので、新しい走行状態が増えたら
    自動で拾われる（PREP/MAPPING だけ除外。向き合わせ・帰還は明示）。"""
    from th_state.state_core import MODE_STATES, is_driving
    _, _, _, attrs = state_core_bundle
    expected = set()
    for mode, row in attrs.items():
        run = (row or {}).get("run_state")
        if run is not None and mode != "PREP":
            expected.add((mode, run))
    expected |= {("PANEL_NAV", "ALIGN"), ("SUMMON", "ALIGN"), ("PREP", "RETURN")}
    for mode, states in MODE_STATES.items():
        for state in states:
            assert is_driving(mode, state, attrs) == ((mode, state) in expected), \
                f"is_driving({mode}, {state}) が一覧と違う"
    # PREP の run_state（MAPPING）は走行ではない。
    assert is_driving("PREP", "MAPPING", attrs) is False
    # MANUAL はスティックが走行操作そのものなので、jog 中は PAUSE でも走行中。
    assert is_driving("MANUAL", "PAUSE", attrs, jog_active=False) is False
    assert is_driving("MANUAL", "PAUSE", attrs, jog_active=True) is True
    assert is_driving("MANUAL", "RUN", attrs, jog_active=False) is True
    # 含めないもの（CALIB の S2/S3・READY・BLOCKED・WAIT_CLEAR・PAUSE 自身）。
    for mode, state in [("CALIB", "S2"), ("CALIB", "S3"), ("REPLAY", "LOCALIZE"),
                        ("REPLAY", "READY"), ("PANEL_NAV", "BLOCKED"),
                        ("SUMMON", "WAIT_CLEAR"), ("FOLLOW", "PAUSE"),
                        ("AT_PANEL", "IDLE_P"), ("IDLE", "NONE")]:
        assert is_driving(mode, state, attrs) is False, \
            f"is_driving({mode}, {state}) が真になった"


@pytest.mark.rule("C-16")
def test_presence_lost_pauses_only_when_driving(state_core_bundle):
    """走行中の sys.presence_lost → PAUSE（W-1 を開く）。PREP/RETURN は PREP/PAUSE。
    走行中でなければ弾く（速度上限 0 だけ。状態は変えない）。"""
    core, _, _, _ = state_core_bundle
    for mode, state, dst in [("FOLLOW", "RUN", "PAUSE"),
                             ("MANUAL", "RUN", "PAUSE"),
                             ("TEACH_FOLLOW", "REC", "PAUSE"),
                             ("REPLAY", "RUN", "PAUSE"),
                             ("LINE", "RUN", "PAUSE"),
                             ("PANEL_NAV", "NAV", "PAUSE"),
                             ("PANEL_NAV", "ALIGN", "PAUSE"),
                             ("SUMMON", "ALIGN", "PAUSE"),
                             ("PREP", "RETURN", "PAUSE")]:
        d = core.step(mode, state, "sys.presence_lost", _mk_ctx())
        assert d.accepted is True, f"{mode}/{state} で C-16 が通らない"
        assert (d.to_mode, d.to_state) == (mode, dst), \
            f"{mode}/{state} の行き先が違う: {d.to_mode}/{d.to_state}"
        assert d.rule_id == "C-16"
        assert any(e.name == "open_window" and e.args.get("id") == "W-1"
                   for e in d.effects), f"{mode}/{state} で W-1 が開かない"
    # ジョグ中の MANUAL/PAUSE も走行中として落とす。
    d = core.step("MANUAL", "PAUSE", "sys.presence_lost",
                  _mk_ctx(flags={"jog_active": True}))
    assert d.accepted is True
    assert (d.to_mode, d.to_state) == ("MANUAL", "PAUSE")
    # 走行中でない状態では弾く。
    for mode, state in [("IDLE", "NONE"), ("REPLAY", "READY"),
                        ("REPLAY", "LOCALIZE"), ("AT_PANEL", "IDLE_P"),
                        ("PANEL_NAV", "BLOCKED"), ("SUMMON", "WAIT_CLEAR"),
                        ("CALIB", "S2"), ("PREP", "MAPPING"),
                        ("FOLLOW", "PAUSE"), ("MANUAL", "PAUSE")]:
        d = core.step(mode, state, "sys.presence_lost", _mk_ctx())
        assert d.accepted is False, \
            f"{mode}/{state} で C-16 が通ってしまった"
        assert (d.to_mode, d.to_state) == (mode, state)


# ============================================================
# 1b-7（SG-B11）: 保存＝記録の確定。SAVED からは記録を再開しない。
# T-TEACH-05（ui.run）/ -05J（jog）/ -05M（jog）はいずれも拒否し、
# SAVED のまま留まる。resume_record は出さない（出ると記録を閉じた
# recorder が無視し、状態だけ REC の「記録中なのに記録していない」になる）。
# ============================================================
@pytest.mark.rule("T-TEACH-05")
@pytest.mark.rule("T-TEACH-05J")
@pytest.mark.rule("T-TEACH-05M")
def test_saved_does_not_resume_recording(state_core_bundle):
    """SAVED での「走行」・スティック操作は拒否され、SAVED のまま留まる。"""
    core, _, _, _ = state_core_bundle
    cases = [
        ("TEACH_FOLLOW", "SAVED", "ui.run", "T-TEACH-05"),
        ("TEACH_FOLLOW", "SAVED", "ui.jog.hold", "T-TEACH-05J"),
        ("TEACH_MANUAL", "SAVED", "ui.jog.hold", "T-TEACH-05M"),
    ]
    for mode, state, event, rule_id in cases:
        d = core.step(mode, state, event, _mk_ctx())
        assert d.accepted is False, \
            f"{mode}/{state} {event} が受理された（保存後に記録が再開する）"
        assert d.rule_id == rule_id, \
            f"{mode}/{state} {event}: {d.rule_id}（{rule_id} で拒否されること）"
        assert d.reject_reason_key == "teach_saved_finalized", \
            f"{mode}/{state} {event}: 理由キーが違う: {d.reject_reason_key!r}"
        assert (d.to_mode, d.to_state) == (mode, state), \
            f"{mode}/{state} {event}: SAVED のまま留まらない: {d.to_mode}/{d.to_state}"
        assert "resume_record" not in [e.name for e in d.effects], \
            f"{mode}/{state} {event}: resume_record が出ている"
        assert list(d.effects) == [], \
            f"{mode}/{state} {event}: effect が出ている: {[e.name for e in d.effects]}"


@pytest.mark.rule("T-TEACH-05J")
def test_saved_jog_does_not_leave_saved_via_common(state_core_bundle):
    """TEACH_FOLLOW/SAVED でのスティックは C-01（jog → PAUSE）に落ちない。
    -05J の override_common が先に拾って拒否すること（変異: override_common を
    消すと C-01 が通り SAVED を抜けてしまう → このテストが赤くなる）。"""
    core, _, _, _ = state_core_bundle
    d = core.step("TEACH_FOLLOW", "SAVED", "ui.jog.hold", _mk_ctx())
    assert d.accepted is False
    assert d.rule_id == "T-TEACH-05J"
    assert (d.to_mode, d.to_state) == ("TEACH_FOLLOW", "SAVED")


@pytest.mark.rule("C-03")
def test_saved_holds_on_recoverable_fault(state_core_bundle):
    """TEACH/SAVED での回復フォルトは PAUSE に落とさない（SAVED のまま）。
    落とすと「はい」で REC に入るが recorder は閉じたままで、記録していない
    のに記録中になる（項目 2）。変異: ガードの除外を消すと赤くなる。"""
    core, _, _, _ = state_core_bundle
    for mode in ("TEACH_FOLLOW", "TEACH_MANUAL"):
        d = core.step(mode, "SAVED", "fault.recoverable",
                      _mk_ctx(fault_active=True, fault_severity="MINOR",
                              fault_type="LIDAR_LOST"))
        assert d.accepted is False, \
            f"{mode}/SAVED で C-03 が通ってしまった"
        assert (d.to_mode, d.to_state) == (mode, "SAVED")
    # 対照: REC・PAUSE では従来どおり PAUSE に落ちる。
    d = core.step("TEACH_FOLLOW", "REC", "fault.recoverable",
                  _mk_ctx(fault_active=True, fault_severity="MINOR",
                          fault_type="LIDAR_LOST"))
    assert d.accepted is True
    assert (d.to_mode, d.to_state) == ("TEACH_FOLLOW", "PAUSE")
    assert d.rule_id == "C-03"


# ============================================================
# 1b-7（SG-B3）: W-4 の答えを IDLE で受ける（T-IDLE-01/-02）。
# 「終了」（C-08）や記録途切れ（T-TEACH-06）は IDLE へ抜けてから問うので、
# 答え（ui.save／ui.discard）は IDLE で受けて finalize_route／discard_route
# を出す。IDLE に留まる（モードは変えない）。
# ============================================================
@pytest.mark.rule("T-IDLE-01")
def test_idle_save_finalizes_route(state_core_bundle):
    """IDLE での ui.save → finalize_route（W-4「はい」。IDLE のまま）。"""
    core, _, _, _ = state_core_bundle
    d = core.step("IDLE", "NONE", "ui.save", _mk_ctx())
    assert d.accepted is True, f'IDLE の ui.save が拒否された: {d.reject_reason_key}'
    assert d.rule_id == "T-IDLE-01"
    assert (d.to_mode, d.to_state) == ("IDLE", "NONE")
    assert [e.name for e in d.effects] == ["finalize_route"]


@pytest.mark.rule("T-IDLE-02")
def test_idle_discard_discards_route(state_core_bundle):
    """IDLE での ui.discard → discard_route（W-4「いいえ」。IDLE のまま）。
    変異: effect を finalize_route に変えると赤くなる（破棄なのに保存する）。"""
    core, _, _, _ = state_core_bundle
    d = core.step("IDLE", "NONE", "ui.discard", _mk_ctx())
    assert d.accepted is True, f'IDLE の ui.discard が拒否された: {d.reject_reason_key}'
    assert d.rule_id == "T-IDLE-02"
    assert (d.to_mode, d.to_state) == ("IDLE", "NONE")
    assert [e.name for e in d.effects] == ["discard_route"]
