"""test_measure_ledger.py — th_state/measure_ledger の純コア試験。ROS 不要。

`/measure/result` を 1 指示につき必ず 1 回だけ出すこと（受理・完了・
取り消し・二重 complete の無視と記録）、すぐ終わる指示（DONE を即出す
もの）の扱い、`/measure/command` の判定 ①（bad_argument）を縛る。
正本は docs/plan/detailed/DetailedDesign-measure.md §3.1・§3.2・§6.1。
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'th_state'))

from th_state.measure_ledger import (IMMEDIATE_MEASURE_COMMANDS,  # noqa: E402
                                     KNOWN_MEASURE_COMMANDS, MeasureLedger,
                                     validate_measure_command)


def test_accept_then_complete_emits_exactly_once():
    led = MeasureLedger()
    assert led.accept("c1", "meas.goal") is True
    assert led.pending_command_id == "c1"
    assert led.pending_command == "meas.goal"
    rec = led.complete("c1", "ARRIVED", "")
    assert rec is not None
    assert (rec.command_id, rec.command, rec.result, rec.reason) == (
        "c1", "meas.goal", "ARRIVED", "")
    assert led.pending_command_id == ""


def test_duplicate_complete_is_ignored_and_recorded():
    led = MeasureLedger()
    led.accept("c1", "meas.turn")
    assert led.complete("c1", "DONE", "") is not None
    assert led.complete("c1", "DONE", "") is None     # 二重は出さない
    ignored = led.ignored
    assert len(ignored) == 1
    assert ignored[0].command_id == "c1"


def test_complete_unknown_id_is_ignored_and_recorded():
    led = MeasureLedger()
    assert led.complete("ghost", "DONE", "") is None
    assert len(led.ignored) == 1
    assert led.ignored[0].command_id == "ghost"


def test_cancel_all_cancels_each_pending_once():
    led = MeasureLedger()
    led.accept("c1", "meas.goal")
    led.accept("c2", "meas.turn")
    out = led.cancel_all("aborted")
    assert [(r.command_id, r.result, r.reason) for r in out] == [
        ("c1", "CANCELLED", "aborted"), ("c2", "CANCELLED", "aborted")]
    assert led.pending_command_id == ""
    # 取り消し済みへの complete は出さない
    assert led.complete("c1", "ARRIVED", "") is None
    assert len(led.ignored) == 1


def test_immediate_commands_are_done_right_away():
    """すぐ終わる指示は受理直後に complete(DONE) で 1 回だけ出す。"""
    led = MeasureLedger()
    for i, cmd in enumerate(sorted(IMMEDIATE_MEASURE_COMMANDS)):
        cid = f"imm-{i}"
        assert led.accept(cid, cmd) is True
        rec = led.complete(cid, "DONE", "")
        assert rec is not None and rec.result == "DONE"
        assert led.complete(cid, "DONE", "") is None
    assert led.pending_command_id == ""


def test_goal_turn_translate_stay_pending_until_complete():
    led = MeasureLedger()
    for cmd in ("meas.goal", "meas.turn", "meas.translate"):
        assert cmd not in IMMEDIATE_MEASURE_COMMANDS
        assert cmd in KNOWN_MEASURE_COMMANDS


def test_reaccept_same_id_is_refused():
    led = MeasureLedger()
    assert led.accept("c1", "meas.goal") is True
    assert led.accept("c1", "meas.goal") is False       # 進行中
    led.complete("c1", "ARRIVED", "")
    assert led.accept("c1", "meas.goal") is False       # 完了済み


def test_validate_bad_argument_cases():
    # command が meas. で始まらない（ui.* を含む）
    assert validate_measure_command("c1", "ui.anything") == "bad_argument"
    assert validate_measure_command("c1", "") == "bad_argument"
    assert validate_measure_command("c1", "foo") == "bad_argument"
    # command_id が空
    assert validate_measure_command("", "meas.goal") == "bad_argument"
    # 未知の meas.*
    assert validate_measure_command("c1", "meas.unknown") == "bad_argument"
    # meas.translate の side が FORWARD／BACKWARD 以外
    assert validate_measure_command("c1", "meas.translate", "LEFT") == "bad_argument"
    assert validate_measure_command("c1", "meas.translate", "RIGHT") == "bad_argument"
    assert validate_measure_command("c1", "meas.translate", "") == "bad_argument"


def test_validate_ok_cases():
    assert validate_measure_command("c1", "meas.goal") == ""
    assert validate_measure_command("c1", "meas.translate", "FORWARD", distance_m=0.2) == ""
    assert validate_measure_command("c1", "meas.translate", "BACKWARD", distance_m=0.2) == ""
    for cmd in sorted(KNOWN_MEASURE_COMMANDS):
        # 8-g P3: turn・translate は引数も要る（angle_rad・distance_m）。有効な値を渡す。
        # 8-g P5c: set_limits は 4 項目が要る（0 は指定なしで通る）。
        assert validate_measure_command(
            "c1", cmd, "FORWARD", angle_rad=0.5, distance_m=0.2,
            v_max_mps=0.0, w_max_radps=0.0, a_max_mps2=0.0,
            alpha_max_radps2=0.0) == "", cmd


def test_validate_set_limits_bad_argument():
    """8-g P5c（§6.6）: 負・有限でないは bad_argument、0 は通る。"""
    import math
    z = dict(v_max_mps=0.0, w_max_radps=0.0, a_max_mps2=0.0, alpha_max_radps2=0.0)
    assert validate_measure_command("c1", "meas.set_limits", **z) == ""
    assert validate_measure_command("c1", "meas.set_limits", **{**z, "v_max_mps": -0.1}) == "bad_argument"
    assert validate_measure_command("c1", "meas.set_limits", **{**z, "w_max_radps": -1.0}) == "bad_argument"
    assert validate_measure_command("c1", "meas.set_limits", **{**z, "a_max_mps2": float("inf")}) == "bad_argument"
    assert validate_measure_command("c1", "meas.set_limits", **{**z, "alpha_max_radps2": float("nan")}) == "bad_argument"
    assert validate_measure_command("c1", "meas.set_limits", **{**z, "v_max_mps": math.nan}) == "bad_argument"
    assert validate_measure_command(
        "c1", "meas.set_limits", v_max_mps=0.1, w_max_radps=0.2,
        a_max_mps2=0.3, alpha_max_radps2=0.4) == ""
