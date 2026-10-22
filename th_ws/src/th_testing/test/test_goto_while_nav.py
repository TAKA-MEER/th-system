"""SM-3.1.2-118 — 移動中・一時停止中（PANEL_NAV / SUMMON / HOME_NAV の全状態）は
行き先を選べない。遷移表の拒否行 T-NAV-goto-reject。ROS 不要。

行き先を選べる状態（AT_HOME の IDLE_H / PAUSE・AT_PANEL の IDLE_P・IDLE）が
巻き込まれていないことも縛る。"""
import pytest

from th_state import state_core

from test_working_guard import _ctx

_NAV_MODES = ("PANEL_NAV", "SUMMON", "HOME_NAV")


@pytest.mark.parametrize("kind", ["PANEL", "HOME", "SUMMON"])
@pytest.mark.parametrize("mode", _NAV_MODES)
def test_goto_rejected_in_every_state_including_pause(mode, kind, state_core_bundle):
    core = state_core_bundle[0]
    states = sorted(state_core.MODE_STATES[mode])
    assert "PAUSE" in states
    for st in states:
        d = core.step(mode, st, "ui.goto", _ctx(kind))
        assert d.accepted is False, (mode, st, kind)
        assert d.reject_reason_key == "goto_while_nav", (mode, st, kind)
        # 状態は変えない。
        assert (d.to_mode, d.to_state) == (mode, st)
        assert d.effects == ()


@pytest.mark.parametrize("kind,mode", [("PANEL", "PANEL_NAV"), ("HOME", "HOME_NAV"),
                                       ("SUMMON", "SUMMON")])
@pytest.mark.parametrize("src", [("AT_HOME", "IDLE_H"), ("AT_HOME", "PAUSE"),
                                 ("AT_PANEL", "IDLE_P"), ("IDLE", "NONE")])
def test_goto_still_accepted_from_choosable_states(src, kind, mode, state_core_bundle):
    core = state_core_bundle[0]
    d = core.step(src[0], src[1], "ui.goto", _ctx(kind))
    assert d.accepted is True, (src, kind)
    assert d.to_mode == mode


def test_abort_and_run_still_work_while_paused(state_core_bundle):
    """一時停止中の逃げ道（中断・走行）は残る。"""
    core = state_core_bundle[0]
    d = core.step("PANEL_NAV", "PAUSE", "ui.abort", _ctx("PANEL"))
    assert d.accepted is True and d.to_mode == "AT_HOME" and d.to_state == "IDLE_H"
    d = core.step("PANEL_NAV", "PAUSE", "ui.run", _ctx("PANEL"))
    assert d.accepted is True
