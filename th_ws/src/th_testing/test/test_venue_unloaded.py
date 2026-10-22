"""SM-3.1.2-119 — 会場地図を持たない間は自律移動を始めない・再開しない。
遷移表の拒否行 T-VUL-goto / T-VUL-run。ROS 不要。

塞ぐのは「行き先を選ぶ」（AT_HOME・AT_PANEL）と「走行／はい／再検索」で再開する
（PANEL_NAV・SUMMON・HOME_NAV の PAUSE・BLOCKED）だけ。手動走行・非常停止・中断は残る。
未受信・LOCALIZING・MAPPING（ctx.venue_unloaded=False）は何も塞がない。"""
import dataclasses

import pytest

from test_working_guard import _ctx

_NAV_MODES = ("PANEL_NAV", "SUMMON", "HOME_NAV")


def _unloaded(kind="PANEL"):
    return dataclasses.replace(_ctx(kind), venue_unloaded=True)


@pytest.mark.parametrize("kind", ["PANEL", "HOME", "SUMMON"])
@pytest.mark.parametrize("src", [("AT_HOME", "IDLE_H"), ("AT_HOME", "PAUSE"),
                                 ("AT_PANEL", "IDLE_P"), ("AT_PANEL", "WORKING")])
def test_goto_rejected_without_venue_map(src, kind, state_core_bundle):
    core = state_core_bundle[0]
    d = core.step(src[0], src[1], "ui.goto", _unloaded(kind))
    assert d.accepted is False, (src, kind)
    assert d.reject_reason_key == "venue_not_loaded"
    assert (d.to_mode, d.to_state) == src
    assert d.effects == ()


@pytest.mark.parametrize("event", ["ui.run", "ui.resume_yes"])
@pytest.mark.parametrize("mode", _NAV_MODES)
def test_resume_rejected_without_venue_map(mode, event, state_core_bundle):
    core = state_core_bundle[0]
    d = core.step(mode, "PAUSE", event, _unloaded())
    assert d.accepted is False, (mode, event)
    assert d.reject_reason_key == "venue_not_loaded"
    assert (d.to_mode, d.to_state) == (mode, "PAUSE")
    assert d.effects == ()


@pytest.mark.parametrize("mode", _NAV_MODES)
def test_reroute_rejected_without_venue_map(mode, state_core_bundle):
    core = state_core_bundle[0]
    d = core.step(mode, "BLOCKED", "ui.reroute", _unloaded())
    assert d.accepted is False
    assert d.reject_reason_key == "venue_not_loaded"


@pytest.mark.parametrize("kind,mode", [("PANEL", "PANEL_NAV"), ("HOME", "HOME_NAV"),
                                       ("SUMMON", "SUMMON")])
@pytest.mark.parametrize("src", [("AT_HOME", "IDLE_H"), ("AT_PANEL", "IDLE_P")])
def test_goto_still_accepted_with_venue_map_or_unknown(src, kind, mode, state_core_bundle):
    """venue_unloaded が偽（読み込み済み・作成中・未受信）なら従来どおり通る。"""
    core = state_core_bundle[0]
    d = core.step(src[0], src[1], "ui.goto", _ctx(kind))
    assert d.accepted is True, (src, kind)
    assert d.to_mode == mode


@pytest.mark.parametrize("mode", _NAV_MODES)
def test_run_still_accepted_with_venue_map(mode, state_core_bundle):
    core = state_core_bundle[0]
    d = core.step(mode, "PAUSE", "ui.run", _ctx("PANEL"))
    assert d.accepted is True and d.to_state == "NAV"


@pytest.mark.parametrize("mode", _NAV_MODES)
def test_abort_and_stop_still_work_without_venue_map(mode, state_core_bundle):
    """中断・停止は会場地図が無くても使える（待機場所まで手で戻すのに要る）。"""
    core = state_core_bundle[0]
    d = core.step(mode, "PAUSE", "ui.abort", _unloaded())
    assert d.accepted is True and d.to_mode == "AT_HOME" and d.to_state == "IDLE_H"
    d = core.step(mode, "NAV", "ui.stop", _unloaded())
    assert d.accepted is True and d.to_state == "PAUSE"


def test_venue_unloaded_does_not_touch_other_modes(state_core_bundle):
    """対象外のモード（IDLE の行き先・前日 PREP）は巻き込まない。"""
    core = state_core_bundle[0]
    d = core.step("IDLE", "NONE", "ui.goto", _unloaded("PANEL"))
    assert d.accepted is True
    d = core.step("PREP", "RETURN", "ui.stop", _unloaded())
    assert d.accepted is True
