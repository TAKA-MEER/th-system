"""test_transit_entry_guards_1b11.py — 1b-11（SG-B13・SG-B14）のホスト試験。

state_core を直接 step() し、方式選択の前提と地図更新のガードを縛る
（ROS2 不要。brief 1b-11 §試験のホストの行）:

- SG-B13: 経路 0 本で教示再生への遷移が理由付きで拒否される／経路があれば通る。
  リードデバイス未接続で電子リード走行が理由付きで拒否される／接続済みなら通る。
- SG-B14: map_update_available が真のときだけ T-REPLAY-08 が通り、
  偽（OFF または地図無し）のときは map_update_off で拒否される。
"""
import pytest

from th_state import state_core


@pytest.fixture
def core(state_core_bundle):
    return state_core_bundle[0]


def _ctx(**over):
    kwargs = dict(
        prev_mode="IDLE", prev_state="NONE", prev_sub="",
        flags={}, zone="OUT", candidate_count=0,
        target_selected=False, target_confident=False,
        fault_active=False, fault_severity="", fault_type="",
        hw_estop=False, ui_estop=False,
        route_ids=(), pin_kinds=(), leash_present=False, leash_taut=False,
        line_visible=False, camera_present=False,
        check_item="", check_result="", calib_item="", calib_preview_sane=False,
        map_update_available=False, now_ms=0, arg={},
    )
    kwargs.update(over)
    return state_core.Context(**kwargs)


# ── SG-B13: 教示再生は経路 0 本で押せない ──────────────────────────
def test_enter_replay_rejected_without_routes(core):
    """経路 0 本 → no_route_recorded で拒否（C-13-replay-no-route）。"""
    d = core.step("IDLE", "NONE", "ui.enter_mode",
                  _ctx(route_ids=(), arg={"mode": "REPLAY"}))
    assert not d.accepted
    assert d.reject_reason_key == "no_route_recorded"
    assert d.rule_id == "C-13-replay-no-route"


def test_enter_replay_accepted_with_routes(core):
    """経路があれば通る（C-13。ガードの入れすぎ検出）。"""
    d = core.step("IDLE", "NONE", "ui.enter_mode",
                  _ctx(route_ids=("R1",), arg={"mode": "REPLAY"}))
    assert d.accepted, d.reject_reason_key
    assert (d.to_mode, d.to_state) == ("REPLAY", "ROUTE_SEL")


def test_enter_other_modes_unaffected_by_route_gate(core):
    """経路 0 本でも手動走行は通る（拒否行の標的違い検出）。"""
    d = core.step("IDLE", "NONE", "ui.enter_mode",
                  _ctx(route_ids=(), arg={"mode": "MANUAL"}))
    assert d.accepted, d.reject_reason_key


# ── SG-B13: 電子リードはデバイス未接続で押せない ───────────────────
def test_enter_leash_rejected_without_device(core):
    """leash_present 偽 → device_not_connected で拒否（C-13-leash-no-device）。"""
    d = core.step("IDLE", "NONE", "ui.enter_mode",
                  _ctx(leash_present=False, arg={"mode": "LEASH"}))
    assert not d.accepted
    assert d.reject_reason_key == "device_not_connected"
    assert d.rule_id == "C-13-leash-no-device"


def test_enter_leash_accepted_with_device(core):
    """接続済みなら通る（ガードの入れすぎ検出）。"""
    d = core.step("IDLE", "NONE", "ui.enter_mode",
                  _ctx(leash_present=True, arg={"mode": "LEASH"}))
    assert d.accepted, d.reject_reason_key
    assert (d.to_mode, d.to_state) == ("LEASH", "DEV_CHECK")


# ── SG-B14: T-REPLAY-08 は map_update_available が真のときだけ通る ──
def test_replay_save_accepted_when_map_update_available(core):
    """フラグ ON＋地図あり → SAVED へ（T-REPLAY-08。commit_map_patch を出す）。"""
    d = core.step("REPLAY", "PAUSE", "ui.save",
                  _ctx(flags={"map_update": True}, map_update_available=True))
    assert d.accepted, d.reject_reason_key
    assert d.rule_id == "T-REPLAY-08"
    assert (d.to_mode, d.to_state) == ("REPLAY", "SAVED")
    assert [e.name for e in d.effects] == ["commit_map_patch"]


def test_replay_save_rejected_when_flag_off(core):
    """フラグ OFF → map_update_off で拒否（T-REPLAY-08X）。"""
    d = core.step("REPLAY", "PAUSE", "ui.save",
                  _ctx(flags={"map_update": False}, map_update_available=True))
    assert not d.accepted
    assert d.reject_reason_key == "map_update_off"
    assert d.rule_id == "T-REPLAY-08X"


def test_replay_save_rejected_when_no_map(core):
    """フラグ ON でも地図無し → map_update_off で拒否（T-REPLAY-08X）。"""
    d = core.step("REPLAY", "PAUSE", "ui.save",
                  _ctx(flags={"map_update": True}, map_update_available=False))
    assert not d.accepted
    assert d.reject_reason_key == "map_update_off"


# ── SG-B19: 地図・経路が読めないときは原因どおりの案内 ─────────────
def test_route_no_map_guides_with_cause(core):
    """地図の無い経路 → LOCALIZE のまま route_no_map の案内（T-REPLAY-12）。"""
    d = core.step("REPLAY", "LOCALIZE", "evt.route_no_map", _ctx())
    assert d.accepted
    assert d.rule_id == "T-REPLAY-12"
    assert (d.to_mode, d.to_state) == ("REPLAY", "LOCALIZE")
    assert [(e.name, e.args.get("key")) for e in d.effects] == [
        ("guide", "route_no_map")]


def test_route_unreadable_guides_with_cause(core):
    """読めない経路ファイル → LOCALIZE のまま route_unreadable の案内（T-REPLAY-13）。"""
    d = core.step("REPLAY", "LOCALIZE", "evt.route_unreadable", _ctx())
    assert d.accepted
    assert d.rule_id == "T-REPLAY-13"
    assert [(e.name, e.args.get("key")) for e in d.effects] == [
        ("guide", "route_unreadable")]
