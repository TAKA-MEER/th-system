"""test_replay_route_selection.py — 「再生を押しても機体が動かない」の回帰試験。

2026-09-02 実機で再現した不具合:

  共通遷移 C-01（ジョグ介入）と C-03（回復可能フォルト）は `mode=* state=*` で
  `PAUSE` へ落とす。そのため経路をまだ積んでいない準備状態
  （`REPLAY` の `ROUTE_SEL` / `LOCALIZE` / `READY`）からも `PAUSE` に入る。
  `PAUSE` からの `ui.run`（`T-REPLAY-07`）は「経路は既に積んである」前提で
  `resume_path` を出すだけなので、**経路選択が丸ごと飛ばされ、再生は受理される
  のに追従する対象が無い**状態になる。

  実機再現手順: `REPLAY` に入る（`ROUTE_SEL`）→ ジョグを握る → `PAUSE` →
  再生が受理され `RUN` へ。`load_route` は一度も発火しない。
  起点合わせでジョグを触る／`ESP32_DISCONNECTED` が一度出るだけで発生した。

対策（VISION.md §2.5「実機フィードバックによる設計変更」2026-09-02）:
  A. `C-01` / `C-03` の `to_state` を `$pause_unless_prep` にし、
     `attributes.yaml` の `prep_states` に挙げた状態では `PAUSE` へ落とさない。
  B. `T-REPLAY-07` / `T-REPLAY-10` に `route_loaded` ガードを課し、
     `T-REPLAY-11`（`PAUSE` から経路を選び直す）で復帰路を用意する。

**安全は緩めていない**ことをここで縛る: 走行中（`RUN`）のフォルトは従来どおり
`PAUSE` に落ちるし、`prep_states` を宣言していないモードの挙動は不変。
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


# ── A: 準備状態は PAUSE へ落とさない ────────────────────────────
@pytest.mark.parametrize("prep", ["ROUTE_SEL", "LOCALIZE", "READY"])
def test_jog_does_not_drop_replay_prep_states_to_pause(core, prep):
    """起点合わせのためにジョグを握っても、経路選択の進捗を失わない。"""
    d = core.step("REPLAY", prep, "ui.jog.hold", _ctx())
    assert d.accepted
    assert d.to_state == prep, f"{prep} からジョグで {d.to_state} に飛んだ"


@pytest.mark.parametrize("prep", ["ROUTE_SEL", "LOCALIZE", "READY"])
def test_recoverable_fault_does_not_drop_replay_prep_states_to_pause(core, prep):
    """ESP32_DISCONNECTED 等が一度出ただけで経路選択が飛ばない。"""
    d = core.step("REPLAY", prep, "fault.recoverable",
                  _ctx(fault_active=True, fault_severity="RECOVERABLE",
                       fault_type="ESP32_DISCONNECTED"))
    assert d.accepted
    assert d.to_state == prep, f"{prep} からフォルトで {d.to_state} に飛んだ"


# ── A の裏: 安全側の挙動は変えていない ──────────────────────────
def test_recoverable_fault_still_pauses_a_running_replay(core):
    """走行中のフォルトは従来どおり PAUSE。ここを緩めたら安全上の後退。"""
    d = core.step("REPLAY", "RUN", "fault.recoverable",
                  _ctx(fault_active=True, fault_severity="RECOVERABLE",
                       fault_type="ESP32_DISCONNECTED"))
    assert d.accepted
    assert d.to_state == "PAUSE"


def test_modes_without_prep_states_are_unchanged(core):
    """prep_states を宣言していないモードは従来どおり PAUSE（影響範囲の封じ込め）。"""
    d = core.step("FOLLOW", "SELECT", "fault.recoverable",
                  _ctx(fault_active=True, fault_severity="RECOVERABLE",
                       fault_type="ESP32_DISCONNECTED"))
    assert d.accepted
    assert d.to_state == "PAUSE"


# ── B: 経路未読込のまま再生させない ─────────────────────────────
def test_run_from_pause_is_rejected_without_a_loaded_route(core):
    """これが「再生は受理されたのに動かない」の本体。空振りを拒否する。"""
    d = core.step("REPLAY", "PAUSE", "ui.run", _ctx(route_loaded=False))
    assert not d.accepted, "経路未読込なのに再生が受理された"


def test_run_from_pause_is_accepted_once_a_route_is_loaded(core):
    """正常系まで塞いでいないこと（ガードの入れすぎ検出）。"""
    d = core.step("REPLAY", "PAUSE", "ui.run", _ctx(route_loaded=True))
    assert d.accepted
    assert d.to_state == "RUN"
    assert [e.name for e in d.effects] == ["resume_path"]


def test_run_from_saved_is_also_guarded(core):
    assert not core.step("REPLAY", "SAVED", "ui.run", _ctx(route_loaded=False)).accepted
    assert core.step("REPLAY", "SAVED", "ui.run", _ctx(route_loaded=True)).accepted


# ── B: 詰まったときの復帰路 ─────────────────────────────────────
def test_route_can_be_reselected_from_pause(core):
    """PAUSE に落ちても経路を選び直せる（落ちたまま詰まない）。"""
    d = core.step("REPLAY", "PAUSE", "ui.route_select",
                  _ctx(route_ids=("R1",), arg={"id": "R1", "reverse": False}))
    assert d.accepted
    assert d.to_state == "LOCALIZE"
    assert [e.name for e in d.effects] == ["load_route"], \
        "経路を選び直したのに load_route が出ない"
    assert d.effects[0].args["route_id"] == "R1"


# ── 実機で踏んだ手順そのもの ────────────────────────────────────
def test_full_repro_sequence_no_longer_skips_route_loading(core):
    """実機の再現手順を通しで流し、経路選択を飛ばせないことを確かめる。"""
    # 1) REPLAY に入る → ROUTE_SEL
    assert core.initial_state("REPLAY") == "ROUTE_SEL"
    # 2) ROUTE_SEL のまま再生 → 拒否（元から正しい）
    assert not core.step("REPLAY", "ROUTE_SEL", "ui.run", _ctx()).accepted
    # 3) ジョグを握る → 修正前は PAUSE。今は ROUTE_SEL のまま
    d = core.step("REPLAY", "ROUTE_SEL", "ui.jog.hold", _ctx())
    assert d.to_state == "ROUTE_SEL"
    # 4) もう一度再生 → ROUTE_SEL なので拒否されたまま（経路を選ぶまで走らない）
    assert not core.step("REPLAY", d.to_state, "ui.run", _ctx()).accepted
    # 5) 経路を選ぶ → ここで初めて load_route が出る
    d = core.step("REPLAY", "ROUTE_SEL", "ui.route_select",
                  _ctx(route_ids=("R1",), arg={"id": "R1", "reverse": False}))
    assert d.accepted and [e.name for e in d.effects] == ["load_route"]
