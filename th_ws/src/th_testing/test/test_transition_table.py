"""test_transition_table.py — transitions.yaml のトレーサビリティ試験（WP-STATE-01）

DetailedDesign-state.md §11 の要件 1・2・2r・2f・3 に対応する。
th_state/th_state/state_core.py は rclpy に依存しないので、この試験も ROS2 を必要としない。
"""
import re
from collections import Counter

import pytest

import conftest
from th_state import state_core


# ============================================================
# 準備: transitions.yaml をモジュールレベルで読み込む（parametrize に使うため）
# ============================================================
_CONFIG_DIR = conftest.th_state_config_dir()

import yaml as _yaml

with open(_CONFIG_DIR / "transitions.yaml", encoding="utf-8") as _f:
    _TRANSITIONS = _yaml.safe_load(_f)
with open(_CONFIG_DIR / "mode_entry.yaml", encoding="utf-8") as _f:
    _MODE_ENTRY = _yaml.safe_load(_f)
with open(_CONFIG_DIR / "attributes.yaml", encoding="utf-8") as _f:
    _ATTRIBUTES = _yaml.safe_load(_f)

_SPEC_ID_RE = re.compile(r"SM-3\.1\.[12]-\d+")

# DetailedDesign-state.md §4.4.3 — 1つの正本IDに複数の詳細行が対応する組（これが行数差の全部）。
# SM-3.1.1-11 は 2026-09-01 の UI 非常停止復帰変更で 1 → 4 行に分割（C-09 / -09b / -09c / -09d。
# Spec-modes.md §3.1.1 SM-3.1.1-11 の 1 行が「解除／戻る／メニューへ」を包含する）。
SPEC_FANOUT = {
    "SM-3.1.1-10": 2, "SM-3.1.1-11": 4, "SM-3.1.2-004": 2, "SM-3.1.2-038": 2,
    "SM-3.1.2-043": 2, "SM-3.1.2-069": 3, "SM-3.1.2-094": 4,
}


def _load_spec_ids(spec_path):
    text = spec_path.read_text(encoding="utf-8")
    return set(_SPEC_ID_RE.findall(text))


# ============================================================
# §11-1: 全行に spec_ref がある
# ============================================================
def test_all_rows_have_spec_ref():
    missing = [row["id"] for row in _TRANSITIONS if not row.get("spec_ref")]
    assert not missing, f"spec_ref が無い行: {missing}"


# ============================================================
# §11-2 / §11-2r / §11-2f: 正本との突き合わせ
# ============================================================
def test_spec_rows_covered(spec_modes_file):
    """Spec-modes.md §3.1.1 の17行・§3.1.2 の102行すべてに、対応する行が1つ以上ある。
    欠けたら失敗する（行数は主張せず SM- の ID を機械的に数える）。"""
    spec_ids = _load_spec_ids(spec_modes_file)
    covered = {row["spec_ref"] for row in _TRANSITIONS}
    missing = spec_ids - covered
    assert not missing, f"transitions.yaml に無い正本ID: {sorted(missing)}"


def test_no_extra_rows(spec_modes_file):
    """逆方向: 全行の spec_ref が指す正本の ID が実在する（過剰行の検出）。"""
    spec_ids = _load_spec_ids(spec_modes_file)
    used = {row["spec_ref"] for row in _TRANSITIONS}
    extra = used - spec_ids
    assert not extra, f"正本に無い spec_ref: {sorted(extra)}"


def test_fanout_is_exactly_the_declared_six():
    """1正本IDに複数行が対応するのは §4.4.3 の6組だけ（分割の検出）。
    test_spec_rows_covered と test_no_extra_rows だけでは
    「1行を2行に割った」ことを検出できないため、この3本目が要る。"""
    spec_refs = [row["spec_ref"] for row in _TRANSITIONS]
    fanout = {k: v for k, v in Counter(spec_refs).items() if v > 1}
    assert fanout == SPEC_FANOUT, f"宣言されていない fan-out: {fanout}"


# ============================================================
# §11-3: 全行に対応する pytest がある
# ============================================================
def test_every_rule_has_a_test(request):
    """@pytest.mark.rule(<id>) で申告されたテストを収集し、transitions.yaml の id 集合と
    突き合わせる。全 128 行がどこかのテストでカバーされていることを検査する。"""
    declared_ids = {row["id"] for row in _TRANSITIONS}
    covered_ids = set()
    for item in request.session.items:
        for mark in item.iter_markers(name="rule"):
            covered_ids.update(mark.args)
    missing = declared_ids - covered_ids
    assert not missing, f"pytest.mark.rule で申告されていない遷移: {sorted(missing)}"


# ============================================================
# 全行の一般的な発火可能性（§11-3 のマーカー収集対象。他の必須8試験がカバーしない
# 残りの行を機械的に埋める）。
#
# 各ガードを真にする最小限の ctx 上書き（Context フィールドのみ・時刻やファイルは読まない）。
# mode_entry_allowed は mode_entry.yaml の表を引く必要があるため個別に扱う。
# ============================================================
_STATIC_GUARD_OVERRIDES = {
    "jog_allowed": {},
    "fault_stops_mode": {"fault_type": ""},
    "fault_cleared": {"fault_active": False},
    "fault_cleared_and_ui_released": {"fault_active": False, "ui_estop": False, "hw_estop": False},
    "estop_ui_allowed": {},
    "not_checking_estop": {"check_item": ""},
    "checking_estop_item": {"check_item": "ESTOP"},
    "no_critical_fault": {"fault_severity": ""},
    "hw_released": {"hw_estop": False},
    "hw_released_and_no_critical": {"hw_estop": False, "fault_severity": ""},
    "leash_slack": {"leash_taut": False},
    "can_finish": {},
    "goto_allowed": {"flags": {"working": False}, "arg": {"kind": "HOME"}},
    "candidate_exists": {"candidate_count": 1},
    "target_selected": {"target_selected": True},
    "target_confident": {"target_selected": True, "target_confident": True},
    "route_arg_valid": {"arg": {"new": True}},
    "route_exists": {"route_ids": ("R1",), "arg": {"id": "R1"}},
    "route_loaded": {"route_loaded": True},
    "home_pin_exists": {"pin_kinds": ("HOME",)},
    "map_update": {"flags": {"map_update": True}, "map_update_available": True},
    "line_visible": {"line_visible": True},
    "leash_taut": {"leash_taut": True},
    "preview_sane": {"calib_preview_sane": True},
    "check_result_ok": {"check_result": "OK"},
    "ng_and_calibrable": {"check_result": "NG", "check_item": "IMU"},
    "ng_and_not_calibrable": {"check_result": "NG", "check_item": "MOTOR"},
    "estop_resume_prev": {"estop_from_ui": True, "fault_active": False,
                          "fault_severity": "", "hw_estop": False, "prev_mode": "MANUAL"},
}
assert set(_STATIC_GUARD_OVERRIDES) | {"mode_entry_allowed"} == set(
    __import__("th_state.guards", fromlist=["GUARDS"]).GUARDS)


def _base_ctx_kwargs():
    return dict(
        prev_mode="FOLLOW", prev_state="RUN", prev_sub="MAPPING",
        flags={}, zone="OUT", candidate_count=0,
        target_selected=False, target_confident=False,
        fault_active=False, fault_severity="", fault_type="",
        hw_estop=False, ui_estop=False,
        route_ids=(), pin_kinds=(), leash_present=False, leash_taut=False,
        line_visible=False, camera_present=False,
        check_item="", check_result="", calib_item="", calib_preview_sane=False,
        map_update_available=False, now_ms=0, arg={},
    )


def _concrete_mode(row):
    m = row["mode"]
    if isinstance(m, list):
        return m[0]
    if m == "*":
        return "FOLLOW"
    return m


def _concrete_state(row, mode):
    s = row["state"]
    if isinstance(s, list):
        candidates = s
    elif s == "*":
        candidates = sorted(state_core.MODE_STATES[mode])
    else:
        candidates = [s]
    valid = [c for c in candidates if c in state_core.MODE_STATES[mode]]
    return sorted(valid)[0] if valid else sorted(state_core.MODE_STATES[mode])[0]


def _concrete_event(row):
    e = row["event"]
    return e[0] if isinstance(e, list) else e


def _scenario_for(row):
    """row を発火させる (mode, state, event, ctx) を組み立てる（best-effort）。"""
    mode = _concrete_mode(row)
    state = _concrete_state(row, mode)
    event = _concrete_event(row)

    kwargs = _base_ctx_kwargs()
    guard = row.get("guard")
    if guard == "mode_entry_allowed":
        targets = _MODE_ENTRY.get(mode, [])
        kwargs["arg"] = {"mode": targets[0]} if targets else {}
    elif guard is not None:
        override = _STATIC_GUARD_OVERRIDES.get(guard, {})
        for key, val in override.items():
            if isinstance(val, dict) and isinstance(kwargs.get(key), dict):
                merged = dict(kwargs[key])
                merged.update(val)
                kwargs[key] = merged
            else:
                kwargs[key] = val
    return mode, state, event, state_core.Context(**kwargs)


def _row_test_id(row):
    return row["id"]


# 静的（収集時）に @pytest.mark.rule(<id>) を付与する。テスト本体の実行中に
# add_marker するのではなく collection time に付与しないと、同じセッション内の
# 他のテスト（test_every_rule_has_a_test）がまだ実行されていない項目のマークを
# 見られない（pytest はファイル内で定義順に実行するため）。
_ROW_PARAMS = [pytest.param(row, id=row["id"], marks=pytest.mark.rule(row["id"]))
               for row in _TRANSITIONS]


@pytest.mark.parametrize("row", _ROW_PARAMS)
def test_row_transition_fires(row, state_core_bundle):
    """§11-3 のマーカー収集対象。ガードを満たす ctx を組み立てて、この行が実際に
    accepted な遷移（または reject: true 行なら明示的な拒否）を返すことを確認する。

    注意: T-ATP-03 のように、より優先度の高い共通行（C-01）と全く同じ結果になる
    ため事実上その共通行に隠れる行が1つだけ存在する（AT_PANEL の設計書 §4.2 の
    記載どおり）。rule_id の厳密一致までは要求せず、accepted / reject の可否と
    reject_reason_key を検査する。"""
    core, _, _, _ = state_core_bundle
    mode, state, event, ctx = _scenario_for(row)
    decision = core.step(mode, state, event, ctx)

    if row.get("reject"):
        assert decision.accepted is False, (
            f"{row['id']}: reject:true の行なのに accepted=True "
            f"(mode={mode}, state={state}, event={event})")
        assert decision.reject_reason_key == row["reject_reason_key"]
        return

    assert decision.accepted is True, (
        f"{row['id']}: 期待した遷移が採用されなかった "
        f"(mode={mode}, state={state}, event={event}, "
        f"reject_reason_key={decision.reject_reason_key!r})")
    assert "$" not in decision.to_mode
    assert "$" not in decision.to_state
