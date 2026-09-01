"""guards.py — 状態遷移のガード述語（27件）

DetailedDesign-state.md §4.1.1 が正。ガードは (mode, state, ctx) の 3 引数を取る
純粋関数で、Context のフィールドと mode / state だけを読む（S-4）。
時刻・ファイル・環境変数は読まない。

rclpy には依存しない（WP-STATE-01 の対象。state_core.py と同じ制約）。
"""
from typing import Callable, Dict

# モードによって PAUSE を持たない集合（§4.1.1 末尾。state_core.py の同名集合と同じ定義）。
_NO_PAUSE_MODES = {"INIT", "IDLE", "ESTOP", "CARRY", "OPCHECK", "CALIB"}

# C-01 (jog_allowed) の除外表（Spec-modes.md §3.1.1 ジョグ介入の除外）。
_JOG_EXCLUDED_MODES = {"INIT", "IDLE", "ESTOP", "CARRY", "OPCHECK", "CALIB",
                        "MANUAL", "TEACH_MANUAL"}

# C-03 (fault_stops_mode) が PERSON_TRACKER_LOST を PAUSE に落とすモード集合。
_TRACKER_FAULT_STOPS_MODES = {"FOLLOW", "TEACH_FOLLOW", "SUMMON"}

# ng_and_calibrable / ng_and_not_calibrable の項目分類（F-16）。
_CALIBRABLE_ITEMS = {"IMU", "LIDAR"}
_NOT_CALIBRABLE_ITEMS = {"ESTOP", "MOTOR"}


def _jog_allowed(mode, state, ctx) -> bool:
    if mode in _JOG_EXCLUDED_MODES:
        return False
    if mode == "SUMMON" and state == "WAIT_CLEAR":
        return False
    return True


def _fault_stops_mode(mode, state, ctx) -> bool:
    if mode in _NO_PAUSE_MODES:
        return False
    if ctx.fault_type == "PERSON_TRACKER_LOST":
        if mode not in _TRACKER_FAULT_STOPS_MODES:
            return False
        if mode == "PREP":
            return False
    return True


def _fault_cleared(mode, state, ctx) -> bool:
    return not ctx.fault_active


def _fault_cleared_and_ui_released(mode, state, ctx) -> bool:
    return not ctx.fault_active and not ctx.ui_estop and not ctx.hw_estop


def _estop_ui_allowed(mode, state, ctx) -> bool:
    return mode != "CARRY"


def _not_checking_estop(mode, state, ctx) -> bool:
    return not (mode == "OPCHECK" and ctx.check_item == "ESTOP")


def _checking_estop_item(mode, state, ctx) -> bool:
    return mode == "OPCHECK" and ctx.check_item == "ESTOP"


def _no_critical_fault(mode, state, ctx) -> bool:
    return ctx.fault_severity != "CRITICAL"


def _hw_released(mode, state, ctx) -> bool:
    return not ctx.hw_estop


def _hw_released_and_no_critical(mode, state, ctx) -> bool:
    return not ctx.hw_estop and ctx.fault_severity != "CRITICAL"


# UI ボタン起因の ESTOP からの復帰先が「押下前のモード」になれる条件
# （2026-09-01 変更。Spec-modes.md §3.1.1 SM-3.1.1-11 / -11b）。
# 重大フォルトを伴わず、物理ボタンも押されておらず、押下前が動作系モードのとき。
_ESTOP_PREV_NONRESUMABLE = {"", "INIT", "IDLE", "ESTOP", "CARRY"}


def _estop_resume_prev(mode, state, ctx) -> bool:
    return (ctx.estop_from_ui
            and not ctx.fault_active
            and ctx.fault_severity != "CRITICAL"
            and not ctx.hw_estop
            and ctx.prev_mode not in _ESTOP_PREV_NONRESUMABLE)


def _leash_slack(mode, state, ctx) -> bool:
    return not ctx.leash_taut


def _can_finish(mode, state, ctx) -> bool:
    if mode == "ESTOP":
        return False
    if mode == "CARRY":
        return not ctx.hw_estop
    return True


def _mode_entry_allowed(mode, state, ctx) -> bool:
    """mode_entry.yaml の表と前提条件を見る。前提条件（DetailedDesign-transit.md §0.4）は
    WP-STATE-01 の対象外（§2 に読む節として挙げられていない）なので、ここでは表のみを見る。
    実際の許可表参照は StateCore 側が mode_entry.yaml を注入する形にする。"""
    raise NotImplementedError(
        "mode_entry_allowed は StateCore が mode_entry.yaml を束縛してから使う"
        "（guards.build_guards() を参照）")


def _goto_allowed(mode, state, ctx) -> bool:
    if ctx.flags.get("working"):
        return False
    if ctx.arg.get("kind") == "PANEL":
        return "PANEL" in ctx.pin_kinds
    return True


def _candidate_exists(mode, state, ctx) -> bool:
    return ctx.candidate_count > 0


def _target_selected(mode, state, ctx) -> bool:
    return ctx.target_selected


def _target_confident(mode, state, ctx) -> bool:
    return ctx.target_selected and ctx.target_confident


def _route_arg_valid(mode, state, ctx) -> bool:
    if ctx.arg.get("new") is True:
        return True
    return ctx.arg.get("id") in ctx.route_ids


def _route_exists(mode, state, ctx) -> bool:
    return ctx.arg.get("id") in ctx.route_ids


def _home_pin_exists(mode, state, ctx) -> bool:
    return "HOME" in ctx.pin_kinds


def _map_update(mode, state, ctx) -> bool:
    return bool(ctx.flags.get("map_update")) and ctx.map_update_available


def _line_visible(mode, state, ctx) -> bool:
    return ctx.line_visible


def _leash_taut(mode, state, ctx) -> bool:
    return ctx.leash_taut


def _preview_sane(mode, state, ctx) -> bool:
    return ctx.calib_preview_sane


def _check_result_ok(mode, state, ctx) -> bool:
    return ctx.check_result == "OK"


def _ng_and_calibrable(mode, state, ctx) -> bool:
    return ctx.check_result == "NG" and ctx.check_item in _CALIBRABLE_ITEMS


def _ng_and_not_calibrable(mode, state, ctx) -> bool:
    return ctx.check_result == "NG" and ctx.check_item in _NOT_CALIBRABLE_ITEMS


# 28 件。§4.1.1 のとおり（`not_working` 削除で 27 → `estop_resume_prev` 追加で 28）。
GUARDS: Dict[str, Callable] = {
    "jog_allowed": _jog_allowed,
    "fault_stops_mode": _fault_stops_mode,
    "fault_cleared": _fault_cleared,
    "fault_cleared_and_ui_released": _fault_cleared_and_ui_released,
    "estop_ui_allowed": _estop_ui_allowed,
    "not_checking_estop": _not_checking_estop,
    "checking_estop_item": _checking_estop_item,
    "no_critical_fault": _no_critical_fault,
    "hw_released": _hw_released,
    "hw_released_and_no_critical": _hw_released_and_no_critical,
    "leash_slack": _leash_slack,
    "can_finish": _can_finish,
    "mode_entry_allowed": _mode_entry_allowed,
    "goto_allowed": _goto_allowed,
    "candidate_exists": _candidate_exists,
    "target_selected": _target_selected,
    "target_confident": _target_confident,
    "route_arg_valid": _route_arg_valid,
    "route_exists": _route_exists,
    "home_pin_exists": _home_pin_exists,
    "map_update": _map_update,
    "line_visible": _line_visible,
    "leash_taut": _leash_taut,
    "preview_sane": _preview_sane,
    "check_result_ok": _check_result_ok,
    "ng_and_calibrable": _ng_and_calibrable,
    "ng_and_not_calibrable": _ng_and_not_calibrable,
    "estop_resume_prev": _estop_resume_prev,
}

assert len(GUARDS) == 28, len(GUARDS)


def build_guards(mode_entry: Dict) -> Dict[str, Callable]:
    """mode_entry.yaml の許可表を束縛した mode_entry_allowed を差し替えた GUARDS のコピーを返す。

    mode_entry_allowed は §4.1.1 のとおり「表を見る」ガードだが、guards.py 単体では
    表（mode_entry.yaml）を持たない。StateCore.__init__ がこの関数経由で束縛する。
    前提条件（DetailedDesign-transit.md §0.4）は WP-STATE-01 の対象外なので、
    ここでは mode_entry.yaml の許可表だけを見る。
    """
    guards = dict(GUARDS)

    def _bound_mode_entry_allowed(mode, state, ctx) -> bool:
        allowed_targets = mode_entry.get(mode, [])
        target = ctx.arg.get("mode")
        return target in allowed_targets

    guards["mode_entry_allowed"] = _bound_mode_entry_allowed
    return guards
