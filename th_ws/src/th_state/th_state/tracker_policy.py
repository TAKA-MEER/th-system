"""tracker_policy.py — 人物追跡（tracker_enabled）フラグの純粋ロジック

Spec-modes.md §9（E-9）と §5.1（CL-X-1）のポリシーを、state_manager.py が
モード名の文字列比較を書かずに満たすための参照点（N-1 対応。モード名の
文字列集合はこのファイルにだけ置き、ノード側はここから import して使う）。

rclpy を import しない（`th_testing` から直接 pytest できる）。
"""
from __future__ import annotations

# Spec-modes.md §9（E-9）「動かさない」モード。このモード群へ遷移したら
# tracker_enabled を false に自動停止する（ESTOP / CARRY は対象外）。
TRACKER_AUTOSTOP_MODES = frozenset({
    "INIT", "IDLE", "MANUAL", "TEACH_MANUAL", "REPLAY", "LINE", "LEASH",
    "OPCHECK", "CALIB",
})

# Spec-modes.md §5.1（CL-X-1-2）「人物追跡を要するモード」での OFF 拒否。
# SUMMON はどの状態でも、PREP は REGISTER（ピン登録中）だけが対象
# （brief-tracker-default-off §3.1）。
_TRACKER_OFF_DENIED = frozenset({
    ("SUMMON", "*"),
    ("PREP", "REGISTER"),
})

# OFF 拒否時に /system/set_flag が返す reject_reason_key。
# WebUI 側（i18n/reasons.js）に同値の表示文言がある。
TRACKER_OFF_DENIED_REASON = "tracker_required"


def tracker_autostop(mode: str) -> bool:
    """「動かさない」モード群に含まれるか。

    true を返すモードへ遷移したとき、tracker_enabled を false に落とす。
    """
    return mode in TRACKER_AUTOSTOP_MODES


def tracker_off_denied(mode: str, state: str) -> bool:
    """いま (mode, state) にいるとき tracker_enabled を OFF にできるか。

    「要 person」の状態では OFF を拒否する（Spec-modes.md §5.1-2）。
    SUMMON はモード全体（どの状態でも）、PREP は REGISTER だけが対象。
    """
    if (mode, "*") in _TRACKER_OFF_DENIED:
        return True
    return (mode, state) in _TRACKER_OFF_DENIED