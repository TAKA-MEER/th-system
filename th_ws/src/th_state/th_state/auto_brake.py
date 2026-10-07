"""auto_brake.py — 自動ブレーキの有効／無効を決める純粋関数（1b-15 SG-B10）。

`rclpy` を import しない（`zones.py` / `tracker_policy.py` と同じ制約）。
`th_testing` から直接 pytest できること。

正本は Spec-safety.md §2.1 と Spec-webui.md §3.5:
  - 自律系（attributes.yaml の `auto_brake_default: on_locked`）は無効化できない。
  - 手動系（手動走行・教示（手動））とジョグ介入中は、ゾーン既定に従う。
    場外は OFF（警告のみ）、場内は ON。どちらも試験員が切り替えられる（既定であって固定ではない）。
  - 画面の申告が途絶している・ゾーン NA のときは ON（安全側）。

状態の正本は機体側（state_manager）。ここは「モード＋ゾーン＋ジョグ＋試験員の要求 → 有効／無効」
の写像だけを持つ。ノード名・トピック名・パラメータは持ち込まない。
"""
from typing import Mapping, Optional

# attributes.yaml の auto_brake_default の値。
LOCKED = "on_locked"
DEFAULT_ON = "on"

# 切替要求を受け付けない理由（reject_reason_key。web_ui の i18n/reasons.js と対）。
AUTO_BRAKE_LOCKED_REASON = "auto_brake_locked"


def override_allowed(mode_attrs: Mapping[str, object], jog_active: bool) -> bool:
    """試験員が切り替えてよいか。

    手動系のモード（auto_brake_default が on_locked でない）か、ジョグ介入中
    （ジョグ中は元のモードが自律系でも手動系として扱う。Spec-modes.md §3.1.1）のとき真。
    それ以外（自律系の走行・待機など）は無効化不可。
    """
    return mode_attrs.get("auto_brake_default") != LOCKED or bool(jog_active)


def zone_default(mode_attrs: Mapping[str, object], zone: str,
                 jog_active: bool) -> bool:
    """試験員の要求が無いときの既定値。

    場内（IN）と NA は ON。場外（OUT）は手動系なら attributes.yaml の既定
    （MANUAL / TEACH_MANUAL は off）、自律系モードのジョグ中は off（ジョグはゾーン既定に従う）。
    自律系（on_locked）でジョグ中でないときは常に ON。zone が未知の値でも ON に倒す。
    """
    if zone != "OUT":
        return True
    if not override_allowed(mode_attrs, jog_active):
        return True
    return mode_attrs.get("auto_brake_default") == DEFAULT_ON


def effective_auto_brake(mode_attrs: Mapping[str, object], zone: str,
                         jog_active: bool,
                         override: Optional[bool]) -> bool:
    """/system/state.auto_brake の値。

    override は試験員の要求（None なら無し）。切り替えを許さない状況（自律系・ゾーン NA）では
    要求を無視する。ゾーン NA（画面途絶・使用中 0 台を含む）は常に ON（安全側）。
    """
    if zone not in ("IN", "OUT"):
        return True
    if override is not None and override_allowed(mode_attrs, jog_active):
        return bool(override)
    return zone_default(mode_attrs, zone, jog_active)
