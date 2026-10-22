"""map_update_policy.py — 地図更新（map_update）フラグの純粋ロジック

地図更新 ON（書き足し。Spec-open F-41）は、ON にしても自己位置推定（slam_toolbox）が
地図作成モードに切り替わらず、地図は読み取り専用のままで一度も効いていなかった。
当面は ON にできない（ユーザー決定 2026-10-22。Spec-webui S-14・S-21、Spec-onsite §4.0.1・§4.0.2）。
OFF への変更は受ける。

rclpy を import しない（`th_testing` から直接 pytest できる）。
"""
from __future__ import annotations

# ON への変更を拒否したときに /system/set_flag が返す reject_reason_key。
# WebUI 側（i18n/reasons.js）に同値の表示文言がある。
MAP_UPDATE_ON_DENIED_REASON = "map_update_unavailable"


def map_update_on_denied(flag: str, value: bool) -> bool:
    """この set_flag 要求が「地図更新を ON にする」ものか（=拒否すべきか）。"""
    return flag == "map_update" and bool(value)
