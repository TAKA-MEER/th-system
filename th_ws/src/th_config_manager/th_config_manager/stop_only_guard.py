"""
stop_only_guard.py
==================
Spec.md SD-9 の純関数（ROS 依存なし）。

地図の破棄・切替・読み込みと設定値の変更を機体側で受け付けるのは、
IDLE と PREP の機体が自分で走らない状態（MAPPING／REGISTER／EDIT／SAVED）で、
ジョグ中でないときに限る。RETURN・PAUSE・走行系の各モード・手動走行では拒否する
（2026-10-07 ユーザー決定。画面のボタン無効化だけに頼らず機体側でも拒否する）。
/system/state が未受信・古いときは拒否する（安全側）。

鮮度の既定値（STATE_STALE_SEC）は registry.yaml の state_stale_ms（1500）と
同じ。jog_gate / obstacle_limiter / safety_monitor が同じ値で /system/state の
途絶を判定している（consumers 参照）。
"""

from typing import Optional

# PREP のうち機体が自分で走らない状態（Spec.md SD-9）。
ALLOWED_PREP_STATES = frozenset({"MAPPING", "REGISTER", "EDIT", "SAVED"})

# /system/state の鮮度の上限 [秒]。registry の state_stale_ms=1500 と同じ値。
STATE_STALE_SEC = 1.5


def stop_only_allows(mode: Optional[str],
                      state: Optional[str],
                      jog_active: bool,
                      state_received: bool,
                      state_age_sec: Optional[float],
                      stale_sec: float = STATE_STALE_SEC) -> "tuple[bool, str]":
    """停止中だけの操作（地図・設定値）をいま受け付けてよいか（純関数）。

    - mode: /system/state の mode（"IDLE" / "PREP" / ...）。None は未受信扱い
    - state: /system/state の state（"MAPPING" / ...）。IDLE の判定には使わない
      （IDLE に状態は無く、state_manager は "NONE" を出す）
    - jog_active: /system/state の jog_active（手動ジョグ介入中）
    - state_received: /system/state を一度でも受け取ったか
    - state_age_sec: 最後に受けてからの経過 [秒]（monotonic。呼び出し側が測る）
    - stale_sec: 鮮度の上限 [秒]。ちょうど上限は新鮮側に含める（<=）

    戻り値: (許可なら True, 拒否理由キー。許可なら "")。理由キーは
    "state_not_received" / "state_stale" / "jog_active" /
    "mode_state_not_stopped:<mode>/<state>" のいずれか。
    """
    if not state_received or mode is None:
        return False, "state_not_received"
    if state_age_sec is None or state_age_sec > stale_sec:
        return False, "state_stale"
    if jog_active:
        return False, "jog_active"
    if mode == "IDLE":
        return True, ""
    if mode == "PREP" and state in ALLOWED_PREP_STATES:
        return True, ""
    return False, f"mode_state_not_stopped:{mode}/{state}"
