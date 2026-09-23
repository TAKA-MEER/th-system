"""dr_spaam_lifecycle_controller_core.py — DR-SPAAM の起動/停止判断の純ロジック

brief-tracker-default-off §3.2: tracker_enabled（/system/state）と lifecycle 状態
（lifecycle_msgs/msg/State）から ChangeState に渡す transition_id を決める。

rclpy を import しない（`th_testing` から直接 pytest できる）。
ID は lifecycle_msgs（Lifecycle State / Transition）の数値と一致させる。
"""
from __future__ import annotations

# 何もしない（アクションなしの意味でのみ使う特別値。実 transition_id 0 と衝突させない）
TRANSITION_NONE = -1

# lifecycle_msgs/msg/Transition の PRIMARY_STATE_* に対応
STATE_UNCONFIGURED = 1
STATE_INACTIVE = 2
STATE_ACTIVE = 3

# lifecycle_msgs/msg/Transition の TRANSITION_* に対応
TRANSITION_CONFIGURE = 0
TRANSITION_ACTIVATE = 2
TRANSITION_DEACTIVATE = 3

TRANSITION_LABELS = {
    TRANSITION_ACTIVATE: '起動(activate)',
    TRANSITION_DEACTIVATE: '停止(deactivate)',
    TRANSITION_CONFIGURE: 'configure',
}


def desired_lifecycle_transition(tracker_enabled: bool, state_id: int) -> int:
    """現在の lifecycle 状態から、すべき transition_id を返す。

    - tracker_enabled=True かつ ACTIVE         → 何もしない
    - tracker_enabled=True かつ INACTIVE       → ACTIVATE（起動）
    - tracker_enabled=False かつ ACTIVE        → DEACTIVATE（停止）
    - tracker_enabled=False かつ INACTIVE      → 何もしない
    - 遷移中（configuring / activating 等）や UNCONFIGURED → 何もしない
      （次の同期ティックで追いつく。起動時は launch の auto_configure=true に任せる）
    """
    if not tracker_enabled and state_id == STATE_ACTIVE:
        return TRANSITION_DEACTIVATE
    if tracker_enabled and state_id == STATE_INACTIVE:
        return TRANSITION_ACTIVATE
    return TRANSITION_NONE