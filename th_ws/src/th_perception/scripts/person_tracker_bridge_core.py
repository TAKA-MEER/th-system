"""
person_tracker_bridge_core.py
================================
ROS2 非依存の純粋ロジック: FollowingPosition.status + stop_following から
PersonStatus.is_lost / lost_reason / confidence を導出する。
pytest で直接テスト可能 (rclpy を import しない)。
"""
from __future__ import annotations

from dataclasses import dataclass

STATUS_EXISTS_LEG = 1
TRACKED_CONFIDENCE = 0.9  # dr_spaam_param.yaml の conf_thresh を通過した検出のみ EXISTS_LEG になるため固定値


@dataclass
class BridgeDecision:
    is_lost: bool
    lost_reason: str
    confidence: float


def classify(status: int, stop_following: bool) -> BridgeDecision:
    """
    status: FollowingPosition.status (0=NO_EXISTS, 1=EXISTS_LEG, ...)
    stop_following: 直近の sobits_follower/.../stop_following (Bool)
                    (= PersonTracker 内の detection_lost OR target_changed_latched_)

    stop_following は2つの独立した条件のORなので、status == EXISTS_LEG (detection_lost で
    はない) にもかかわらず stop_following が真であれば、それは target_changed_latched_
    (対象切替の疑い) によるものだと判別できる。
    """
    if status != STATUS_EXISTS_LEG:
        return BridgeDecision(is_lost=True, lost_reason="DETECTION_LOST", confidence=0.0)
    if stop_following:
        return BridgeDecision(is_lost=True, lost_reason="TARGET_SWITCHED", confidence=0.0)
    return BridgeDecision(is_lost=False, lost_reason="", confidence=TRACKED_CONFIDENCE)


def match_selected_index(candidates, followed_xy, is_lost, match_tol_m=0.35) -> int:
    """追跡中の座標 followed_xy に最も近い候補の index を返す。
    is_lost or followed_xy None or 最近傍が match_tol_m 超 → -1。
    candidates: list[tuple[float, float]]  (base_link 系 x,y)
    """
    if is_lost or followed_xy is None or not candidates:
        return -1
    fx, fy = followed_xy
    best_i = -1
    best_d = None
    for i, (cx, cy) in enumerate(candidates):
        d = ((cx - fx) ** 2 + (cy - fy) ** 2) ** 0.5
        if best_d is None or d < best_d:
            best_d = d
            best_i = i
    if best_d is None or best_d > match_tol_m:
        return -1
    return best_i


@dataclass
class AutoSelectState:
    single_since_ms: int | None = None  # 候補がちょうど1つになった時刻。0 or 2+ で None
    fired: bool = False                 # 一度発火したら再発火しない


def auto_select_step(state: AutoSelectState, n_candidates: int, now_ms: int,
                     hold_ms: int, already_selected: bool) -> tuple[AutoSelectState, int]:
    """候補数の履歴から自動選択すべき index を返す（該当なければ -1）。
    - n_candidates == 1 かつ not already_selected:
        single_since_ms が None なら now_ms をセット
        now_ms - single_since_ms >= hold_ms なら index 0 を返し single_since_ms を「発火済み」に
    - n_candidates != 1 → single_since_ms = None
    - 一度発火したら候補数が 1 のままでも再発火しない
    """
    if already_selected or state.fired:
        return state, -1
    if n_candidates == 1:
        if state.single_since_ms is None:
            state.single_since_ms = now_ms
            return state, -1
        if now_ms - state.single_since_ms >= hold_ms:
            state.fired = True
            state.single_since_ms = None
            return state, 0
        return state, -1
    state.single_since_ms = None
    return state, -1
