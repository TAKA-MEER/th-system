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


GRACE_CONFIDENCE = 0.3  # 猶予中に見せる confidence（TRACKED_CONFIDENCE より低く「不確か」を表す）


@dataclass
class LostGraceState:
    """is_lost のデバウンス（猶予）状態。dataclass で持ち、時刻は呼び出し側が
    渡す（auto_select_step / AutoSelectState と同じ流儀。ROS Time に依存しない
    ので pytest で直接テストできる）。"""
    last_seen_ms: int | None = None  # is_lost=False だった直近の時刻。まだ一度も検出できていなければ None


def apply_lost_grace(
    state: "LostGraceState", decision: "BridgeDecision", now_ms: int, grace_ms: int,
) -> "tuple[LostGraceState, BridgeDecision]":
    """classify() の判定に猶予（デバウンス）を適用する（brief-onsite-ux2 F-4）。

    脚検出は歩行中に一瞬抜けるため、classify() が is_lost=True を返した
    フレームが 1 枚来ただけで即 evt.target_lost（画面には「対象を見失って
    います」）を発行すると点滅して見える（実機ログで確定。1788867908〜911 の
    4 秒間に evt.target_lost が 4 回、という事例あり）。「最後に検出できて
    いた時刻」から grace_ms 以内は is_lost=False を維持し、点滅を吸収する。

    呼び出し側（person_tracker_bridge.py）の evt.target_lost は edge 検出
    （is_lost が False→True に変わった瞬間だけ発行）なので、この関数が返す
    「猶予で False のまま」を見ている限り、猶予中に一度も発行されない
    （＝「復帰したらイベントは一切出さない」を自動的に満たす）。

    - decision.is_lost が False（検出できている）: state.last_seen_ms を
      now_ms に更新し、decision をそのまま返す。
    - decision.is_lost が True で、まだ一度も検出できていない
      （last_seen_ms is None）: 猶予の起点が無いのでそのまま lost。
    - decision.is_lost が True で、last_seen_ms から grace_ms 未満しか
      経っていない: 猶予中。is_lost=False・lost_reason=""・
      confidence=GRACE_CONFIDENCE で「まだ見えている」ことにする。
      last_seen_ms は更新しない（実際には見えていないので、猶予の残り時間は
      ここから消費され続ける＝再検出が無いまま grace_ms を過ぎれば確定 lost）。
    - decision.is_lost が True で、grace_ms 以上経過: 猶予切れ。
      decision をそのまま返す（confirmed lost）。
    """
    if not decision.is_lost:
        state.last_seen_ms = now_ms
        return state, decision
    if state.last_seen_ms is None:
        return state, decision
    if now_ms - state.last_seen_ms < grace_ms:
        return state, BridgeDecision(is_lost=False, lost_reason="", confidence=GRACE_CONFIDENCE)
    return state, decision


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
