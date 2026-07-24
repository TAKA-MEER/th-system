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
