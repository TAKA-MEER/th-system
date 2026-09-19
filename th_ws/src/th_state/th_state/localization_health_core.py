"""localization_health_core.py — localization_health の ROS2 非依存の純粋コア（WP-SAFE-05）。

rclpy を import しない（`th_testing` から直接 pytest できる。connectivity_core.py
と同じ制約・流儀）。

Spec-safety.md §3.5.0 の A と C を判定する（B は範囲外。`O-e1`）。
数値リテラルを書かない（R2）。しきい値・期待ノード名はすべて呼び出し側
（localization_health.py。registry.yaml 由来の ROS2 パラメータ）から渡す。
"""
from dataclasses import dataclass
from typing import Iterable, Optional, Tuple

REASON_OK = ""
REASON_STALE = "stale"        # A: map→odom が凍結
REASON_NODE_DOWN = "node_down"  # C: 推定ノード不在
# B 用の予約。範囲外（O-e1。信号源未確定）のため出さない。
# REASON_LOW_CONFIDENCE = "low_confidence"


@dataclass(frozen=True)
class Params:
    """registry.yaml 由来（localization_*）。"""
    stale_ms: int
    warmup_ms: int
    expected_nodes: Tuple[str, ...]


@dataclass(frozen=True)
class HealthReport:
    ok: bool
    reason: str
    node_present: bool
    # 最後に見た map→odom の古さ [s]。一度も見ていなければ inf。
    transform_age_sec: float


def evaluate(now_ms: int, boot_ms: int, last_transform_ms: Optional[int],
             present_nodes: Iterable[str], p: Params) -> HealthReport:
    """A（凍結）と C（ノード不在）を判定する。

    起動猶予（warmup_ms）の間は ok を出す。推定ノードの discovery・初回
    map→odom に時間がかかるため。起動失敗（C）は猶予後に node_down で捕まえる。
    """
    present = set(present_nodes)
    node_present = any(n in present for n in p.expected_nodes)
    if last_transform_ms is None:
        age_sec = float("inf")
    else:
        age_sec = (now_ms - last_transform_ms) / 1000.0

    if now_ms - boot_ms < p.warmup_ms:
        return HealthReport(ok=True, reason=REASON_OK,
                            node_present=node_present, transform_age_sec=age_sec)
    if not node_present:
        return HealthReport(ok=False, reason=REASON_NODE_DOWN,
                            node_present=node_present, transform_age_sec=age_sec)
    if age_sec * 1000.0 > p.stale_ms:
        return HealthReport(ok=False, reason=REASON_STALE,
                            node_present=node_present, transform_age_sec=age_sec)
    return HealthReport(ok=True, reason=REASON_OK,
                        node_present=node_present, transform_age_sec=age_sec)
