"""localization_health_core.py — localization_health の ROS2 非依存の純粋コア（WP-SAFE-05）。

rclpy を import しない（`th_testing` から直接 pytest できる。connectivity_core.py
と同じ制約・流儀）。

Spec-safety.md §3.5.0 の A・C・B′ を判定する（ゆっくり間違っていく `O-e2` は
範囲外）。数値リテラルを書かない（R2）。しきい値・期待ノード名はすべて
呼び出し側（localization_health.py。registry.yaml 由来の ROS2 パラメータ）から渡す。

優先順位（`node_down` ＞ `stale` ＞ `jump`。呼び出し側がこの順で評価し、
構造で保証する）:
- `node_down` が先: 推定器が居なければ他の判断が無意味。操作者への文言も
  違う（spec §3.5.0「C は A に含まれるが別に持つ」）。
- `stale` が次: 凍結値を比べる意味が無い。凍結中は移動量 0 なので `jump` は
  そもそも出ないが、順序として明記する。
- `jump` が最後: 2 tick とも新鮮なときだけ意味がある。`evaluate()` が ok の
  ときだけ呼び出し側が評価する。
"""
import math
from dataclasses import dataclass
from typing import Iterable, Optional, Tuple

REASON_OK = ""
REASON_STALE = "stale"        # A: map→odom が凍結
REASON_NODE_DOWN = "node_down"  # C: 推定ノード不在
REASON_JUMP = "jump"          # B′: map→odom が比較周期に許容超で動いた
# ゆっくり間違っていく用（O-e2）の予約。範囲外のため出さない。
# REASON_LOW_CONFIDENCE = "low_confidence"


@dataclass(frozen=True)
class Params:
    """registry.yaml 由来（localization_* / jump_*）。"""
    stale_ms: int
    warmup_ms: int
    expected_nodes: Tuple[str, ...]
    jump_window_ms: int
    jump_translation_m: float
    jump_rotation_rad: float


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


@dataclass(frozen=True)
class TransformSample:
    """ある tick で見た map→odom の値。平面（x, y, yaw）だけ見る。"""
    t_ms: int
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class JumpReport:
    is_jump: bool
    trans_m: float
    rot_rad: float


def _wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def detect_jump(prev: Optional[TransformSample], curr: TransformSample,
                p: Params) -> JumpReport:
    """B′: 比較周期のあいだの map→odom の動きが許容を超えたか（WP-SAFE-05B）。

    呼び出し側（ノード）が tick ごとに呼ぶ。tick 間隔が比較周期
   （jump_window_ms）になるよう、ノードのタイマ周期を合わせる。
    境界（ぴったり）は出さない。

    出さない場合:
    - `prev` が無い（初回・TF 不連続で呼び出し側が捨てた直後。
      読み直し直後の飛びを拾わないため）
    - 2 点の時刻差が `stale_ms` を超える（A の時間スケールを超えた不連続。
      凍結→復帰の飛びは A の担当。`stale_ms` の流用であり新規リテラルは無い）
    - 時刻が戻っている（時計の異常。安全側に倒して出さない）
    """
    if prev is None:
        return JumpReport(is_jump=False, trans_m=0.0, rot_rad=0.0)
    dt_ms = curr.t_ms - prev.t_ms
    if dt_ms < 0 or dt_ms > p.stale_ms:
        return JumpReport(is_jump=False, trans_m=0.0, rot_rad=0.0)
    trans = math.hypot(curr.x - prev.x, curr.y - prev.y)
    rot = abs(_wrap_pi(curr.yaw - prev.yaw))
    return JumpReport(
        is_jump=(trans > p.jump_translation_m or rot > p.jump_rotation_rad),
        trans_m=trans,
        rot_rad=rot,
    )
