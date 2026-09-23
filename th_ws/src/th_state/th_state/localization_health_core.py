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
REASON_RESTARTING = "restarting"  # O-e3: 計画的な再起動中（保留。フォルトではない）
REASON_RESTART_TIMEOUT = "restart_timeout"  # O-e3: 再起動が上限を超過（本物の異常）
REASON_INACTIVE = "inactive"  # WP-SAFE-05修正: 自律走行しないモードのため監視外
# （safety_monitor には ok=true として届く。transform_age_sec／node_present は生値）
# ゆっくり間違っていく用（O-e2）の予約。範囲外のため出さない。
# REASON_LOW_CONFIDENCE = "low_confidence"

# Spec-safety.md §3.5.0「使っていない間は監視しない」のモード表
# （2026-09-23 具体化）。「使っている」＝自己位置推定に従って自律走行する
# モードにいる間。モード単位で決める（向き合わせ・自動再試行のような「人が
# 押さなくても動き出す」場面を取りこぼさないため）。試験準備だけは地図作成が
# 中心なので自動帰還（RETURN）の間に限る。
MONITORED_MODES = frozenset({"REPLAY", "PANEL_NAV", "SUMMON", "HOME_NAV"})
PREP_MONITORED_STATES = frozenset({"RETURN"})


def is_localization_in_use(mode: Optional[str], state: Optional[str]) -> bool:
    """/system/state の mode/state から監視対象かを返す（純関数）。

    - REPLAY / PANEL_NAV / SUMMON / HOME_NAV はモード全体（状態は見ない）
    - PREP は状態が RETURN のときだけ
    - それ以外すべて・未受信（None）は監視しない（起動中は INIT なので同じ）
    """
    if mode is None:
        return False
    if mode in MONITORED_MODES:
        return True
    if mode == "PREP":
        return state in PREP_MONITORED_STATES
    return False


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


def check_planned_restart(now_ms: int,
                          restarting: Optional[bool],
                          true_since_ms: Optional[int],
                          false_since_ms: Optional[int],
                          restart_max_ms: int,
                          post_restart_grace_ms: int) -> Optional[Tuple[bool, str]]:
    """O-e3: 計画的な再起動の保留判定（純関数。数値リテラルを持たない）。

    ノードは /slam_control/estimator_restarting の受信値と edge 時刻
    （自分の時計）を渡す。戻り値 None ＝保留対象外（従来どおり A・C・B′ を
    評価する）。それ以外は (ok, reason) をそのまま使う:
    - 再起動中（True 受信中）は (True, 'restarting')。A・C・B′ を評価しない
    - True になってからの経過が restart_max_ms 超は (False, 'restart_timeout')
      （立て直しが終わらない本物の異常。publisher が死んで True のまま
      止まっていても、自分の時計で測るため上限で救える）
    - 終わって post_restart_grace_ms 以内は (True, 'restarting')
      （最初の補正を待つ。A・C を評価しない）
    - 一度も知らせが無い／False（再起動の edge を見ていない）→ None
      （＝計画的でないものとして従来どおり検知する。安全側）

    境界は既存の流儀に揃える（超えたら ng・以内は ok。evaluate の
    stale・warmup と同じ向き）: 上限は `>` で超えたら故障、猶予は `<` で
    以内なら保留（ちょうどは猶予明け＝通常評価）。
    """
    if restarting is True:
        if true_since_ms is not None and now_ms - true_since_ms > restart_max_ms:
            return (False, REASON_RESTART_TIMEOUT)
        return (True, REASON_RESTARTING)
    if (restarting is False and false_since_ms is not None
            and now_ms - false_since_ms < post_restart_grace_ms):
        return (True, REASON_RESTARTING)
    return None


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
