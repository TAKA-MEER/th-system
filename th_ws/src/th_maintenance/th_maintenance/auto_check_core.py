"""auto_check_core.py — 起動時の自動点検の判定（純粋。ROS2 非依存）。

Spec-ops.md §2.6（毎回の起動で自動判定できる項目だけ自動実行）と
DetailedDesign-maintenance.md §2.7 の写実。`scripts/opcheck_auto.py` が
このモジュールを import して判定に使う。

対象は「自動で判定できる項目だけ」（データ受信・値の範囲・死角マスクの
ズレ・物理ボタンの状態）。**走行を伴う確認（MOTOR）は自動では行わない**
（機体は絶対に動かさない）。

設計方針（check_core.py と同じ不変条件）:
  - ファイル・環境・ROS2 を一切触らない。引数だけで完結する
    （ホストの素の pytest で直接走れるようにするため）。
  - 既存の判定（judge_imu・judge_gyro_unit・judge_lidar・judge_estop の
    考え方）は `check_core` を import して使い回す。**check_core.py の
    既存関数は変えない**。
  - 開発モードの鮮度判定（dev_mode_core.hpp と同じ考え方:
    未受信・3 秒以上更新なし・読めない JSON は消さない）も純粋関数
    `is_suppressed()` としてここに置く。Python 側の既存実装は
    connectivity 側のゲート式（should_emit_link_ok）にしかなく、
    そのまま流用できないため、同じ作法で書き起こす。
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import Sequence

from th_maintenance.check_core import (
    CheckParams,
    judge_gyro_unit,
    judge_imu,
    judge_lidar,
)

# /system/dev_mode の鮮度上限（dev_mode_core.hpp と同じ 3 秒）。
DEV_MODE_STALE_MS = 3000.0

AUTO_ITEMS = ("ESTOP", "IMU", "LIDAR")

OVERALL_RESULTS = ("OK", "WARN", "NG", "CHECKING")


@dataclass(frozen=True)
class AutoItemVerdict:
    """自動点検の 1 項目の判定。`result` は "OK" / "WARN" / "NG" / "CHECKING"。"""

    result: str
    reason: str = ""


@dataclass(frozen=True)
class AutoOverall:
    """総合ステータス。`overall` は "OK" / "WARN" / "NG" / "CHECKING"。"""

    overall: str
    items: dict = field(default_factory=dict)


def judge_estop_auto(estop_alive: bool, pressed: bool) -> AutoItemVerdict:
    """物理ボタンの自動判定（押す・離すの確認ではなく今の状態だけ）。

    - 状態が届いていない → NG("no_data")
    - 押されたまま → NG("pressed")（解除を案内する。運用は止めない＝警告のみ。
      運用開始そのものの阻止は connectivity_checker の link_ok ゲートが担う）
    - 届いていて押されていない → OK
    """
    if not estop_alive:
        return AutoItemVerdict("NG", "no_data")
    if pressed:
        return AutoItemVerdict("NG", "pressed")
    return AutoItemVerdict("OK")


def judge_imu_auto(calib_status: int, gyro_bias_rad_s: float,
                   max_wz_rad_s: float | None, alive: bool,
                   p: CheckParams) -> AutoItemVerdict:
    """IMU の自動判定（check_core.judge_imu + judge_gyro_unit の合成）。

    `max_wz_rad_s` が None（窓内サンプル無し）でも生死は judge_imu が見る。
    単位取り違え（wz_implausible）は NG、サンプル無しは WARN に倒す
    （judge_gyro_unit の区分をそのまま使う）。
    """
    base = judge_imu(calib_status, gyro_bias_rad_s, alive, p)
    if base.result == "NG":
        return AutoItemVerdict("NG", base.reason)
    unit = judge_gyro_unit(max_wz_rad_s, p)
    if unit.result == "NG":
        return AutoItemVerdict("NG", unit.reason)
    if base.result == "WARN" or unit.result == "WARN":
        reason = base.reason if base.result == "WARN" else unit.reason
        return AutoItemVerdict("WARN", reason)
    return AutoItemVerdict("OK")


def judge_lidar_auto(alive: bool, period_s: float | None,
                     ranges: Sequence[float], angle_increment_deg: float,
                     configured_ranges: Sequence[float],
                     p: CheckParams) -> AutoItemVerdict:
    """LiDAR の自動判定（check_core.judge_lidar の写し）。

    死活・周期・全周の有効性・死角マスクのズレを順に見る。
    """
    verdict = judge_lidar(
        alive=alive,
        period_s=period_s,
        ranges=ranges,
        angle_increment_deg=angle_increment_deg,
        configured_ranges=configured_ranges,
        p=p,
    )
    return AutoItemVerdict(verdict.result, verdict.reason)


def combine_overall(estop: AutoItemVerdict, imu: AutoItemVerdict,
                    lidar: AutoItemVerdict) -> AutoOverall:
    """3 項目から総合ステータスを作る。

    - まだ何も届いていない（3 項目とも CHECKING）→ CHECKING（判定中）
    - 1 つでも NG → NG（理由は NG の項目名を `+` で連結）
    - NG が無く 1 つでも WARN → WARN
    - CHECKING が残っていれば（NG/WARN 無し）→ CHECKING
    - すべて OK → OK
    """
    items = {"ESTOP": estop, "IMU": imu, "LIDAR": lidar}
    results = {k: v.result for k, v in items.items()}
    if all(r == "CHECKING" for r in results.values()):
        return AutoOverall("CHECKING", items)
    ng = sorted(k for k, r in results.items() if r == "NG")
    if ng:
        return AutoOverall("NG", items)
    if any(r == "WARN" for r in results.values()):
        return AutoOverall("WARN", items)
    if any(r == "CHECKING" for r in results.values()):
        return AutoOverall("CHECKING", items)
    return AutoOverall("OK", items)


def is_suppressed(dev_mode_raw: str | None, now_ms: float,
                  recv_ms: float | None) -> bool:
    """開発モードの `opcheck` で警告を消してよいか（純粋関数）。

    `dev_mode_core.hpp` と同じ考え方。**`effective` に `opcheck` が明示
    されているときだけ**真。以下はすべて偽（＝消さない＝警告を出す）:
      - 未受信（`recv_ms` が None）
      - 3 秒以上更新なし（`now_ms - recv_ms > DEV_MODE_STALE_MS`）
      - 読めない JSON（parse 失敗・`effective` が無い）
      - `dev_mode` が偽・`effective.opcheck` が真でない
    """
    if dev_mode_raw is None or recv_ms is None:
        return False
    if (now_ms - recv_ms) > DEV_MODE_STALE_MS:
        return False
    try:
        obj = json.loads(dev_mode_raw)
    except (json.JSONDecodeError, TypeError, ValueError):
        return False
    if not isinstance(obj, dict):
        return False
    if obj.get("dev_mode") is not True:
        return False
    effective = obj.get("effective")
    if not isinstance(effective, dict):
        return False
    return effective.get("opcheck") is True
