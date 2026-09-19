"""
odom_source.py
==============
自己位置源（EKF 出力 vs 生 odom）の選択ロジック（ROS2 非依存）

route_recorder.py / replay_runner.py は教示・再生の自己位置を決める。self 位置源に
EKF 出力 (/odometry/filtered) を使いたいが、EKF が動いていないと (imu_enabled の
構成や起動失敗で) そのトピックが出ない。また /odom (10Hz) よりレートが低い (7.7Hz)。

そこで「EKF 出力が新鮮ならそれを使い、古ければ生 odom にフォールバックする」選択を
ここに置く。WS-9D の要旨: EKF の恩恵 (/odometry/filtered) が誰にも使われていなかった
ため、長距離の教示・再生でクローラのスリップによるヨードリフトが効いていた。

ROS2 を一切 import しない純関数にして pytest で直接叩ける形にする
(route_replay_core.py / route_record_core.py と同じ二層構造)。
"""

from __future__ import annotations

from typing import Optional, Tuple

Pose2D = Tuple[float, float, float]   # (x, y, yaw[rad])
OdomSample = Tuple[Pose2D, int]       # (pose, stamp_ms)


def pick_odom_source(
    filtered: Optional[OdomSample],
    raw: Optional[OdomSample],
    now_ms: int,
    stale_ms: int,
) -> Tuple[Optional[Pose2D], Optional[str]]:
    """使うべき pose と、その出所を返す。

    filtered / raw は None または (pose, stamp_ms) のタプル。pose は (x, y, yaw)。
    戻り値: (pose, source) で source は 'filtered' | 'raw' | None。

    - filtered が新鮮（now_ms - stamp_ms <= stale_ms）ならそれを使う
    - そうでなく raw が新鮮なら raw
    - どちらも無い/古いなら (None, None)

    境界: 経過時間がちょうど stale_ms は新鮮側に含める（<=）。
    stamp_ms が未来（負の経過時間）でも新鮮扱いにする（クロックのずれで永久に
    古い判定になるのを避ける）。どちらかが None でも落ちない。
    """
    def _is_fresh(sample: Optional[OdomSample]) -> bool:
        if sample is None:
            return False
        _pose, stamp_ms = sample
        return now_ms - stamp_ms <= stale_ms

    if _is_fresh(filtered):
        pose, _ = filtered
        return (pose, 'filtered')

    if _is_fresh(raw):
        pose, _ = raw
        return (pose, 'raw')

    return (None, None)
