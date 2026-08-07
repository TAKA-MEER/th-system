"""
face_planner_core.py
=====================
FACING モード（超信地旋回のみで正対・展示用）のコアロジック

テスト可能性のため、ROS2 インポートを一切含まない純粋な Python モジュール。
ROS2 ノード側 (face_planner.py) はこのモジュールを import して使用する。

FACING は一切並進せず、試験員の base_link 相対ベアリングへその場で正対し続けるだけの
モード。ブース展示などロボットを設置スペースから動かせない場面向け。

このモジュール自体は odom 非依存(ベアリングは atan2(person_y, person_x) で直接
求まる)。ただし呼び出し側(face_planner.py)は検知間隔(DR-SPAAM 約2Hz)の間も
旋回し続けるため、検知時点の base_link 相対値をそのまま使い回すと自機の旋回分だけ
ベアリングが古びる。face_planner.py は follow_planner_mapless.py と同じ方式で
odom 経由の自己移動補償を行ってから、補償済みの (x, y) をこのモジュールへ渡す
(costmap・Nav2・SLAM 地図には依存しない)。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from th_planning.mapless_follow_core import should_stop_for_lost


@dataclass
class FaceParams:
    k_ang:                     float = 2.0   # 角度ゲイン
    w_max_rad_s:                float = 0.5  # rad/s 旋回速度の上限
    max_angular_accel_rad_s2:   float = 3.0  # rad/s^2 旋回速度の加速側レート制限
    max_angular_decel_rad_s2:   float = 6.0  # rad/s^2 旋回速度の減速側レート制限(加速側より
                                              # 大きく: 目標付近での通り過ぎを検知してから
                                              # 素早く止まれるようにする)。実機検証(2026-08-07)
                                              # では、当初疑っていた「振幅の大きい発振」の主因は
                                              # これではなく人物トラッカー側の誤対応付け(他の脚
                                              # への乗り移り)だったと判明した。コマンドの滑らかさ
                                              # 自体には寄与するため維持するが、単独では発振を
                                              # 解消しない(w_max_rad_s の説明は
                                              # th_bringup/config/planning_params.yaml 参照)
    angle_tolerance_rad:        float = 0.05 # rad 正対完了とみなす角度誤差(不感帯)
    min_confidence:              float = 0.5  # 0.0-1.0 これ未満なら旋回停止
    lost_debounce_sec:           float = 0.3  # s is_lost がこの秒数継続したら停止
    update_rate_hz:               float = 10.0


@dataclass
class FaceOutput:
    angular_z: float = 0.0
    # "tracking" / "aligned" / "person_lost" / "low_confidence"
    reason:    str   = "tracking"


def bearing_to(person_x: float, person_y: float) -> float:
    """base_link 相対座標から正対すべき角度を返す(並進しないため絶対座標系は無関係)"""
    return math.atan2(person_y, person_x)


def face_control(bearing_rad: float, k_ang: float, angle_tolerance: float) -> float:
    """ベアリングへの P 制御角速度。角度誤差が許容内なら 0(不感帯)。"""
    if abs(bearing_rad) < angle_tolerance:
        return 0.0
    return k_ang * bearing_rad


def rate_limit_angular(target: float, prev: float,
                        max_accel_delta: float, max_decel_delta: float) -> float:
    """
    符号付き角速度のレート制限。|target| が |prev| より大きい(加速)か小さい(減速)かで
    別々の上限を使う。mapless_follow_core.rate_limit() の並進版(加減速を別枠にする)と
    同じ考え方だが、角速度は符号が反転し得るため単純な大小比較ではなく大きさで
    加速/減速を判定する。減速側を大きくすることで、目標を通り過ぎたと分かってから
    素早く止まれる(実機検証: 加減速を同じレートにすると振幅の大きい発振になる)。
    """
    decelerating = abs(target) < abs(prev)
    max_delta = max_decel_delta if decelerating else max_accel_delta
    if target > prev + max_delta:
        return prev + max_delta
    if target < prev - max_delta:
        return prev - max_delta
    return target


class FaceCore:
    """
    face_planner の中核ロジック(ROS2 非依存)。
    ROS2 ノードから update_rate_hz で update() を呼び出す。
    """

    def __init__(self, params: FaceParams = None):
        self.params = params or FaceParams()
        self._prev_angular_z = 0.0

    def reset(self) -> None:
        """モード切替時などにレート制限の起点をゼロへ戻す"""
        self._prev_angular_z = 0.0

    def update(
        self,
        person_x: float,
        person_y: float,
        confidence: float,
        is_lost: bool,
        lost_elapsed_sec: float,
    ) -> FaceOutput:
        p = self.params
        max_accel_delta = p.max_angular_accel_rad_s2 / p.update_rate_hz
        max_decel_delta = p.max_angular_decel_rad_s2 / p.update_rate_hz

        if should_stop_for_lost(is_lost, lost_elapsed_sec, p.lost_debounce_sec):
            w = rate_limit_angular(0.0, self._prev_angular_z, max_accel_delta, max_decel_delta)
            self._prev_angular_z = w
            return FaceOutput(angular_z=w, reason="person_lost")

        if confidence < p.min_confidence:
            w = rate_limit_angular(0.0, self._prev_angular_z, max_accel_delta, max_decel_delta)
            self._prev_angular_z = w
            return FaceOutput(angular_z=w, reason="low_confidence")

        bearing = bearing_to(person_x, person_y)
        target_w = face_control(bearing, p.k_ang, p.angle_tolerance_rad)
        target_w = max(-p.w_max_rad_s, min(p.w_max_rad_s, target_w))

        w = rate_limit_angular(target_w, self._prev_angular_z, max_accel_delta, max_decel_delta)
        self._prev_angular_z = w

        reason = "aligned" if abs(bearing) < p.angle_tolerance_rad else "tracking"
        return FaceOutput(angular_z=w, reason=reason)
