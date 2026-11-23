"""adjust_core.py — measure_adjuster の純粋コア（8-g P3）。

正本は docs/plan/detailed/DetailedDesign-measure.md §6.4 と
docs/plan/spec/Spec-modes.md SM-3.1.2-127・-128・-136・-137。

- 旋回は始点からのヨーの変化を巻き戻さずに積み上げて比べる（目標を
  (-π, π] に丸めると π を超える指示が逆向きの近道を回る）。
- 並進は始点からの進んだ距離で、間は向きを保つ。

ROS2 を import しない（`th_state/state_core.py` と同じ制約）。
"""
from __future__ import annotations

import math


def wrap_pi(angle: float) -> float:
    """角度を (-π, π] へ巻き戻す。"""
    return math.atan2(math.sin(angle), math.cos(angle))


def accumulate_yaw(yaw: float, prev_yaw: float, accumulated: float) -> float:
    """前回のヨーからの増分（巻き戻さずに）を accumulated へ足す。"""
    return accumulated + wrap_pi(yaw - prev_yaw)


def turn_done(accumulated: float, target: float, tolerance: float) -> bool:
    """積み上げた旋回角度が目標に許容差で届いたか（符号を問わず一致）。"""
    return abs(target - accumulated) <= tolerance


def translate_done(distance: float, target: float, tolerance: float) -> bool:
    """進んだ距離が目標に許容差で届いたか。"""
    return abs(target - distance) <= tolerance


def turn_wzr(accumulated: float, target: float, kp: float,
             w_max: float, w_min: float) -> float:
    """旋回の角速度指令（正＝左回り）。大きさは [w_min, w_max] に収める
    （venue_nav_core.align_cmd_wz と同じ，不感帯の上を保つ）。"""
    err = target - accumulated
    if w_min > w_max:
        w_min = w_max
    magnitude = min(w_max, max(w_min, abs(kp * err)))
    return math.copysign(magnitude, err) if err != 0 else 0.0


def heading_hold(yaw: float, start_yaw: float, kp: float, w_max: float) -> float:
    """並進の間の向き保持（yaw のずれを打ち消す角速度）。"""
    err = wrap_pi(start_yaw - yaw)
    return max(-w_max, min(w_max, kp * err))
