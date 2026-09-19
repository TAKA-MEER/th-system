"""
wait_clear_core.py
==================
退避待ちゲートの純ロジック（ROS2 非依存）

SUMMON/WAIT_CLEAR で試験員がゴールから clear_distance_m 以上離れ
clear_hold_ms 継続したら evt.clear_ok、clear_timeout_ms 超で
evt.clear_timeout(POINT へ戻す)。見失い中は hold を 0 に戻す。

ROS2 ノード (wait_clear_gate.py) はこのモジュールを import して使用する。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Tuple


@dataclass
class WaitClearParams:
    clear_distance_m: float = 1.0
    clear_hold_ms: int = 1500
    clear_timeout_ms: int = 30000


@dataclass
class WaitClearState:
    hold_accum_ms: int = 0
    elapsed_ms: int = 0
    fired_ok: bool = False
    fired_timeout: bool = False


def step(state: WaitClearState, dt_ms: int,
         distance_m: float | None, target_visible: bool,
         params: WaitClearParams) -> Tuple[WaitClearState, str, List[str]]:
    """1 ティック進める。

    Returns:
        (新 state, verdict, 発火イベント名リスト)

    - target_visible=False or distance is None → hold_accum_ms=0, verdict="NOT_CLEAR"
    - distance_m >= clear_distance_m → hold_accum_ms += dt_ms
        - hold_accum_ms >= clear_hold_ms かつ not fired_ok → events=["evt.clear_ok"],
          fired_ok=True, verdict="OK"
        - まだ → verdict="WAITING"
    - distance_m < clear_distance_m → hold_accum_ms=0, verdict="NOT_CLEAR"
    - elapsed_ms += dt_ms. elapsed_ms >= clear_timeout_ms かつ not fired_ok かつ not fired_timeout
        → events=["evt.clear_timeout"], fired_timeout=True
    - 一度 fired_ok / fired_timeout になったら以後イベントを二重に出さない
    """
    state = WaitClearState(
        hold_accum_ms=state.hold_accum_ms,
        elapsed_ms=state.elapsed_ms,
        fired_ok=state.fired_ok,
        fired_timeout=state.fired_timeout,
    )
    events: List[str] = []

    # hold 判定
    if not target_visible or distance_m is None:
        state.hold_accum_ms = 0
        verdict = "NOT_CLEAR"
    elif distance_m >= params.clear_distance_m:
        state.hold_accum_ms += dt_ms
        if state.hold_accum_ms >= params.clear_hold_ms and not state.fired_ok:
            state.fired_ok = True
            events.append("evt.clear_ok")
            verdict = "OK"
        else:
            verdict = "WAITING"
    else:
        state.hold_accum_ms = 0
        verdict = "NOT_CLEAR"

    # タイムアウト
    state.elapsed_ms += dt_ms
    if (state.elapsed_ms >= params.clear_timeout_ms
            and not state.fired_ok and not state.fired_timeout):
        state.fired_timeout = True
        events.append("evt.clear_timeout")

    return state, verdict, events


def remaining_sec(state: WaitClearState, params: WaitClearParams) -> float:
    """clear_timeout_ms までの残り秒。"""
    return max(0.0, (params.clear_timeout_ms - state.elapsed_ms) / 1000.0)


def reset() -> WaitClearState:
    """WAIT_CLEAR を抜けた/入ったときに呼ぶ。"""
    return WaitClearState()
