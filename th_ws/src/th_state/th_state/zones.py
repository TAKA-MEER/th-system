"""zones.py — 画面からゾーンを導く純粋関数（WP-STATE-02）。

`rclpy` を import しない（`state_core.py` / `guards.py` と同じ制約。CLAUDE.md／
DetailedDesign-state.md §1）。`th_testing` から直接 pytest できること。

DetailedDesign-names.md §4.1 が「唯一の正」と明言している `derive_limits()` の実装。
DetailedDesign-state.md §6 も「ここに擬似コードを再掲しない」としたうえで同じ関数を指すので、
実装はここ 1 箇所だけに置く（`jog_gate` と `obstacle_limiter` はこの実装を将来 import する）。

数値リテラルを書かない（R2）。`ui_active_window_s` は呼び出し側（ノード）が
ROS2 パラメータとして受け取り、引数で渡す。画面 ID・ゾーン名・速度上限パラメータ名は
DetailedDesign-names.md §4 の表の転記であり、文字列の集合なので R2 の対象外
（state_core.py の MODES 集合と同じ扱い）。
"""
from dataclasses import dataclass
from typing import Dict, Mapping

# ============================================================
# DetailedDesign-names.md §4 — 画面（15）とゾーン・速度上限の表そのもの。
# ============================================================
SCREEN_ZONE: Dict[str, str] = {
    "S-00": "NA",
    "S-01": "OUT",
    "S-10": "OUT",
    "S-11": "OUT",
    "S-12": "OUT",
    "S-13": "OUT",
    "S-14": "OUT",
    "S-15": "OUT",
    "S-16": "OUT",
    "S-20": "IN",
    "S-21": "IN",
    "S-30": "NA",
    "S-31": "NA",
    "S-40": "NA",
    "S-50": "OUT",
}

SCREEN_SPEED_LIMIT: Dict[str, str] = {
    "S-00": "stop",
    "S-01": "stop",
    "S-10": "v_max",
    "S-11": "v_max",
    "S-12": "v_max",
    "S-13": "v_max",
    "S-14": "v_max",
    "S-15": "v_max",
    "S-16": "v_leash",
    "S-20": "v_slow",
    "S-21": "v_slow",
    "S-30": "v_check",
    "S-31": "stop",
    "S-40": "v_calib",
    "S-50": "stop",
}

assert set(SCREEN_ZONE.keys()) == set(SCREEN_SPEED_LIMIT.keys())

# 3値の優先順位（zone の合成専用。速度上限の合成には使わない。§4.1 の禁止事項）。
# 複数端末が異なるゾーンの画面を同時に使用中のときだけ効く（通常運用は単一画面が大半）。
_ZONE_PRIORITY = ("IN", "NA", "OUT")

# speed_limit 名同士の「厳しさ」順（左ほど厳しい）。実際の数値は registry.yaml 側の責務なので
# ここでは名前の順序だけを持つ（R2）。stop が最も厳しいことは定義上自明。
_LIMIT_STRICTNESS = ("stop", "v_check", "v_calib", "v_slow", "v_leash", "v_max")


@dataclass(frozen=True)
class ScreenInput:
    """`/ui/active_screen` の1端末ぶんのスナップショット。"""
    screen_id: str
    interacting: bool
    last_input_ms: int


@dataclass(frozen=True)
class Limits:
    zone: str            # "IN" / "OUT" / "NA"
    speed_limit: str      # パラメータ名 or "stop"
    auto_brake: bool


def derive_limits(screens: Mapping[str, ScreenInput], now_ms: int,
                   ui_active_window_s: int) -> Limits:
    """DetailedDesign-names.md §4.1 の `derive_limits()`。

    「使用中」= interacting かつ最後の操作から ui_active_window_s 以内
    （Spec-safety.md §6.2 と同一定義）。途絶・使用中 0 台はフェイルセーフで
    zone=NA・speed_limit=stop・auto_brake=True に倒す。
    """
    window_ms = ui_active_window_s * 1000
    active = [s for s in screens.values()
              if s.interacting and now_ms - s.last_input_ms <= window_ms]

    if not active:
        return Limits(zone="NA", speed_limit="stop", auto_brake=True)

    zones_present = {SCREEN_ZONE[s.screen_id] for s in active}
    zone = next(z for z in _ZONE_PRIORITY if z in zones_present)

    # 速度上限は「各画面の上限の最小値」（§4.1）。名前の厳しさ順で最も厳しいものを採る。
    limits_present = {SCREEN_SPEED_LIMIT[s.screen_id] for s in active}
    speed_limit = next(l for l in _LIMIT_STRICTNESS if l in limits_present)

    auto_brake = zone != "OUT"
    return Limits(zone=zone, speed_limit=speed_limit, auto_brake=auto_brake)
