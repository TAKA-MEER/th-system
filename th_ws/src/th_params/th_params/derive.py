"""derive.py — class:b パラメータの導出式（純粋関数）。

`DetailedDesign-params.md` §3 の全式 ＋ `DetailedDesign-safety.md` §3.1 の
`floor_distance`（`d_floor` は制動距離から求めてはいけないため、別の式として定義されている）。
合わせて 12 関数（`DetailedDesign-wp0.md` `WP-PARAM-01` §4.1）。

不変条件 P-1: **ファイルも環境変数も読まない。**registry を知らない・呼ばれた引数だけで完結する。
"""
from __future__ import annotations

import math

# ---------------------------------------------------------------------------
# §3.1 【B】制動の連鎖（最も広く効く）
# ---------------------------------------------------------------------------


def braking_distance(v: float, a: float, t_delay: float) -> float:
    """Spec-safety.md §2.2 / Spec-params.md §3【B】 / F-06（停止距離の定義）"""
    return v * v / (2.0 * a) + v * t_delay


# ---------------------------------------------------------------------------
# §3.1.1 速度は「止まれる距離」から逆算する
# ---------------------------------------------------------------------------


def speed_from_braking_distance(d_allow: float, a: float, t_delay: float) -> float:
    """braking_distance(v, a, t_delay) == d_allow を満たす v。
    v²/(2a) + v·t = D を解く（正の根）。"""
    return a * (-t_delay + math.sqrt(t_delay ** 2 + 2.0 * d_allow / a))


def v_max_from_ceiling(ceiling_mps: float, headroom_ratio: float) -> float:
    """出力上限を使い切らず、補正の余地を残す。Spec-params.md §1"""
    return ceiling_mps * headroom_ratio


# ---------------------------------------------------------------------------
# safety.md §3.1 — d_floor は速度非依存の機体クリアランス
# ---------------------------------------------------------------------------


def floor_distance(body_half_length_m: float, floor_margin_m: float) -> float:
    """速度に依存しない。『これ以上近づいたら何が起きていてもおかしい』距離。
    ★ obstacle_stop_distance_m（制動距離ベース）とは独立した式であること（A2a/A2b の前提）。"""
    return body_half_length_m + floor_margin_m


# ---------------------------------------------------------------------------
# §3.1.2 その他の導出
# ---------------------------------------------------------------------------


def person_backstop_ms(grace_ms: float, link_p99_ms: float, factor: float) -> float:
    """safety_monitor の遅いバックストップ。挙動側の grace より必ず長い。"""
    return max(grace_ms * factor, link_p99_ms * factor)


def clear_distance(body_half_length_m: float, clear_margin_m: float) -> float:
    """退避待ちゲート：対象がゴールから離れるべき距離。"""
    return body_half_length_m + clear_margin_m


def hysteresis_band(floor_distance_m: float, ratio: float) -> float:
    """★ d_floor スケール。d_behavior から導くと帯が d_behavior を超えうる（A11・P-4）。"""
    return floor_distance_m * ratio


# ---------------------------------------------------------------------------
# §3.2 【A】停止精度の連鎖
# ---------------------------------------------------------------------------


def combined_heading_error(two_point_deg: float, nav_tolerance_deg: float) -> tuple[float, float]:
    """(二乗和平方根, 最悪の重なり) を返す。Spec-onsite.md §3.3 / F-22"""
    rss = math.hypot(two_point_deg, nav_tolerance_deg)
    worst = two_point_deg + nav_tolerance_deg
    return rss, worst


def two_point_angle_error_deg(spacing_m: float, sigma_m: float) -> float:
    """2 点指示が与える向きの誤差。L と σ から。Spec-params.md §1"""
    return math.degrees(math.atan2(sigma_m * math.sqrt(2.0), spacing_m))


# ---------------------------------------------------------------------------
# §3.3 タイムアウトの 2 本制約
# ---------------------------------------------------------------------------


def timeout_lower_bound_ms(p99_gap_ms: float, margin_ratio: float) -> float:
    """誤フォルトを出さない下限"""
    return p99_gap_ms * (1.0 + margin_ratio)


def timeout_upper_bound_ms(v_max: float, intrusion_budget_m: float) -> float:
    """タイムアウトまでに走ってよい距離からの上限"""
    return intrusion_budget_m / v_max * 1000.0


def v_max_from_intrusion_budget(intrusion_budget_m: float, timeout_ms: float) -> float:
    """A1 のクランプ（§3.3 手順2・N-7）: timeout_upper_bound_ms の逆関数。
    採用する timeout（下限）を固定した上で、intrusion_budget_m を満たす v_max を逆算する。
    v_max を下げる側にのみ使う（1 回だけ。不動点反復にしない。P-5）。

    IEEE754 の丸めにより `(budget / x) * x`（x = timeout_ms/1000）が `budget` を
    1 ULP 超えることがある——`assertions.a1_intrusion_budget()` は厳密比較
    （`intrusion > intrusion_budget_m`）なので、超えたままだと「クランプして
    ちょうど満たしたはずの v_max」で直後に A1 が再び違反する。A1（危険な組合せを
    構成不能にする表明）を緩めるのではなく、クランプ側が厳密に満たしに行く——
    `math.nextafter(v, 0.0)` で v_max を下向きに 1 ULP ずつ下げ、
    `v * x ≤ budget` になった時点で止める（実測で最大 1 回、有界）。"""
    x = timeout_ms / 1000.0
    v = intrusion_budget_m / x
    while v * x > intrusion_budget_m:
        v = math.nextafter(v, 0.0)
    return v


# ---------------------------------------------------------------------------
# §3.4 【C】経路長の連鎖
# ---------------------------------------------------------------------------


def deviation_budget_m(corridor_width_m: float, body_width_m: float, margin_m: float) -> float:
    return (corridor_width_m - body_width_m) / 2.0 - margin_m
