"""assertions.py — 起動時アサーション（危険な組合せを構成不能にする。純粋関数）。

`DetailedDesign-params.md` §4 の **A1〜A11**（`WP-PARAM-01` の対象範囲。A12 は
`DetailedDesign-safety.md` §1.3 を参照する必要があり、パケット §2 の参照節に
含まれていないため実装しない — R1「§2 に無い仕様は実装しない」）。

**A13**（`N-15` レビュー・`WP-SAFE-03` の派生作業で追加）: `th_state.zones.LIMIT_STRICTNESS`
の並びが `registry.yaml` の解決済みの数値の昇順と一致することを検査する。
`th_state/zones.py` の `LIMIT_STRICTNESS` は「speed_limit 名の厳しさ順」という
**数値の代理**であり、実際の数値（`registry.yaml`。measured 値が入ると変わりうる）と
順序が食い違うと `combine_speed_limits()` が実際より高い（緩い）上限を返し、
速度上限という安全機構が静かに無効化される。この関数だけ `th_state` の並びを
（呼び出し側が）引数として渡す必要があり、P-1「registry を知らない」の対称として
「`th_state` も知らない・引数だけで完結する」——`order` をハードコードしない。

各関数は `list[str]` を返す。**空リスト＝合格。**非空＝違反メッセージ（複数可）。
「拒否」か「警告に留める」かは呼び出し側（`export.py`）が判断する
（A1・A6 は params.md 上は「警告」、他の多くは「起動を拒否」だが、
違反の有無そのものはどの関数も同じ形——非空リスト——で表す）。

不変条件 P-1: **ファイルも環境変数も読まない。**registry を知らない・引数だけで完結する。
"""
from __future__ import annotations

from typing import Any, Iterable, Mapping, Sequence

# 自律走行に使う速度軸だけに A2a を課す（§4.1。極低速軸には課さない）
AUTONOMOUS_SPEED_AXES = ("v_max", "v_slow", "v_reverse")


def a1_intrusion_budget(v_max: float, timeout_ms: float, intrusion_budget_m: float,
                         label: str = "timeout") -> list[str]:
    """A1: v_max × (timeout_ms/1000) ≤ intrusion_budget_m。

    違反時の対処（params.md §4）は「v_max をクランプし、警告を出す。blocking 指定なら
    起動を拒否」——このクランプ自体は `export._apply_v_max_clamp()` が §3.3 の一度きりの
    `v_max` 引き下げ（P-5）として `resolve_registry()` の中で実施する（N-7）。
    したがって `resolved` から取り出した `v_max` / `timeout_ms` は通常クランプ済みであり、
    ここで違反が出るのはクランプが A5 と衝突して見送られた場合（N-7 選択肢b）。
    この関数自体は違反の有無だけを判定する。
    """
    intrusion = v_max * (timeout_ms / 1000.0)
    if intrusion > intrusion_budget_m:
        return [f"A1 違反({label}): v_max({v_max})×{label}({timeout_ms}ms)/1000={intrusion} "
                f"が intrusion_budget_m({intrusion_budget_m}) を超える"]
    return []


def a2a_floor_before_behavior(floor_m: float, stop_m: float, axis: str = "") -> list[str]:
    """A2a: obstacle_floor_distance_m < obstacle_stop_distance_m。

    **自律走行に使う軸だけに課す**（`v_max` / `v_slow` / `v_reverse`。§4.1）。
    `v_jog_panel` 等の極低速軸には課してはいけない——呼び出し側がその判断をする
    （`a2a_floor_before_behavior_axes` が `AUTONOMOUS_SPEED_AXES` で自動的に絞る）。
    """
    if not (floor_m < stop_m):
        suffix = f"（軸: {axis}）" if axis else ""
        return [f"A2a 違反{suffix}: obstacle_floor_distance_m({floor_m}) が "
                f"obstacle_stop_distance_m({stop_m}) 以上"]
    return []


def a2a_floor_before_behavior_axes(floor_m: float,
                                    stop_distance_by_axis: Mapping[str, float]) -> list[str]:
    """A2a をすべての自律走行軸（`AUTONOMOUS_SPEED_AXES`）について検査する（§1.4）。
    `v_jog_panel` / `v_check` / `v_calib` は意図的に対象外（§4.1）。
    """
    errors: list[str] = []
    for axis in AUTONOMOUS_SPEED_AXES:
        if axis not in stop_distance_by_axis:
            continue
        errors.extend(a2a_floor_before_behavior(floor_m, stop_distance_by_axis[axis], axis))
    return errors


def a2b_floor_before_follow(floor_m: float, follow_stop_m: float) -> list[str]:
    """A2b: obstacle_floor_distance_m < follow_stop_distance_m。
    （リミッタが追従対象の脚で発火しないための検査。FMEA①）"""
    if not (floor_m < follow_stop_m):
        return [f"A2b 違反: obstacle_floor_distance_m({floor_m}) が "
                f"follow_stop_distance_m({follow_stop_m}) 以上"]
    return []


def a3_tracker_lost_grace(tracker_lost_grace_ms: float, person_timeout_ms: float) -> list[str]:
    """A3: 0 < tracker_lost_grace_ms < person_timeout_ms"""
    if not (0 < tracker_lost_grace_ms < person_timeout_ms):
        return [f"A3 違反: tracker_lost_grace_ms({tracker_lost_grace_ms}) が "
                f"0 と person_timeout_ms({person_timeout_ms}) の間にない"]
    return []


def a4_jog_lease_vs_manual_joy(jog_lease_ms: float, manual_joy_timeout_s: float) -> list[str]:
    """A4: jog_lease_ms ≥ twist_mux の manual_joy timeout（秒→ms に換算して比較）"""
    manual_joy_timeout_ms = manual_joy_timeout_s * 1000.0
    if not (jog_lease_ms >= manual_joy_timeout_ms):
        return [f"A4 違反: jog_lease_ms({jog_lease_ms}) が "
                f"manual_joy_timeout({manual_joy_timeout_s}s={manual_joy_timeout_ms}ms) 未満"]
    return []


def a5_speed_ordering(v_reverse: float, v_slow: float, v_max: float,
                       v_jog_panel: float) -> list[str]:
    """A5: v_reverse ≤ v_slow ≤ v_max かつ v_jog_panel ≤ v_reverse"""
    errors: list[str] = []
    if not (v_reverse <= v_slow <= v_max):
        errors.append(f"A5 違反: v_reverse({v_reverse}) ≤ v_slow({v_slow}) ≤ v_max({v_max}) でない")
    if not (v_jog_panel <= v_reverse):
        errors.append(f"A5 違反: v_jog_panel({v_jog_panel}) ≤ v_reverse({v_reverse}) でない")
    return errors


def a6_esp32_timeout_vs_watchdog(esp32_timeout_ms: float, watchdog_ms: float) -> list[str]:
    """A6: esp32_timeout_ms > WATCHDOG_MS（ESP32 側 600 ms）。違反時は警告（拒否ではない）。"""
    if not (esp32_timeout_ms > watchdog_ms):
        return [f"A6 警告: esp32_timeout_ms({esp32_timeout_ms}) が "
                f"WATCHDOG_MS({watchdog_ms}) 以下"]
    return []


def a7_cmd_vel_stale_vs_watchdog(cmd_vel_stale_ms: float, watchdog_ms: float) -> list[str]:
    """A7: cmd_vel_stale_ms < WATCHDOG_MS（ESP32 より先に PC 側で止まること。FMEA②）"""
    if not (cmd_vel_stale_ms < watchdog_ms):
        return [f"A7 違反: cmd_vel_stale_ms({cmd_vel_stale_ms}) が "
                f"WATCHDOG_MS({watchdog_ms}) 以上（ESP32 より PC 側が先に止まらない）"]
    return []


def a8_blocking_placeholders(rows: Iterable[Mapping[str, Any]], stage: int,
                              nodes: Sequence[str] | None = None) -> list[str]:
    """A8: status:placeholder かつ blocking:true かつ blocking_from_stage ≤ stage かつ
    consumers が今回の launch（`nodes`）に含まれる行が 0 であること。

    `nodes` が None のときは絞り込みをしない（＝どのノードも起動対象とみなす。
    `--stage` の既定「最大値＝全部止める」と同じ思想で、安全側に倒す）。
    """
    errors: list[str] = []
    for row in rows:
        if row.get("status") != "placeholder":
            continue
        if not row.get("blocking"):
            continue
        blocking_from_stage = row.get("blocking_from_stage")
        if blocking_from_stage is None or blocking_from_stage > stage:
            continue
        consumers = row.get("consumers") or []
        if nodes is not None and not (set(consumers) & set(nodes)):
            continue
        errors.append(
            f"A8 違反: {row.get('name')} が placeholder のまま起動しようとしている"
            f"（blocking_from_stage={blocking_from_stage} ≤ stage={stage}, "
            f"consumers={consumers}）")
    return errors


def a9_consumers_nonempty(rows: Iterable[Mapping[str, Any]]) -> list[str]:
    """A9: 全パラメータの consumers が空でない（CI 専用。死んだパラメータの検出）。"""
    errors: list[str] = []
    for row in rows:
        if not row.get("consumers"):
            errors.append(f"A9 違反: {row.get('name')} の consumers が空")
    return errors


def a10_wheel_radius_scale(wheel_radius_scale: float, max_dev: float) -> list[str]:
    """A10: |wheel_radius_scale − 1| ≤ wheel_radius_scale_max_dev。
    10% を超えるずれは校正ではなく機械的な異常。"""
    if not (abs(wheel_radius_scale - 1.0) <= max_dev):
        return [f"A10 違反: |wheel_radius_scale({wheel_radius_scale}) − 1| が "
                f"wheel_radius_scale_max_dev({max_dev}) を超える"]
    return []


def a11_hysteresis_band(floor_m: float, hysteresis_band_m: float, stop_m: float) -> list[str]:
    """A11: obstacle_floor_distance_m + hysteresis_band_m < obstacle_stop_distance_m
    （ヒステリシス帯が d_behavior を越えない。P-4 の実装がこの式を満たすことの検査）"""
    if not (floor_m + hysteresis_band_m < stop_m):
        return [f"A11 違反: obstacle_floor_distance_m({floor_m}) + "
                f"hysteresis_band_m({hysteresis_band_m}) が "
                f"obstacle_stop_distance_m({stop_m}) 以上"]
    return []


def a13_speed_limit_strictness_order(order: Sequence[str],
                                      values_by_name: Mapping[str, float]) -> list[str]:
    """A13: `order`（`th_state.zones.LIMIT_STRICTNESS` を渡す想定）の並びが、
    `values_by_name` の解決済みの数値の昇順と一致すること。

    `order` は「左ほど厳しい（低速）」という順序の名前列。`values_by_name` は
    `order` に含まれる名前のうち、`resolve_registry()` で具体的な数値に解決できた
    （`placeholder` でない）ものだけを渡すこと——未測定の軸は比較から除外する
    （呼び出し側が省けばこの関数は自動的に飛ばす）。`"stop"` は registry に行を
    持たない特別な名前で、常に 0.0 として扱う（`values_by_name` に無くてよい）。

    違反時（登場順で見て値が前より小さい＝逆転している）は起動を拒否する
    （呼び出し側 `export.py` が `errors` に積む）。`LIMIT_STRICTNESS` は
    「speed_limit 名の厳しさ順」という数値の代理であり、ここが崩れると
    `combine_speed_limits()`（`th_state/zones.py`）が実際より高い上限を返しうる。
    """
    errors: list[str] = []
    prev_name: str | None = None
    prev_value: float | None = None
    for name in order:
        if name == "stop":
            value = 0.0
        elif name in values_by_name:
            value = values_by_name[name]
        else:
            continue  # placeholder（未測定）または registry に無い名前は比較から除外
        if prev_value is not None and value < prev_value:
            errors.append(
                f"A13 違反: LIMIT_STRICTNESS の並びで '{prev_name}'({prev_value}) の次に "
                f"'{name}'({value}) が来るが、数値の昇順になっていない"
                f"（'{name}' の方が値が小さい＝速いのに、並びでは '{prev_name}' より厳しい"
                "側に置かれていない）")
        else:
            prev_name, prev_value = name, value
    return errors
