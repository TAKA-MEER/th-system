"""export.py — registry.yaml からノード別 ROS2 パラメータ YAML を生成する（純粋関数＋薄い CLI）。

`DetailedDesign-params.md` §5。**生成（この CLI）と監査（`params_audit` ノード。WP-PARAM-02）を分ける。**
launch の `OpaqueFunction` からもここからも同じものを呼ぶ。

    python3 -m th_params.export --registry <path> --out <dir> --stage <n> [--sim] [--nodes a,b,c]

終了コード: 0=成功 / 1=アサーション違反（launch を止める） / 2=スキーマ違反。

不変条件: `resolve_registry()` / `build_outputs()` 等の計算部分は**引数だけで完結する純粋関数**。
ファイル I/O は `main()` に閉じ込める（P-1 は schema/derive/assertions だけの制約だが、
この CLI でも計算とファイル I/O を分けておくとテストしやすい）。
"""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
from typing import Any, Mapping

import yaml

from th_params import assertions, derive, schema
# A13（N-15）: th_state.zones.LIMIT_STRICTNESS の並びが registry.yaml の解決済みの
# 数値の昇順と一致することを検査するために必要（th_params/package.xml に
# <depend>th_state</depend> を追加してある）。
from th_state.zones import LIMIT_STRICTNESS

# ---------------------------------------------------------------------------
# 導出値の解決（registry.yaml の formula 名 → derive.py の純粋関数の組合せ）
# ---------------------------------------------------------------------------
#
# registry.yaml の `formula` は「合成式」の名前であることがある
# （例: `braking_distance_plus_margin` は `derive.braking_distance()` にマージンを
# 足した合成であり、`derive.py` の 12 関数のいずれとも 1:1 ではない。
# `DetailedDesign-params.md` §1.4・§6.4 が示す名前をそのまま使う）。

PLACEHOLDER = schema.TBD

# §3.3: timeout_lower_bound_ms の margin_ratio。v_max とは独立な固定係数。
# `_call_formula()`（timeout_from_bounds）と `_apply_v_max_clamp()` の両方が使うため
# ここに 1 か所だけ置く（値が 2 箇所に散らばって食い違うのを防ぐ）。
TIMEOUT_MARGIN_RATIO = 1.0


class ResolutionError(Exception):
    """formula 名が未知、または derived_from が循環しているときに送出する。"""


def _resolve_one(name: str, rows_by_name: Mapping[str, dict],
                  values: dict[str, tuple[str, Any]],
                  visiting: set[str]) -> tuple[str, Any]:
    """1 パラメータの (status, value) を再帰的に解決する。"""
    if name in values:
        return values[name]
    if name not in rows_by_name:
        raise ResolutionError(f"未知のパラメータ '{name}' が derived_from に現れた")
    if name in visiting:
        raise ResolutionError(f"derived_from が循環している: {name}")

    row = rows_by_name[name]
    status = row.get("status")

    if status in ("given", "measured"):
        result = (status, row.get("value"))
        values[name] = result
        return result

    if status == "placeholder":
        result = ("placeholder", PLACEHOLDER)
        values[name] = result
        return result

    if status != "derived":
        raise ResolutionError(f"未知の status '{status}' ({name})")

    visiting.add(name)
    derived_from = row.get("derived_from") or []
    dep_statuses = []
    deps: dict[str, Any] = {}
    for dep in derived_from:
        dep_status, dep_value = _resolve_one(dep, rows_by_name, values, visiting)
        dep_statuses.append(dep_status)
        deps[dep] = dep_value
    visiting.discard(name)

    out_status = schema.resolved_status(dep_statuses)
    if out_status == "placeholder":
        result = ("placeholder", PLACEHOLDER)
        values[name] = result
        return result

    value = _call_formula(row.get("formula"), row, deps, rows_by_name, values)
    result = ("derived", value)
    values[name] = result
    return result


def _call_formula(formula: str | None, row: dict, deps: dict[str, Any],
                   rows_by_name: Mapping[str, dict],
                   values: dict[str, tuple[str, Any]]) -> Any:
    """registry.yaml の `formula` 名を derive.py の呼び出しに変換する。"""
    name = row.get("name")
    value_by = row.get("value_by")

    if formula == "v_max_from_ceiling":
        return derive.v_max_from_ceiling(deps["drivetrain_ceiling_mps"], deps["v_max_headroom_ratio"])

    if formula == "speed_from_braking_distance":
        d_allow_name = [d for d in row["derived_from"] if d != "brake_accel_mps2" and d != "brake_delay_s"][0]
        return derive.speed_from_braking_distance(
            deps[d_allow_name], deps["brake_accel_mps2"], deps["brake_delay_s"])

    if formula == "floor_distance":
        return derive.floor_distance(deps["body_half_length_m"], deps["floor_margin_m"])

    if formula == "clear_distance":
        return derive.clear_distance(deps["body_half_length_m"], deps["clear_margin_m"])

    if formula == "hysteresis_band":
        return derive.hysteresis_band(deps["obstacle_floor_distance_m"], deps["hysteresis_ratio"])

    if formula == "deviation_budget_m":
        return derive.deviation_budget_m(deps["corridor_width_m"], deps["body_width_m"],
                                          deps["clear_margin_m"])

    if formula == "person_backstop_ms":
        # link_gap_p99_ms は value_by あり(esp32/lidar/ui)。person 側は safety_monitor 全体の
        # バックストップなので、3 系統のうち最大の p99 を使う（最も遅いリンクに合わせる）。
        grace_status, grace_value = values["tracker_lost_grace_ms"]
        p99_status, p99_value = values["link_gap_p99_ms"]
        if p99_status == "placeholder" or grace_status == "placeholder":
            raise ResolutionError("person_backstop_ms は placeholder 入力では呼ばれない")
        p99_worst = max(p99_value.values()) if isinstance(p99_value, dict) else p99_value
        return derive.person_backstop_ms(grace_value, p99_worst, 1.0)

    if formula == "timeout_from_bounds":
        # §3.3 手順1: 下限を採る。手順2（上限を超える場合に v_max を1回だけ下げる。P-5・N-7）は
        # `_apply_v_max_clamp()` が `resolve_registry()` の冒頭・この関数が呼ばれるより前に
        # 一度だけ済ませてあるため、ここで参照する `deps["v_max"]`（= values["v_max"]）は
        # 既にクランプ後の値になっている。したがってここで改めて upper と比較する必要はない
        # （クランプが A5 と衝突して見送られた場合は v_max が自然値のまま残り、timeout が
        # upper を超えたままになるが、それは意図どおり——呼び出し側の A1 チェックが
        # 起動を拒否する。N-7 選択肢b。DetailedDesign-open.md 参照）。
        p99_status, p99_value = values["link_gap_p99_ms"]
        if p99_status == "placeholder":
            raise ResolutionError("timeout_from_bounds は placeholder 入力では呼ばれない")
        axis = "lidar" if name == "lidar_timeout_ms" else "esp32"
        p99_for_axis = p99_value[axis] if isinstance(p99_value, dict) else p99_value
        return derive.timeout_lower_bound_ms(p99_for_axis, TIMEOUT_MARGIN_RATIO)

    if formula == "braking_distance_plus_margin":
        margin_name = [d for d in row["derived_from"]
                       if d not in ("brake_accel_mps2", "brake_delay_s")][0]
        a = deps["brake_accel_mps2"]
        t_delay = deps["brake_delay_s"]
        margin = deps[margin_name]
        if value_by:
            result = {}
            for axis in value_by:
                axis_status, axis_value = _resolve_one(axis, rows_by_name, values, set())
                if axis_status == "placeholder":
                    # 個別の軸だけ placeholder のときは None にする（sentinel 文字列を
                    # 生成物へ埋め込まない。行全体の status は derived のままでも軸ごとに
                    # None が混じりうる — P-2 を生成 YAML にも及ぼすための措置）。
                    result[axis] = None
                else:
                    result[axis] = derive.braking_distance(axis_value, a, t_delay) + margin
            return result
        v_axis_status, v_axis_value = _resolve_one("v_max", rows_by_name, values, set())
        if v_axis_status == "placeholder":
            raise ResolutionError("braking_distance_plus_margin(scalar) は v_max が placeholder")
        return derive.braking_distance(v_axis_value, a, t_delay) + margin

    raise ResolutionError(f"未知の formula '{formula}' ({name})")


def _apply_v_max_clamp(rows_by_name: Mapping[str, dict], values: dict[str, tuple[str, Any]],
                        clamp_warnings: list[str] | None, clamp_errors: list[str] | None) -> None:
    """A1 のクランプ（`DetailedDesign-params.md` §3.3 手順2・§4 A1・N-7）。

    `resolve_registry()` の一般ループより**前に**呼ぶこと。`v_max` を使う経路は
    `derived_from` だけではない——`lidar_timeout_ms` / `esp32_timeout_ms` の
    `derived_from` には確かに `v_max` があるが、`obstacle_stop_distance_m`（`value_by`
    に `v_max` を持つ軸の解決経由）や `follow_stop_distance_m`（`value_by` を持たない
    スカラー分岐で `_call_formula()` が `_resolve_one("v_max", ...)` を直接呼ぶ経由。
    `export.py` の `braking_distance_plus_margin` 参照）は `derived_from` に `v_max` を
    **持たないまま** v_max を使う（`derived_from: [v_max]` を grep しても
    `lidar_timeout_ms` / `esp32_timeout_ms` の 2 行しか出ないのはそのため——
    「この配置は過剰では」と早合点しないこと）。`v_slow` / `v_reverse` / `v_jog_panel` は
    そもそも v_max に依存しない（この関数の下方、A5 適合性チェックのコメント参照）。
    クランプ後の `v_max` を `values["v_max"]` に上書きしてから一般ループへ入ることで、
    上記のどちらの経路であっても**新しい v_max で 1 度だけ再計算される**
    （`_resolve_one` は `values` に既にあれば再計算しない memoize 方式なので、
    事前に確定させておく必要がある。P-5: 不動点反復にしない）。

    N-7（`DetailedDesign-open.md`）: クランプは A5（`v_reverse ≤ v_slow ≤ v_max`）と衝突しうる。
    ここでは**選択肢(b)**を採る——クランプ後の `v_max` が `v_slow` を下回るなら、
    クランプを適用せず `v_max` を自然値のまま残す。この場合 intrusion budget 超過が
    解消されないため、後段の `run_assertions()` の A1 チェックが改めて違反を検出し
    起動を拒否する（`clamp_errors` にも衝突の理由を明示するメッセージを積む）。
    速度体系全体を比例縮小する選択肢(a)は採らない——registry に書いた値と実際に
    動く値が乖離し追跡できなくなるため。A1 を満たさないまま起動させる選択肢(c)も
    採らない——A1 の存在意義（危険な組合せを構成不能にする）に反するため。
    """
    if "v_max" not in rows_by_name or "intrusion_budget_m" not in rows_by_name \
            or "link_gap_p99_ms" not in rows_by_name:
        return

    try:
        v_max_status, v_max_natural = _resolve_one("v_max", rows_by_name, values, set())
        budget_status, budget = _resolve_one("intrusion_budget_m", rows_by_name, values, set())
        p99_status, p99_value = _resolve_one("link_gap_p99_ms", rows_by_name, values, set())
    except ResolutionError:
        return
    if "placeholder" in (v_max_status, budget_status, p99_status):
        return

    needed_v_max_by_axis: dict[str, float] = {}
    for axis, timeout_name in (("lidar", "lidar_timeout_ms"), ("esp32", "esp32_timeout_ms")):
        if timeout_name not in rows_by_name:
            continue
        p99_for_axis = p99_value[axis] if isinstance(p99_value, dict) else p99_value
        lower = derive.timeout_lower_bound_ms(p99_for_axis, TIMEOUT_MARGIN_RATIO)
        upper = derive.timeout_upper_bound_ms(v_max_natural, budget)
        if lower > upper:
            needed_v_max_by_axis[timeout_name] = derive.v_max_from_intrusion_budget(budget, lower)

    if not needed_v_max_by_axis:
        return  # A1 は違反しない。クランプ不要

    v_max_needed = min(needed_v_max_by_axis.values())  # 最も厳しい軸に合わせる
    axes_desc = ", ".join(sorted(needed_v_max_by_axis))

    # A5 適合性チェック: v_slow / v_reverse / v_jog_panel は v_max に依存しないので
    # （derived_from に v_max を持たない）ここで先に解決しても循環にならない。
    conflicting = []
    for name in ("v_slow", "v_reverse", "v_jog_panel"):
        if name not in rows_by_name:
            continue
        try:
            status, value = _resolve_one(name, rows_by_name, values, set())
        except ResolutionError:
            continue
        if status == "placeholder":
            continue
        if value > v_max_needed:
            conflicting.append(f"{name}({value})")

    if conflicting:
        msg = (f"N-7: A1 のクランプ判定で v_max を {v_max_natural} → {v_max_needed} へ"
               f"下げる必要があった（{axes_desc} が intrusion_budget_m={budget} を超過）が、"
               f"{', '.join(conflicting)} を下回り A5 に違反するためクランプを適用しない。"
               "安全な速度が存在しない構成として v_max は自然値のまま残し、"
               "intrusion budget 超過を A1 の起動拒否に委ねる"
               "（N-7 選択肢b。DetailedDesign-open.md 参照）。")
        if clamp_errors is not None:
            clamp_errors.append(msg)
        return

    values["v_max"] = ("derived", v_max_needed)
    msg = (f"N-7: A1 のクランプを適用した。v_max を {v_max_natural} → {v_max_needed} へ下げた"
           f"（{axes_desc} が intrusion_budget_m={budget} を超過したため）。"
           "v_max に依存する派生値はこの新しい v_max で 1 度だけ再計算する（P-5）。")
    if clamp_warnings is not None:
        clamp_warnings.append(msg)


def resolve_registry(rows: list[dict], clamp_warnings: list[str] | None = None,
                      clamp_errors: list[str] | None = None,
                      apply_v_max_clamp: bool = True) -> dict[str, tuple[str, Any]]:
    """registry の全行を解決し、{name: (status, value)} を返す。

    値が正しく計算できない行（placeholder が伝播した行）は ('placeholder', schema.TBD) 相当。

    `apply_v_max_clamp`（既定 True）: False にすると A1 のクランプ（`_apply_v_max_clamp`）を
    一切行わない。`--sim` では A1〜A11・A13 が全てスキップされる既存挙動（`main()` 参照）に
    合わせ、呼び出し側が `--sim` のときに False を渡す。

    `clamp_warnings` / `clamp_errors`: クランプが実際に何をしたか（適用した／A5 と衝突して
    見送った）を追記する出力用リスト。呼び出し側は `run_assertions()` に同じリストを渡すと
    最終的な `(errors, warnings)` にそのまま合流する。
    """
    rows_by_name = {row["name"]: row for row in rows}
    values: dict[str, tuple[str, Any]] = {}
    if apply_v_max_clamp:
        _apply_v_max_clamp(rows_by_name, values, clamp_warnings, clamp_errors)
    for name in rows_by_name:
        try:
            _resolve_one(name, rows_by_name, values, set())
        except ResolutionError:
            values[name] = ("placeholder", PLACEHOLDER)
    return values


# ---------------------------------------------------------------------------
# 出力（ノード別 YAML ＋ twist_mux.yaml）
# ---------------------------------------------------------------------------


def _jsonable(value: Any) -> Any:
    """YAML に書けない型（tuple 等）を素直な型に変換する。"""
    if isinstance(value, tuple):
        return list(value)
    return value


def build_node_outputs(rows: list[dict], resolved: Mapping[str, tuple[str, Any]]
                        ) -> dict[str, dict]:
    """consumers ごとにノード別 YAML の中身を組み立てる。twist_mux も consumers の 1 つとして扱う。

    `status: placeholder` の値は sentinel 文字列（`schema.TBD`）をそのまま書かず `null` にする。
    P-2（sentinel は registry.yaml にしか現れない）を生成物にも及ぼすための措置。
    実際に placeholder のまま起動できてしまうかどうかは A8（起動時アサーション）が別途守る。
    """
    outputs: dict[str, dict] = {}
    for row in rows:
        name = row["name"]
        status, value = resolved.get(name, (row.get("status"), row.get("value")))
        out_value = None if status == "placeholder" else _jsonable(value)
        for consumer in row.get("consumers") or []:
            node_params = outputs.setdefault(consumer, {})
            node_params[name] = out_value
    return outputs


def compute_digest(resolved: Mapping[str, tuple[str, Any]]) -> str:
    """params_digest（§5.1）: sha1(sorted(f"{name}={status}:{value}"))[:12]"""
    parts = sorted(f"{name}={status}:{value}" for name, (status, value) in resolved.items())
    joined = "\n".join(parts).encode("utf-8")
    return hashlib.sha1(joined).hexdigest()[:12]


def run_assertions(rows: list[dict], resolved: Mapping[str, tuple[str, Any]],
                    stage: int, nodes: list[str] | None,
                    include_a8: bool = True,
                    clamp_warnings: list[str] | None = None,
                    clamp_errors: list[str] | None = None) -> tuple[list[str], list[str]]:
    """A1〜A11・A13 のうち registry から機械的に評価できるものを回す
    （A12 は未実装。A9 は CI 専用でここでは呼ばない）。

    戻り値は `(errors, warnings)`。**どのアサーションが起動を拒否（errors）で、
    どれが警告に留まる（warnings）かは `DetailedDesign-params.md` §4 のアサーション表が
    根拠。**現時点で警告扱いなのは A6 と、A1 のクランプが実際に適用された場合（N-7）。

    A1 自体は `resolved` に入っている `v_max` / `*_timeout_ms` を検査するだけであり、
    それらは `resolve_registry()` が `_apply_v_max_clamp()` で**既にクランプ後の値**に
    差し替え済みのはず——クランプが成功していれば intrusion budget を満たすため、
    ここでの A1 チェックは通常もう違反しない。クランプが A5 と衝突して見送られた場合
    （N-7 選択肢b。`DetailedDesign-open.md` 参照）は `v_max` が自然値のまま残るため、
    この A1 チェックが改めて違反を検出し起動を拒否する。

    `clamp_warnings` / `clamp_errors`: `resolve_registry()` に渡したのと同じリストを
    ここにも渡すと、クランプが実際に何をしたか（適用した／A5 衝突で見送った）の
    メッセージがそのまま最終的な `(errors, warnings)` に合流する（新しい仕組みを
    作らず、既存の warnings 経路に載せる）。

    `include_a8`（既定 True）: False にすると A8（blocking placeholder の検査）を
    呼ばない。A8 は「未測定の値を抱えたまま**起動**させない」ための完全性の門で、
    launch（`params_generation.run_generation()` 経由）の責務。一方 `/params/set` は
    「これから当てようとしている値そのものが妥当か」（A1〜A7・A10・A11 の物理的整合性）
    だけを見ればよく、registry 全体に残る**無関係な**未測定値（例:
    `link_gap_p99_ms` のような `blocking_from_stage` 付きの placeholder）で
    毎回拒否されるのはおかしい。呼び出し側（`params_audit._cb_set`）は
    `include_a8=False` で呼ぶ。
    """
    errors: list[str] = []
    warnings: list[str] = []
    rows_by_name = {row["name"]: row for row in rows}

    def val(name: str) -> Any:
        return resolved[name][1]

    def is_placeholder(name: str) -> bool:
        # D1-a: 名前が resolved に無い（＝registry に存在しない名前）場合も
        # placeholder 扱いにする。呼び出し側は全て `if not is_placeholder(x):` の
        # 形で使っているので、これは「その名前が無ければ、それに依存するアサーションは
        # 飛ばす」という意味になる。名前の綴り違いそのものの検出は A10 / CI 側
        # （schema.validate_registry 等）が受け持つ範囲であり、ここで KeyError を
        # 投げてサービスコールバックごと落とすべきではない。
        return name not in resolved or resolved[name][0] == "placeholder"

    # A8: blocking placeholder（stage / nodes を考慮）。sim では呼ばない（呼び出し側で制御）。
    if include_a8:
        errors.extend(assertions.a8_blocking_placeholders(rows, stage, nodes))

    # A2a / A2b / A11: obstacle_floor_distance_m と各種距離の関係
    if not is_placeholder("obstacle_floor_distance_m"):
        floor_m = val("obstacle_floor_distance_m")
        if not is_placeholder("obstacle_stop_distance_m"):
            stop_by_axis = val("obstacle_stop_distance_m")
            concrete_axes = {axis: d for axis, d in stop_by_axis.items() if d is not None}
            errors.extend(assertions.a2a_floor_before_behavior_axes(floor_m, concrete_axes))
            if not is_placeholder("hysteresis_band_m"):
                hysteresis_m = val("hysteresis_band_m")
                for axis in assertions.AUTONOMOUS_SPEED_AXES:
                    if axis in concrete_axes:
                        errors.extend(assertions.a11_hysteresis_band(
                            floor_m, hysteresis_m, concrete_axes[axis]))
        if not is_placeholder("follow_stop_distance_m"):
            errors.extend(assertions.a2b_floor_before_follow(floor_m, val("follow_stop_distance_m")))

    # A3
    if not is_placeholder("tracker_lost_grace_ms") and not is_placeholder("person_timeout_ms"):
        errors.extend(assertions.a3_tracker_lost_grace(
            val("tracker_lost_grace_ms"), val("person_timeout_ms")))

    # A4
    if "jog_lease_ms" in rows_by_name and "manual_joy_timeout" in rows_by_name:
        errors.extend(assertions.a4_jog_lease_vs_manual_joy(
            val("jog_lease_ms"), val("manual_joy_timeout")))

    # A5
    if not any(is_placeholder(n) for n in ("v_reverse", "v_slow", "v_max", "v_jog_panel")):
        errors.extend(assertions.a5_speed_ordering(
            val("v_reverse"), val("v_slow"), val("v_max"), val("v_jog_panel")))

    # A6（警告。params.md §4）/ A7（拒否）
    if "esp32_watchdog_ms" in rows_by_name:
        watchdog_ms = val("esp32_watchdog_ms")
        if not is_placeholder("esp32_timeout_ms"):
            warnings.extend(assertions.a6_esp32_timeout_vs_watchdog(val("esp32_timeout_ms"), watchdog_ms))
        if "cmd_vel_stale_ms" in rows_by_name:
            errors.extend(assertions.a7_cmd_vel_stale_vs_watchdog(val("cmd_vel_stale_ms"), watchdog_ms))

    # A10
    if "wheel_radius_scale" in rows_by_name and "wheel_radius_scale_max_dev" in rows_by_name:
        errors.extend(assertions.a10_wheel_radius_scale(
            val("wheel_radius_scale"), val("wheel_radius_scale_max_dev")))

    # A1: v_max と lidar/esp32 timeout の intrusion_budget チェック
    if not is_placeholder("v_max") and not is_placeholder("intrusion_budget_m"):
        budget = val("intrusion_budget_m")
        for timeout_name in ("lidar_timeout_ms", "esp32_timeout_ms"):
            if timeout_name in rows_by_name and not is_placeholder(timeout_name):
                errors.extend(assertions.a1_intrusion_budget(
                    val("v_max"), val(timeout_name), budget, label=timeout_name))

    # A13（N-15）: th_state.zones.LIMIT_STRICTNESS の並びが registry.yaml の解決済みの
    # 数値の昇順と一致していること。未測定（placeholder）の軸は比較から除外する。
    # "stop" は registry に行が無いので values_by_name には含めない
    # （a13_speed_limit_strictness_order() 側で 0.0 として扱う）。
    strictness_values = {
        name: val(name) for name in LIMIT_STRICTNESS
        if name != "stop" and name in rows_by_name and not is_placeholder(name)
    }
    errors.extend(assertions.a13_speed_limit_strictness_order(LIMIT_STRICTNESS, strictness_values))

    # A1 クランプ（N-7）: resolve_registry() 側で集めたメッセージをそのまま合流させる。
    # 適用できた（警告）／A5 衝突で見送った（起動拒否の理由を明示する追加エラー）の
    # どちらも、新しい仕組みを作らず既存の (errors, warnings) 経路に載せる。
    if clamp_warnings:
        warnings.extend(clamp_warnings)
    if clamp_errors:
        errors.extend(clamp_errors)

    return errors, warnings


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="th_params.registry から生成物を書き出す")
    parser.add_argument("--registry", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--stage", type=int, default=8, help="A8 判定用の現在段階。既定は最大値")
    parser.add_argument("--sim", action="store_true", help="allow_placeholder: true")
    parser.add_argument("--nodes", default=None, help="今回の launch で起動するノード（カンマ区切り）")
    args = parser.parse_args(argv)

    with open(args.registry, encoding="utf-8") as f:
        rows = yaml.safe_load(f)

    schema_errors = schema.validate_registry(rows)
    if schema_errors:
        for e in schema_errors:
            print(e, file=sys.stderr)
        return 2

    # A1 のクランプ（N-7）は --sim では適用しない。A1〜A11・A13 が全てスキップされる
    # 既存挙動（このブロックの if not args.sim: と同じ思想）を壊さないため。
    clamp_warnings: list[str] = []
    clamp_errors: list[str] = []
    resolved = resolve_registry(rows, clamp_warnings=clamp_warnings, clamp_errors=clamp_errors,
                                 apply_v_max_clamp=not args.sim)

    nodes = args.nodes.split(",") if args.nodes else None
    if not args.sim:
        errors, warnings = run_assertions(rows, resolved, args.stage, nodes,
                                           clamp_warnings=clamp_warnings, clamp_errors=clamp_errors)
        # 警告（A6・クランプ適用時）は stderr に出すが、終了コードには影響させない（生成は続行する）。
        for w in warnings:
            print(w, file=sys.stderr)
        if errors:
            for e in errors:
                print(e, file=sys.stderr)
            return 1

    import os
    os.makedirs(args.out, exist_ok=True)

    node_outputs = build_node_outputs(rows, resolved)
    for node_name, node_params in node_outputs.items():
        out_path = os.path.join(args.out, f"{node_name}.yaml")
        with open(out_path, "w", encoding="utf-8") as f:
            yaml.safe_dump({node_name: {"ros__parameters": node_params}}, f,
                            allow_unicode=True, sort_keys=True)

    digest = compute_digest(resolved)
    with open(os.path.join(args.out, "params_digest.json"), "w", encoding="utf-8") as f:
        json.dump({"params_digest": digest}, f)

    return 0


if __name__ == "__main__":
    sys.exit(main())
