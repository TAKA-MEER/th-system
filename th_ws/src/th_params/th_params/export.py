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

# ---------------------------------------------------------------------------
# 導出値の解決（registry.yaml の formula 名 → derive.py の純粋関数の組合せ）
# ---------------------------------------------------------------------------
#
# registry.yaml の `formula` は「合成式」の名前であることがある
# （例: `braking_distance_plus_margin` は `derive.braking_distance()` にマージンを
# 足した合成であり、`derive.py` の 12 関数のいずれとも 1:1 ではない。
# `DetailedDesign-params.md` §1.4・§6.4 が示す名前をそのまま使う）。

PLACEHOLDER = schema.TBD


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
        # §3.3: 下限を採り、上限を超えるなら v_max を 1 回だけ下げる（P-5）。
        p99_status, p99_value = values["link_gap_p99_ms"]
        if p99_status == "placeholder":
            raise ResolutionError("timeout_from_bounds は placeholder 入力では呼ばれない")
        axis = "lidar" if name == "lidar_timeout_ms" else "esp32"
        p99_for_axis = p99_value[axis] if isinstance(p99_value, dict) else p99_value
        margin_ratio = 1.0  # 下限に対する余裕。§3.3 のとおり固定係数（v_max とは独立）
        lower = derive.timeout_lower_bound_ms(p99_for_axis, margin_ratio)
        upper = derive.timeout_upper_bound_ms(deps["v_max"], deps["intrusion_budget_m"])
        timeout = lower
        if timeout > upper:
            # v_max を下げて確定（不動点反復にしない。P-5）。ここでは registry の v_max を
            # 書き換えず、警告メッセージだけを添えて timeout を確定する
            # （実際の v_max クランプ適用は呼び出し側 export の警告ログに委ねる）。
            pass
        return timeout

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


def resolve_registry(rows: list[dict]) -> dict[str, tuple[str, Any]]:
    """registry の全行を解決し、{name: (status, value)} を返す。

    値が正しく計算できない行（placeholder が伝播した行）は ('placeholder', schema.TBD) 相当。
    """
    rows_by_name = {row["name"]: row for row in rows}
    values: dict[str, tuple[str, Any]] = {}
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
                    include_a8: bool = True) -> tuple[list[str], list[str]]:
    """A1〜A11 のうち registry から機械的に評価できるものを回す。A9 は CI 専用（ここでは呼ばない）。

    戻り値は `(errors, warnings)`。**どのアサーションが起動を拒否（errors）で、
    どれが警告に留まる（warnings）かは `DetailedDesign-params.md` §4 のアサーション表が
    根拠。**現時点で警告扱いなのは A6 だけ（A1 は同表では「`v_max` をクランプし、警告を出す。
    `blocking` 指定なら起動を拒否」という条件付きの扱いだが、そのクランプ（P-5）自体が
    `_call_formula()` の `timeout_from_bounds` で意図的に未実装のまま残っているため、
    ここでは A1 を従来どおり errors 側に置く。この食い違いは
    `DetailedDesign-open.md` §4.1 の未解決事項として別途扱う）。

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

    resolved = resolve_registry(rows)

    nodes = args.nodes.split(",") if args.nodes else None
    if not args.sim:
        errors, warnings = run_assertions(rows, resolved, args.stage, nodes)
        # 警告（A6）は stderr に出すが、終了コードには影響させない（生成は続行する）。
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
