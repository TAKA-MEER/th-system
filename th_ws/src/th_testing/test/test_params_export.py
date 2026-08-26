"""test_params_export.py — th_params/export.py を検証する（WP-PARAM-01 §7）。"""
import pytest

from th_params import derive, export, schema


def test_twist_mux_generated(registry_rows):
    """twist_mux.yaml が生成対象（consumers）に入っていること。"""
    resolved = export.resolve_registry(registry_rows)
    outputs = export.build_node_outputs(registry_rows, resolved)
    assert "twist_mux" in outputs
    assert "manual_joy_timeout" in outputs["twist_mux"]


def test_digest_stable(registry_rows):
    """同じ registry から同じ params_digest が得られる。"""
    resolved1 = export.resolve_registry(registry_rows)
    resolved2 = export.resolve_registry(registry_rows)
    digest1 = export.compute_digest(resolved1)
    digest2 = export.compute_digest(resolved2)
    assert digest1 == digest2
    assert len(digest1) == 12


def test_digest_changes_when_a_value_changes(registry_rows):
    """registry の値が変われば digest も変わる（provenance として機能する）。"""
    import copy
    rows_copy = copy.deepcopy(registry_rows)
    resolved_before = export.resolve_registry(rows_copy)
    digest_before = export.compute_digest(resolved_before)

    for row in rows_copy:
        if row["name"] == "brake_delay_s":
            row["value"] = row["value"] + 0.5
            break
    resolved_after = export.resolve_registry(rows_copy)
    digest_after = export.compute_digest(resolved_after)
    assert digest_before != digest_after


def test_resolve_registry_propagates_placeholder(registry_rows):
    """S5: 依存元が placeholder のあいだ、依存する行（v_slow / obstacle_stop_distance_m）も
    placeholder に伝播すること。

    registry の現在の status に直接束縛すると、実測が進むたびにこのテストが落ちる
    （brake_accel_mps2 は 2026-08-21 に実測されて measured になった）。検証したいのは
    S5 の伝播機構そのものであって「brake_accel_mps2 が今 placeholder であること」では
    ないので、実物の formula グラフ（derived_from の連鎖）は使いつつ、起点だけを
    deepcopy 上で意図的に placeholder に落として伝播を確認する。
    """
    import copy
    rows_copy = copy.deepcopy(registry_rows)
    rows_by_name = {row["name"]: row for row in rows_copy}
    origin = rows_by_name["brake_accel_mps2"]
    origin["status"] = "placeholder"
    origin["value"] = schema.TBD
    origin["blocking"] = True  # class:c は S1 により status!=measured で blocking:true が必須

    resolved = export.resolve_registry(rows_copy)
    assert resolved["brake_accel_mps2"][0] == "placeholder"
    assert resolved["v_slow"][0] == "placeholder"
    assert resolved["obstacle_stop_distance_m"][0] == "placeholder"


def test_intrusion_budget_m_is_placeholder_and_propagates_safely(registry_rows):
    """N-2（DetailedDesign-open.md §4.1）: intrusion_budget_m は現状 placeholder だが、
    値が決まった後も「起点が placeholder のとき A1 がガードされる」という検証意図は
    生き続ける。registry の現在の status に直接束縛すると、値が給値された時点で
    このテストが落ちてしまう（先回りして書き換える理由）ため、実物の formula グラフは
    使いつつ、起点だけを deepcopy 上で意図的に placeholder に落として確認する。

    S5 により lidar_timeout_ms / esp32_timeout_ms も placeholder に伝播するが、
    それが正しい振る舞い（伝播して困るなら値を決めるべき、というのが設計の意図）。
    伝播先の解決・生成が例外を投げずに完走することを確認する。
    """
    import copy
    rows_copy = copy.deepcopy(registry_rows)
    rows_by_name = {row["name"]: row for row in rows_copy}
    origin = rows_by_name["intrusion_budget_m"]
    origin["status"] = "placeholder"
    origin["value"] = schema.TBD
    origin["blocking"] = True

    resolved = export.resolve_registry(rows_copy)
    assert resolved["intrusion_budget_m"][0] == "placeholder"
    assert resolved["lidar_timeout_ms"][0] == "placeholder"
    assert resolved["esp32_timeout_ms"][0] == "placeholder"

    # 生成物が例外なく組み上がり、sentinel も漏れない
    outputs = export.build_node_outputs(rows_copy, resolved)
    assert outputs["safety_monitor"]["intrusion_budget_m"] is None
    assert outputs["safety_monitor"]["lidar_timeout_ms"] is None
    assert outputs["safety_monitor"]["esp32_timeout_ms"] is None

    # A1（intrusion_budget_m を使う起動時アサーション）は placeholder 入力ではガードされ、
    # 例外を投げずに単に評価をスキップすること（export.run_assertions 経由）
    errors, warnings = export.run_assertions(rows_copy, resolved, stage=0, nodes=None)
    assert not any("A1" in e for e in errors)


def test_resolve_registry_computes_speed_independent_values(registry_rows):
    """obstacle_floor_distance_m は brake_accel_mps2 に依存しないので、
    brake が未測定でも計算できる（safety.md §3.1 の設計意図）。"""
    resolved = export.resolve_registry(registry_rows)
    status, value = resolved["obstacle_floor_distance_m"]
    assert status == "derived"
    assert value == resolved_floor_value(registry_rows)


def resolved_floor_value(registry_rows):
    rows_by_name = {r["name"]: r for r in registry_rows}
    return rows_by_name["body_half_length_m"]["value"] + rows_by_name["floor_margin_m"]["value"]


def test_resolve_registry_value_by_axes_fully_given():
    """value_by を持つ formula が、依存が全部揃った場合に軸ごとの辞書を返すこと
    （実際の registry では brake_accel_mps2 が常に placeholder のため、この経路は
    ここで合成 registry を使って検証する）。"""
    rows = [
        {"name": "brake_accel_mps2", "class": "b", "status": "given", "value": 1.0,
         "consumers": ["x"], "spec_ref": "t"},
        {"name": "brake_delay_s", "class": "b", "status": "given", "value": 0.1,
         "consumers": ["x"], "spec_ref": "t"},
        {"name": "safety_margin_m", "class": "b", "status": "given", "value": 0.2,
         "consumers": ["x"], "spec_ref": "t"},
        {"name": "v_max", "class": "b", "status": "given", "value": 1.0,
         "consumers": ["x"], "spec_ref": "t"},
        {"name": "v_slow", "class": "a", "status": "placeholder", "value": schema.TBD,
         "consumers": ["x"], "spec_ref": "t"},
        {"name": "obstacle_stop_distance_m", "class": "b", "status": "derived", "value": None,
         "value_by": ["v_max", "v_slow"],
         "derived_from": ["brake_accel_mps2", "brake_delay_s", "safety_margin_m"],
         "formula": "braking_distance_plus_margin", "consumers": ["x"], "spec_ref": "t"},
    ]
    resolved = export.resolve_registry(rows)
    status, value = resolved["obstacle_stop_distance_m"]
    assert status == "derived"
    assert isinstance(value, dict)
    assert value["v_max"] is not None
    assert value["v_slow"] is None  # v_slow は placeholder なので None（sentinel は書かない）


def test_build_node_outputs_never_leaks_tbd_measure_sentinel(registry_rows):
    """P-2: 生成物に sentinel 文字列がそのまま出ないこと（None に変換される）。"""
    import json
    resolved = export.resolve_registry(registry_rows)
    outputs = export.build_node_outputs(registry_rows, resolved)
    dumped = json.dumps(outputs)
    assert schema.TBD not in dumped


def _registry_with_esp32_timeout_below_watchdog(registry_rows):
    """`esp32_timeout_ms` が `esp32_watchdog_ms` 以下になり、かつ **A1 は違反しない**
    registry のコピーを作る（A6 だけを単独で違反させたい。§4 の表では A6 は「警告」・
    A1 は「拒否」なので、両方を同時に踏むと A6 が errors を空にできることを検証できない）。
    class:b の行なので `status: given` で直接値を置いても S1〜S5（schema.py）に違反しない。

    v_max / intrusion_budget_m が given/derived 済み（placeholder でない）場合は
    A1 の上限（budget/v_max*1000）も踏まえて、その内側で watchdog 未満の値を選ぶ。
    どちらかが placeholder のままなら A1 はそもそもガードされるので、watchdog 未満
    であることだけを条件にする。
    """
    import copy
    rows_copy = copy.deepcopy(registry_rows)
    rows_by_name = {row["name"]: row for row in rows_copy}
    watchdog_ms = rows_by_name["esp32_watchdog_ms"]["value"]

    resolved_before = export.resolve_registry(rows_copy)
    v_max_status, v_max_value = resolved_before["v_max"]
    budget_status, budget_value = resolved_before["intrusion_budget_m"]

    timeout_ms = watchdog_ms - 50
    if v_max_status != "placeholder" and budget_status != "placeholder":
        a1_upper_ms = budget_value / v_max_value * 1000.0
        timeout_ms = min(timeout_ms, a1_upper_ms - 10)
    assert timeout_ms > 0, "test setup: 選んだ esp32_timeout_ms が非正になった"

    timeout_row = rows_by_name["esp32_timeout_ms"]
    timeout_row["status"] = "given"
    timeout_row["value"] = timeout_ms
    return rows_copy, watchdog_ms, timeout_ms


def test_a6_esp32_timeout_below_watchdog_is_warning_not_error(registry_rows):
    """`DetailedDesign-params.md` §4: A6 は違反時「警告」であり、他のアサーション
    （多くは「起動を拒否」）と違って errors に混ざってはいけない。
    esp32_timeout_ms が WATCHDOG_MS 以下でも run_assertions の errors は空、
    warnings に A6 のメッセージが入ること。"""
    rows_copy, watchdog_ms, timeout_ms = _registry_with_esp32_timeout_below_watchdog(registry_rows)
    resolved = export.resolve_registry(rows_copy)
    assert resolved["esp32_timeout_ms"] == ("given", timeout_ms)
    assert timeout_ms <= watchdog_ms

    errors, warnings = export.run_assertions(rows_copy, resolved, stage=0, nodes=None)
    assert not any("A6" in e for e in errors)
    assert any("A6" in w for w in warnings)


def test_main_completes_with_zero_exit_when_a6_violated(registry_rows, tmp_path):
    """A6 が警告に留まる結果として、`export.main()`（launch から呼ばれる CLI 本体）が
    GenerationError 相当の失敗にならず、終了コード 0 で生成物を書き切ること
    （このアサーションが errors に混ざっていた旧実装では、良好なリンクほど
    esp32_timeout_ms が縮み A6 が発火し、実機の bringup が起動できなくなっていた）。"""
    import yaml
    rows_copy, _watchdog_ms, _timeout_ms = _registry_with_esp32_timeout_below_watchdog(registry_rows)

    registry_path = tmp_path / "registry.yaml"
    with open(registry_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(rows_copy, f, allow_unicode=True)

    out_dir = tmp_path / "out"
    rc = export.main(["--registry", str(registry_path), "--out", str(out_dir), "--stage", "0"])
    assert rc == 0
    assert (out_dir / "safety_monitor.yaml").exists()
    assert (out_dir / "params_digest.json").exists()


# ---------------------------------------------------------------------------
# N-7（DetailedDesign-open.md）: A1 の v_max クランプ（`_apply_v_max_clamp`）
# ---------------------------------------------------------------------------
#
# 現行 registry.yaml では drivetrain_ceiling_mps が小さく（v_max ≈ 1.12 m/s）、
# クランプは発火しない（発火させるには v_max が概ね 1.55 m/s を超える必要がある）。
# 以下のテストは drivetrain_ceiling_mps だけを人工的に底上げして発火させる。


def test_a1_clamp_applies_and_recomputes_v_max_dependents_once(registry_rows):
    """クランプが発火する場合: (1) v_max が期待どおり下がる (2) 警告が出る
    (3) v_max に依存する派生値（follow_stop_distance_m 等）が新しい v_max で
    再計算される (4) 1 度しか発火しない（反復しない。P-5）。

    venue_clearance_m 等は現行 registry で placeholder のままなので v_slow も
    placeholder になり、A5 適合性チェックは対象外になる（クランプは素直に適用される）。
    """
    import copy
    rows_copy = copy.deepcopy(registry_rows)
    rows_by_name = {r["name"]: r for r in rows_copy}
    rows_by_name["drivetrain_ceiling_mps"]["value"] = 10.0

    clamp_warnings: list[str] = []
    clamp_errors: list[str] = []
    resolved = export.resolve_registry(rows_copy, clamp_warnings=clamp_warnings,
                                        clamp_errors=clamp_errors)

    assert clamp_errors == []
    assert len(clamp_warnings) == 1  # (4) 1 度だけ発火する

    v_max_status, v_max_value = resolved["v_max"]
    assert v_max_status == "derived"
    v_max_natural = derive.v_max_from_ceiling(10.0, rows_by_name["v_max_headroom_ratio"]["value"])
    assert v_max_value < v_max_natural  # (1) 下がっている

    budget = rows_by_name["intrusion_budget_m"]["value"]
    p99 = rows_by_name["link_gap_p99_ms"]["value"]
    lower_esp32 = derive.timeout_lower_bound_ms(p99["esp32"], export.TIMEOUT_MARGIN_RATIO)
    lower_lidar = derive.timeout_lower_bound_ms(p99["lidar"], export.TIMEOUT_MARGIN_RATIO)
    expected_v_max = min(derive.v_max_from_intrusion_budget(budget, lower_esp32),
                          derive.v_max_from_intrusion_budget(budget, lower_lidar))
    assert v_max_value == pytest.approx(expected_v_max)  # (1) 期待値どおり

    # timeout 自体は下限のまま（v_max のクランプに反復して依存しない。P-5）
    esp32_status, esp32_value = resolved["esp32_timeout_ms"]
    lidar_status, lidar_value = resolved["lidar_timeout_ms"]
    assert (esp32_status, esp32_value) == ("derived", pytest.approx(lower_esp32))
    assert (lidar_status, lidar_value) == ("derived", pytest.approx(lower_lidar))

    # (2) 警告が出る。かつ post-clamp では A1 はもう違反しない
    errors, warnings = export.run_assertions(rows_copy, resolved, stage=0, nodes=None,
                                              clamp_warnings=clamp_warnings,
                                              clamp_errors=clamp_errors)
    assert not any("A1" in e for e in errors)
    assert any("N-7" in w for w in warnings)

    # (3) follow_stop_distance_m が新しい（クランプ後の）v_max で再計算されている
    a = rows_by_name["brake_accel_mps2"]["value"]
    t_delay = rows_by_name["brake_delay_s"]["value"]
    person_margin = rows_by_name["person_margin_m"]["value"]
    expected_follow_stop = derive.braking_distance(v_max_value, a, t_delay) + person_margin
    follow_status, follow_value = resolved["follow_stop_distance_m"]
    assert follow_status == "derived"
    assert follow_value == pytest.approx(expected_follow_stop)

    # クランプ前の（自然な）v_max で計算した値とは異なる＝実際に再計算された証拠
    stale_follow_stop = derive.braking_distance(v_max_natural, a, t_delay) + person_margin
    assert follow_value != pytest.approx(stale_follow_stop)


def test_a1_clamp_skipped_when_it_would_break_a5(registry_rows):
    """N-7 選択肢(b): クランプ後の v_max が v_slow を下回り A5（v_slow ≤ v_max）に
    違反する場合は、クランプを適用せず v_max を自然値のまま残す。intrusion budget
    超過が解消されないままなので、A1 が改めて違反を検出して起動を拒否する
    （DetailedDesign-open.md N-7 参照。速度体系全体を比例縮小する選択肢(a)や、
    A1 を満たさないまま起動させる選択肢(c)は採らないと決めた）。"""
    import copy
    rows_copy = copy.deepcopy(registry_rows)
    rows_by_name = {r["name"]: r for r in rows_copy}
    rows_by_name["drivetrain_ceiling_mps"]["value"] = 10.0
    venue = rows_by_name["venue_clearance_m"]
    venue["status"] = "given"
    venue["value"] = 5.0

    clamp_warnings: list[str] = []
    clamp_errors: list[str] = []
    resolved = export.resolve_registry(rows_copy, clamp_warnings=clamp_warnings,
                                        clamp_errors=clamp_errors)

    v_max_status, v_max_value = resolved["v_max"]
    v_max_natural = derive.v_max_from_ceiling(10.0, rows_by_name["v_max_headroom_ratio"]["value"])
    assert v_max_status == "derived"
    assert v_max_value == pytest.approx(v_max_natural)  # クランプを適用していない

    # テスト設定の前提確認: クランプが必要な状況で、かつクランプ後の v_max（v_max_needed）
    # が v_slow を下回る（＝ A5 と衝突する）ことを、クランプと同じ計算で検証する
    budget = rows_by_name["intrusion_budget_m"]["value"]
    p99 = rows_by_name["link_gap_p99_ms"]["value"]
    lower_esp32 = derive.timeout_lower_bound_ms(p99["esp32"], export.TIMEOUT_MARGIN_RATIO)
    lower_lidar = derive.timeout_lower_bound_ms(p99["lidar"], export.TIMEOUT_MARGIN_RATIO)
    v_max_needed = min(derive.v_max_from_intrusion_budget(budget, lower_esp32),
                        derive.v_max_from_intrusion_budget(budget, lower_lidar))
    assert v_max_needed < v_max_natural  # クランプが必要な状況になっている

    v_slow_status, v_slow_value = resolved["v_slow"]
    assert v_slow_status == "derived"
    assert v_slow_value > v_max_needed  # テスト設定として A5 と衝突する状況になっている

    assert clamp_warnings == []
    assert any("A5" in e for e in clamp_errors)

    errors, warnings = export.run_assertions(rows_copy, resolved, stage=0, nodes=None,
                                              clamp_warnings=clamp_warnings,
                                              clamp_errors=clamp_errors)
    assert any("A1" in e for e in errors)  # intrusion budget 超過が残ったまま検出される
    assert any("N-7" in e and "A5" in e for e in errors)  # クランプ見送りの理由も明示される


def test_a1_clamp_disabled_when_apply_v_max_clamp_false(registry_rows):
    """`apply_v_max_clamp=False`（`main()` が `--sim` のときに渡す）では
    クランプを一切行わない（『--sim では A1〜A11 が全てスキップされる』既存挙動を壊さない）。"""
    import copy
    rows_copy = copy.deepcopy(registry_rows)
    rows_by_name = {r["name"]: r for r in rows_copy}
    rows_by_name["drivetrain_ceiling_mps"]["value"] = 10.0

    clamp_warnings: list[str] = []
    clamp_errors: list[str] = []
    resolved = export.resolve_registry(rows_copy, clamp_warnings=clamp_warnings,
                                        clamp_errors=clamp_errors, apply_v_max_clamp=False)
    v_max_status, v_max_value = resolved["v_max"]
    v_max_natural = derive.v_max_from_ceiling(10.0, rows_by_name["v_max_headroom_ratio"]["value"])
    assert v_max_status == "derived"
    assert v_max_value == pytest.approx(v_max_natural)
    assert clamp_warnings == []
    assert clamp_errors == []


def test_main_sim_flag_skips_v_max_clamp(registry_rows, tmp_path):
    """CLI 経由（`--sim`）でも同様にクランプが発火しないこと。
    発火する条件を作った上で `--sim` を付け、生成された safety_monitor.yaml の
    v_max 依存パラメータが自然値のままであることを確認する。"""
    import copy
    import yaml
    rows_copy = copy.deepcopy(registry_rows)
    rows_by_name = {r["name"]: r for r in rows_copy}
    rows_by_name["drivetrain_ceiling_mps"]["value"] = 10.0

    registry_path = tmp_path / "registry.yaml"
    with open(registry_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(rows_copy, f, allow_unicode=True)

    out_dir = tmp_path / "out"
    rc = export.main(["--registry", str(registry_path), "--out", str(out_dir),
                       "--stage", "0", "--sim"])
    assert rc == 0

    v_max_natural = derive.v_max_from_ceiling(10.0, rows_by_name["v_max_headroom_ratio"]["value"])
    with open(out_dir / "params_audit.yaml", encoding="utf-8") as f:
        params_audit_out = yaml.safe_load(f)
    assert params_audit_out["params_audit"]["ros__parameters"]["v_max"] == pytest.approx(
        v_max_natural)


# ---------------------------------------------------------------------------
# N-15（DetailedDesign-open.md）: A13 speed_limit strictness order
#
# th_state.zones.LIMIT_STRICTNESS の並びが registry.yaml の解決済みの数値の昇順と
# 一致しているかを検査する。v_slow / v_reverse / v_jog_panel は現行 registry.yaml では
# 逆算元（venue_clearance_m / blind_clearance_m / panel_clearance_m）が placeholder
# なので、以下は意図的に v_jog_panel の行を status:given に上書きして「measured 値が
# 入って LIMIT_STRICTNESS の位置と食い違った」状況を再現する（レビュー指摘の再発防止）。
# ---------------------------------------------------------------------------
def test_baseline_registry_passes_a13_because_speed_axes_are_still_placeholder(registry_rows):
    """現行 registry.yaml（v_slow/v_reverse/v_jog_panel が placeholder）では
    A13 は比較対象が無く合格する（回帰確認: 導入によって既存の起動が壊れていない）。"""
    resolved = export.resolve_registry(registry_rows)
    errors, _warnings = export.run_assertions(registry_rows, resolved, stage=0, nodes=None)
    assert not any("A13" in e for e in errors)


def test_a13_fires_when_v_jog_panel_measured_value_breaks_the_order(registry_rows):
    """**実際に発火することの確認**（レビュー指摘）。`v_jog_panel` に
    `v_calib`(0.15) より小さい値が測定値として入った状況（＝現在の
    `LIMIT_STRICTNESS` の並び `... v_calib, v_jog_panel, v_slow ...` と矛盾する）を
    作り、`run_assertions()` が実際に A13 違反で起動を拒否することを確認する。"""
    import copy
    rows_copy = copy.deepcopy(registry_rows)
    rows_by_name = {r["name"]: r for r in rows_copy}
    # 逆算式を経由せず、測定済みのふりをして直接値を入れる（PT-2 と同じ「given 扱い」）。
    rows_by_name["v_jog_panel"]["status"] = "given"
    rows_by_name["v_jog_panel"]["value"] = 0.1  # v_calib(0.15) より小さい → 順序矛盾
    rows_by_name["v_jog_panel"]["derived_from"] = None
    rows_by_name["v_jog_panel"]["formula"] = None

    resolved = export.resolve_registry(rows_copy)
    assert resolved["v_jog_panel"] == ("given", 0.1)  # 前提: 実際に concrete な値になっている

    errors, _warnings = export.run_assertions(rows_copy, resolved, stage=0, nodes=None)
    assert any("A13" in e and "v_jog_panel" in e for e in errors)


def test_a13_does_not_fire_when_v_jog_panel_measured_value_matches_the_order(registry_rows):
    """対照テスト: `v_calib`(0.15) より大きい値なら（`LIMIT_STRICTNESS` の位置どおり
    v_calib と v_slow の間に収まっていれば）A13 は違反にならない。"""
    import copy
    rows_copy = copy.deepcopy(registry_rows)
    rows_by_name = {r["name"]: r for r in rows_copy}
    rows_by_name["v_jog_panel"]["status"] = "given"
    rows_by_name["v_jog_panel"]["value"] = 0.3  # v_calib(0.15) より大きい
    rows_by_name["v_jog_panel"]["derived_from"] = None
    rows_by_name["v_jog_panel"]["formula"] = None

    resolved = export.resolve_registry(rows_copy)
    errors, _warnings = export.run_assertions(rows_copy, resolved, stage=0, nodes=None)
    assert not any("A13" in e for e in errors)


def test_main_rejects_startup_when_a13_violated(registry_rows, tmp_path):
    """`export.main()`（launch から呼ばれる CLI 本体）のレベルで、A13 違反が
    終了コード 1（アサーション違反。launch を止める）になることを確認する。"""
    import copy
    import yaml
    rows_copy = copy.deepcopy(registry_rows)
    rows_by_name = {r["name"]: r for r in rows_copy}
    rows_by_name["v_jog_panel"]["status"] = "given"
    rows_by_name["v_jog_panel"]["value"] = 0.1  # v_calib(0.15) より小さい → 順序矛盾
    rows_by_name["v_jog_panel"]["derived_from"] = None
    rows_by_name["v_jog_panel"]["formula"] = None

    registry_path = tmp_path / "registry.yaml"
    with open(registry_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(rows_copy, f, allow_unicode=True)

    out_dir = tmp_path / "out"
    rc = export.main(["--registry", str(registry_path), "--out", str(out_dir), "--stage", "0"])
    assert rc == 1
