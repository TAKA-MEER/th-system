"""test_params_export.py — th_params/export.py を検証する（WP-PARAM-01 §7）。"""
from th_params import export, schema


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
