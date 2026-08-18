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
    """S5: brake_accel_mps2 が placeholder のあいだ、依存する v_slow も placeholder。"""
    resolved = export.resolve_registry(registry_rows)
    assert resolved["brake_accel_mps2"][0] == "placeholder"
    assert resolved["v_slow"][0] == "placeholder"
    assert resolved["obstacle_stop_distance_m"][0] == "placeholder"


def test_intrusion_budget_m_is_placeholder_and_propagates_safely(registry_rows):
    """N-2（DetailedDesign-open.md §4.1）: intrusion_budget_m は人が決めるまで未決 = placeholder。
    S5 により lidar_timeout_ms / esp32_timeout_ms も placeholder に伝播するが、
    それが正しい振る舞い（伝播して困るなら値を決めるべき、というのが設計の意図）。
    伝播先の解決・生成が例外を投げずに完走することを確認する。
    """
    resolved = export.resolve_registry(registry_rows)
    assert resolved["intrusion_budget_m"][0] == "placeholder"
    assert resolved["lidar_timeout_ms"][0] == "placeholder"
    assert resolved["esp32_timeout_ms"][0] == "placeholder"

    # 生成物が例外なく組み上がり、sentinel も漏れない
    outputs = export.build_node_outputs(registry_rows, resolved)
    assert outputs["safety_monitor"]["intrusion_budget_m"] is None
    assert outputs["safety_monitor"]["lidar_timeout_ms"] is None
    assert outputs["safety_monitor"]["esp32_timeout_ms"] is None

    # A1（intrusion_budget_m を使う起動時アサーション）は placeholder 入力ではガードされ、
    # 例外を投げずに単に評価をスキップすること（export.run_assertions 経由）
    errors = export.run_assertions(registry_rows, resolved, stage=0, nodes=None)
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
